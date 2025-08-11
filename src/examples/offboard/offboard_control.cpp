#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>
#include <array>
#include <cmath>
#include <sstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cstdlib>
#include <geometry_msgs/msg/vector3.hpp>

// Helper: build a float[3] from doubles
inline std::array<float, 3> make_position(double x, double y, double z) {
    return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}


using namespace std::chrono_literals;
using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
using VehicleCommand     = px4_msgs::msg::VehicleCommand;
using VehicleOdometry    = px4_msgs::msg::VehicleOdometry;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl()
    : Node("offboard_control")
    {
        // --- set the lader timer ---
        last_leader_odom_time_ = this->now();
        // --- parameters ---
        declare_parameter<std::string>("uav_namespace", "px4_1");
        declare_parameter<int>("mission_id", 1);
        declare_parameter<int>("mode", 0);

        get_parameter("uav_namespace", uav_namespace_);
        get_parameter("mission_id", mission_id_);
        get_parameter("mode", mode_);

        // derive system ID from namespace suffix ("px4_2" → 2 → +1 → 3)
        inst_ = std::stoi(uav_namespace_.substr(uav_namespace_.find('_') + 1));
        target_system_id_ = static_cast<uint8_t>(inst_ + 1);
        
        // Handeling Mode Selection
        if(mode_ == 2){
            RCLCPP_INFO(this->get_logger(), "Running in MODE 2 : Keyboard Control is available");
            leader_max_accel_ = 2.5;
            using_keyboard_input_ = true;
        }
        else if(mode_ == 1){
            RCLCPP_INFO(this->get_logger(), "Running in MODE 1 : Auto Mode is active");
            auto_mode_ = true;
        }


        std::string prefix = "/" + uav_namespace_;

        // --- publishers ---
        offb_mode_pub_ = create_publisher<OffboardControlMode>(prefix + "/fmu/in/offboard_control_mode", 10);
        traj_sp_pub_   = create_publisher<TrajectorySetpoint>(prefix + "/fmu/in/trajectory_setpoint", 10);
        cmd_pub_       = create_publisher<VehicleCommand>(prefix + "/fmu/in/vehicle_command", 10);
        if(inst_ == 1) leader_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/leader_odometry1", rclcpp::QoS(10).best_effort());
        else if(inst_ == 2) leader_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/leader_odometry2", rclcpp::QoS(10).best_effort());
        
        
        // --- subs ---
        // PX4 publishes VehicleOdometry with Best Effort reliability
        rclcpp::QoS odom_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                                  .best_effort()
                                  .durability_volatile();
        
        // Self Odom Subscription 
        std::string self_topic = prefix + "/fmu/out/vehicle_odometry";
        self_odom_sub_ = create_subscription<VehicleOdometry>(
            self_topic,
            odom_qos,
            [this](VehicleOdometry::SharedPtr msg) {
                self_odom_ = *msg;
            }
        );
        // Leader Odom Sub
        leader_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/leader_odometry1", odom_qos,
    [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {

        // previous PX4 timestamp stored in the last received message
        uint64_t prev_px4_ts = leader_odom_.timestamp; // 0 if none yet
        uint64_t new_px4_ts  = msg->timestamp;       // PX4 timestamps are microseconds

        // Minimum advance to consider this a new update (in microseconds).
        // Use 0 if you want any change to count, otherwise a small threshold
        // helps ignore duplicate/identical repeated messages.
        const uint64_t MIN_DELTA_US = 1000; // 1 ms

        if (prev_px4_ts == 0) {
            // First message ever received for this subscription.
            // Option: you can add extra checks here (age vs now) if desired.
            leader_odom_ = *msg;
            last_leader_odom_time_ = this->now();
            received_leader_pose_ = true;
            RCLCPP_INFO(get_logger(), "First leader odom accepted (px4 ts: %lu).", new_px4_ts);
        } else if (new_px4_ts > prev_px4_ts + MIN_DELTA_US) {
            // PX4 timestamp advanced — this is a fresh update; accept it.
            leader_odom_ = *msg;
            last_leader_odom_time_ = this->now();
            received_leader_pose_ = true;
            // Optional debug:
            // RCLCPP_DEBUG(get_logger(), "Fresh leader odom (delta_us=%lu).", new_px4_ts - prev_px4_ts);
        } else {
            // Timestamp has not advanced (duplicate/stale). Ignore for timeout purposes.
            RCLCPP_DEBUG(get_logger(), "Ignoring stale/duplicate leader odom (px4 ts: %lu).", new_px4_ts);
        }
    });
       
        
        // Getting Waypoits using Topic
        
        std::string waypoint_topic = "/leader/waypoint";
        waypoint_sub_ = create_subscription<TrajectorySetpoint>(
            waypoint_topic, 10,
            [this](TrajectorySetpoint::SharedPtr msg) {
                if(inst_ == leader_id_){
                current_target_ = *msg;
                new_waypoint_ = true;
                RCLCPP_INFO(get_logger(), "New waypoint received by leader %d.", inst_ );
                }
            }
        );
        // subscribe to text commands
        
        command_sub_ = create_subscription<std_msgs::msg::String>(
            "/control_command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::string s = msg->data;
                std::vector<std::string> tokens;
                std::stringstream ss(s);
                std::string item;
                while (std::getline(ss, item, ',')) {
                    // trim whitespace
                    item.erase(item.begin(), std::find_if(item.begin(), item.end(), [](unsigned char c){ return !std::isspace(c); }));
                    item.erase(std::find_if(item.rbegin(), item.rend(), [](unsigned char c){ return !std::isspace(c); }).base(), item.end());
                    tokens.push_back(item);
                }
                if (tokens.size() == 4) {
                    std::string cmd = tokens[0];
                    try {
                        formation_length_ = std::stod(tokens[1]);
                        formation_direction_ = static_cast<char>(tokens[2][0]);
                        formation_type_ = tokens[3];
                    } catch (const std::exception &e) {
                        RCLCPP_WARN(get_logger(), "Error parsing control_command: %s", e.what());
                        return;
                    }
                    RCLCPP_INFO(get_logger(), "Parsed command='%s', length=%.1f, dir='%c', formation='%s'",
                                cmd.c_str(), formation_length_, formation_direction_, formation_type_.c_str());
                        if (cmd == "disarm") {
                            if (inst_ == leader_id_) {
                                leader_odom_pub_.reset(); // kill the leader's odom publisher 
                                rclcpp::sleep_for(std::chrono::seconds(3));
                                timer_->cancel();
                                publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0.0);
                                return;
                            }                      
                        }
                    if (formation_type_ == "triangle") triangle(formation_length_, formation_direction_);
                    else if (formation_type_ == "square") square(formation_length_, formation_direction_);
                    else if (formation_type_ == "line")   line(formation_length_, formation_direction_);
                    else RCLCPP_WARN(get_logger(), "Unknown formation '%s'", formation_type_.c_str());
                } else {
                    RCLCPP_WARN(get_logger(), "Invalid format: '%s'", s.c_str());
                }
            }
        );
        
        // ------------------------- Auto Mode ------------------------------
        // setting path_ for leader in auto mode:
        if(auto_mode_){
            if(inst_ == leader_id_) set_path_();
            
            path_index_pub_ = this->create_publisher<std_msgs::msg::UInt32>("leader/path_index", 10);

            path_index_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
            "leader/path_index", 10,
            [this](const std_msgs::msg::UInt32::SharedPtr msg) {
                last_known_wp_idx_ = msg->data;
            });
        }

        //--------------------------------------------------------------------
        // keyboard Sub
        keyboard_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
            "/keyboard_input", 10,
            [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
                keyboard_input_ = *msg;
                current_target_.position[0] += keyboard_input_.x;
                current_target_.position[1] += keyboard_input_.y;
                current_target_.position[2] += keyboard_input_.z;
                RCLCPP_INFO(get_logger(), "New waypoint received from Keyboard Input.");
                new_waypoint_ = true;
                last_keyboard_input_time_ = this->now();
                using_keyboard_input_ = true;
            }
        );


        // Default Formation:
        triangle(5, 'h');

        // --- offboard timer ---
        timer_ = create_wall_timer(100ms, [this]() {
            if (received_leader_pose_) {
                auto now_time = this->now();
                double dt = (now_time - last_leader_odom_time_).seconds();

                // switch leader
                if (dt > leader_timeout_sec_ && !using_backup_leader_) {
                    RCLCPP_WARN(get_logger(), "Leader timeout. Switching to px4_2 as leader.");
                    if(inst_ == leader_id_+1) rclcpp::sleep_for(std::chrono::milliseconds(100)); // this is just to make sure all drones execute this part
                    using_backup_leader_ = true;
                    switch_to_new_leader("/leader_odometry2");
                    // update offsets here too
                    
                }
            }
            
            
            // switch to offboard + arm
            if (offboard_sp_counter_ == 10) {
                publish_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            }

            // leader publishes odometry 
            if (inst_ == leader_id_) {
                leader_odom_pub_->publish(self_odom_);
            }

            // publish offboard mode
            OffboardControlMode m{};
            m.position = true;
            m.timestamp = now().nanoseconds() / 1000;
            offb_mode_pub_->publish(m);

            // Publish path index if Auto mode is true
            if(auto_mode_){
                if (mission_id_ == leader_id_) {
                    std_msgs::msg::UInt32 msg;
                    msg.data = current_wp_idx_;
                    path_index_pub_->publish(msg);
                }
            }
            
            // publish trajectory
            publish_trajectory();

            offboard_sp_counter_++;
        });
    }


private:
    // compute formation offsets (NED frame)
        std::array<std::array<double,3>,5> offsets_ned = {{
            {{ 0.0,  0.0,  0.0}},  // unused 0
            {{ 0.0,  0.0,  0.0}},  // px4_1 (leader)
            {{ 0.0,  0.0,  0.0}},  // px4_2
            {{ 0.0,  -2.0,  0.0}},  // px4_3
            {{ 0.0,  0.0,  0.0}}   // px4_4
        }};
    
    // ------------------ formations ----------------------
    // ---- formation params ----
    std::string formation_type_ = "triangle";
    double formation_length_ = 5.0;
    char formation_direction_ = 'h';
    bool triangle_active{false};
    bool square_active{false};
    bool line_active{false};
    
    // triangle:

        void triangle(double len, char direction)
        {
            triangle_active = true;
            square_active = false;
            line_active = false;

            double height;
            double half_base;
            if(leader_id_ == 1){
                // height
                height = sqrt(3)/2 * len; 
                double offset_height = -(height -2);
                
                // base
                half_base = (len-4)/2;

                if(direction == 'h'){
                    RCLCPP_INFO(get_logger(), "Changing Formation to triangle 'h'");
                    offsets_ned[2] = {-half_base , offset_height , 0};
                    offsets_ned[3] = { 0, offset_height , 0};
                    offsets_ned[4] = { half_base, offset_height , 0};

                }
                else if(direction == 'v'){
                    RCLCPP_INFO(get_logger(), "Changing Formation to triangle 'v'");
                    offsets_ned[2] = {-half_base , 2 , -height};
                    offsets_ned[3] = { 0, 2 , -height};
                    offsets_ned[4] = { half_base, 2 , -height};
                }
            }
            else if(leader_id_ == 2){
                height = sqrt(3)/2 * len;
                half_base = len/2;
                if(direction == 'h'){
                    offsets_ned[3] = { -half_base - 2, -height , 0};
                    offsets_ned[4] = { half_base - 4 , -height , 0};
                }
                else if(direction =='v'){
                    offsets_ned[3] = { -half_base - 2, 0 , -height};
                    offsets_ned[4] = { half_base - 4 , 0, -height};
                }
            }
            
            off_x_ = offsets_ned[inst_][0];
            off_y_ = offsets_ned[inst_][1];
            off_z_ = offsets_ned[inst_][2];
        }
    // square:

        void square(double len, char direction){
            triangle_active = false;
            square_active = true;
            line_active = false;

            double diameter = sqrt(2) * len;
            if(leader_id_ == 1){
                if(direction == 'h') {
                    RCLCPP_INFO(get_logger(), "Changing Formation to square 'h'");
                    offsets_ned[2] = {-(diameter/2) + 2 , -(diameter/2) + 2  , 0};
                    offsets_ned[3] = { 0, -diameter +2 , 0};
                    offsets_ned[4] = { (diameter/2) - 2 , -(diameter/2) + 2  , 0};
                }
                else if(direction == 'v'){
                    RCLCPP_INFO(get_logger(), "Changing Formation to square 'v'");
                    offsets_ned[2] = {-(diameter/2) + 2 , 2  , -(diameter/2)};
                    offsets_ned[3] = { 0, 2 , -diameter};
                    offsets_ned[4] = { (diameter/2) - 2 , 2  , -(diameter/2)};
                }
            }
            else if(leader_id_ == 2){
                RCLCPP_INFO(get_logger(), "Square formation is not available for 3 drones.");
            } 

            off_x_ = offsets_ned[inst_][0];
            off_y_ = offsets_ned[inst_][1];
            off_z_ = offsets_ned[inst_][2];
        }

    // line:
        void line(double len, char direction){
            triangle_active = false;
            square_active = false;
            line_active = true;
            
            if(leader_id_ == 1){
                if(direction =='h'){
                        RCLCPP_INFO(get_logger(), "Changing Formation to line 'h'");
                        offsets_ned[2] = { 2 , -len/3 +2 , 0};
                        offsets_ned[3] = { 0, -len+2 , 0};
                        offsets_ned[4] = { -2 , -2*len/3 +2 , 0};
                    }
                else if(direction == 'v'){
                    RCLCPP_INFO(get_logger(), "Changing Formation to line 'v'");
                        offsets_ned[2] = { 2, 2 , -len/3  };
                        offsets_ned[3] = {  0, 2 , -len  };
                        offsets_ned[4] = { -2 , 2, -2*len/3  };
                    }
                }
            
            else if(leader_id_ == 2){
                if(direction =='h'){
                        offsets_ned[3] = { -2, -len/2 , 0};
                        offsets_ned[4] = { -4 , -len  , 0};
                    }
                else if(direction == 'v'){
                        offsets_ned[3] = { -2 , 0 , -len/2  };
                        offsets_ned[4] = { -4 , 0, -len };
                    }
            }
            off_x_ = offsets_ned[inst_][0];
            off_y_ = offsets_ned[inst_][1];
            off_z_ = offsets_ned[inst_][2];
        }

    // leader situation
    rclcpp::Time last_leader_odom_time_;
    double leader_timeout_sec_ = 0.5;  // time to wait before declaring leader dead
    bool using_backup_leader_ = false;

    // change leader
    void switch_to_new_leader(const std::string& new_topic) {
        RCLCPP_INFO(get_logger(), "Running switch Function.... ");
        leader_id_ ++;  // update leader ID
        leader_odom_sub_.reset();
        
        RCLCPP_INFO(get_logger(), "New Leader ID: %d", leader_id_);
        if(inst_ == leader_id_){
            set_path_(); // let the new leader to know the path 
            current_wp_idx_ = last_known_wp_idx_;
            RCLCPP_INFO(get_logger(), "Current idx: %ld", current_wp_idx_);
            RCLCPP_INFO(get_logger(), "Last Known idx: %d", last_known_wp_idx_);    
        }

        leader_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        new_topic ,
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
        [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            leader_odom_ = *msg;
            last_leader_odom_time_ = this->now();
            received_leader_pose_ = true;
        });

        if (triangle_active) triangle(formation_length_, formation_direction_);
        else if (square_active) square(formation_length_, formation_direction_);
        else if (line_active) line(formation_length_, formation_direction_);   
        }


    // --- ROS interfaces ---
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr  offb_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr   traj_sp_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr       cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr path_index_pub_; // Auto Mode
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr leader_odom_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr keyboard_sub_; // keyboard
    rclcpp::Subscription<VehicleOdometry>::SharedPtr   self_odom_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr   leader_odom_sub_;
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr path_index_sub_; // Auto mode
    
    // --- state & params ---
    std::string uav_namespace_;
    int inst_ = 0;
    int mission_id_;
    uint8_t target_system_id_{1};
    int leader_id_ =1;
    int mode_ = 0;                           // Mode param
    bool auto_mode_ = false;                 // Auto Mode
    std::vector<std::array<float, 3>> path_; // Auto mode 
    size_t current_wp_idx_ = 0;              // Auto mode
    uint32_t last_known_wp_idx_ = 0;         // Auto mode
    VehicleOdometry leader_odom_;
    VehicleOdometry self_odom_;
    
    // Keyboard Params
    geometry_msgs::msg::Vector3 keyboard_input_;
    rclcpp::Time last_keyboard_input_time_;
    bool using_keyboard_input_ = false;

    //--------------------------------------- speed sync
    const int    leader_wait_threshold_ = 20;      // ticks at 100 ms → 2 s hold
    const double leader_max_speed_      = 12.0;    // m/s
    double leader_max_accel_            = 1.5;     // m/s²
    const double leader_dt_             = 0.125;   // seconds per control loop (100/80 Hz)

    // For leader waypoint motion
    double leader_current_speed_ = 0.0;
    bool leader_started_ = false;
    int leader_wait_counter_ = 0;
    //----------------------------------------
    
    //----------------- Path For Auto Mode --------------------
    void set_path_() {
    path_= {
        {0.0f, 0.0f, -5.0f},
        {10.0f, 10.0f, -5.0f},
        {20.0f, 50.0f, -10.0f},
        {15.0f, -10.0f, -10.0f},
        {-15.0f, 20.0f, -10.0f},        
        {0.0f, 0.0f, -5.0f},
        {0.0f, 0.0f, 0.0f}
        };
    }   

    // Waypoint to move the formation
    TrajectorySetpoint current_target_;

    // formation offset (for followers)
    double off_x_{0}, off_y_{0}, off_z_{0};
    
    
    // counters & flags
    uint64_t offboard_sp_counter_{0};
    bool disarming_ = false;
    bool new_waypoint_ = false;
    bool received_leader_pose_ = false;
    

    // send a vehicle_command
    void publish_command(uint16_t cmd, float p1 = 0.0f, float p2 = 0.0f) {
        VehicleCommand vc{};
        vc.command          = cmd;
        vc.param1           = p1;
        vc.param2           = p2;
        vc.target_system    = target_system_id_;
        vc.target_component = 1;
        vc.source_system    = target_system_id_;
        vc.source_component = 1;
        vc.from_external    = true;
        vc.timestamp        = now().nanoseconds() / 1000;
        cmd_pub_->publish(vc);
    }

    // build and publish the trajectory setpoint
    void publish_trajectory() {
        TrajectorySetpoint sp{};
        sp.timestamp = now().nanoseconds() / 1000;

        double lx = 0.0, ly = 0.0, lz = 0.0;

        if(received_leader_pose_){
            lx = leader_odom_.position[0];
            ly = leader_odom_.position[1];
            lz = leader_odom_.position[2];
        }

        
        if ((mission_id_ == leader_id_ || !received_leader_pose_)) {
            // leader logic
            if(offboard_sp_counter_ <= 100){
                sp.position = make_position(lx, ly, -5.0);
            }
            else {
                
                if(new_waypoint_){
                    Eigen::Vector3d current_pos(lx, ly, lz);
                    Eigen::Vector3d goal(current_target_.position[0],
                                        current_target_.position[1],
                                        current_target_.position[2]);

                    Eigen::Vector3d delta = goal - current_pos;
                    double dist = delta.norm();

                    if (dist < 0.3) {
                        leader_current_speed_ = 0.0;
                        leader_started_       = false;
                        if(auto_mode_) current_wp_idx_++;
                        new_waypoint_ = false;
                        RCLCPP_INFO(get_logger(), "Holding Position...");
                        sp.position = make_position(goal.x(), goal.y(), goal.z());
                    } 
                    else {
                        if (!leader_started_) {
                            if (leader_wait_counter_++ < leader_wait_threshold_) {
                                sp.position = make_position(current_pos.x(),
                                                            current_pos.y(),
                                                            current_pos.z());
                            } else {
                                leader_started_ = true;
                            }
                            traj_sp_pub_->publish(sp);
                            return;
                        }

                        if (leader_current_speed_ < leader_max_speed_) {
                            leader_current_speed_ += leader_max_accel_ * leader_dt_;
                            if (leader_current_speed_ > leader_max_speed_) {
                                leader_current_speed_ = leader_max_speed_;
                            }
                        }
                        Eigen::Vector3d dir = delta.normalized();
                        RCLCPP_INFO(get_logger(), "Distance to waypoint: %.2f", dist);
                        Eigen::Vector3d vel = dir * leader_current_speed_;
                        Eigen::Vector3d next = current_pos + vel * leader_dt_;
                        sp.position = make_position(next.x(), next.y(), next.z());
                    }
                }

                else {
                    sp.position = make_position(lx, ly, lz);                    
                    if (auto_mode_ && current_wp_idx_ < path_.size()) {
                        current_target_.position[0] = path_[current_wp_idx_][0];
                        current_target_.position[1] = path_[current_wp_idx_][1];
                        current_target_.position[2] = path_[current_wp_idx_][2];
                        new_waypoint_ = true;
                    }  
                }
            } 
        } 
        else {
            if(offboard_sp_counter_<= 70){
                sp.position = make_position(0.0, 0.0, -5.0);
            }
            else {
                sp.position = make_position(lx + off_x_, ly + off_y_, lz + off_z_);
            }
            
        }

        sp.yaw = 0.0f;
        traj_sp_pub_->publish(sp);
    }
    
    // publish trajectory on keyboard mode
//     void publish_trajectory_kb(){
//         TrajectorySetpoint sp{};
//         sp.timestamp = now().nanoseconds() / 1000;
//         Eigen::Vector3d current_pos(self_odom_.position[0], self_odom_.position[1], self_odom_.position[2]);

//         double lx = 0.0, ly = 0.0, lz = 0.0;

//         if(received_leader_pose_){
//             lx = leader_odom_.position[0];
//             ly = leader_odom_.position[1];
//             lz = leader_odom_.position[2];
//         }

//         if(new_waypoint_){
//             if(leader_id_ == mission_id_){
//                 Eigen::Vector3d goal(current_target_.position[0],
//                                     current_target_.position[1],
//                                     current_target_.position[2]);
//                 Eigen::Vector3d delta = goal - current_pos;
//                 double dist = delta.norm();    
//                 if (dist < 0.3) {
//                     new_waypoint_ = false;
//                     RCLCPP_INFO(get_logger(), "Holding Position...");
//                     sp.position = make_position(goal.x(), goal.y(), goal.z());
//                 }
//                 else{
//                     sp.position = make_position(goal.x(), goal.y(), goal.z());
//                 } 
//             }    
//         }
        
//         else{
//             if(offboard_sp_counter_<= 70){
//                 sp.position = make_position(0, 0, -5);
//             }
//             else{
//                 sp.position = make_position(current_pos[0],current_pos[1], current_pos[2]);
//             }
            
//         }

//         sp.yaw = 0.0f;
//         traj_sp_pub_->publish(sp);
//     }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}