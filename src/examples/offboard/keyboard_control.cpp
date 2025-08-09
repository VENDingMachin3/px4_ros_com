#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <termios.h>
#include <unistd.h>
#include <chrono>

using namespace std::chrono_literals;

class KeyboardControl : public rclcpp::Node {
public:
    KeyboardControl() : Node("keyboard_control_node") {
        input_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/keyboard_input", 10);
        position_.x = 0.0;
        position_.y = 0.0;
        position_.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Keyboard Control Started (WASDQE)");

        timer_ = this->create_wall_timer(100ms, std::bind(&KeyboardControl::read_key_and_publish, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr input_pub_;
    geometry_msgs::msg::Vector3 position_;
    rclcpp::TimerBase::SharedPtr timer_;

    void read_key_and_publish() {
        char c = getch();
        switch (c) {
            case 'w':
                position_.x = 1;
                position_.y = 0;
                position_.z = 0; 
                break;
            
            case 's': 
                position_.x = -1;
                position_.y = 0;
                position_.z = 0; 
                break;
            case 'a': 
                position_.x = 0;
                position_.y = 1;
                position_.z = 0; 
                break;
            case 'd': 
                position_.x = 0;
                position_.y = -1;
                position_.z = 0; 
                break;
            case 'q': 
                position_.x = 0;
                position_.y = 0;
                position_.z = 1; 
                break;
            case 'e': 
                position_.x = 0;
                position_.y = 0;
                position_.z = -1; 
                break;
            default: return;
        }

        RCLCPP_INFO(this->get_logger(), "Keyboard key: %c", c);

        input_pub_->publish(position_);
    }

    char getch() {
        struct termios oldt, newt;
        char ch;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardControl>());
    rclcpp::shutdown();
    return 0;
}
