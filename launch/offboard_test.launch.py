
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control_1', # needs to have different name for each drone
            output='screen',
            parameters=[
                {'uav_namespace': 'px4_1'}, # px4_1 -> instance 1
                {'mission_id': 1}
            ]
        )
        
        
    ])
