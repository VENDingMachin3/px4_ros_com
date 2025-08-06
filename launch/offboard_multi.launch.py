
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
        ,Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control_2',# needs to have different name for each drone
            output='screen',
            parameters=[
                {'uav_namespace': 'px4_2'},# px4_2 -> instance 2
                {'mission_id': 2}
            ]
        ),
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control_3',# needs to have different name for each drone
            output='screen',
            parameters=[
                {'uav_namespace': 'px4_3'},# px4_3 -> instance 3
                {'mission_id': 3}
            ]
        )
        ,Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control_4', # needs to have different name for each drone
            output='screen',
            parameters=[
                {'uav_namespace': 'px4_4'}, # px4_4 -> instance 4
                {'mission_id': 4}
            ]
        )
        
    ])
