from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the "mode" argument with a default value of 0
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',  # for default mode
        description='Mode selection: 0, 1, or 2  -> Default, Auto, or Keyboard Control '
    )

    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        mode_arg,  # Add the launch argument to the LaunchDescription

        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control_1',
            output='screen',
            parameters=[
                {'uav_namespace': 'px4_1'},
                {'mission_id': 1},
                {'mode': mode}  # Pass the mode to the node
            ]
        ),
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control_2',
            output='screen',
            parameters=[
                {'uav_namespace': 'px4_2'},
                {'mission_id': 2},
                {'mode': mode}
            ]
        ),
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control_3',
            output='screen',
            parameters=[
                {'uav_namespace': 'px4_3'},
                {'mission_id': 3},
                {'mode': mode}
            ]
        ),
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control_4',
            output='screen',
            parameters=[
                {'uav_namespace': 'px4_4'},
                {'mission_id': 4},
                {'mode': mode}
            ]
        )
    ])
