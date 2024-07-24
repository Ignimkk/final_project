from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lrobot',
            executable='robot_control',
            name='robot_control',
            output='screen'
        ),
    ])
