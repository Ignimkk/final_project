from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_vel_to_gpio',
            executable='cmd_vel_to_gpio_node',
            name='cmd_vel_to_gpio_node',
            output='screen'
        ),
    ])
