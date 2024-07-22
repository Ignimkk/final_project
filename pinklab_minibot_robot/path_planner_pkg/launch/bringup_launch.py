import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pid_params_file = os.path.join(
        get_package_share_directory('path_planner_pkg'),
        'config',
        'pid_params.yaml'
    )

    dwb_params_file = os.path.join(
        get_package_share_directory('path_planner_pkg'),
        'config',
        'dwb_local_planner_params.yaml'
    )

    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('path_planner_pkg'),
            'maps',
            'map.yaml'),
        description='Full path to map yaml file to load'
    )

    pid_controller_node = Node(
        package='path_planner_pkg',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[pid_params_file]
    )

    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'bringup_launch.py'
        )),
        launch_arguments={
            'params_file': dwb_params_file,
            'map': LaunchConfiguration('map')
        }.items()
    )

    return LaunchDescription([
        map_yaml_file,
        pid_controller_node,
        navigation2_launch
    ])
