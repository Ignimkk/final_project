from setuptools import setup
import os
from glob import glob

package_name = 'path_planner_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/dwb_local_planner_params.yaml', 'config/pid_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/bringup_launch.py']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools', 'Pillow'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_publisher = path_planner_pkg.goal_publisher:main',
            'goal_publisher_with_click = path_planner_pkg.goal_publisher_with_click:main',
            'path_planner = path_planner_pkg.path_planner:main',
            'pid_controller = path_planner_pkg.pid_controller:main',
            'navigate_to_pose_client = path_planner_pkg.navigate_to_pose_client:main',
            'path_planning_node = path_planner_pkg.path_planning_node:main',
            'test = path_planner_pkg.test:main',
            'odom_publisher = path_planner_pkg.odom_publisher:main',
        ],
    },
)
