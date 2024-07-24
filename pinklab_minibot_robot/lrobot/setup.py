from setuptools import find_packages, setup
import os

package_name = 'lrobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lrobot_launch.py']),
        ('share/' + package_name + '/config', ['lrobot/poses.json']),  
        (os.path.join(package_name, 'lrobot'), ['lrobot/poses.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ys',
    maintainer_email='ysu0415@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control = lrobot.robot_control:main',
            'ui_node = lrobot.ui_node:main'
        ],
    },
)
