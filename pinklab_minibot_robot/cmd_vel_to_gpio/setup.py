from setuptools import setup, find_packages

package_name = 'cmd_vel_to_gpio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cmd_vel_to_gpio_launch.py']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A package to convert cmd_vel to GPIO signals',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = cmd_vel_to_gpio.cmd_vel_to_gpio_node:main',
        ],
    },
)
