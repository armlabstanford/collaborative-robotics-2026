"""
Phone Teleoperation Launch File for TidyBot2.

Launches the phone teleop server that provides a web interface
for controlling the robot from a mobile phone browser.

This should be launched AFTER the main robot launch file
(sim.launch.py or real.launch.py).

Usage:
    # Terminal 1: Start simulation (or real hardware)
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Start phone teleop server
    ros2 launch tidybot_bringup phone_teleop.launch.py

    # Then open http://<ROBOT_IP>:5000 on your phone
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_port = DeclareLaunchArgument(
        'port', default_value='5000',
        description='Port for the phone teleop web server'
    )
    declare_host = DeclareLaunchArgument(
        'host', default_value='0.0.0.0',
        description='Host address to bind the web server'
    )

    phone_teleop_node = Node(
        package='tidybot_control',
        executable='phone_teleop_server',
        name='phone_teleop_server',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'host': LaunchConfiguration('host'),
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_host,
        phone_teleop_node,
    ])
