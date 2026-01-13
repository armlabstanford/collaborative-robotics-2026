"""
Real Hardware Launch File for TidyBot2.

Launches:
- Interbotix arm drivers (real hardware)
- RealSense camera driver
- Robot state publisher (URDF + TF)
- Arm controllers (left + right)
- RViz (optional)

Usage:
    ros2 launch tidybot_bringup real.launch.py
    ros2 launch tidybot_bringup real.launch.py use_rviz:=false

Prerequisites:
- Interbotix SDK installed: pip install interbotix-xs-modules
- RealSense SDK installed: pip install pyrealsense2
- Arms connected via USB
- RealSense camera connected via USB
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_bringup = FindPackageShare('tidybot_bringup')
    pkg_description = FindPackageShare('tidybot_description')

    # Declare arguments
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for visualization'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',  # FALSE for real hardware!
        description='Use simulation time (false for real robot)'
    )

    # URDF from xacro
    urdf_path = PathJoinSubstitution([pkg_description, 'urdf', 'tidybot_wx200.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_path])

    # ========== NODES ==========

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Right arm HARDWARE interface (replaces MuJoCo bridge for right arm)
    right_arm_hw = Node(
        package='tidybot_control',
        executable='interbotix_arm_node',
        name='right_arm_hw',
        output='screen',
        parameters=[{
            'arm_name': 'right',
            'robot_model': 'wx200',
            'publish_rate': 100.0,
        }]
    )

    # Left arm HARDWARE interface (replaces MuJoCo bridge for left arm)
    left_arm_hw = Node(
        package='tidybot_control',
        executable='interbotix_arm_node',
        name='left_arm_hw',
        output='screen',
        parameters=[{
            'arm_name': 'left',
            'robot_model': 'wx200',
            'publish_rate': 100.0,
        }]
    )

    # Right arm controller (SAME as simulation!)
    right_arm_controller = Node(
        package='tidybot_control',
        executable='arm_controller_node',
        name='right_arm_controller',
        output='screen',
        parameters=[{
            'arm_name': 'right',
            'control_rate': 50.0,
        }]
    )

    # Left arm controller (SAME as simulation!)
    left_arm_controller = Node(
        package='tidybot_control',
        executable='arm_controller_node',
        name='left_arm_controller',
        output='screen',
        parameters=[{
            'arm_name': 'left',
            'control_rate': 50.0,
        }]
    )

    # RealSense camera (replaces MuJoCo camera rendering)
    # Uses the official realsense2_camera package
    # Install: sudo apt install ros-humble-realsense2-camera
    realsense_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            'color_fps': 30.0,
            'depth_fps': 30.0,
        }],
        remappings=[
            # Remap to match simulation topic names
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera/depth/image_raw'),
        ]
    )

    # RViz
    rviz_config = PathJoinSubstitution([pkg_bringup, 'rviz', 'tidybot.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        # Arguments
        declare_use_rviz,
        declare_use_sim_time,

        # Nodes
        robot_state_publisher,

        # HARDWARE DRIVERS (instead of MuJoCo bridge)
        right_arm_hw,
        left_arm_hw,
        realsense_camera,

        # CONTROLLERS (same as simulation!)
        right_arm_controller,
        left_arm_controller,

        rviz,
    ])
