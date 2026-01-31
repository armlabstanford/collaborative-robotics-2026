"""
Real Hardware Launch File for TidyBot2.

Launches:
- Phoenix 6 mobile base driver
- Interbotix xs_sdk arm drivers (dual U2D2 setup)
- RealSense camera driver
- Robot state publisher (URDF + TF)
- Image compression (optional, for remote clients)
- RViz (optional)

Hardware Setup (Dual U2D2):
    - U2D2 #1 (/dev/ttyUSB0): Right arm (IDs 1-9) + Pan-tilt (IDs 21-22)
    - U2D2 #2 (/dev/ttyUSB1): Left arm (IDs 11-19)

Usage:
    ros2 launch tidybot_bringup real.launch.py
    ros2 launch tidybot_bringup real.launch.py use_rviz:=false
    ros2 launch tidybot_bringup real.launch.py use_base:=false  # Disable base
    ros2 launch tidybot_bringup real.launch.py use_left_arm:=false  # Right arm only

Prerequisites:
- Phoenix 6 library installed (via uv): uv add phoenix6
- Interbotix SDK installed: source ~/interbotix_humble_ws/install/setup.bash
- RealSense SDK installed: sudo apt install ros-humble-realsense2-camera
- Arms connected via USB (dual U2D2 setup)
- RealSense camera connected via USB
- Phoenix 6 CAN bus connected
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional logic for xs_sdk nodes."""
    pkg_bringup = FindPackageShare('tidybot_bringup')
    pkg_description = FindPackageShare('tidybot_description')

    # Get launch configuration values
    use_base = LaunchConfiguration('use_base').perform(context) == 'true'
    use_arms = LaunchConfiguration('use_arms').perform(context) == 'true'
    use_left_arm = LaunchConfiguration('use_left_arm').perform(context) == 'true'
    use_pan_tilt = LaunchConfiguration('use_pan_tilt').perform(context) == 'true'
    use_camera = LaunchConfiguration('use_camera').perform(context) == 'true'
    use_compression = LaunchConfiguration('use_compression').perform(context) == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    load_configs = LaunchConfiguration('load_configs').perform(context) == 'true'

    # Get project root for uv packages
    tidybot2_path = os.environ.get('TIDYBOT2_PATH', '/home/locobot/tidybot2')
    home_dir = os.path.dirname(tidybot2_path)
    project_root = os.path.join(home_dir, 'collaborative-robotics-2026')

    # UV virtual environment site-packages
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    uv_site_packages = os.path.join(project_root, '.venv', 'lib', f'python{python_version}', 'site-packages')

    # Build PYTHONPATH with uv packages
    existing_pythonpath = os.environ.get('PYTHONPATH', '')
    new_pythonpath = f"{uv_site_packages}:{existing_pythonpath}" if existing_pythonpath else uv_site_packages

    # Environment for nodes needing uv packages
    hw_node_env = {
        'PYTHONPATH': new_pythonpath,
        'TIDYBOT2_PATH': '/home/locobot/tidybot2',
    }

    nodes = []

    # URDF from xacro
    urdf_path = PathJoinSubstitution([pkg_description, 'urdf', 'tidybot_wx250s.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_path])

    # Robot state publisher (always needed)
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    ))

    # Joint state aggregator - combines joint states from both arms into /joint_states
    # This is needed because each xs_sdk publishes to its own namespace
    if use_arms:
        source_list = ['/right_arm/joint_states']
        if use_left_arm:
            source_list.append('/left_arm/joint_states')

        nodes.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_aggregator',
            output='screen',
            parameters=[{
                'source_list': source_list,
                'rate': 50,
            }]
        ))

    # Phoenix 6 mobile base driver
    if use_base:
        nodes.append(Node(
            package='tidybot_control',
            executable='phoenix6_base_node',
            name='phoenix6_base',
            output='screen',
            additional_env=hw_node_env,
            parameters=[{
                'max_linear_vel': 0.5,
                'max_angular_vel': 1.57,
                'max_linear_accel': 0.25,
                'max_angular_accel': 0.79,
                'publish_rate': 50.0,
            }]
        ))

    # Interbotix xs_sdk nodes for arms
    if use_arms:
        # Right arm + pan-tilt on U2D2 #1 (/dev/ttyUSB0)
        if use_pan_tilt:
            right_motor_config = PathJoinSubstitution([pkg_bringup, 'config', 'right_arm_pantilt.yaml'])
            right_mode_config = PathJoinSubstitution([pkg_bringup, 'config', 'right_arm_pantilt_modes.yaml'])
        else:
            right_motor_config = PathJoinSubstitution([pkg_bringup, 'config', 'right_arm.yaml'])
            right_mode_config = PathJoinSubstitution([pkg_bringup, 'config', 'modes.yaml'])

        nodes.append(Node(
            package='interbotix_xs_sdk',
            executable='xs_sdk',
            name='xs_sdk',
            namespace='right_arm',
            parameters=[{
                'motor_configs': right_motor_config,
                'mode_configs': right_mode_config,
                'load_configs': load_configs,
            }],
            output='screen',
        ))

        # Left arm on U2D2 #2 (/dev/ttyUSB1)
        if use_left_arm:
            left_motor_config = PathJoinSubstitution([pkg_bringup, 'config', 'left_arm.yaml'])
            left_mode_config = PathJoinSubstitution([pkg_bringup, 'config', 'left_arm_modes.yaml'])

            nodes.append(Node(
                package='interbotix_xs_sdk',
                executable='xs_sdk',
                name='xs_sdk',
                namespace='left_arm',
                parameters=[{
                    'motor_configs': left_motor_config,
                    'mode_configs': left_mode_config,
                    'load_configs': load_configs,
                }],
                output='screen',
            ))

        # Gripper wrapper - translates sim-compatible /right_gripper/cmd and /left_gripper/cmd
        # to Interbotix SDK commands for seamless sim-to-real transfer
        nodes.append(Node(
            package='tidybot_control',
            executable='gripper_wrapper_node',
            name='gripper_wrapper',
            output='screen',
        ))

    # RealSense camera
    if use_camera:
        nodes.append(Node(
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
                ('/camera/color/image_raw', '/camera/color/image_raw'),
                ('/camera/depth/image_rect_raw', '/camera/depth/image_raw'),
            ]
        ))

    # Image compression for remote clients
    if use_compression:
        nodes.append(Node(
            package='tidybot_network_bridge',
            executable='image_compression_node',
            name='image_compression',
            output='screen',
            additional_env=hw_node_env,
            parameters=[{
                'jpeg_quality': 80,
                'png_level': 3,
                'target_fps': 15.0,
            }]
        ))

    # RViz
    if use_rviz:
        rviz_config = PathJoinSubstitution([pkg_bringup, 'rviz', 'tidybot.rviz'])
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}]
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'use_base', default_value='true',
            description='Launch Phoenix 6 mobile base driver'
        ),
        DeclareLaunchArgument(
            'use_arms', default_value='true',
            description='Launch Interbotix arm drivers'
        ),
        DeclareLaunchArgument(
            'use_left_arm', default_value='true',
            description='Launch left arm on U2D2 #2 (/dev/ttyUSB1)'
        ),
        DeclareLaunchArgument(
            'use_pan_tilt', default_value='true',
            description='Enable pan-tilt on U2D2 #1 (with right arm)'
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Launch RealSense camera driver'
        ),
        DeclareLaunchArgument(
            'use_compression', default_value='false',
            description='Launch image compression for remote clients'
        ),
        DeclareLaunchArgument(
            'load_configs', default_value='true',
            description='Load motor configs from YAML files'
        ),

        # Setup function handles conditional node creation
        OpaqueFunction(function=launch_setup),
    ])
