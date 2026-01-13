"""
Simulation Launch File for TidyBot2.

Launches:
- MuJoCo bridge (physics simulation)
- Robot state publisher (URDF + TF)
- Arm controllers (left + right)
- RViz (optional)

Usage:
    ros2 launch tidybot_bringup sim.launch.py
    ros2 launch tidybot_bringup sim.launch.py use_rviz:=false
    ros2 launch tidybot_bringup sim.launch.py show_mujoco_viewer:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_bringup = FindPackageShare('tidybot_bringup')
    pkg_description = FindPackageShare('tidybot_description')
    pkg_bridge = FindPackageShare('tidybot_mujoco_bridge')

    # Get model path from TIDYBOT_REPO_ROOT environment variable
    # This is set by setup_env.bash
    repo_root = os.environ.get('TIDYBOT_REPO_ROOT', '')
    if not repo_root:
        # Fallback: assume standard location
        repo_root = os.path.expanduser('~/Documents/collaborative-robotics-2026')

    model_path = os.path.join(repo_root, 'simulation', 'assets', 'mujoco', 'scene_wx200_bimanual.xml')

    # Declare arguments
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz for visualization'
    )

    declare_show_viewer = DeclareLaunchArgument(
        'show_mujoco_viewer', default_value='true',
        description='Show MuJoCo viewer window'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    # URDF from xacro
    urdf_path = PathJoinSubstitution([pkg_description, 'urdf', 'tidybot_wx200.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_path])

    # ========== NODES ==========

    # Robot state publisher (publishes TF from URDF + joint_states)
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

    # MuJoCo Bridge - THE SIMULATION BACKEND
    mujoco_bridge = Node(
        package='tidybot_mujoco_bridge',
        executable='mujoco_bridge_node',
        name='mujoco_bridge',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'sim_rate': 500.0,
            'publish_rate': 100.0,
            'camera_rate': 30.0,
            'show_viewer': LaunchConfiguration('show_mujoco_viewer'),
        }]
    )

    # Right arm controller
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

    # Left arm controller
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
        declare_show_viewer,
        declare_use_sim_time,

        # Nodes
        robot_state_publisher,
        mujoco_bridge,
        right_arm_controller,
        left_arm_controller,
        rviz,
    ])
