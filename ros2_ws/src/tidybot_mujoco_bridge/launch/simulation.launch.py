"""Launch file for MuJoCo simulation bridge."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the MuJoCo model
    # Adjust this path based on your workspace structure
    default_model_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
        '..', '..', 'simulation', 'assets', 'mujoco', 'scene_wx200_bimanual.xml'
    )
    default_model_path = os.path.abspath(default_model_path)

    # Declare arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Path to MuJoCo XML model'
    )

    sim_rate_arg = DeclareLaunchArgument(
        'sim_rate',
        default_value='500.0',
        description='Simulation rate in Hz'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Joint state publish rate in Hz'
    )

    camera_rate_arg = DeclareLaunchArgument(
        'camera_rate',
        default_value='30.0',
        description='Camera image publish rate in Hz'
    )

    show_viewer_arg = DeclareLaunchArgument(
        'show_viewer',
        default_value='true',
        description='Show MuJoCo viewer window'
    )

    # MuJoCo bridge node
    mujoco_bridge = Node(
        package='tidybot_mujoco_bridge',
        executable='mujoco_bridge_node',
        name='mujoco_bridge',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'sim_rate': LaunchConfiguration('sim_rate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'camera_rate': LaunchConfiguration('camera_rate'),
            'show_viewer': LaunchConfiguration('show_viewer'),
        }]
    )

    return LaunchDescription([
        model_path_arg,
        sim_rate_arg,
        publish_rate_arg,
        camera_rate_arg,
        show_viewer_arg,
        mujoco_bridge,
    ])
