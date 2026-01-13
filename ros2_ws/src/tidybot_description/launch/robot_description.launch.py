"""Launch file for TidyBot2 robot description (URDF + robot_state_publisher)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('tidybot_description')

    # Path to xacro file
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'tidybot_wx200.urdf.xacro'])

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Process xacro to URDF
    robot_description = Command(['xacro ', urdf_path])

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
    ])
