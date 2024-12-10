
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    replay_share_dir = get_package_share_directory('replay_logs')
    bridge_config = os.path.join(replay_share_dir, 'config', 'ros_gazebo_topics.yaml')
    print(bridge_config)
    # Declare the config_file argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=bridge_config,
        description='Path to the configuration file for parameter_bridge'
    )
    
    # Bridge Node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')}
            ],
        output='screen'
    )

    replay_logs = Node(
        package='replay_logs',
        executable='replay',
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        bridge,
        replay_logs
    ])