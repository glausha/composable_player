from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_uri', description='Path to bag file'),
        DeclareLaunchArgument('storage_id', default_value='mcap'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('start_paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_clock', default_value='false'),
        DeclareLaunchArgument('clock_frequency', default_value='100.0'),
        Node(
            package='composable_player',
            executable='player_node',
            name='player',
            parameters=[{
                'bag_uri': LaunchConfiguration('bag_uri'),
                'storage_id': LaunchConfiguration('storage_id'),
                'rate': LaunchConfiguration('rate'),
                'loop': LaunchConfiguration('loop'),
                'start_paused': LaunchConfiguration('start_paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_clock': LaunchConfiguration('publish_clock'),
                'clock_frequency': LaunchConfiguration('clock_frequency'),
            }],
            output='screen',
        ),
    ])
