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
            }],
            output='screen',
        ),
    ])
