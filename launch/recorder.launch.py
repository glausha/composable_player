from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_uri', description='Output bag path'),
        DeclareLaunchArgument('storage_id', default_value='mcap'),
        DeclareLaunchArgument('all_topics', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('max_bag_size', default_value='0'),
        DeclareLaunchArgument('max_bag_duration', default_value='0'),
        Node(
            package='composable_player',
            executable='recorder_node',
            name='recorder',
            parameters=[{
                'output_uri': LaunchConfiguration('output_uri'),
                'storage_id': LaunchConfiguration('storage_id'),
                'all_topics': LaunchConfiguration('all_topics'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'max_bag_size': LaunchConfiguration('max_bag_size'),
                'max_bag_duration': LaunchConfiguration('max_bag_duration'),
            }],
            output='screen',
        ),
    ])
