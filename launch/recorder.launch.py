from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_uri', description='Output bag path'),
        DeclareLaunchArgument('storage_id', default_value='mcap'),
        DeclareLaunchArgument('all_topics', default_value='true'),
        Node(
            package='composable_player',
            executable='recorder_node',
            name='recorder',
            parameters=[{
                'output_uri': LaunchConfiguration('output_uri'),
                'storage_id': LaunchConfiguration('storage_id'),
                'all_topics': LaunchConfiguration('all_topics'),
            }],
            output='screen',
        ),
    ])
