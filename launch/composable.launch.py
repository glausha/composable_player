from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_uri', default_value='', description='Bag file path for player'),
        DeclareLaunchArgument('output_uri', default_value='', description='Output path for recorder'),
        DeclareLaunchArgument('storage_id', default_value='mcap'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('all_topics', default_value='true'),
        DeclareLaunchArgument('enable_player', default_value='true'),
        DeclareLaunchArgument('enable_recorder', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_clock', default_value='false'),
        DeclareLaunchArgument('clock_frequency', default_value='100.0'),
        DeclareLaunchArgument('max_bag_size', default_value='0'),
        DeclareLaunchArgument('max_bag_duration', default_value='0'),

        ComposableNodeContainer(
            name='composable_player_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[],
            output='screen',
        ),

        LoadComposableNodes(
            target_container='composable_player_container',
            condition=IfCondition(LaunchConfiguration('enable_player')),
            composable_node_descriptions=[
                ComposableNode(
                    package='composable_player',
                    plugin='composable_player::PlayerNode',
                    name='player',
                    parameters=[{
                        'bag_uri': LaunchConfiguration('bag_uri'),
                        'storage_id': LaunchConfiguration('storage_id'),
                        'rate': LaunchConfiguration('rate'),
                        'loop': LaunchConfiguration('loop'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'publish_clock': LaunchConfiguration('publish_clock'),
                        'clock_frequency': LaunchConfiguration('clock_frequency'),
                    }],
                ),
            ],
        ),

        LoadComposableNodes(
            target_container='composable_player_container',
            condition=IfCondition(LaunchConfiguration('enable_recorder')),
            composable_node_descriptions=[
                ComposableNode(
                    package='composable_player',
                    plugin='composable_player::RecorderNode',
                    name='recorder',
                    parameters=[{
                        'output_uri': LaunchConfiguration('output_uri'),
                        'storage_id': LaunchConfiguration('storage_id'),
                        'all_topics': LaunchConfiguration('all_topics'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'max_bag_size': LaunchConfiguration('max_bag_size'),
                        'max_bag_duration': LaunchConfiguration('max_bag_duration'),
                    }],
                ),
            ],
        ),
    ])
