from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=join(
            get_package_share_directory('basic_concepts'),
            'config',
            'params.yaml'
        ),
        description='Path to the configuration file.'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Whether debug messages should be shown.'
    )

    publisher_demo_node_1 = Node(
        package='basic_concepts',
        executable='publisher_demo',
        name='publisher_demonstration_1',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'), # load parameters from config file
            {
                'debug_enabled': LaunchConfiguration('debug'),
                'publish_rate': 1
            }
        ]
    )

    publisher_demo_node_2 = Node(
        package='basic_concepts',
        executable='publisher_demo',
        name='publisher_demonstration_2',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'), # load parameters from config file
            {
                'debug_enabled': LaunchConfiguration('debug'),
                'publish_rate': 2
            }
        ]
    )

    subscriber_demo_node_1 = Node(
        package='basic_concepts',
        executable='subscriber_demo',
        name='subscriber_demonstration_1',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file') # load parameters from config file
        ]
    )

    subscriber_demo_node_2 = Node(
        package='basic_concepts',
        executable='subscriber_demo',
        name='subscriber_demonstration_2',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file') # load parameters from config file
        ]
    )

    subscriber_demo_node_3 = Node(
        package='basic_concepts',
        executable='subscriber_demo',
        name='subscriber_demonstration_3',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file') # load parameters from config file
        ]
    )

    return LaunchDescription([
        config_file_arg,
        debug_arg,

        publisher_demo_node_1,
        publisher_demo_node_2,
        subscriber_demo_node_1,
        subscriber_demo_node_2,
        subscriber_demo_node_3
    ])