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

    publisher_demo_node = Node(
        package='basic_concepts',
        executable='publisher_demo',
        name='publisher_demonstration',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'), # load parameters from config file
            {
                'debug_enabled': LaunchConfiguration('debug')
            }
        ]
    )

    subscriber_demo_node = Node(
        package='basic_concepts',
        executable='subscriber_demo',
        name='subscriber_demonstration',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file') # load parameters from config file
        ]
    )

    return LaunchDescription([
        config_file_arg,
        debug_arg,

        publisher_demo_node,
        subscriber_demo_node
    ])