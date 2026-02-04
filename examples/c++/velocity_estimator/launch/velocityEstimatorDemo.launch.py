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
            get_package_share_directory('velocity_estimator'),
            'config',
            'params.yaml'
        ),
        description='Path to the configuration file.'
    )

    pose_publisher_node = Node(
        package='velocity_estimator',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ]
    )

    velocity_estimator_node = Node(
        package='velocity_estimator',
        executable='velocity_estimator',
        name='velocity_estimator',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file')
        ]
    )

    return LaunchDescription([
        config_file_arg,

        pose_publisher_node,
        velocity_estimator_node
    ])
