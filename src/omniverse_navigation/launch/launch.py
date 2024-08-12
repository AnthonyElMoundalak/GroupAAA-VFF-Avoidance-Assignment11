import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('omniverse_navigation'),
        'config',
        'AvoidanceNodeConfig.yaml'
    )
    return LaunchDescription([
        Node(
            package='omniverse_navigation',
            executable='avoidance_node',
            name='avoidance_node',
            output='screen',
            parameters=[
                config
            ]
        ),
    ])

