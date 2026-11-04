import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('perception'),
        'config', 'ring_detector.yaml')

    return LaunchDescription([
        Node(
            package='perception',
            executable='ring_detector_node',
            name='ring_detector_node',
            parameters=[params],
            output='screen',
        ),
    ])
