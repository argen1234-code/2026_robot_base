import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('perception')
    hand_params = os.path.join(share, 'config', 'hand_detector.yaml')

    hand_detector = Node(
        package='perception',
        executable='hand_detector_node',
        name='hand_detector_node',
        parameters=[hand_params],
        output='screen',
    )

    viewer = Node(
        package='perception',
        executable='dual_viewer',
        name='dual_viewer',
        parameters=[{
            'hand_image_topic': '/perception/hand_image',
            'hand_status_topic': '/perception/hand_debug',
            'target_image_topic': '/perception/debug_image',
            'depth_image_topic': '/camera/depth/image_raw',
            'http_port': 80,
        }],
        output='screen',
    )

    return LaunchDescription([hand_detector, viewer])
