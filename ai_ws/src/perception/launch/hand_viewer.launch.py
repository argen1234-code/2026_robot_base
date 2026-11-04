import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('perception')
    params = os.path.join(share, 'config', 'hand_detector.yaml')
    use_gui = LaunchConfiguration('use_gui')
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='Also open a native OpenCV window; the browser viewer '
                    'at port 8089 is always on')

    detector = Node(
        package='perception',
        executable='hand_detector_node',
        name='hand_detector_node',
        parameters=[params],
        output='screen',
    )

    viewer = Node(
        package='perception',
        executable='target_viewer',
        name='hand_viewer',
        parameters=[{
            'image_topic': '/perception/hand_image',
            'status_topic': '/perception/hand_debug',
            'http_port': 8089,
            'use_gui': ParameterValue(use_gui, value_type=bool),
        }],
        output='screen',
    )

    return LaunchDescription([use_gui_arg, detector, viewer])
