import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('perception')
    use_gui = LaunchConfiguration('use_gui')
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='Also open a native OpenCV window (may not work in '
                    'Wayland/container); the browser viewer is always on')

    ring_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'ring_detector.launch.py')),
    )

    viewer = Node(
        package='perception',
        executable='target_viewer',
        name='target_viewer',
        parameters=[{
            'image_topic': '/perception/debug_image',
            'status_topic': '/perception/target_debug',
            'http_port': 8088,
            'use_gui': ParameterValue(use_gui, value_type=bool),
        }],
        output='screen',
    )

    return LaunchDescription([use_gui_arg, ring_detector, viewer])
