import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    task_share = get_package_share_directory('task_manager')
    perception_share = get_package_share_directory('perception')
    params = os.path.join(task_share, 'config', 'task_manager.yaml')

    dry_run = LaunchConfiguration('dry_run')
    dry_run_arg = DeclareLaunchArgument(
        'dry_run', default_value='true',
        description='If true, velocity_mux publishes zero /cmd_vel')

    ring_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch',
                         'ring_detector.launch.py')),
    )

    velocity_mux = Node(
        package='task_manager',
        executable='velocity_mux',
        name='velocity_mux',
        parameters=[
            params,
            {'dry_run': ParameterValue(dry_run, value_type=bool)},
        ],
        output='screen',
    )

    target_searcher = Node(
        package='task_manager',
        executable='target_searcher',
        name='target_searcher',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([
        dry_run_arg,
        ring_detector,
        velocity_mux,
        target_searcher,
    ])
