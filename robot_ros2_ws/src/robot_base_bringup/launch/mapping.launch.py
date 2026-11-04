"""Single-lidar SLAM mapping mode."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = get_package_share_directory('robot_base_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),
        DeclareLaunchArgument('visualization', default_value='foxglove'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(base, 'config', 'slam_toolbox.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(base, 'launch', 'robot_base.launch.py')),
            launch_arguments={
                'visualization': LaunchConfiguration('visualization'),
                'slam': 'true', 'lidar': 'true',
                'lidar_filter': 'true', 'lidar_port': LaunchConfiguration('lidar_port'),
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items()),
    ])
