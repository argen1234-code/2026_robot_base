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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(base, 'launch', 'robot_base.launch.py')),
            launch_arguments={
                'visualization': 'rviz', 'slam': 'true', 'lidar': 'true',
                'lidar_filter': 'true', 'lidar_port': LaunchConfiguration('lidar_port'),
            }.items()),
    ])
