"""Load a saved map and start Nav2; motor output is opt-in."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('robot_base_bringup')
    base_launch = os.path.join(bringup, 'launch', 'robot_base.launch.py')
    nav_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    nav_params = os.path.join(bringup, 'config', 'nav2_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Saved map YAML file'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),
        DeclareLaunchArgument('motor_enable', default_value='false'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/stm32'),
        DeclareLaunchArgument('wechat', default_value='false', description='Enable TLS MQTT bridge'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'visualization': 'rviz', 'slam': 'false', 'lidar': 'true',
                'lidar_filter': 'true', 'lidar_port': LaunchConfiguration('lidar_port'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                'map': LaunchConfiguration('map'), 'params_file': nav_params,
                'use_sim_time': 'false', 'slam': 'false', 'use_localization': 'true',
                'autostart': 'true', 'use_composition': 'false',
            }.items()),
        Node(
            package='robot_base_bringup', executable='chassis_bridge.py',
            name='chassis_bridge', output='screen',
            parameters=[{
                'port': LaunchConfiguration('mcu_port'),
                'enabled': LaunchConfiguration('motor_enable'),
                'command_timeout': 0.25, 'max_linear': 0.20, 'max_angular': 0.45,
            }],
            condition=IfCondition(LaunchConfiguration('motor_enable'))),
        Node(
            package='robot_base_bringup', executable='wechat_bridge.py',
            name='wechat_bridge', output='screen',
            condition=IfCondition(LaunchConfiguration('wechat'))),
    ])
