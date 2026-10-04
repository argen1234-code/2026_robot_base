"""Build a live map and run Nav2 against it, without a saved map file."""

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
    nav2 = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),
        DeclareLaunchArgument('visualization', default_value='rviz'),
        DeclareLaunchArgument('motor_enable', default_value='false'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/stm32'),
        DeclareLaunchArgument('wechat', default_value='false', description='Enable TLS MQTT bridge'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'lidar_port': LaunchConfiguration('lidar_port'),
                'visualization': LaunchConfiguration('visualization'),
            }.items()),
        # mapping.launch.py already owns slam_toolbox and map->odom. Nav2 only
        # starts the planners/controllers here; no second SLAM or AMCL instance.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': os.path.join(bringup, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'false', 'autostart': 'true', 'use_composition': 'False',
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
