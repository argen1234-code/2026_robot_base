"""先验 posegraph + slam_toolbox localization + Nav2。

注意：slam_toolbox 的定位/续建使用 serialize_map 生成的
`<name>.posegraph` / `<name>.data`，不是 map_saver 的 `.yaml/.pgm`。
本文件不再启动 map_server/AMCL；map -> odom 由 slam_toolbox
localization 模式发布。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('robot_base_bringup')
    base_launch = os.path.join(bringup, 'launch', 'robot_base.launch.py')
    nav_launch = os.path.join(bringup, 'launch', 'nav2_navigation.launch.py')
    nav_params = os.path.join(bringup, 'config', 'nav2_params_prior.yaml')
    default_slam_params = os.path.join(bringup, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_slam_params,
            description='slam_toolbox localization 参数（含 map_file_name/mode）'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),
        DeclareLaunchArgument('motor_enable', default_value='false'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/stm32'),
        DeclareLaunchArgument('visualization', default_value='foxglove',
                              choices=['rviz', 'foxglove', 'none'],
                              description='可视化方式：Foxglove（默认）、RViz2 或不启动'),
        DeclareLaunchArgument('wechat', default_value='false',
                              description='Enable TLS MQTT bridge'),

        # slam_toolbox localization：发布 map->odom 和 /map；不发 AMCL。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'visualization': LaunchConfiguration('visualization'),
                'slam': 'true', 'lidar': 'true',
                'lidar_filter': 'true',
                'lidar_port': LaunchConfiguration('lidar_port'),
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items()),

        # 只启动 Nav2 的规划/控制链，不启动 map_server/amcl。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                'params_file': nav_params,
                'use_sim_time': 'false', 'autostart': 'true',
            }.items()),

        # chassis_bridge 始终启动，motor_enable 只控制 ARM。
        Node(
            package='robot_base_bringup', executable='chassis_bridge.py',
            name='chassis_bridge', output='screen',
            parameters=[os.path.join(bringup, 'config', 'imu_calibration.yaml'), {
                'port': LaunchConfiguration('mcu_port'),
                'enabled': ParameterValue(LaunchConfiguration('motor_enable'),
                                          value_type=bool),
                'command_timeout': 0.25, 'max_linear': 0.24, 'max_angular': 0.55,
            }]),
        Node(
            package='robot_base_bringup', executable='wechat_bridge.py',
            name='wechat_bridge', output='screen',
            condition=IfCondition(LaunchConfiguration('wechat'))),
    ])
