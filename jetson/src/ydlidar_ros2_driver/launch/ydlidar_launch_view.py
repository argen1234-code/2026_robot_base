#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# 移植说明（2026_robot_base）：
#   1. 原文件用 LifecycleNode 启动，但驱动 ydlidar_ros2_driver_node 里的 main()
#      用的是 rclcpp::Node::make_shared（普通节点），并非生命周期节点。
#      用 LifecycleNode 启动一个非生命周期节点，生命周期服务不存在，
#      后续若要发 transition 会失败。这里改成 Node。
#   2. 原文件无条件发布 base_link -> laser_frame 的静态 TF。
#      本仓库的 URDF 已经把雷达作为 laser_frame 挂在 base_link 下，
#      robot_state_publisher 会发布同一条 TF；两者同时发布会互相打架，
#      RViz 报 "TF_REPEATED_DATA"。因此改为由 publish_static_tf 控制，
#      默认 false（用 URDF 的），单独跑本文件调试雷达时可设 true。


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    publish_static_tf = LaunchConfiguration('publish_static_tf')

    rviz_config_file = os.path.join(share_dir, 'config', 'ydlidar.rviz')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'TminiPro.yaml'),
        description='FPath to the ROS2 parameters file to use.')
    static_tf_declare = DeclareLaunchArgument(
        'publish_static_tf', default_value='false',
        description='是否发布 base_link -> laser_frame 静态 TF。'
                    '本仓库 URDF 已提供该 TF，默认关闭；'
                    '单独调试雷达（不加载 URDF）时可设为 true。')

    driver_node = Node(package='ydlidar_ros2_driver',
                       executable='ydlidar_ros2_driver_node',
                       name='ydlidar_ros2_driver_node',
                       output='screen',
                       emulate_tty=True,
                       parameters=[parameter_file],
                       namespace='/',
                       )

    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02', '0', '0', '0', '1',
                               'base_link', 'laser_frame'],
                    condition=IfCondition(publish_static_tf),
                    )

    rviz2_node = Node(package='rviz2',
                      executable='rviz2',
                      name='rviz2',
                      arguments=['-d', rviz_config_file],
                      )

    return LaunchDescription([
        params_declare,
        static_tf_declare,
        driver_node,
        tf2_node,
        rviz2_node,
    ])
