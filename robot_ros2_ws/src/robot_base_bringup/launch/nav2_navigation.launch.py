"""精简版 Nav2 启动：只起本车实际用到的节点。

替代 nav2_bringup/launch/navigation_launch.py。

为什么不用官方那个：
    官方 navigation_launch.py 会启动 route_server、docking_server、
    collision_monitor、waypoint_follower 等本车完全用不到的节点，并把它们
    一起交给 lifecycle_manager 按【固定顺序】拉起。其中 route_server
    配置时要加载插件、会阻塞自己的执行器约 200ms，在 Jetson 这种负载较高的
    机器上偶发"来不及响应"：

        [lifecycle_manager_navigation]: Configuring route_server
        [ERROR]: Failed to change state for node: route_server
        [ERROR]: Failed to bring up all requested nodes. Aborting bringup.

    而它在列表里排第 4，一旦失败，【后面的节点全部不会被激活】——
    包括 bt_navigator（目标点动作服务就在它上面）和 velocity_smoother。
    结果是 RViz 里点了目标点既没路径出来、车也不动，而且时好时坏。

本文件只启动必要节点，并把 lifecycle_manager 的节点列表缩到最小，
从根上消除这个竞态。

⚠️ cmd_vel 重映射必须保留（与官方一致）：
    controller_server / behavior_server / velocity_smoother 都加
    remappings + [('cmd_vel', 'cmd_vel_nav')]
    整条链才是 controller -> cmd_vel_nav -> smoother -> cmd_vel_smoothed
    nav2_params.yaml 里的 cmd_vel_in_topic/cmd_vel_out_topic 依赖这个约定。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    # 只管理我们用到的节点，顺序无关（route_server 已移除）
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def node(pkg, exe, name, extra_remaps=None):
        return Node(
            package=pkg, executable=exe, name=name,
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + (extra_remaps or []),
        )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', description='Nav2 参数文件'),
        DeclareLaunchArgument('log_level', default_value='info'),

        # ⚠️ 这三个的 cmd_vel 要重映射成 cmd_vel_nav，链才接得上
        node('nav2_controller', 'controller_server', 'controller_server',
             [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_planner', 'planner_server', 'planner_server'),
        node('nav2_smoother', 'smoother_server', 'smoother_server'),
        node('nav2_behaviors', 'behavior_server', 'behavior_server',
             [('cmd_vel', 'cmd_vel_nav')]),
        node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
             [('cmd_vel', 'cmd_vel_nav')]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                # ⚠️ 默认 4.0s。本车节点配置较慢（Jetson 负载高、插件加载耗时），
                #    调大到 15s，避免节点还在配置就被判定失联。
                'bond_timeout': 15.0,
            }],
        ),
    ])
