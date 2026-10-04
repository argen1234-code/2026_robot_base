"""仅看模型：不启动硬件、不启动 ros2_control、不启动控制器。

用于在任何东西碰硬件之前，先确认模型结构和关节轴方向是否正确。

    ros2 launch robot_base_description view_robot.launch.py
    ros2 launch robot_base_description view_robot.launch.py use_joint_state_publisher_gui:=false
    ros2 launch robot_base_description view_robot.launch.py rviz:=false

用的是 rviz/view_robot.rviz（Fixed Frame = base_footprint，带 RobotModel 显示）。
整机建图运行时请用 robot_base_bringup 的 robot_base.launch.py + rviz/robot_base.rviz。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_base_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_base.urdf.xacro')

    # ⚠️ 用 view_robot.rviz，不要用 robot_base.rviz。
    #    robot_base.rviz 是建图视图：Fixed Frame 是 map，且没有 RobotModel 显示。
    #    本 launch 不启动 ros2_control / SLAM，map 和 odom 都不存在，
    #    用那份配置 RViz 会报 "Fixed Frame [map] does not exist" 且看不到模型。
    #    view_robot.rviz 的 Fixed Frame 是 base_footprint（URDF 直接提供），
    #    并带 RobotModel 显示。
    rviz_config = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')

    use_gui = LaunchConfiguration('use_joint_state_publisher_gui')
    use_rviz = LaunchConfiguration('rviz')

    # 纯可视化时不带 <ros2_control> 块，URDF 更干净，
    # 也便于后续把同一份模型直接丢给 Gazebo（它自带硬件接口）。
    #
    # ParameterValue(..., value_type=str) 是必须的：
    # 否则 launch_ros 会把 URDF 文本当 YAML 解析而报错。
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=false',
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='true',
            description='用滑条调节关节角。需要图形界面。'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz2。'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 30.0,
            }],
        ),

        # robot_state_publisher 3.3.4（Jazzy）仍然没有 publish_default_positions
        # 参数，轮子的 TF 只有等 /joint_states 有数据后才会出现。
        # 所以下面两个节点必须且只能跑一个。
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_gui),
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
