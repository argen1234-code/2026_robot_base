"""拉起整个机器人底座栈：ros2_control 硬件 + 控制器 + TF。

    ros2 launch robot_base_bringup robot_base.launch.py
    ros2 launch robot_base_bringup robot_base.launch.py rviz:=false

启动内容：
  1. robot_state_publisher   —— 发布 TF（由 URDF 决定）
  2. ros2_control_node       —— 加载 robot_base_driver 硬件插件
  3. joint_state_broadcaster —— 发布 /joint_states
  4. diff_drive_controller   —— 接收 cmd_vel，发布 /odom 与 odom->base_link TF

启动后验证：
    ros2 control list_controllers          # 两个控制器都应为 active
    ros2 topic echo /odom --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('robot_base_description')
    bringup_share = get_package_share_directory('robot_base_bringup')

    xacro_file = os.path.join(desc_share, 'urdf', 'robot_base.urdf.xacro')
    controllers_yaml = os.path.join(bringup_share, 'config', 'controllers.yaml')
    rviz_config = os.path.join(desc_share, 'rviz', 'robot_base.rviz')

    use_rviz = LaunchConfiguration('rviz')

    # 带 <ros2_control> 块，硬件由 robot_base_driver 提供
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str)

    # 1) 硬件 + 控制器管理器。
    #    controllers.yaml 也传给本节点，controller_manager 从中读取
    #    各控制器的 type 与 update_rate。
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        remappings=[
            # controllers.yaml 里设了 use_stamped_vel: false，
            # 因此控制器订阅的是 ~/cmd_vel_unstamped（geometry_msgs/Twist）。
            # 把它桥接成通用的 /cmd_vel，好处是 teleop_twist_keyboard、
            # nav2 等只发 Twist 到 /cmd_vel 的节点可以直接用。
            ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel'),
            # 控制器默认把里程计发在私有命名空间 ~/odom 下，
            # 桥接成 /odom 以符合惯例（RViz 配置和 nav2 都期望 /odom）。
            ('/diff_drive_controller/odom', '/odom'),
        ],
        output='both',
    )

    # 2) TF 发布
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # 3) 先起 joint_state_broadcaster
    #    类型用 -t 显式指定：controllers.yaml 里刻意不写 type，
    #    以免 controller_manager 自动加载而与 spawner 抢（详见该文件注释）。
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster',
        ],
        output='screen',
    )

    # 4) 等 joint_state_broadcaster 起来后再起 diff_drive_controller，
    #    避免两个 spawner 同时抢 controller_manager 的服务。
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-type', 'diff_drive_controller/DiffDriveController',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    delay_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz2。'),
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_diff_drive,
        rviz,
    ])
