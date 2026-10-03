"""Gazebo 仿真：把底盘整套跑在仿真里。

    ros2 launch robot_base_bringup gazebo.launch.py
    ros2 launch robot_base_bringup gazebo.launch.py gui:=false    # 只跑服务器（无 Gazebo 界面）
    ros2 launch robot_base_bringup gazebo.launch.py visualization:=foxglove

与 robot_base.launch.py（真机路径）的区别：

  | | 真机路径 | 本文件（仿真） |
  |---|---|---|
  | 雷达数据 | ydlidar 驱动读串口 | **Gazebo 仿真**，经 ros_gz_bridge 桥接 |
  | 轮子驱动 | robot_base_driver（**桩实现，不会动**） | **Gazebo 物理引擎**（经 gz_ros2_control） |
  | 时钟 | 系统时钟 | **仿真时钟**（use_sim_time=true） |
  | cmd_vel | 车不会动 | **车真的会跑** |

两条路径共用同一份 URDF，靠 xacro 的 use_gazebo 参数切换硬件插件与传感器来源。

⚠️ 本文件【不启动 ros2_control_node】：
   gz_ros2_control 插件会在 Gazebo 进程内部自己起一个 controller_manager，
   再起一个会撞车。这里只用 spawner 往那个 controller_manager 里加载控制器。

启动内容：
  1. Gazebo 服务器（加载世界文件）
  2. robot_state_publisher —— URDF 用 use_gazebo:=true
  3. ros_gz_sim create    —— 把机器人从 /robot_description 生成到仿真里
  4. ros_gz_bridge        —— 仿真雷达 -> /scan，仿真时钟 -> /clock
  5. scan_to_scan_filter_chain —— 角度滤波 -> /scan_filtered
  6. joint_state_broadcaster + diff_drive_controller（spawner 加载）
  7. rf2o_laser_odometry —— 激光里程计，发布 /odom 与 odom->base_footprint
                             （真机只有雷达这一个物理传感器，仿真保持一致）
  8. slam_toolbox —— 2D SLAM 建图（可用 slam:=false 关掉）
  9. Gazebo GUI（可用 gui:=false 关掉）与 RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                              IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction)
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory('robot_base_description')
    bringup_share = get_package_share_directory('robot_base_bringup')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc_share, 'urdf', 'robot_base.urdf.xacro')
    world_file = os.path.join(bringup_share, 'worlds', 'robot_base_world.sdf')
    controllers_yaml = os.path.join(bringup_share, 'config', 'controllers.yaml')
    slam_params_yaml = os.path.join(bringup_share, 'config', 'slam_toolbox.yaml')
    laser_filters_yaml = os.path.join(bringup_share, 'config', 'laser_filters.yaml')
    rviz_config = os.path.join(desc_share, 'rviz', 'robot_base.rviz')

    use_gui = LaunchConfiguration('gui')
    use_slam = LaunchConfiguration('slam')
    foxglove_address = LaunchConfiguration('foxglove_address')
    foxglove_port = LaunchConfiguration('foxglove_port')

    # use_gazebo:=true 会：
    #   - 把 ros2_control 硬件插件换成 gz_ros2_control/GazeboSimSystem
    #   - 引入雷达传感器、摩擦系数、gz_ros2_control 插件等 <gazebo> 标签
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=true']),
        value_type=str)

    # 1) Gazebo 服务器。
    #    -r 立即开始仿真（否则要手动按播放），-s 只跑服务器（渲染由无界面后端做）。
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', ['-r -s ', world_file])],
    )

    # Gazebo GUI 客户端（连到上面那个服务器）
    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        condition=IfCondition(use_gui),
        output='screen',
    )

    # 2) TF 与 /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    # 3) 把机器人生成到仿真里。
    #    -topic robot_description 直接从话题读 URDF（robot_state_publisher 会
    #    以 transient_local 发布），这样不用把 URDF 再拼一遍。
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'robot_base',
                   '-z', '0.15'],       # 略抬高一点，避免出生瞬间陷进地面
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 4) 话题桥接：Gazebo Transport <-> ROS 2
    #    格式 topic@ROS类型<方向>GZ类型，[ 表示 Gazebo -> ROS。
    #    ⚠️ 传感器的话题名以 URDF 里 <topic> 标签为准（当前是 /lidar）；
    #       这里把它重映射成 ROS 惯例的 /scan。
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/robot_base_world/model/robot_base/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/lidar', '/scan'),
                    ('/world/robot_base_world/model/robot_base/joint_state', '/joint_states')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 5) 角度滤波（与真机路径同一份配置）
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[laser_filters_yaml, {'use_sim_time': True}],
        output='screen',
    )

    # 6) 控制器。⚠️ 不启动 ros2_control_node —— controller_manager 由
    #    gz_ros2_control 插件在 Gazebo 进程内创建，spawner 直接连它。
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers_yaml,
                   # 话题重映射：仿真里同样把控制器的私有话题桥到通用名字
                   '--controller-ros-args',
                   '-r /diff_drive_controller/cmd_vel:=/cmd_vel '
                   '-r /diff_drive_controller/odom:=/odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ⚠️ 必须延迟再加载控制器。
    # Gazebo 启动阶段（加载世界 + 生成机器人 + 物理引擎初始化）很吃资源，
    # gz_ros2_control 在 Gazebo 进程内建的那个 controller_manager 更新循环
    # 跟不上，会导致 "Switch controller timed out after 5 seconds!"，
    # 表现为 joint_state_broadcaster 加载成功但激活失败（/joint_states 无数据、
    # 轮子不出现在 TF 里）。实测延迟 12 秒可稳定通过。
    delay_jsp = TimerAction(period=12.0, actions=[joint_state_broadcaster_spawner])

    # 等 joint_state_broadcaster 起来再起 diff_drive，避免两个 spawner 抢服务
    delay_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # 7) 激光里程计（rf2o）。
    #    ⚠️ 真机上的物理传感器【只有激光雷达】，没有可用的轮式编码器，
    #       所以 odom 一律由 rf2o 从雷达数据推算 —— 仿真里也保持一致，
    #       这样仿真才真能验证真机那套链路。
    #
    #    因此 controllers.yaml 里的 enable_odom_tf 保持 false：
    #       diff_drive_controller 让出 odom -> base_footprint，由 rf2o 发布。
    #    仿真里【不使用】Gazebo 物理引擎算出的轮式里程计 ——
    #    它虽然真实，但真机上拿不到，用了就失去验证意义。
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic': '/scan_filtered',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # 8) 2D SLAM 建图。
    #    与真机路径用同一份 slam_toolbox.yaml、同样是 lifelong 模式，
    #    唯一差别是 use_sim_time=true。
    #
    #    ⚠️ LifecycleNode，必须显式发 configure + activate，
    #       否则只有 lifecycle 样板接口，/scan 订阅与 /map 发布都不会创建。
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='lifelong_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_yaml, {'use_sim_time': True}],
        condition=IfCondition(use_slam),
        output='screen',
        namespace='',
    )

    slam_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(use_slam),
    )

    slam_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[slam_toolbox] 已 configure，正在 activate'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(use_slam),
    )

    # ⚠️ 不再用 -f odom 覆盖固定坐标系：
    #    有了 SLAM 后 map 坐标系存在，用配置里默认的 map 才是正确的 SLAM 视图。
    #    若用 slam:=false 关掉 SLAM，map 就不存在了，那时需要改回 -f odom。
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('visualization'), 'rviz')),
        output='screen',
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'address': foxglove_address, 'port': foxglove_port, 'use_sim_time': True}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('visualization'), 'foxglove')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo GUI（false = 只跑服务器）。'),
        DeclareLaunchArgument('visualization', default_value='rviz',
                              choices=['rviz', 'foxglove', 'none'],
                              description='可视化方式：RViz2、Foxglove Bridge 或不启动。'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='是否启动 slam_toolbox 建图（发布 /map 与 map->odom）。'),
        DeclareLaunchArgument('foxglove_address', default_value='0.0.0.0',
                              description='Foxglove Bridge 监听地址。'),
        DeclareLaunchArgument('foxglove_port', default_value='8765',
                              description='Foxglove Bridge WebSocket 端口。'),
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_robot,
        bridge,
        laser_filter_node,
        rf2o_node,
        slam_node,
        slam_configure,
        slam_activate,
        rviz,
        foxglove_bridge,
    ])
