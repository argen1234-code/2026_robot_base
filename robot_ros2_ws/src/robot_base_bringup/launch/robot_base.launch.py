"""拉起整个机器人底座栈：ros2_control 硬件 + 控制器 + TF。

    ros2 launch robot_base_bringup robot_base.launch.py
    ros2 launch robot_base_bringup robot_base.launch.py visualization:=none

启动内容：
  1. robot_state_publisher   —— 发布 TF（由 URDF 决定）
  2. ros2_control_node       —— 加载 robot_base_driver 硬件插件
  3. joint_state_broadcaster —— 发布 /joint_states
  4. diff_drive_controller   —— 接收 cmd_vel，发布 /odom 与 odom->base_link TF
  5. ydlidar_ros2_driver     —— 发布 /scan（激光雷达，可用 lidar:=false 关掉）
  6. scan_to_scan_filter_chain —— 对 /scan 做角度过滤，发布 /scan_filtered
                                 （RViz 与下游订阅的是过滤后的这一份）
  7. rf2o_laser_odometry     —— 激光里程计原始输出 /odom_rf2o
  7b. odom_stabilizer        —— 轮速 ZUPT，输出 /odom_stabilized（不发 TF）
  7c. wheel_odom             —— 轮速积分，输出 /wheel/odom（不发 TF）
  7d. ekf_filter_node        —— 融合 rf2o+ZUPT、轮速、IMU gyro z，
                                 发布 /odom 与唯一的 odom->base_footprint TF
  8. slam_toolbox            —— 2D SLAM 异步建图，
                                 发布 /map 与 map->odom 的 TF（可用 slam:=false 关掉）

话题流向：ydlidar -> /scan -> [距离+后向角度滤波] -> /scan_filtered
          -> slam_toolbox -> /map
          -> rf2o -> /odom_rf2o -> odom_stabilizer -> /odom_stabilized
          /chassis/wheel_speed -> wheel_odom -> /wheel/odom
          /chassis/imu -> EKF gyro z
          -> ekf_filter_node -> /odom + odom->base_footprint TF

TF 树（与 TF2 教程里的移动机器人树形图一致）：
    map -> odom -> base_footprint -> base_link -> link_laser -> laser_frame
     ▲      ▲            ▲
     │      │            └─ URDF（robot_state_publisher）
     │      └─ ekf_filter_node（融合里程计；唯一 odom->base_footprint）
     └─ slam_toolbox

启动后验证：
    ros2 control list_controllers          # 两个控制器都应为 active
    ros2 topic echo /odom --once
    ros2 topic hz /scan                    # 雷达应约 10 Hz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    desc_share = get_package_share_directory('robot_base_description')
    bringup_share = get_package_share_directory('robot_base_bringup')
    lidar_share = get_package_share_directory('ydlidar_ros2_driver')

    xacro_file = os.path.join(desc_share, 'urdf', 'robot_base.urdf.xacro')
    controllers_yaml = os.path.join(bringup_share, 'config', 'controllers.yaml')
    rviz_config = os.path.join(desc_share, 'rviz', 'robot_base.rviz')

    # 雷达参数文件。驱动包自带各型号的 yaml（X2/X4/G1/G2/TG/TminiPro...），
    # 本机实测插的是 Tmini Plus（Model Code 151，230400 bps），所以默认用它。
    # ⚠️ Tmini Plus 不输出强度信息：用 TminiPro.yaml（intensity: true）会持续
    #    报 Checksum error 并反复切换强度位，故单独保留 TminiPlus.yaml。
    # 换型号只需 --lidar-params-file 指定驱动包里对应的那份 yaml。
    default_lidar_params = os.path.join(lidar_share, 'params', 'TminiPlus.yaml')
    laser_filters_yaml = os.path.join(bringup_share, 'config', 'laser_filters.yaml')
    slam_params_yaml = os.path.join(bringup_share, 'config', 'slam_toolbox.yaml')

    use_lidar = LaunchConfiguration('lidar')
    use_lidar_filter = LaunchConfiguration('lidar_filter')
    use_slam = LaunchConfiguration('slam')

    # 带 <ros2_control> 块，硬件由 robot_base_driver 提供
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str)

    # 1) 硬件 + 控制器管理器。
    #    controllers.yaml 也传给本节点，controller_manager 从中读取
    #    update_rate。（控制器的 type 写在控制器自己的段里，见该文件注释。）
    #
    #    ⚠️ 控制器的重映射【不要】写在这里了。Jazzy 起把重映射挂在
    #    controller_manager 节点上已废弃，运行时会收到：
    #      "The use of remapping arguments to the controller_manager node is
    #       deprecated. Please use the '--controller-ros-args' argument of the
    #       spawner to pass remapping arguments to the controller node."
    #    改为在对应 spawner 上用 --controller-ros-args 传，见下面 diff_drive
    #    的 spawner。
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
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

    # 默认仍用 joint_state_publisher 提供轮子关节状态。
    # 新 URDF 里虽然已经有 <ros2_control> 段，但 robot_base_driver 仍是桩实现，
    # 仓库默认不启动 controller_manager；实际运动走 chassis_bridge + MCU。
    wheel_states = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    # 3) 先起 joint_state_broadcaster
    #    ⚠️ Jazzy 的 spawner 没有 --controller-type/-t 参数了，
    #    类型改为从 --param-file 里 <控制器名>.ros__parameters.type 读。
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
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
            '--param-file', controllers_yaml,
            # 控制器的私有话题重映射走这里（见上面 control_node 处的说明）。
            # 该参数按空格切分成 ROS 参数传给控制器节点。
            '--controller-ros-args',
            # 控制器只订阅 ~/cmd_vel，类型固定为 geometry_msgs/msg/TwistStamped
            # （Humble 的 use_stamped_vel 与 ~/cmd_vel_unstamped 均已移除）。
            # 桥接成通用的 /cmd_vel。⚠️ 发消息的一方必须带 header.stamp：
            #   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
            #       --ros-args -p stamped:=true
            # ⚠️ 这里【不再】把 '/diff_drive_controller/odom' 重映射到 /odom：
            #    /odom 现在由激光里程计 rf2o 发布（见下方 rf2o_node）。
            #    两个节点同时发 /odom 会让订阅者收到交错的两份数据。
            #    轮式里程计仍可在 /diff_drive_controller/odom 上单独查看。
            '-r /diff_drive_controller/cmd_vel:=/cmd_vel',
        ],
        output='screen',
    )

    delay_diff_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    # 5) 激光雷达。ydlidar 驱动是普通 Node（不是 lifecycle），
    #    直接发布 /scan（sensor_msgs/LaserScan）与 /point_cloud。
    #    frame_id 由参数文件给定为 laser_frame，与 URDF 里的雷达坐标系同名，
    #    因此 TF 链 odom -> base_footprint -> base_link -> link_laser ->
    #    laser_frame 由 robot_state_publisher 提供，不需要额外 static TF。
    #
    #    ⚠️ 默认端口 /dev/ydlidar 由 udev 规则绑定（见 README「雷达端口绑定」），
    #       这样插拔/重启后设备名不会变。需要 dialout 组权限；
    #       没权限或没配 udev 时本节点会报错退出，但不影响底盘其余部分。
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[
            LaunchConfiguration('lidar_params_file'),
            # 放在参数文件之后，保证命令行能覆盖文件里的值
            {'port': LaunchConfiguration('lidar_port')},
        ],
        condition=IfCondition(use_lidar),
        output='screen',
    )

    # 6) 激光雷达角度过滤。
    #    订阅雷达原始数据 /scan，按 config/laser_filters.yaml 里的规则过滤，
    #    重新发布到 /scan_filtered。RViz 与下游（nav2 costmap 等）应订阅后者。
    #
    #    当前规则（2026-11-04 静态 bag 实测后更新）：laser_frame +Y 是车头，
    #    只做 0.20–10 m 距离截断，不再屏蔽车尾（车尾扇区实测无自回波）。
    #    改范围只改那个 yaml，不必动本文件。
    #
    #    ⚠️ 关掉滤波（lidar_filter:=false）时 /scan_filtered 不存在，
    #       RViz 里 LaserScan 显示会是空的 —— 因为 RViz 已改为订阅过滤后的话题。
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[laser_filters_yaml],
        condition=IfCondition(use_lidar_filter),
        output='screen',
    )

    # 7) 激光里程计。
    #    通过配准相邻两帧 LaserScan 估计平面运动，发布 /odom 与
    #    odom -> base_footprint 的 TF，给 slam_toolbox 当运动先验。
    #
    #    为什么需要它：轮式里程计目前是桩实现（robot_base_driver.read()
    #    把命令回声成状态再积分），完全没有反映真实运动。而雷达是真的，
    #    所以激光里程计给出的才是车（或被推着走时）的真实运动。
    #
    #    ⚠️ TF 归属：odom -> base_footprint 现在由本节点发布，
    #       controllers.yaml 里 diff_drive_controller 的 enable_odom_tf
    #       必须同时为 false，否则两个节点抢同一条 TF。
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            # 用角度过滤后的数据。rf2o 是扫描配准，用原始 /scan 也行，
            # 但过滤掉后方 90° 能减少车体自身/拖线的干扰。
            'laser_scan_topic': '/scan_filtered',
            # rf2o 只输出原始激光里程计；/odom 和 odom->base_footprint
            # 由下面的 odom_stabilizer 发布（轮速静止时冻结，消除静态漂移）。
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            # base_frame_id 必须是 base_footprint：它决定 rf2o 发布
            # odom -> <base_frame_id>，写 base_link 会让 base_link 又有两个父节点。
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            # 留空表示不从上位姿初始化，从原点开始
            'init_pose_from_topic': '',
            'freq': 10.0,      # 与雷达 10Hz 一致
        }],
        output='screen',
    )

    # 7b) 零速保持（ZUPT）：订阅 rf2o 与 MCU 轮速。
    #     实测 rf2o 在整车完全静止时仍会持续漂移（35 s 约 12° yaw），
    #     静态障碍因此在 odom 系里“漂”。小车静止时冻结输出位姿，
    #     轮速恢复后用偏置无缝续接 rf2o；没有轮速数据时透传。
    #     EKF 模式下本节点只输出 /odom_stabilized，不再发 TF。
    odom_stabilizer_node = Node(
        package='robot_base_bringup',
        executable='odom_stabilizer.py',
        name='odom_stabilizer',
        parameters=[{
            'odom_in': '/odom_rf2o',
            'odom_out': '/odom_stabilized',
            'wheel_topic': '/chassis/wheel_speed',
            'target_topic': '/chassis/wheel_target',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_tf': False,
            # 实测静止轮速噪声 ±0.12 rad/s：进入静止 0.20，退出 0.30。
            'stationary_speed_rad_s': 0.20,
            'release_speed_rad_s': 0.30,
            'stationary_hold_s': 0.30,
            'release_hold_s': 0.15,
            'target_motion_threshold': 0.05,
            'target_timeout_s': 0.50,
            'wheel_timeout_s': 0.50,
        }],
        output='screen',
    )

    # 7c) 轮式里程计：/chassis/wheel_speed -> /wheel/odom，供 EKF 融合。
    wheel_odom_node = Node(
        package='robot_base_bringup',
        executable='wheel_odom.py',
        name='wheel_odom',
        parameters=[{
            'wheel_topic': '/chassis/wheel_speed',
            'odom_topic': '/wheel/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'wheel_radius': 0.041713,
            'wheel_separation': 0.148,
            'publish_rate': 50.0,
            'wheel_timeout': 0.50,
        }],
        output='screen',
    )

    # 7d) EKF 松耦合：rf2o+ZUPT、轮式 twist、IMU gyro z -> /odom 和
    #     唯一的 odom -> base_footprint TF。slam_toolbox 继续发 map->odom。
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(bringup_share, 'config', 'ekf.yaml')],
        # EKF 原生输出 /odometry/filtered，重映射成 Nav2/现有代码使用的 /odom。
        remappings=[('odometry/filtered', '/odom')],
        output='screen',
    )

    # 8) 2D SLAM 异步建图。
    #    订阅 /scan_filtered，发布 /map 与 map->odom 的 TF。
    #
    #    ⚠️ slam_toolbox 是 LifecycleNode，不是普通节点：
    #       直接 ros2 run 起的话只有 lifecycle 样板接口，/scan 订阅和 /map
    #       发布都要等 configure + activate 之后才创建。所以这里必须显式发
    #       生命周期转换事件（configure -> activate），否则它一直不工作。
    #
    #    ⚠️ use_sim_time 必须为 false（真机用系统时钟）。官方 launch 默认是
    #       true，那会让节点一直等 /clock 而卡在未激活状态。
    slam_node = LifecycleNode(
        package='slam_toolbox',
        # 标准异步建图，避免实验性 lifelong 节点频繁淘汰位姿图节点。
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': False},
        ],
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('visualization'), 'rviz')),
        output='screen',
    )

    # Foxglove Studio 通过 WebSocket 连接，不依赖本机 OpenGL/Ogre；
    # 本机 RoboStack aarch64 的 RViz2 会在 Ogre 初始化时段错误，故默认用它。
    foxglove_preview = Node(
        package='robot_base_bringup',
        executable='foxglove_preview.py',
        name='foxglove_preview',
        arguments=[
            '--width', os.environ.get('FOXGLOVE_PREVIEW_WIDTH', '240'),
            '--fps', os.environ.get('FOXGLOVE_PREVIEW_FPS', '3'),
            '--quality', os.environ.get('FOXGLOVE_PREVIEW_QUALITY', '40'),
        ],
        prefix='nice -n 10',
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('visualization'), 'foxglove')),
        output='screen',
    )
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'address': LaunchConfiguration('foxglove_address'),
            'port': LaunchConfiguration('foxglove_port'),
            # Bound stale image queues so maps/TF keep reaching live clients.
            'use_compression': True,
            'send_buffer_limit': 2000000,
            'message_backlog_size': 64,
            'max_qos_depth': 2,
            # Only this bridge sees the small viewer copies. ROS algorithms
            # continue subscribing to the original sensor/detection topics.
            'topic_whitelist': ['^(?!/foxglove_preview/).*'],
            'best_effort_qos_topic_whitelist': [
                '/camera/.*image.*', '/perception/.*image.*'],
        }],
        remappings=[
            ('/camera/color/image_raw/compressed', '/foxglove_preview/color/compressed'),
            ('/camera/depth/image_raw', '/foxglove_preview/depth/image_raw'),
            ('/camera/depth/camera_info', '/foxglove_preview/depth/camera_info'),
            ('/perception/person_image/compressed', '/foxglove_preview/person/compressed'),
            ('/perception/hand_image/compressed', '/foxglove_preview/hand/compressed'),
        ],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('visualization'), 'foxglove')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'visualization', default_value='foxglove',
            choices=['rviz', 'foxglove', 'none'],
            description='可视化方式：Foxglove（默认，WebSocket）、RViz2 或不启动。'),
        DeclareLaunchArgument(
            'foxglove_address', default_value='0.0.0.0',
            description='Foxglove Bridge 监听地址。'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='Foxglove Bridge 监听端口（Studio 里填 ws://<树莓派IP>:8765）。'),
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='是否启动激光雷达驱动（发布 /scan）。'),
        DeclareLaunchArgument(
            'lidar_params_file', default_value=default_lidar_params,
            description='YDLidar 参数文件路径。换型号时指向驱动包 params/ 下对应 yaml。'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='是否启动 slam_toolbox（发布 /map 与 map->odom）。'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=slam_params_yaml,
            description='slam_toolbox 参数 YAML；统一入口会生成 mode/map_file_name 覆盖文件。'),
        DeclareLaunchArgument(
            'lidar_filter', default_value='true',
            description='是否对雷达数据做角度过滤（发布 /scan_filtered）。'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ydlidar',
            description='YDLidar 串口设备。默认用 udev 规则绑定的稳定名字 '
                        '/dev/ydlidar（见 README「雷达端口绑定」）；'
                        '没配 udev 规则时可传 /dev/ttyUSB0。'),
        robot_state_publisher,
        wheel_states,
        lidar_node,
        laser_filter_node,
        rf2o_node,
        odom_stabilizer_node,
        wheel_odom_node,
        ekf_node,
        slam_node,
        slam_configure,
        slam_activate,
        rviz,
        foxglove_preview,
        foxglove,
    ])
