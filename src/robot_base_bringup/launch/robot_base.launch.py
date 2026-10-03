"""拉起整个机器人底座栈：ros2_control 硬件 + 控制器 + TF。

    ros2 launch robot_base_bringup robot_base.launch.py
    ros2 launch robot_base_bringup robot_base.launch.py visualization:=foxglove

启动内容：
  1. robot_state_publisher   —— 发布 TF（由 URDF 决定）
  2. ros2_control_node       —— 加载 robot_base_driver 硬件插件
  3. joint_state_broadcaster —— 发布 /joint_states
  4. diff_drive_controller   —— 接收 cmd_vel，发布 /odom 与 odom->base_link TF
  5. ydlidar_ros2_driver     —— 发布 /scan（激光雷达，可用 lidar:=false 关掉）
  6. scan_to_scan_filter_chain —— 对 /scan 做角度过滤，发布 /scan_filtered
                                 （RViz 与下游订阅的是过滤后的这一份）
  7. rf2o_laser_odometry     —— 激光里程计，发布 /odom 与 odom->base_footprint 的 TF
  8. slam_toolbox            —— 2D SLAM 持续建图（lifelong 模式），
                                 发布 /map 与 map->odom 的 TF（可用 slam:=false 关掉）

话题流向：ydlidar -> /scan -> [角度滤波] -> /scan_filtered -> slam_toolbox -> /map

TF 树（与 TF2 教程里的移动机器人树形图一致）：
    map -> odom -> base_footprint -> base_link -> laser_frame
     ▲      ▲            ▲
     │      │            └─ URDF（robot_state_publisher）
     │      └─ rf2o_laser_odometry（激光里程计）
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
    # 本机实测插的是 Tmini Pro（230400 bps），所以默认用它。
    # 换型号只需 --lidar-params-file 指定驱动包里对应的那份 yaml。
    default_lidar_params = os.path.join(lidar_share, 'params', 'TminiPro.yaml')
    laser_filters_yaml = os.path.join(bringup_share, 'config', 'laser_filters.yaml')
    slam_params_yaml = os.path.join(bringup_share, 'config', 'slam_toolbox.yaml')

    use_lidar = LaunchConfiguration('lidar')
    use_lidar_filter = LaunchConfiguration('lidar_filter')
    use_slam = LaunchConfiguration('slam')
    foxglove_address = LaunchConfiguration('foxglove_address')
    foxglove_port = LaunchConfiguration('foxglove_port')

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
    #    frame_id 由参数文件给定为 laser_frame，与 URDF 里的雷达 link 同名，
    #    因此 TF 链 odom -> base_link -> laser_frame 由 robot_state_publisher 提供，
    #    不需要额外的 static_transform_publisher。
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
    #    当前规则：laser_frame +Y 是车头，屏蔽车尾 ±45°（雷达角
    #             -135° 到 -45°），保留以车头为中心的 270°。
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
            'odom_topic': '/odom',
            'publish_tf': True,
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

    # 8) 2D SLAM 持续建图（lifelong 模式）。
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
        # lifelong（终身建图）节点。与 async 版的核心区别：它会主动淘汰过时节点，
        # 环境变化时地图会持续演化，而不是把新旧两套墙都留在图上。
        # 想换回单次建图模式改成 'async_slam_toolbox_node' 即可，
        # 但参数文件也要换（lifelong_* 那几项 async 版没声明，会报未声明参数错误）。
        executable='lifelong_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_yaml,
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

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'address': foxglove_address, 'port': foxglove_port}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('visualization'), 'foxglove')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'visualization', default_value='rviz',
            choices=['rviz', 'foxglove', 'none'],
            description='可视化方式：RViz2、Foxglove Bridge 或不启动。'),
        DeclareLaunchArgument(
            'lidar', default_value='true',
            description='是否启动激光雷达驱动（发布 /scan）。'),
        DeclareLaunchArgument(
            'lidar_params_file', default_value=default_lidar_params,
            description='YDLidar 参数文件路径。换型号时指向驱动包 params/ 下对应 yaml。'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='是否启动 slam_toolbox 建图（发布 /map 与 map->odom）。'),
        DeclareLaunchArgument(
            'lidar_filter', default_value='true',
            description='是否对雷达数据做角度过滤（发布 /scan_filtered）。'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ydlidar',
            description='YDLidar 串口设备。默认用 udev 规则绑定的稳定名字 '
                        '/dev/ydlidar（见 README「雷达端口绑定」）；'
                        '没配 udev 规则时可传 /dev/ttyUSB0。'),
        DeclareLaunchArgument('foxglove_address', default_value='0.0.0.0',
                              description='Foxglove Bridge 监听地址。'),
        DeclareLaunchArgument('foxglove_port', default_value='8765',
                              description='Foxglove Bridge WebSocket 端口。'),
        robot_state_publisher,
        wheel_states,
        lidar_node,
        laser_filter_node,
        rf2o_node,
        slam_node,
        slam_configure,
        slam_activate,
        rviz,
        foxglove_bridge,
    ])
