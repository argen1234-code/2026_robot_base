import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package's shared directory. / 获取包的共享目录。
    params_file=os.path.join(get_package_share_directory('largemodel'), "config", "yahboom.yaml") 

    # Launch arguments. / 启动参数。
    text_chat_mode = LaunchConfiguration('text_chat_mode')
    text_chat_mode_arg = DeclareLaunchArgument(
        'text_chat_mode', default_value='false')

    # enable_nav starts our task/gate nodes.  The Nav2 stack itself is launched
    # by the navigation project; keep it separate so the voice stack can still
    # run without a map or Nav2.
    enable_nav = LaunchConfiguration('enable_nav')
    enable_nav_arg = DeclareLaunchArgument('enable_nav', default_value='false')

    # The unified CLI decides who owns /dev/astra-color.  uvc = model_service
    # owns it; ros_compressed = a standalone publisher owns it and this node
    # consumes the JPEG topic.
    camera_source = LaunchConfiguration('camera_source')
    camera_source_arg = DeclareLaunchArgument(
        'camera_source', default_value='uvc')
    # In decision mode the hierarchical decision layer owns motion commands;
    # direct summon/move tools are not registered.
    decision_mode = LaunchConfiguration('decision_mode')
    decision_mode_arg = DeclareLaunchArgument(
        'decision_mode', default_value='false')
    # Mapping mode points this at /ai_nav_disabled so voice chat cannot move.
    nav_goal_topic = LaunchConfiguration('nav_goal_topic')
    nav_goal_topic_arg = DeclareLaunchArgument(
        'nav_goal_topic', default_value='/goal_pose')

    # Define nodes. / 定义节点。
    model_server = Node(
        package='largemodel',
        executable='model_service',
        name='model_service',
        parameters=[
            params_file,
            {
                'text_chat_mode': text_chat_mode,
                'camera_source': camera_source,
                'decision_mode': ParameterValue(decision_mode, value_type=bool),
            }  # Dynamic parameter, command line arguments override the homonymous parameters in yahboom.yaml and are passed to the node. / 动态参数，命令行参数覆盖yahboom.yaml同名参数再传给节点。
        ],
        remappings=[('/goal_pose', nav_goal_topic)],
        output='screen'
    )

    asr_server = Node(
        package='largemodel',
        executable='asr',
        name='asr',
        parameters=[params_file],
        output='screen',
        condition=UnlessCondition(text_chat_mode)
    )

    motion_gate_node = Node(
        package='largemodel',
        executable='motion_gate',
        name='motion_gate',
        parameters=[params_file],
        output='screen',
        condition=IfCondition(enable_nav),
    )

    nav_task_node = Node(
        package='largemodel',
        executable='nav_task',
        name='nav_task',
        parameters=[params_file],
        output='screen',
        condition=IfCondition(enable_nav),
    )

    return LaunchDescription([
        text_chat_mode_arg,    # Declare launch arguments. / 声明启动参数。
        enable_nav_arg,        # Start task/gate nodes when true. / 是否启动导航任务层。
        camera_source_arg,     # uvc | ros_compressed.
        decision_mode_arg,     # Decision layer owns motion.
        nav_goal_topic_arg,    # /goal_pose or /ai_nav_disabled.
        model_server,          # Start the model service node. / 启动模型服务节点。
        asr_server,            # Start the ASR user interaction node. / 启动asr用户交互节点。
        motion_gate_node,      # Safe cmd_vel forwarding. / 安全速度门控。
        nav_task_node,         # Nav2 action client. / Nav2 任务客户端。
    ])





