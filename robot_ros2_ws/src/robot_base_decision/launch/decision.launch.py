"""Hierarchical decision layers.

The top-level task_coordinator is always started.  Each stack decision layer
is optional so ``robot.sh`` can run only the selected stacks.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_nav = LaunchConfiguration('start_nav_decision')
    start_voice = LaunchConfiguration('start_voice_decision')
    start_vision = LaunchConfiguration('start_vision_decision')
    initial_task = LaunchConfiguration('initial_task')
    execute = LaunchConfiguration('execute')
    preview = LaunchConfiguration('preview')
    config = LaunchConfiguration('config')
    http_port = LaunchConfiguration('vision_http_port')

    return LaunchDescription([
        DeclareLaunchArgument('start_nav_decision', default_value='true'),
        DeclareLaunchArgument('start_voice_decision', default_value='true'),
        DeclareLaunchArgument('start_vision_decision', default_value='true'),
        DeclareLaunchArgument('initial_task', default_value='none'),
        DeclareLaunchArgument('execute', default_value='false'),
        DeclareLaunchArgument('preview', default_value='false'),
        DeclareLaunchArgument(
            'config',
            default_value='/home/pi/2026_robot_base/config/person_approach.json'),
        DeclareLaunchArgument('vision_http_port', default_value='8878'),

        Node(
            package='robot_base_decision',
            executable='task_coordinator',
            name='task_coordinator',
            output='screen',
            parameters=[{
                'initial_task': initial_task,
                'execute': ParameterValue(execute, value_type=bool),
                'preview': ParameterValue(preview, value_type=bool),
            }],
        ),
        Node(
            package='robot_base_decision',
            executable='nav_decision',
            name='nav_decision',
            output='screen',
            condition=IfCondition(start_nav),
        ),
        # Use ExecuteProcess for the argparse-based nodes.  launch_ros.Node
        # appends --ros-args -r __node:=..., which interferes with the
        # voice node's /asr subscription in this environment.
        GroupAction(
            condition=IfCondition(start_voice),
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'robot_base_decision',
                         'voice_decision'],
                    output='screen',
                    condition=UnlessCondition(execute),
                ),
                ExecuteProcess(
                    cmd=['ros2', 'run', 'robot_base_decision',
                         'voice_decision', '--execute'],
                    output='screen',
                    condition=IfCondition(execute),
                ),
            ],
        ),
        # Preview vision node: no --execute, never moves.
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_base_decision', 'vision_decision',
                 '--config', config, '--http-port', http_port],
            output='screen',
            condition=UnlessCondition(execute),
        ),
        # Motion-capable vision node: follow mode, task activation still comes
        # from the coordinator / web UI (no --auto).
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_base_decision', 'vision_decision',
                 '--config', config, '--execute', '--follow',
                 '--http-port', http_port],
            output='screen',
            condition=IfCondition(execute),
        ),
    ])
