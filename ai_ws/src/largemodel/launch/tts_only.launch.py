import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取参数文件的路径
    params_file = os.path.join(
        get_package_share_directory('largemodel'), 
        "config", 
        "yahboom.yaml"
    )

    # 定义只启动 TTS 节点的配置
    tts_server_only = Node(
        package='largemodel',
        executable='tts_only',
        name='tts_only_node',
        parameters=[params_file],
        output='screen'
    )

    # 返回 LaunchDescription 对象
    return LaunchDescription([
        tts_server_only
    ]) 