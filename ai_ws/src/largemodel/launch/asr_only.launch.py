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

    # 定义只启动 ASR 节点的配置
    asr_server_only = Node(
        package='largemodel',
        executable='asr',
        name='asr', # 将节点名称改回 'asr' 以加载yahboom.yaml中的正确参数
        parameters=[params_file],
        output='screen'
    )

    # 返回 LaunchDescription 对象
    return LaunchDescription([
        asr_server_only
    ]) 