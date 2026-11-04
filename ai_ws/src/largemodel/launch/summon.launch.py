"""
Convenience launch: voice stack + summon task layer.

The Nav2 / Cartographer stack is deliberately NOT included here because it
lives in the navigation project.  Launch order should be:

  1. navigation bringup (map server, localization, Nav2)
  2. ros2 launch largemodel summon.launch.py

Use text_chat_mode:=true to debug without the ASR node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('largemodel')
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'largemodel_control.launch.py')
        ),
        launch_arguments={
            'enable_nav': 'true',
        }.items(),
    )

    return LaunchDescription([control])
