"""统一启动入口。

  # 建图（默认加载 ~/maps 最新 posegraph 续建；没有则新建）
  ros2 launch robot_base_bringup bringup.launch.py \
    slam:=true motor_enable:=true visualization:=foxglove

  # 先验地图 localization（默认最新 posegraph，或 map:=指定）
  ros2 launch robot_base_bringup bringup.launch.py \
    slam:=false map:=/home/pi/maps/robot_map \
    motor_enable:=true visualization:=foxglove

map 参数是 **posegraph 前缀**（不要带 .posegraph/.yaml）：
  slam_toolbox 用 <prefix>.posegraph/<prefix>.data；
  map_saver 的 .yaml/.pgm 不能用于 slam_toolbox localization。

slam=true 时：
  · map 为空 → 找 ~/maps 下最新 *.posegraph；没有 → 从零建图；
  · map 指定 → 加载该 posegraph 继续建图。
slam=false 时：
  · map 为空 → 用最新 posegraph；仍没有 → 启动前报错；
  · map 指定 → 用该 posegraph。
"""

import glob
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _is_true(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _latest_posegraph(maps_dir: str):
    files = glob.glob(os.path.join(maps_dir, '*.posegraph'))
    if not files:
        return None
    latest = max(files, key=os.path.getmtime)
    return latest[:-len('.posegraph')]


def _make_slam_params(base_yaml: str, mode: str, map_prefix,
                      map_start_pose, map_start_at_dock: bool) -> str:
    with open(base_yaml) as fh:
        doc = yaml.safe_load(fh)
    params = doc['slam_toolbox']['ros__parameters']
    params['mode'] = mode
    if map_prefix:
        params['map_file_name'] = map_prefix
        # slam_toolbox declares this as double_array; YAML [0,0,0] would be an
        # integer array and is rejected during configure.
        params['map_start_pose'] = [float(v) for v in map_start_pose]
        params['map_start_at_dock'] = map_start_at_dock
    out = os.path.join(
        tempfile.gettempdir(),
        f'robot_slam_params_{os.getpid()}_{mode}.yaml')
    with open(out, 'w') as fh:
        yaml.safe_dump(doc, fh, sort_keys=False)
    return out


def _prepare(context):
    slam = _is_true(context.launch_configurations.get('slam', 'true'))
    map_arg = context.launch_configurations.get('map', '').strip()
    maps_dir = os.path.expanduser(
        context.launch_configurations.get('maps_dir', '~/maps'))
    map_start_at_dock = _is_true(
        context.launch_configurations.get('map_start_at_dock', 'false'))
    try:
        map_start_pose = yaml.safe_load(
            context.launch_configurations.get('map_start_pose',
                                              '[0.0, 0.0, 0.0]'))
    except yaml.YAMLError as exc:
        raise RuntimeError(f'map_start_pose 不是合法 YAML 列表: {exc}')
    if (not isinstance(map_start_pose, (list, tuple))
            or len(map_start_pose) != 3):
        raise RuntimeError('map_start_pose 必须是 [x, y, yaw] 三项')

    if map_arg.lower() == 'latest':
        map_prefix = _latest_posegraph(maps_dir)
    elif map_arg:
        if map_arg.lower() in ('none', 'fresh'):
            if not slam:
                raise RuntimeError('slam:=false 不能使用 map:=none')
            map_prefix = None
        else:
            if map_arg.endswith('.yaml') or map_arg.endswith('.pgm'):
                raise RuntimeError(
                    'slam_toolbox 需要 .posegraph 前缀，不能用 map_saver 的 '
                    '.yaml/.pgm；请用 serialize_map 保存 posegraph。')
            prefix = map_arg[:-len('.posegraph')] if map_arg.endswith(
                '.posegraph') else map_arg
            if not os.path.exists(prefix + '.posegraph'):
                raise RuntimeError(f'找不到 posegraph: {prefix}.posegraph')
            map_prefix = prefix
    else:
        map_prefix = _latest_posegraph(maps_dir)
    if not slam and map_prefix is None:
        raise RuntimeError(
            f'slam:=false 需要 posegraph，但 {maps_dir} 下没有 '
            '*.posegraph；请先用 serialize_map 保存。')

    mode = 'mapping' if slam else 'localization'
    base_params = os.path.join(
        get_package_share_directory('robot_base_bringup'),
        'config', 'slam_toolbox.yaml')
    slam_params_file = _make_slam_params(
        base_params, mode, map_prefix, map_start_pose, map_start_at_dock)

    bringup = get_package_share_directory('robot_base_bringup')
    sub = 'mapping_navigation.launch.py' if slam else 'navigation.launch.py'
    args = {
        'motor_enable': context.launch_configurations.get(
            'motor_enable', 'false'),
        'visualization': context.launch_configurations.get(
            'visualization', 'foxglove'),
        'lidar_port': context.launch_configurations.get(
            'lidar_port', '/dev/ydlidar'),
        'mcu_port': context.launch_configurations.get(
            'mcu_port', '/dev/stm32'),
        'wechat': context.launch_configurations.get('wechat', 'false'),
        'slam_params_file': slam_params_file,
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', sub)),
        launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='true=建图；false=slam_toolbox localization 先验地图'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='posegraph 前缀；空=~/maps 最新，slam=false 时必需'),
        DeclareLaunchArgument(
            'maps_dir', default_value='~/maps',
            description='自动寻找最新 *.posegraph 的目录'),
        DeclareLaunchArgument(
            'map_start_pose', default_value='[0.0, 0.0, 0.0]',
            description='加载 posegraph 的初始位姿 [x, y, yaw]'),
        DeclareLaunchArgument(
            'map_start_at_dock', default_value='false',
            description='使用 posegraph 里保存的 dock 位姿'),
        DeclareLaunchArgument('motor_enable', default_value='false'),
        DeclareLaunchArgument('visualization', default_value='foxglove',
                              choices=['foxglove', 'rviz', 'none']),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ydlidar'),
        DeclareLaunchArgument('mcu_port', default_value='/dev/stm32'),
        DeclareLaunchArgument('wechat', default_value='false'),
        OpaqueFunction(function=_prepare),
    ])
