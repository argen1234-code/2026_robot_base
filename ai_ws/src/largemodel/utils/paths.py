"""统一的 largemodel 资源路径解析。

优先使用环境变量 LARGEMODEL_SRC_ROOT；其次兼容旧的开发路径
~/yahboom_ws/src/largemodel；最后退回 ament 安装目录。
"""

import os


def package_root() -> str:
    env = os.environ.get("LARGEMODEL_SRC_ROOT")
    if env and os.path.isdir(env):
        return env

    dev = os.path.expanduser("~/yahboom_ws/src/largemodel")
    if os.path.isdir(dev):
        return dev

    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory("largemodel")
