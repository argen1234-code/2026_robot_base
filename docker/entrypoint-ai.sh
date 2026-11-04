#!/usr/bin/env bash
# 容器启动时按顺序 source：ROS → AI 工作区 → Orbbec。
set -e

source /opt/ros/humble/setup.bash
[ -f /root/yahboom_ws/install/setup.bash ] && source /root/yahboom_ws/install/setup.bash
[ -f /root/orbbec_ws/install/setup.bash ] && source /root/orbbec_ws/install/setup.bash

exec "$@"
