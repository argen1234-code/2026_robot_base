#!/usr/bin/env bash
# 启动雷达驱动 + 直连 STM32 速度标定；跑完自动停雷达。
#
# 用法：
#   bash tools/run_speed_calibration.sh --revs 5 --speed 0.10
#   bash tools/run_speed_calibration.sh --revs 5 --speed 0.10 --direction backward
#
# 标定前：
#   - ROS 导航栈必须已停止（不能有 chassis_bridge 占用 /dev/stm32）
#   - 小车前方留出约 3 m，正前方约 2~2.5 m 放一个纸箱/挡板给雷达
# 注意：不要用 set -u。micromamba 的 activate.d 脚本会引用未定义的
# CONDA_BUILD，在 -u 下会直接退出。
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

export CONDA_BUILD="${CONDA_BUILD:-0}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
if [ -x "$HOME/.local/bin/micromamba" ]; then
  eval "$("$HOME/.local/bin/micromamba" shell hook -s bash)"
  micromamba activate ros_jazzy
fi
# shellcheck disable=SC1091
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

if pgrep -f "chassis_bridge|mapping_navigation|navigation.launch" >/dev/null; then
  echo "ERROR: 导航/底盘桥接还在运行，先停栈再标定" >&2
  exit 1
fi

LIDAR_LOG=/tmp/ydlidar_speedcal.log
setsid ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
  publish_static_tf:=false >"$LIDAR_LOG" 2>&1 &
LIDAR_PID=$!

cleanup() {
  kill -INT -- "-$LIDAR_PID" 2>/dev/null || true
  wait "$LIDAR_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "等待 /scan ..."
for _ in $(seq 1 20); do
  if ros2 topic list 2>/dev/null | grep -qx '/scan'; then
    echo "/scan 已出现"
    break
  fi
  sleep 1
done

python3 tools/speed_calibrate.py "$@"
RC=$?
exit "$RC"
