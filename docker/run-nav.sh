#!/usr/bin/env bash
# 启动导航容器（Humble 回退环境；主线导航计划在主机 Jazzy 上运行）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:-ros_nav_dev}"
shift || true
IMAGE="${IMAGE_NAME:-yahboom/ros-nav:snapshot-20261104}"

MEM_ARGS=()
if ! docker info 2>/dev/null | grep -q "No memory limit support"; then
  MEM_ARGS=(--memory=4g --memory-swap=4g --cpus=2)
else
  echo "WARN: memory cgroup 未启用，跳过容器内存限额（earlyoom/ulimit 兜底）" >&2
fi

DEVICE_ARGS=()
for dev in /dev/ttyACM0 /dev/ttyUSB0 /dev/ydlidar /dev/stm32; do
  [ -e "$dev" ] && DEVICE_ARGS+=(--device="$dev")
done

X11_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
  X11_ARGS+=(-e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1
             -v /tmp/.X11-unix:/tmp/.X11-unix
             -v "$HOME/.Xauthority:/root/.Xauthority" -e XAUTHORITY=/root/.Xauthority)
fi

if [ "$#" -eq 0 ]; then
  set -- bash
fi

exec docker run -it --name "$NAME" --net=host \
  "${MEM_ARGS[@]}" "${DEVICE_ARGS[@]}" "${X11_ARGS[@]}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}" \
  -v "$ROOT:/root/2026_robot_base:rw" \
  "$IMAGE" bash -lc '
    source /opt/ros/humble/setup.bash
    [ -f /root/2026_robot_base/robot_ros2_ws/install/setup.bash ] && \
      source /root/2026_robot_base/robot_ros2_ws/install/setup.bash
    exec "$@"
  ' bash "$@"
