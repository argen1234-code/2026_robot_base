#!/usr/bin/env bash
# 启动 AI 容器。用法：
#   docker/run-ai.sh [容器名] [命令...]
# 默认交互式 bash；例如 docker/run-ai.sh ros_humble_ai_dev \
#   ros2 launch largemodel largemodel_control.launch.py
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:-ros_humble_ai_dev}"
shift || true
IMAGE="${IMAGE_NAME:-yahboom/ros-humble-ai:2026.11}"

ENV_ARGS=()
[ -f "$ROOT/.env" ] && ENV_ARGS=(--env-file "$ROOT/.env")

# 当前内核若未启用 memory cgroup，--memory 会直接报错，这里自动跳过。
MEM_ARGS=()
if ! docker info 2>/dev/null | grep -q "No memory limit support"; then
  MEM_ARGS=(--memory=3.5g --memory-swap=3.5g --cpus=2)
else
  echo "WARN: memory cgroup 未启用，跳过容器内存限额（earlyoom/ulimit 兜底）" >&2
fi

DEVICE_ARGS=()
[ -e /dev/video0 ] && DEVICE_ARGS+=(--device=/dev/video0)
[ -e /dev/snd ] && DEVICE_ARGS+=(--device=/dev/snd)
[ -e /dev/bus/usb ] && DEVICE_ARGS+=(--device=/dev/bus/usb)

X11_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
  X11_ARGS+=(-e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1
             -v /tmp/.X11-unix:/tmp/.X11-unix
             -v "$HOME/.Xauthority:/root/.Xauthority" -e XAUTHORITY=/root/.Xauthority)
fi
if [ -d /run/user/1000/pulse ]; then
  X11_ARGS+=(-e PULSE_SERVER=unix:/run/user/1000/pulse/native
             -v /run/user/1000/pulse:/run/user/1000/pulse:ro
             -v "$HOME/.config/pulse:/root/.config/pulse:ro")
fi

if [ "$#" -eq 0 ]; then
  set -- bash
fi

exec docker run -it --name "$NAME" --net=host --privileged \
  "${MEM_ARGS[@]}" "${ENV_ARGS[@]}" "${DEVICE_ARGS[@]}" "${X11_ARGS[@]}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}" \
  -v "$ROOT/ai_ws:/root/yahboom_ws:rw" \
  -v "$ROOT/third_party/orbbec_ws:/root/orbbec_ws:ro" \
  "$IMAGE" /usr/local/bin/entrypoint-ai.sh "$@"
