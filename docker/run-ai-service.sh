#!/usr/bin/env bash
# 后台启动 AI 服务（可配合 systemd 使用）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${CONTAINER_NAME:-ros_humble_ai_service}"
IMAGE="${IMAGE_NAME:-yahboom/ros-humble-ai:2026.11-mqtt}"
LAUNCH="${AI_LAUNCH:-largemodel_control.launch.py}"
# 不要继承调用 shell 的 ROS_DOMAIN_ID（主机导航是 1）；AI 默认固定 2。
# 需要同域调试时显式 AI_ROS_DOMAIN_ID=1。
AI_DOMAIN="${AI_ROS_DOMAIN_ID:-2}"

ENV_ARGS=()
[ -f "$ROOT/.env" ] && ENV_ARGS=(--env-file "$ROOT/.env")

MEM_ARGS=()
if ! docker info 2>/dev/null | grep -q "No memory limit support"; then
  MEM_ARGS=(--memory=3.5g --memory-swap=3.5g --cpus=2)
else
  echo "WARN: memory cgroup 未启用，跳过容器内存限额" >&2
fi

DEVICE_ARGS=(--device=/dev/snd)
[ -e /dev/mic-kws ] && DEVICE_ARGS+=(--device=/dev/mic-kws)
[ -e /dev/video0 ] && DEVICE_ARGS+=(--device=/dev/video0)

# 主机是 PipeWire/PulseAudio；容器内的 pygame/ALSA 需要走 Pulse 混音，
# 否则 USB 声卡被主机音频服务占用，pygame.mixer.init() 会报 Device busy。
PULSE_ARGS=()
PULSE_SOCKET="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/pulse/native"
if [ -S "$PULSE_SOCKET" ]; then
  PULSE_ARGS=(-e PULSE_SERVER="unix:$PULSE_SOCKET"
              -e SDL_AUDIODRIVER=pulseaudio
              -v "${PULSE_SOCKET%/native}":"${PULSE_SOCKET%/native}":ro
              -v "$HOME/.config/pulse:/root/.config/pulse:ro")
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true
exec docker run -d --name "$NAME" --net=host --restart unless-stopped \
  "${MEM_ARGS[@]}" "${ENV_ARGS[@]}" "${DEVICE_ARGS[@]}" "${PULSE_ARGS[@]}" \
  -e ROS_DOMAIN_ID="$AI_DOMAIN" \
  -v "$ROOT/ai_ws:/root/yahboom_ws:rw" \
  -v "$ROOT/third_party/orbbec_ws:/root/orbbec_ws:ro" \
  "$IMAGE" /usr/local/bin/entrypoint-ai.sh \
  ros2 launch largemodel "$LAUNCH"
