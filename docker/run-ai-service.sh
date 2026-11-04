#!/usr/bin/env bash
# 后台启动 AI 服务（可配合 systemd 使用）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${CONTAINER_NAME:-ros_humble_ai_service}"
IMAGE="${IMAGE_NAME:-yahboom/ros-humble-ai:2026.11}"
LAUNCH="${AI_LAUNCH:-largemodel_control.launch.py}"

ENV_ARGS=()
[ -f "$ROOT/.env" ] && ENV_ARGS=(--env-file "$ROOT/.env")

MEM_ARGS=()
if ! docker info 2>/dev/null | grep -q "No memory limit support"; then
  MEM_ARGS=(--memory=3.5g --memory-swap=3.5g --cpus=2)
else
  echo "WARN: memory cgroup 未启用，跳过容器内存限额" >&2
fi

docker rm -f "$NAME" >/dev/null 2>&1 || true
exec docker run -d --name "$NAME" --net=host --restart unless-stopped \
  "${MEM_ARGS[@]}" "${ENV_ARGS[@]}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-2}" \
  -v "$ROOT/ai_ws:/root/yahboom_ws:rw" \
  -v "$ROOT/third_party/orbbec_ws:/root/orbbec_ws:ro" \
  --device=/dev/snd \
  "$IMAGE" /usr/local/bin/entrypoint-ai.sh \
  ros2 launch largemodel "$LAUNCH"
