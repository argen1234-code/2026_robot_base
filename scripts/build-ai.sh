#!/usr/bin/env bash
# 在 AI 容器内编译 ai_ws。cgroup 未启用时用 ulimit 兜底。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE_NAME:-yahboom/ros-humble-ai:snapshot-20261104}"

ENV_ARGS=()
[ -f "$ROOT/.env" ] && ENV_ARGS=(--env-file "$ROOT/.env")

docker run --rm --name ai-build --net=host \
  "${ENV_ARGS[@]}" \
  -v "$ROOT/ai_ws:/root/yahboom_ws:rw" \
  "$IMAGE" bash -lc '
    ulimit -v 4194304
    source /opt/ros/humble/setup.bash
    cd /root/yahboom_ws
    MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 "$@"
  ' bash "$@"

sudo chown -R pi:pi "$ROOT/ai_ws/build" "$ROOT/ai_ws/install" "$ROOT/ai_ws/log" 2>/dev/null || true
