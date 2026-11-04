#!/usr/bin/env bash
# 重新编译 Orbbec 驱动（仅在需要时执行；install/ 已随仓库本地保存）。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE_NAME:-yahboom/ros-humble-ai:snapshot-20261104}"

mkdir -p "$ROOT/third_party/orbbec_ws/build" "$ROOT/third_party/orbbec_ws/install" \
         "$ROOT/third_party/orbbec_ws/log"

docker run --rm --name orbbec-build --net=host \
  -v "$ROOT/third_party/orbbec_ws:/root/orbbec_ws:rw" \
  "$IMAGE" bash -lc '
    ulimit -v 4194304
    source /opt/ros/humble/setup.bash
    cd /root/orbbec_ws
    MAKEFLAGS=-j2 colcon build --parallel-workers 2 "$@"
  ' bash "$@"

sudo chown -R pi:pi "$ROOT/third_party/orbbec_ws"
