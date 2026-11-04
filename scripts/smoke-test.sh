#!/usr/bin/env bash
# 无硬件冒烟测试：容器内确认 AI 包与 Orbbec 包可见、Python 模块可导入。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE_NAME:-yahboom/ros-humble-ai:snapshot-20261104}"

docker run --rm --net=host \
  -v "$ROOT/ai_ws:/root/yahboom_ws:ro" \
  -v "$ROOT/third_party/orbbec_ws:/root/orbbec_ws:ro" \
  "$IMAGE" bash -lc '
    source /opt/ros/humble/setup.bash
    source /root/yahboom_ws/install/setup.bash
    echo "--- ai packages ---"
    ros2 pkg list | grep -E "^(largemodel|perception|task_manager|interfaces|text_chat)$"
    echo "--- orbbec packages ---"
    source /root/orbbec_ws/install/setup.bash
    ros2 pkg list | grep -E "^orbbec" || true
    echo "--- imports ---"
    python3 -c "import cv2, torch, funasr, mediapipe, piper; print(\"core imports OK\")"
    echo "--- model files ---"
    ls -lh /root/yahboom_ws/src/largemodel/MODELS/asr/SenseVoiceSmall/model.pt \
           /root/yahboom_ws/src/largemodel/MODELS/tts/zh/*.onnx
  '
