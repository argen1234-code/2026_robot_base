#!/usr/bin/env bash
# 下载本地 ASR/TTS 模型到 ai_ws/src/largemodel/MODELS（模型不入库）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODELS="$ROOT/ai_ws/src/largemodel/MODELS"
mkdir -p "$MODELS/asr/SenseVoiceSmall" "$MODELS/tts/zh" "$MODELS/tts/en"

# --- SenseVoiceSmall（约 893MB）---
ASR="$MODELS/asr/SenseVoiceSmall/model.pt"
if [ ! -f "$ASR" ] || [ "$(stat -c%s "$ASR")" -lt 800000000 ]; then
  echo "Downloading SenseVoiceSmall ..."
  uv tool run --from modelscope modelscope download \
    --model iic/SenseVoiceSmall --local_dir "$MODELS/asr/SenseVoiceSmall"
else
  echo "SenseVoiceSmall exists, skip"
fi

# --- Piper 中文 / 英文 ---
fetch() {
  local url="$1" out="$2"
  if [ -s "$out" ]; then echo "$(basename "$out") exists, skip"; return; fi
  echo "Downloading $(basename "$out") ..."
  curl -fL --retry 3 -o "$out" "$url"
}

BASE="https://hf-mirror.com/rhasspy/piper-voices/resolve/main"
fetch "$BASE/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx" \
      "$MODELS/tts/zh/zh_CN-huayan-medium.onnx"
fetch "$BASE/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx.json" \
      "$MODELS/tts/zh/zh_CN-huayan-medium.onnx.json"
fetch "$BASE/en/en_US/libritts/high/en_US-libritts-high.onnx" \
      "$MODELS/tts/en/en_US-libritts-high.onnx"
fetch "$BASE/en/en_US/libritts/high/en_US-libritts-high.onnx.json" \
      "$MODELS/tts/en/en_US-libritts-high.onnx.json"

echo "Done. Models in $MODELS"
du -sh "$MODELS"
