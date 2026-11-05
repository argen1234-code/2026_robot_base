#!/usr/bin/env bash
# 下载 YOLOv8n ONNX（人像检测用）。
#
# 用法：
#   bash scripts/fetch-yolo.sh [目标路径]
# 默认：~/models/yolov8n.onnx
set -euo pipefail

DEST="${1:-$HOME/models/yolov8n.onnx}"
SHA1="68f864475d06e2ec4037181052739f268eeac38d"
mkdir -p "$(dirname "$DEST")"

if [[ -f "$DEST" ]] && echo "$SHA1  $DEST" | sha1sum -c - --status 2>/dev/null; then
  echo "已存在且校验通过: $DEST"
  exit 0
fi
rm -f "$DEST"

URLS=(
  "https://ghfast.top/https://github.com/CVHub520/X-AnyLabeling/releases/download/v0.1.0/yolov8n.onnx"
  "https://github.com/CVHub520/X-AnyLabeling/releases/download/v0.1.0/yolov8n.onnx"
)
for url in "${URLS[@]}"; do
  echo "下载: $url"
  if curl -fL --retry 2 --max-time 300 -o "$DEST.part" "$url"; then
    if echo "$SHA1  $DEST.part" | sha1sum -c - --status 2>/dev/null; then
      mv "$DEST.part" "$DEST"
      echo "完成: $DEST"
      exit 0
    fi
    echo "SHA1 校验失败，换下一个源" >&2
  fi
  rm -f "$DEST.part"
done

echo "YOLO 模型下载失败；请检查网络后重试。" >&2
exit 1
