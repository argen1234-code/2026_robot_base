#!/usr/bin/env bash
# 主机原生启动：导航 ros_jazzy + AI 语音 ai_host，同 domain 1，直接 /goal_pose。
# 不使用 Docker、不使用 MQTT。
#
# 用法：
#   bash scripts/start_host_stack.sh                 # 最新地图，干跑（不 ARM）
#   bash scripts/start_host_stack.sh --motor         # 允许电机
#   bash scripts/start_host_stack.sh --map /home/pi/maps/robot_map_...
#   bash scripts/start_host_stack.sh --viz foxglove  # 默认 none
#   bash scripts/start_host_stack.sh --no-hand       # 不启动手部检测（默认自动）
#   bash scripts/start_host_stack.sh --cam-images    # 转发压缩图给 Foxglove（默认随 --viz foxglove）
#   bash scripts/start_host_stack.sh --person        # YOLO 人像检测（自动启动深度）
#   bash scripts/start_host_stack.sh --depth         # 只启动 Orbbec 深度
#   bash scripts/start_host_stack.sh --no-depth      # 人像仅用彩色（2D）
#   PERSON_CV_THREADS=2 PERSON_MAX_FPS=3 bash scripts/start_host_stack.sh --person
#
# 手部检测：/dev/hand-cam → MediaPipe 独立 venv → UDP → /perception/hand_pose
# 人像检测：YOLOv8n ONNX + 深度 → /perception/person_detections
# 压缩图：/camera/color/image_raw/compressed + /perception/hand_image/compressed
# Ctrl-C 会停止 AI、导航、深度与人像（导航退出自动 DISARM）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST_AI_WS="${HOST_AI_WS:-$HOME/host_ai_ws}"
MAP=""
MOTOR="false"
VIZ="none"
HAND="auto"
CAM_IMAGES="auto"
PERSON="false"
DEPTH="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="${2:-}"; shift 2 ;;
    --motor) MOTOR="true"; shift ;;
    --viz) VIZ="${2:-none}"; shift 2 ;;
    --hand) HAND="true"; shift ;;
    --no-hand) HAND="false"; shift ;;
    --cam-images) CAM_IMAGES="true"; shift ;;
    --no-cam-images) CAM_IMAGES="false"; shift ;;
    --person) PERSON="true"; DEPTH="true"; shift ;;
    --depth) DEPTH="true"; shift ;;
    --no-depth) DEPTH="false"; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "$MAP" ]]; then
  MAP="$(ls -t "$HOME"/maps/*.posegraph 2>/dev/null | head -1 || true)"
  MAP="${MAP%.posegraph}"
fi
[[ -n "$MAP" && -f "$MAP.posegraph" ]] || {
  echo "找不到 posegraph: ${MAP:-<empty>}"; exit 2; }

LOGDIR="${HOST_STACK_LOG_DIR:-$HOME/rosbags/host_stack_logs/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOGDIR"

warn() { echo "[host] WARN: $*" >&2; }
die()  { echo "[host] ERROR: $*" >&2; exit 1; }

echo "[host] 外设检查"
for link in /dev/ydlidar /dev/stm32 /dev/mic-kws /dev/astra-color /dev/hand-cam; do
  if [[ -e "$link" ]]; then
    printf '  %-14s -> %s\n' "$link" "$(readlink "$link" 2>/dev/null || echo present)"
  else
    printf '  %-14s MISSING\n' "$link"
  fi
done
[[ -e /dev/ydlidar ]] || die "/dev/ydlidar 缺失"
[[ "$MOTOR" == "true" && ! -e /dev/stm32 ]] && die "--motor 需要 /dev/stm32"
[[ -e /dev/mic-kws ]] || warn "KWS 缺失；语音唤醒不可用"
arecord -l 2>/dev/null | grep -q "USB Audio" || warn "USB 音频 capture 缺失"
echo "  map            $MAP"

[[ -f "$HOST_AI_WS/install/setup.bash" ]] || \
  die "host_ai_ws 未构建；先运行 colcon build --packages-select interfaces largemodel"
[[ -f "$ROOT/.env" ]] || warn ".env 不存在，云端 ASR/TTS/LLM 可能失败"

HAND_VENV="${HAND_VENV:-$HOME/mediapipe_plain}"
HAND_DEVICE="${HAND_DEVICE:-/dev/hand-cam}"
HAND_UDP="${HAND_UDP:-127.0.0.1:5005}"
HAND_ENABLE="false"
if [[ "$HAND" != "false" && -e "$HAND_DEVICE" && -x "$HAND_VENV/bin/python" ]]; then
  HAND_ENABLE="true"
fi
if [[ "$HAND" == "true" && "$HAND_ENABLE" != "true" ]]; then
  die "--hand 需要 $HAND_DEVICE 和 $HAND_VENV/bin/python"
fi
if [[ "$HAND_ENABLE" != "true" ]]; then
  warn "手部检测未启用（$HAND_DEVICE 或 $HAND_VENV 缺失）"
fi

# 压缩图默认只在开 Foxglove 时启用，避免无谓的编码与 DDS 流量。
HAND_IMAGES="false"
if [[ "$HAND_ENABLE" == "true" ]]; then
  if [[ "$CAM_IMAGES" == "true" ]]; then
    HAND_IMAGES="true"
  elif [[ "$CAM_IMAGES" == "auto" && "$VIZ" == "foxglove" ]]; then
    HAND_IMAGES="true"
  fi
fi

PERSON_MODEL="${PERSON_MODEL:-}"
PERSON_INPUT_SIZE="${PERSON_INPUT_SIZE:-}"
PERSON_BACKEND="${PERSON_BACKEND:-auto}"
PERSON_RUNTIME_DIR="${PERSON_RUNTIME_DIR:-$HOME/vision_runtime}"
if [[ "$PERSON_BACKEND" == "auto" ]]; then
  if [[ -n "$PERSON_MODEL" ]]; then
    if [[ -d "$PERSON_MODEL" ]]; then PERSON_BACKEND="ncnn"; else PERSON_BACKEND="opencv"; fi
  elif [[ -f "$HOME/models/vision_v2/yolov8n_320_ncnn_model/model.ncnn.bin" ]] \
       && PYTHONPATH="$PERSON_RUNTIME_DIR${PYTHONPATH:+:$PYTHONPATH}" \
          "$HOME/micromamba/envs/ai_host/bin/python" -c 'import ncnn; assert hasattr(ncnn, "Net")' >/dev/null 2>&1; then
    PERSON_BACKEND="ncnn"
  else
    PERSON_BACKEND="opencv"
  fi
fi
if [[ -z "$PERSON_MODEL" ]]; then
  if [[ "$PERSON_BACKEND" == "ncnn" ]]; then
    PERSON_MODEL="$HOME/models/vision_v2/yolov8n_320_ncnn_model"
    PERSON_INPUT_SIZE="${PERSON_INPUT_SIZE:-320}"
  elif [[ -f "$HOME/models/yolov8n_320.onnx" ]]; then
    PERSON_MODEL="$HOME/models/yolov8n_320.onnx"
    PERSON_INPUT_SIZE="${PERSON_INPUT_SIZE:-320}"
  else
    PERSON_MODEL="$HOME/models/yolov8n.onnx"
    PERSON_INPUT_SIZE="${PERSON_INPUT_SIZE:-640}"
  fi
fi
PERSON_INPUT_SIZE="${PERSON_INPUT_SIZE:-640}"
PERSON_INPUT_HEIGHT="${PERSON_INPUT_HEIGHT:-$PERSON_INPUT_SIZE}"
if [[ "$PERSON_BACKEND" == "ncnn" ]]; then
  PERSON_CV_THREADS="${PERSON_CV_THREADS:-1}"
  PERSON_MAX_FPS="${PERSON_MAX_FPS:-6}"
else
  PERSON_CV_THREADS="${PERSON_CV_THREADS:-2}"
  PERSON_MAX_FPS="${PERSON_MAX_FPS:-3}"
fi
PERSON_ANNOTATION_FPS="${PERSON_ANNOTATION_FPS:-3}"
PERSON_CONF="${PERSON_CONF:-0.35}"
PERSON_SUPPORT_CONF="${PERSON_SUPPORT_CONF:-0.2}"
DEPTH_WS="${DEPTH_WS:-$HOME/orbbec_jazzy_ws}"
# Astra Pro 在 USB2.0 上 640×480 Y11@30 实测只有 ~20–24 fps（等时带宽不足），
# 320×240@30 可稳定 29.7 fps 且 CPU 约减半。人像距离用归一化坐标映射，
# 320 分辨率不影响取距；如需更高精度可设 DEPTH_WIDTH=640 DEPTH_HEIGHT=480。
DEPTH_WIDTH="${DEPTH_WIDTH:-320}"
DEPTH_HEIGHT="${DEPTH_HEIGHT:-240}"
DEPTH_FPS="${DEPTH_FPS:-30}"
DEPTH_ENABLE="false"
if [[ "$DEPTH" != "false" && -f "$DEPTH_WS/install/setup.bash" \
      && -e /dev/astra-color ]]; then
  DEPTH_ENABLE="true"
fi
if [[ "$DEPTH" == "true" && "$DEPTH_ENABLE" != "true" ]]; then
  warn "深度驱动未启用（$DEPTH_WS/install/setup.bash 或 /dev/astra-color 缺失）"
fi

PERSON_ENABLE="false"
if [[ "$PERSON" == "true" ]]; then
  if [[ "$PERSON_BACKEND" == "ncnn" ]]; then
    [[ -f "$PERSON_MODEL/model.ncnn.param" && -f "$PERSON_MODEL/model.ncnn.bin" ]] || \
      die "--person 需要 NCNN 模型目录 $PERSON_MODEL"
  else
    [[ -f "$PERSON_MODEL" ]] || \
      die "--person 需要模型 $PERSON_MODEL；先运行 bash scripts/fetch-yolo.sh"
  fi
  PERSON_ENABLE="true"
  [[ "$DEPTH_ENABLE" == "true" ]] || warn "深度不可用，人像检测仅输出 2D"
fi

if pgrep -f "[r]os2 launch robot_base_bringup bringup.launch.py" >/dev/null; then
  die "已有导航 launch 在跑，先停掉再启动"
fi
if pgrep -f "[r]os2 launch largemodel largemodel_control.launch.py" >/dev/null; then
  die "已有 AI launch 在跑，先停掉再启动"
fi
if [[ "$PERSON_ENABLE" == "true" ]] && \
    pgrep -u "$(id -u)" -f '(^|/)python(3([.][0-9]+)?)? +[^ ]*/tools/person_detector_node[.]py( |$)' >/dev/null; then
  die "已有独立人像检测进程在跑，先停掉再启动，避免重复推理"
fi
if [[ "$DEPTH_ENABLE" == "true" ]] && \
    pgrep -f "[r]os2 launch orbbec_camera astra.launch.py" >/dev/null; then
  die "已有 Orbbec 深度 launch 在跑，先停掉再启动"
fi

cat >"$LOGDIR/nav_cmd.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=1
ps -o pgid= -p \$\$ | tr -d ' ' > "$LOGDIR/nav.pgid"
exec ros2 launch robot_base_bringup bringup.launch.py \
  slam:=false map:="$MAP" motor_enable:="$MOTOR" visualization:="$VIZ"
EOF
cat >"$LOGDIR/ai_cmd.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$HOST_AI_WS/install/setup.bash"
set -a
source "$ROOT/.env"
set +a
export ROS_DOMAIN_ID=1
ps -o pgid= -p \$\$ | tr -d ' ' > "$LOGDIR/ai.pgid"
exec ros2 launch largemodel largemodel_control.launch.py
EOF
if [[ "$HAND_ENABLE" == "true" ]]; then
HAND_IMAGE_ARGS=""
if [[ "$HAND_IMAGES" == "true" ]]; then
  HAND_IMAGE_ARGS="--image-udp 127.0.0.1:5006 --image-fps 3 --image-width 320 --jpeg-quality 60"
fi
cat >"$LOGDIR/hand_det_cmd.sh" <<EOF
#!/usr/bin/env bash
set -e
ps -o pgid= -p \$\$ | tr -d ' ' > "$LOGDIR/hand_det.pgid"
exec nice -n 5 env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH "$HAND_VENV/bin/python" \\
  "$ROOT/tools/hand_detector_host.py" --device "$HAND_DEVICE" --udp "$HAND_UDP" \\
  $HAND_IMAGE_ARGS
EOF
cat >"$LOGDIR/hand_bridge_cmd.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$HOST_AI_WS/install/setup.bash"
export ROS_DOMAIN_ID=1
ps -o pgid= -p \$\$ | tr -d ' ' > "$LOGDIR/hand_bridge.pgid"
exec python3 "$ROOT/tools/hand_pose_bridge.py"
EOF
chmod +x "$LOGDIR/hand_det_cmd.sh" "$LOGDIR/hand_bridge_cmd.sh"
fi
if [[ "$DEPTH_ENABLE" == "true" ]]; then
cat >"$LOGDIR/depth_cmd.sh" <<EOF
#!/usr/bin/env bash
set -e
source "$ROOT/robot_ros2_ws/install/setup.bash"
source "$DEPTH_WS/install/setup.bash"
export ROS_DOMAIN_ID=1
ps -o pgid= -p \$\$ | tr -d ' ' > "$LOGDIR/depth.pgid"
exec nice -n 5 ros2 launch orbbec_camera astra.launch.py \
  enable_color:=false enable_depth:=true enable_point_cloud:=false \
  depth_format:=Y11 depth_fps:=$DEPTH_FPS \
  depth_width:=$DEPTH_WIDTH depth_height:=$DEPTH_HEIGHT
EOF
chmod +x "$LOGDIR/depth_cmd.sh"
fi

if [[ "$PERSON_ENABLE" == "true" ]]; then
cat >"$LOGDIR/person_cmd.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$HOST_AI_WS/install/setup.bash"
export ROS_DOMAIN_ID=1
ps -o pgid= -p \$\$ | tr -d ' ' > "$LOGDIR/person.pgid"
export OMP_NUM_THREADS="$PERSON_CV_THREADS"
export OPENBLAS_NUM_THREADS=1
export OMP_WAIT_POLICY=PASSIVE
exec nice -n 5 python3 "$ROOT/tools/person_detector_node.py" \
  --model "$PERSON_MODEL" --input-size "$PERSON_INPUT_SIZE" \
  --input-height "$PERSON_INPUT_HEIGHT" --backend "$PERSON_BACKEND" \
  --runtime-dir "$PERSON_RUNTIME_DIR" --conf "$PERSON_CONF" --support-conf "$PERSON_SUPPORT_CONF" \
  --max-fps "$PERSON_MAX_FPS" --cv-threads "$PERSON_CV_THREADS" --annotation-fps "$PERSON_ANNOTATION_FPS"
EOF
chmod +x "$LOGDIR/person_cmd.sh"
fi
chmod +x "$LOGDIR/nav_cmd.sh" "$LOGDIR/ai_cmd.sh"

export PATH="$HOME/.local/bin:$PATH"
export MAMBA_ROOT_PREFIX="$HOME/micromamba"

NAV_PG=""
AI_PG=""
HAND_DET_PG=""
HAND_BRIDGE_PG=""
DEPTH_PG=""
PERSON_PG=""
cleanup() {
  set +e
  trap - EXIT INT TERM
  echo
  echo "[host] 正在停止..."
  [[ -n "$PERSON_PG" ]] && kill -INT -- -"$PERSON_PG" 2>/dev/null
  [[ -n "$DEPTH_PG" ]] && kill -INT -- -"$DEPTH_PG" 2>/dev/null
  [[ -n "$HAND_BRIDGE_PG" ]] && kill -INT -- -"$HAND_BRIDGE_PG" 2>/dev/null
  [[ -n "$HAND_DET_PG" ]] && kill -INT -- -"$HAND_DET_PG" 2>/dev/null
  [[ -n "$AI_PG" ]] && kill -INT -- -"$AI_PG" 2>/dev/null
  [[ -n "$NAV_PG" ]] && kill -INT -- -"$NAV_PG" 2>/dev/null
  sleep 3
  [[ -n "$PERSON_PG" ]] && kill -TERM -- -"$PERSON_PG" 2>/dev/null
  [[ -n "$DEPTH_PG" ]] && kill -TERM -- -"$DEPTH_PG" 2>/dev/null
  [[ -n "$HAND_BRIDGE_PG" ]] && kill -TERM -- -"$HAND_BRIDGE_PG" 2>/dev/null
  [[ -n "$HAND_DET_PG" ]] && kill -TERM -- -"$HAND_DET_PG" 2>/dev/null
  [[ -n "$AI_PG" ]] && kill -TERM -- -"$AI_PG" 2>/dev/null
  [[ -n "$NAV_PG" ]] && kill -TERM -- -"$NAV_PG" 2>/dev/null
  sleep 1
  [[ -n "$PERSON_PG" ]] && kill -KILL -- -"$PERSON_PG" 2>/dev/null
  [[ -n "$DEPTH_PG" ]] && kill -KILL -- -"$DEPTH_PG" 2>/dev/null
  [[ -n "$HAND_BRIDGE_PG" ]] && kill -KILL -- -"$HAND_BRIDGE_PG" 2>/dev/null
  [[ -n "$HAND_DET_PG" ]] && kill -KILL -- -"$HAND_DET_PG" 2>/dev/null
  [[ -n "$AI_PG" ]] && kill -KILL -- -"$AI_PG" 2>/dev/null
  [[ -n "$NAV_PG" ]] && kill -KILL -- -"$NAV_PG" 2>/dev/null
  echo "[host] 已停止（导航 Ctrl-C 时已发送 DISARM）"
}
trap cleanup EXIT INT TERM

echo "[host] 启动导航（ros_jazzy, domain 1, motor=$MOTOR）..."
setsid --wait micromamba run -n ros_jazzy bash "$LOGDIR/nav_cmd.sh" \
  >"$LOGDIR/nav.log" 2>&1 &
NAV_RUN_PID=$!
for _ in $(seq 1 40); do [[ -s "$LOGDIR/nav.pgid" ]] && break; sleep 0.25; done
[[ -s "$LOGDIR/nav.pgid" ]] || die "导航没有写 pgid"
NAV_PG="$(cat "$LOGDIR/nav.pgid")"
for _ in $(seq 1 120); do
  grep -q "Managed nodes are active" "$LOGDIR/nav.log" 2>/dev/null && break
  sleep 0.5
done
grep -q "Managed nodes are active" "$LOGDIR/nav.log" 2>/dev/null \
  || { tail -n 50 "$LOGDIR/nav.log" >&2; die "导航未激活"; }
echo "  导航 active"

echo "[host] 启动 AI（ai_host, domain 1）..."
setsid --wait micromamba run -n ai_host bash "$LOGDIR/ai_cmd.sh" \
  >"$LOGDIR/ai.log" 2>&1 &
AI_RUN_PID=$!
for _ in $(seq 1 40); do [[ -s "$LOGDIR/ai.pgid" ]] && break; sleep 0.25; done
[[ -s "$LOGDIR/ai.pgid" ]] || die "AI 没有写 pgid"
AI_PG="$(cat "$LOGDIR/ai.pgid")"
# 注意：asr 节点也会打印 "Initialization completed"，必须等 model_service
# 的工具注册行，否则会在 AI 真正就绪前误报。
for _ in $(seq 1 60); do
  grep -q "LargeModelService node Initialization completed" "$LOGDIR/ai.log" 2>/dev/null && break
  sleep 0.5
done
grep -q "LargeModelService node Initialization completed" "$LOGDIR/ai.log" 2>/dev/null \
  || warn "AI 未确认初始化，见 $LOGDIR/ai.log"
AI_TOOLS="$(grep -oE "Registered [0-9]+ tools" "$LOGDIR/ai.log" 2>/dev/null | tail -1 || true)"
if [[ -n "$AI_TOOLS" ]]; then
  echo "  AI $AI_TOOLS"
else
  warn "AI 工具数未确认"
fi

if [[ "$HAND_ENABLE" == "true" ]]; then
  echo "[host] 启动手部检测（$HAND_DEVICE → UDP $HAND_UDP → /perception/hand_pose）..."
  setsid --wait bash "$LOGDIR/hand_det_cmd.sh" \
    >"$LOGDIR/hand_det.log" 2>&1 &
  setsid --wait micromamba run -n ai_host bash "$LOGDIR/hand_bridge_cmd.sh" \
    >"$LOGDIR/hand_bridge.log" 2>&1 &
  for _ in $(seq 1 40); do
    [[ -s "$LOGDIR/hand_det.pgid" && -s "$LOGDIR/hand_bridge.pgid" ]] && break
    sleep 0.25
  done
  HAND_DET_PG="$(cat "$LOGDIR/hand_det.pgid" 2>/dev/null || true)"
  HAND_BRIDGE_PG="$(cat "$LOGDIR/hand_bridge.pgid" 2>/dev/null || true)"
  sleep 3
  grep -q "opened" "$LOGDIR/hand_det.log" 2>/dev/null \
    && echo "  手部相机 $HAND_DEVICE opened" \
    || warn "手部检测未确认打开相机，见 $LOGDIR/hand_det.log"
  grep -q "hand bridge UDP" "$LOGDIR/hand_bridge.log" 2>/dev/null \
    && echo "  手部桥接 /perception/hand_pose" \
    || warn "手部桥接未确认启动，见 $LOGDIR/hand_bridge.log"
fi

if [[ "$DEPTH_ENABLE" == "true" ]]; then
  echo "[host] 启动 Orbbec 深度（ros_jazzy, domain 1）..."
  setsid --wait micromamba run -n ros_jazzy bash "$LOGDIR/depth_cmd.sh" \
    >"$LOGDIR/depth.log" 2>&1 &
  for _ in $(seq 1 40); do [[ -s "$LOGDIR/depth.pgid" ]] && break; sleep 0.25; done
  DEPTH_PG="$(cat "$LOGDIR/depth.pgid" 2>/dev/null || true)"
  for _ in $(seq 1 30); do
    grep -q "Start device" "$LOGDIR/depth.log" 2>/dev/null && break
    sleep 0.5
  done
  grep -q "Start device" "$LOGDIR/depth.log" 2>/dev/null \
    && echo "  深度 /camera/depth/image_raw" \
    || warn "深度未确认启动，见 $LOGDIR/depth.log"
fi

if [[ "$PERSON_ENABLE" == "true" ]]; then
  echo "[host] 启动人像检测（YOLOv8n, domain 1）..."
  echo "  model=$PERSON_MODEL backend=$PERSON_BACKEND input=${PERSON_INPUT_SIZE}x${PERSON_INPUT_HEIGHT} max_fps=$PERSON_MAX_FPS cv_threads=$PERSON_CV_THREADS"
  setsid --wait micromamba run -n ai_host bash "$LOGDIR/person_cmd.sh" \
    >"$LOGDIR/person.log" 2>&1 &
  for _ in $(seq 1 40); do [[ -s "$LOGDIR/person.pgid" ]] && break; sleep 0.25; done
  PERSON_PG="$(cat "$LOGDIR/person.pgid" 2>/dev/null || true)"
  for _ in $(seq 1 40); do
    grep -q "person detector ready" "$LOGDIR/person.log" 2>/dev/null && break
    sleep 0.5
  done
  grep -q "person detector ready" "$LOGDIR/person.log" 2>/dev/null \
    && echo "  人像 /perception/person_detections" \
    || warn "人像检测未确认启动，见 $LOGDIR/person.log"
fi

cat <<EOF
[host] READY
  导航 ros_jazzy / AI ai_host / domain 1 / 日志: $LOGDIR
  /dev/mic-kws -> $(readlink /dev/mic-kws 2>/dev/null || echo missing)
  /dev/astra-color -> $(readlink /dev/astra-color 2>/dev/null || echo missing)
  /dev/hand-cam    -> $(readlink /dev/hand-cam 2>/dev/null || echo missing)
  手部检测     -> $(if [[ "$HAND_ENABLE" == "true" ]]; then echo "$HAND_DEVICE → /perception/hand_pose"; else echo "disabled"; fi)
  深度/人像    -> $(if [[ "$PERSON_ENABLE" == "true" ]]; then echo "/camera/depth/image_raw + /perception/person_detections"; elif [[ "$DEPTH_ENABLE" == "true" ]]; then echo "/camera/depth/image_raw"; else echo "disabled"; fi)
  压缩图       -> /camera/color/image_raw/compressed (10 Hz)$(if [[ "$HAND_IMAGES" == "true" ]]; then echo " + /perception/hand_image/compressed"; fi)$(if [[ "$PERSON_ENABLE" == "true" ]]; then echo " + /perception/person_image/compressed"; fi)
  语音目标直接发 /goal_pose，无 MQTT/容器。
  Ctrl-C 统一停止并 DISARM。
EOF

set +e
wait "$NAV_RUN_PID"
RC=$?
set -e
echo "[host] 导航退出 rc=$RC"
exit "$RC"
