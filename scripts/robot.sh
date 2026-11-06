#!/usr/bin/env bash
# 唯一机器人启动入口。
#
#   scripts/robot.sh start [options]
#   scripts/robot.sh stop [--force]
#   scripts/robot.sh status
#
# 默认全栈：导航重定位（最新地图）+ AI 语音 + 视觉 Astra+手部，
# 决策层全部启动，电机不 ARM，Foxglove 关闭。
# 独立视觉：start --task visual-approach|visual-follow [--motor] [--distance 0.05]
# 不加 --motor 预览；加 --motor 启动即运行。网页 http://<pi>:8879。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAMBA="${MAMBA_BIN:-$HOME/.local/bin/micromamba}"
STATE_DIR="${ROBOT_STATE_DIR:-$HOME/.robot}"
MANIFEST="$STATE_DIR/manifest"
LOG_BASE="${ROBOT_LOG_DIR:-$HOME/rosbags/robot}"

ASTRA_COLOR_DEVICE="${ASTRA_COLOR_DEVICE:-/dev/astra-color}"
HAND_VENV="${HAND_VENV:-$HOME/mediapipe_plain}"
HAND_DEVICE="${HAND_DEVICE:-/dev/hand-cam}"
HAND_UDP="${HAND_UDP:-127.0.0.1:5005}"
HAND_FPS="${HAND_FPS:-5}"
DEPTH_WS="${DEPTH_WS:-$HOME/orbbec_jazzy_ws}"
DEPTH_WIDTH="${DEPTH_WIDTH:-320}"
DEPTH_HEIGHT="${DEPTH_HEIGHT:-240}"
DEPTH_FPS="${DEPTH_FPS:-30}"
PERSON_RUNTIME_DIR="${PERSON_RUNTIME_DIR:-$HOME/vision_runtime}"
PERSON_MAX_FPS="${PERSON_MAX_FPS:-6}"
PERSON_CV_THREADS="${PERSON_CV_THREADS:-1}"

die() { echo "[robot] ERROR: $*" >&2; exit 1; }
log() { echo "[robot] $*"; }

usage() {
  sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
}

# ---------------------------------------------------------------- options
NAV="localize"
NAV_SET=0
MAP_ARG=""
MAP_SET=0
AI=1
VISION="both"
FOXGLOVE=0
MOTOR=0
DECISION=1
DECISION_SET=0
NAV_DEC=1
VOICE_DEC=1
VISION_DEC=1
TASK="none"
RESTART=0
LOG_DIR=""
NO_DEPTH=0
VISUAL_ONLY=0
VISUAL_DISTANCE=""
PERSON_DEPTH_MIN_M=0.30
PERSON_DEPTH_MAX_M=8.0

COMMAND="${1:-start}"
STOP_FORCE=0
case "$COMMAND" in
  start) shift ;;
  stop) shift; [[ "${1:-}" == "--force" ]] && STOP_FORCE=1 ;;
  status) shift ;;
  -h|--help|"") usage; exit 0 ;;
  *) die "unknown command: $COMMAND" ;;
esac

if [[ "$COMMAND" == "start" ]]; then
while [[ $# -gt 0 ]]; do
  case "$1" in
    --nav) NAV="${2:?}"; NAV_SET=1; shift 2 ;;
    --map) MAP_ARG="${2:?}"; MAP_SET=1; shift 2 ;;
    --map-start-pose) MAP_START_POSE="${2:?}"; shift 2 ;;
    --map-at-dock) MAP_AT_DOCK=1; shift ;;
    --ai) AI=1; shift ;;
    --no-ai) AI=0; shift ;;
    --vision) VISION="${2:?}"; shift 2 ;;
    --no-depth) NO_DEPTH=1; shift ;;
    --foxglove) FOXGLOVE=1; shift ;;
    --motor) MOTOR=1; shift ;;
    --decision) DECISION=1; DECISION_SET=1; shift ;;
    --no-decision) DECISION=0; DECISION_SET=1; shift ;;
    --no-nav-decision) NAV_DEC=0; shift ;;
    --no-voice-decision) VOICE_DEC=0; shift ;;
    --no-vision-decision) VISION_DEC=0; shift ;;
    --task) TASK="${2:?}"; shift 2 ;;
    --distance) VISUAL_DISTANCE="${2:?}"; shift 2 ;;
    --restart) RESTART=1; shift ;;
    --log-dir) LOG_DIR="${2:?}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1 (see --help)" ;;
  esac
done

case "$NAV" in mapping|localize|none) ;; *) die "--nav must be mapping|localize|none" ;; esac
case "$VISION" in astra|hand|both|none) ;; *) die "--vision must be astra|hand|both|none" ;; esac
case "$TASK" in none|follow|preview|visual-approach|visual-follow) ;; *) die "--task must be none|follow|preview|visual-approach|visual-follow" ;; esac
if [[ "$TASK" == visual-* ]]; then
  [[ "$NAV_SET" == 0 || "$NAV" == none ]] || die "visual tasks require --nav none"
  [[ "$MAP_SET" == 0 && "$FOXGLOVE" == 0 ]] || die "visual tasks do not use maps or Foxglove navigation"
  [[ "$NO_DEPTH" == 0 ]] || die "visual tasks require the depth camera (remove --no-depth)"
  VISUAL_ONLY=1; NAV=none; AI=0; VISION=astra; DECISION=0
  read -r PERSON_DEPTH_MIN_M PERSON_DEPTH_MAX_M VISUAL_MAX_LINEAR VISUAL_MAX_ANGULAR < <(python3 -c \
    'import json,sys; c=json.load(open(sys.argv[1])); print(c["depth_min_m"], c["depth_max_m"], c["max_linear_mps"], c["max_angular_rps"])' \
    "$ROOT/config/visual_follow.json")
  python3 - "$ROOT/robot_ros2_ws/src/robot_base_decision" "$ROOT/config/visual_follow.json" "$VISUAL_DISTANCE" <<'PY'
import json, sys
sys.path.insert(0, sys.argv[1])
from robot_base_decision.visual_follow import validate_config
cfg = json.load(open(sys.argv[2]))
if sys.argv[3]:
    cfg['target_distance_m'] = float(sys.argv[3])
validate_config(cfg)
PY
elif [[ -n "$VISUAL_DISTANCE" ]]; then
  die "--distance is only available with --task visual-approach|visual-follow"
fi
[[ "$FOXGLOVE" == "1" && "$NAV" == "none" ]] && die "--foxglove requires --nav"
[[ "$MOTOR" == "1" && "$NAV" == "none" && "$VISUAL_ONLY" == 0 ]] && die "--motor requires --nav or a visual task"
[[ "$NAV" == "mapping" && "$DECISION_SET" == "1" && "$DECISION" == "1" ]] && \
  die "mapping mode does not support --decision (Foxglove goals only)"
[[ "$NAV" == "mapping" && "$TASK" != "none" ]] && die "mapping mode does not support --task"
[[ "$TASK" != "none" && "$DECISION" == "0" && "$VISUAL_ONLY" == 0 ]] && die "--task requires --decision"

VISION_ASTRA=0; VISION_HAND=0
case "$VISION" in
  astra) VISION_ASTRA=1 ;;
  hand) VISION_HAND=1 ;;
  both) VISION_ASTRA=1; VISION_HAND=1 ;;
esac

# ------------------------------------------------------------ map/mode
MAP_START_POSE="${MAP_START_POSE:-[0.0, 0.0, 0.0]}"
MAP_AT_DOCK="${MAP_AT_DOCK:-0}"
if [[ "$NAV" != "none" && "$MAP_SET" == "0" ]]; then
  if [[ "$NAV" == "mapping" ]]; then MAP_ARG="new"; else MAP_ARG="latest"; fi
fi
if [[ "$NAV" == "none" && "$MAP_SET" == "1" ]]; then
  die "--map requires --nav"
fi
if [[ "$NAV" == "localize" && "$MAP_ARG" == "new" ]]; then
  die "--map new is only valid with --nav mapping"
fi

SLAM="false"
BRINGUP_MAP=""
case "$NAV" in
  mapping)
    SLAM="true"
    case "$MAP_ARG" in
      new) BRINGUP_MAP="none" ;;
      latest) BRINGUP_MAP="latest" ;;
      *) BRINGUP_MAP="$MAP_ARG" ;;
    esac
    ;;
  localize)
    SLAM="false"
    case "$MAP_ARG" in
      latest) BRINGUP_MAP="latest" ;;
      *) BRINGUP_MAP="$MAP_ARG" ;;
    esac
    ;;
esac

if [[ "$NAV" == "localize" && "$BRINGUP_MAP" == "latest" ]]; then
  ls "$HOME"/maps/*.posegraph >/dev/null 2>&1 || \
    die "localize/latest requires a posegraph in ~/maps"
fi
if [[ -n "$BRINGUP_MAP" && "$BRINGUP_MAP" != "none" && "$BRINGUP_MAP" != "latest" ]]; then
  PREFIX="${BRINGUP_MAP%.posegraph}"
  [[ -f "$PREFIX.posegraph" ]] || die "posegraph not found: $PREFIX.posegraph"
  BRINGUP_MAP="$PREFIX"
fi

# ------------------------------------------------------------ calibration
CONFIG="$ROOT/config/person_approach.json"
CALIBRATED="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("calibration_confirmed", False))' "$CONFIG" 2>/dev/null || echo False)"
DECISION_EXECUTE=0
if [[ "$MOTOR" == "1" && "$CALIBRATED" == "True" && "$NAV" == "localize" \
      && "$VISION_ASTRA" == "1" && "$DECISION" == "1" && "$TASK" != "preview" ]]; then
  DECISION_EXECUTE=1
fi
if [[ "$TASK" == "follow" ]]; then
  [[ "$DECISION_EXECUTE" == "1" ]] || \
    die "--task follow requires --motor, calibration_confirmed=true, --nav localize and --vision astra"
fi
if [[ "$TASK" == "preview" ]]; then
  [[ "$NAV" == "localize" && "$VISION_ASTRA" == "1" ]] || \
    die "--task preview requires --nav localize and --vision astra"
  DECISION_EXECUTE=0
fi

# --------------------------------------------------------- decision layers
START_NAV_DEC=0; START_VOICE_DEC=0; START_VISION_DEC=0
if [[ "$NAV" == "mapping" ]]; then DECISION=0; fi
if [[ "$DECISION" == "1" ]]; then
  [[ "$NAV" == "localize" && "$NAV_DEC" == "1" ]] && START_NAV_DEC=1
  [[ "$AI" == "1" && "$VOICE_DEC" == "1" ]] && START_VOICE_DEC=1
  [[ "$NAV" == "localize" && "$VISION_ASTRA" == "1" && "$VISION_DEC" == "1" ]] && \
    START_VISION_DEC=1
  if [[ "$START_NAV_DEC$START_VOICE_DEC$START_VISION_DEC" == "000" ]]; then
    DECISION=0
  fi
fi

# --------------------------------------------------------------- preflight
[[ "$NAV" != "none" && -e /dev/ydlidar ]] || [[ "$NAV" == "none" ]] || \
  die "/dev/ydlidar missing"
[[ "$MOTOR" == "1" && -e /dev/stm32 ]] || [[ "$MOTOR" == "0" ]] || \
  die "/dev/stm32 missing (required by --motor)"
[[ -f "$ROOT/robot_ros2_ws/install/setup.bash" ]] || die "robot_ros2_ws not built"
[[ -f "$HOME/host_ai_ws/install/setup.bash" ]] || [[ "$AI" == "0" ]] || \
  die "host_ai_ws not built (required by --ai)"
if [[ "$VISION_ASTRA" == "1" ]]; then
  [[ -e "$ASTRA_COLOR_DEVICE" ]] || die "/dev/astra-color missing (vision astra)"
fi
if [[ "$VISUAL_ONLY" == 1 ]]; then
  [[ -f "$DEPTH_WS/install/setup.bash" ]] || die "visual task requires Orbbec depth workspace"
fi
if [[ "$AI" == "1" && "$VISION_ASTRA" == "0" ]]; then
  [[ -e "$ASTRA_COLOR_DEVICE" ]] || log "WARN: /dev/astra-color missing; AI camera tools degraded"
fi
if [[ "$VISION_HAND" == "1" ]]; then
  [[ -e "$HAND_DEVICE" ]] || die "/dev/hand-cam missing (vision hand)"
  [[ -x "$HAND_VENV/bin/python" ]] || die "mediapipe_plain venv missing"
fi
[[ -f "$ROOT/.env" ]] || [[ "$AI" == "0" ]] || log "WARN: .env missing; cloud ASR/TTS/LLM may fail"
fi  # COMMAND == start

# ------------------------------------------------------------------ stop
manifest_alive() {
  [[ -f "$MANIFEST" ]] || return 1
  local name pgid pid
  while read -r name pgid pid; do
    [[ -n "${pgid:-}" ]] || continue
    if kill -0 -- "-$pgid" 2>/dev/null; then return 0; fi
  done <"$MANIFEST"
  return 1
}

stop_stack() {
  local force="${1:-0}"
  if [[ ! -f "$MANIFEST" ]]; then
    # 过渡期：停止旧入口残留的栈。
    pkill -INT -f '[s]cripts/start_host_stack\.sh' 2>/dev/null || true
    pkill -INT -f '[t]ools/person_approach_node\.py' 2>/dev/null || true
    log "没有由 robot.sh 启动的栈"
    return 0
  fi
  local -a items=()
  mapfile -t items < <(tac "$MANIFEST")
  log "停止栈（${#items[@]} 个组件）..."
  local item name pgid pid
  for item in "${items[@]}"; do
    read -r name pgid pid <<<"$item"
    [[ -n "${pgid:-}" ]] || continue
    kill -INT -- "-$pgid" 2>/dev/null || true
  done
  local wait_s=15
  [[ "$force" == "1" ]] && wait_s=3
  for _ in $(seq 1 "$wait_s"); do manifest_alive || break; sleep 1; done
  if manifest_alive; then
    log "仍有组件，发送 SIGTERM..."
    for item in "${items[@]}"; do
      read -r name pgid pid <<<"$item"
      [[ -n "${pgid:-}" ]] && kill -TERM -- "-$pgid" 2>/dev/null || true
    done
    sleep 3
  fi
  if manifest_alive; then
    log "仍有组件，发送 SIGKILL..."
    for item in "${items[@]}"; do
      read -r name pgid pid <<<"$item"
      [[ -n "${pgid:-}" ]] && kill -KILL -- "-$pgid" 2>/dev/null || true
    done
    sleep 1
  fi
  rm -f "$MANIFEST"
  log "栈已停止"
}

status_stack() {
  if [[ ! -f "$MANIFEST" ]]; then
    log "没有运行中的栈"
    return 0
  fi
  printf '%-16s %-8s %-8s %s\n' COMPONENT PGID PID STATE
  local name pgid pid
  while read -r name pgid pid; do
    local state="stopped"
    kill -0 -- "-$pgid" 2>/dev/null && state="running"
    printf '%-16s %-8s %-8s %s\n' "$name" "$pgid" "$pid" "$state"
  done <"$MANIFEST"
  ss -ltn 2>/dev/null | grep -E ':(8765|8878|8879)\b' || true
}

case "$COMMAND" in
  stop) stop_stack "$STOP_FORCE"; exit 0 ;;
  status) status_stack; exit 0 ;;
esac

# ----------------------------------------------------------------- start
if [[ "$RESTART" == "1" ]]; then stop_stack 0; fi
if manifest_alive; then
  die "已有栈在运行；使用 --restart 或先 scripts/robot.sh stop"
fi
if pgrep -f '[s]cripts/start_host_stack\.sh' >/dev/null; then
  die "检测到旧栈在运行；先执行 scripts/robot.sh stop"
fi

if [[ -z "$LOG_DIR" ]]; then
  LOG_DIR="$LOG_BASE/$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$LOG_DIR" "$STATE_DIR"
: >"$MANIFEST"
ln -sfn "$LOG_DIR" "$LOG_BASE/latest"
log "日志目录: $LOG_DIR"

launch_cmd() {
  local name="$1"; shift
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 </dev/null &
  local pid=$!
  sleep 0.2
  local pgid
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
  [[ -n "$pgid" ]] || pgid="$pid"
  printf '%s %s %s\n' "$name" "$pgid" "$pid" >>"$MANIFEST"
  log "启动 $name (pid=$pid pgid=$pgid)"
}

wait_log() {
  local name="$1" pattern="$2" timeout="${3:-30}"
  for _ in $(seq 1 "$timeout"); do
    grep -q "$pattern" "$LOG_DIR/$name.log" 2>/dev/null && return 0
    manifest_alive || return 1
    sleep 1
  done
  return 1
}

# ---- navigation ----
if [[ "$NAV" != "none" ]]; then
  VIZ="none"; [[ "$FOXGLOVE" == "1" ]] && VIZ="foxglove"
  MOTOR_BOOL="false"; [[ "$MOTOR" == "1" ]] && MOTOR_BOOL="true"
  cat >"$LOG_DIR/nav.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=1
exec ros2 launch robot_base_bringup bringup.launch.py \\
  "slam:=$SLAM" "map:=$BRINGUP_MAP" "motor_enable:=$MOTOR_BOOL" \\
  "visualization:=$VIZ" "map_start_pose:=$MAP_START_POSE" "map_start_at_dock:=$MAP_AT_DOCK"
EOF
  launch_cmd nav "$MAMBA" run -n ros_jazzy bash "$LOG_DIR/nav.sh"
  wait_log nav 'Managed nodes are active' 150 || {
    tail -n 40 "$LOG_DIR/nav.log" >&2 || true
    stop_stack 0
    die "导航启动失败"
  }
  log "导航 READY"
fi

# ---- Astra color owner ----
AI_CAMERA_SOURCE="uvc"
if [[ "$VISION_ASTRA" == "1" ]]; then
  AI_CAMERA_SOURCE="ros_compressed"
  cat >"$LOG_DIR/color.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=1
exec ros2 run robot_base_bringup astra_color_publisher.py --ros-args \\
  -p device:=$ASTRA_COLOR_DEVICE -p width:=640 -p height:=480 \\
  -p fps:=10.0 -p quality:=70
EOF
  launch_cmd color "$MAMBA" run -n ros_jazzy bash "$LOG_DIR/color.sh"
  wait_log color 'Astra color ready' 20 || {
    tail -n 30 "$LOG_DIR/color.log" >&2 || true
    stop_stack 0
    die "Astra 彩色发布器启动失败（检查 ${ASTRA_COLOR_DEVICE} 是否被占用）"
  }
  log "Astra 彩色 READY"
fi

# ---- AI voice ----
AI_NAV_TOPIC="/goal_pose"
[[ "$NAV" != "localize" ]] && AI_NAV_TOPIC="/ai_nav_disabled"
AI_DECISION="false"
[[ "$START_VOICE_DEC" == "1" ]] && AI_DECISION="true"
if [[ "$AI" == "1" ]]; then
  cat >"$LOG_DIR/ai.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$HOME/host_ai_ws/install/setup.bash"
set -a
source "$ROOT/.env"
set +a
export ROS_DOMAIN_ID=1
exec ros2 launch largemodel largemodel_control.launch.py \\
  "camera_source:=$AI_CAMERA_SOURCE" "decision_mode:=$AI_DECISION" \\
  "nav_goal_topic:=$AI_NAV_TOPIC"
EOF
  launch_cmd ai "$MAMBA" run -n ai_host bash "$LOG_DIR/ai.sh"
  wait_log ai 'LargeModelService node Initialization completed' 90 || \
    log "WARN: AI 未确认初始化，见 $LOG_DIR/ai.log"
  log "AI READY"
fi

# ---- depth + person (Astra vision) ----
if [[ "$VISION_ASTRA" == "1" ]]; then
  if [[ "$NO_DEPTH" == "0" ]]; then
  cat >"$LOG_DIR/depth.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$ROOT/robot_ros2_ws/install/setup.bash"
source "$DEPTH_WS/install/setup.bash"
export ROS_DOMAIN_ID=1
exec ros2 launch orbbec_camera astra.launch.py \\
  enable_color:=false enable_depth:=true enable_point_cloud:=false \\
  depth_format:=Y11 depth_fps:=$DEPTH_FPS depth_width:=$DEPTH_WIDTH depth_height:=$DEPTH_HEIGHT
EOF
  launch_cmd depth "$MAMBA" run -n ros_jazzy bash "$LOG_DIR/depth.sh"
  if ! wait_log depth 'Start device' 45; then
    if [[ "$VISUAL_ONLY" == 1 ]]; then
      stop_stack 0
      die "纯视觉模式深度启动失败，见 $LOG_DIR/depth.log"
    fi
    log "WARN: 深度未确认启动，见 $LOG_DIR/depth.log"
  fi
  log "深度 READY"
  fi

  PERSON_MODEL="${PERSON_MODEL:-$HOME/models/vision_v2/yolov8n_320_ncnn_model}"
  PERSON_BACKEND="${PERSON_BACKEND:-ncnn}"
  if [[ ! -f "$PERSON_MODEL/model.ncnn.param" ]]; then
    if [[ -f "$HOME/models/yolov8n_320.onnx" ]]; then
      PERSON_MODEL="$HOME/models/yolov8n_320.onnx"
    else
      PERSON_MODEL="$HOME/models/yolov8n.onnx"
    fi
    PERSON_BACKEND="opencv"
  fi
  PERSON_INPUT_SIZE="${PERSON_INPUT_SIZE:-}"
  if [[ -z "$PERSON_INPUT_SIZE" ]]; then
    if [[ "$PERSON_BACKEND" == "ncnn" ]]; then
      PERSON_INPUT_SIZE=320
    elif [[ "$PERSON_MODEL" == *yolov8n_320.onnx ]]; then
      PERSON_INPUT_SIZE=320
    else
      PERSON_INPUT_SIZE=640
    fi
  fi
  cat >"$LOG_DIR/person.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$HOME/host_ai_ws/install/setup.bash"
export ROS_DOMAIN_ID=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE
exec python3 "$ROOT/tools/person_detector_node.py" \\
  --model "$PERSON_MODEL" --backend "$PERSON_BACKEND" \\
  --input-size "$PERSON_INPUT_SIZE" --input-height "$PERSON_INPUT_SIZE" \\
  --runtime-dir "$PERSON_RUNTIME_DIR" --max-fps "$PERSON_MAX_FPS" --cv-threads "$PERSON_CV_THREADS" \\
  --depth-min-m "$PERSON_DEPTH_MIN_M" --depth-max-m "$PERSON_DEPTH_MAX_M"
EOF
  launch_cmd person "$MAMBA" run -n ai_host bash "$LOG_DIR/person.sh"
  if ! wait_log person 'person detector ready' 45; then
    if [[ "$VISUAL_ONLY" == 1 ]]; then
      stop_stack 0
      die "纯视觉模式人像检测启动失败，见 $LOG_DIR/person.log"
    fi
    log "WARN: 人像检测未确认启动，见 $LOG_DIR/person.log"
  fi
  log "人像检测 READY"
fi

# ---- hand camera ----
if [[ "$VISION_HAND" == "1" ]]; then
  HAND_IMAGE_ARGS=()
  if [[ "$FOXGLOVE" == "1" ]]; then
    HAND_IMAGE_ARGS=(--image-udp 127.0.0.1:5006 --image-fps 3 --image-width 320 --jpeg-quality 60)
  fi
  launch_cmd hand_det nice -n 5 env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH \
    "$HAND_VENV/bin/python" "$ROOT/tools/hand_detector_host.py" \
    --device "$HAND_DEVICE" --udp "$HAND_UDP" --fps "$HAND_FPS" "${HAND_IMAGE_ARGS[@]}"
  cat >"$LOG_DIR/hand_bridge.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$HOME/host_ai_ws/install/setup.bash"
export ROS_DOMAIN_ID=1
exec python3 "$ROOT/tools/hand_pose_bridge.py"
EOF
  launch_cmd hand_bridge "$MAMBA" run -n ai_host bash "$LOG_DIR/hand_bridge.sh"
  wait_log hand_det 'opened' 20 || log "WARN: 手部相机未确认打开，见 $LOG_DIR/hand_det.log"
  log "手部检测 READY"
fi

# ---- standalone visual path: no laser / SLAM / Nav2 / decision coordinator ----
if [[ "$VISUAL_ONLY" == 1 ]]; then
  if [[ "$MOTOR" == 1 ]]; then
    cat >"$LOG_DIR/chassis.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=1
exec ros2 run robot_base_bringup chassis_bridge.py --ros-args \\
  --params-file "$ROOT/robot_ros2_ws/src/robot_base_bringup/config/imu_calibration.yaml" \\
  -p enabled:=true -p max_linear:=$VISUAL_MAX_LINEAR -p max_angular:=$VISUAL_MAX_ANGULAR \\
  -r cmd_vel_smoothed:=/visual_follow/unused_nav_velocity \\
  -r /person_nav/status:=/visual_follow/unused_nav_status \\
  -r /person_nav/centering_target:=/visual_follow/unused_nav_centering
EOF
    launch_cmd chassis "$MAMBA" run -n ros_jazzy bash "$LOG_DIR/chassis.sh"
  fi
  cat >"$LOG_DIR/visual.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=1
exec ros2 run robot_base_decision visual_follow \\
  --config "$ROOT/config/visual_follow.json" --auto \\
  $([[ "$TASK" == visual-follow ]] && echo --follow) \\
  $([[ "$MOTOR" == 1 ]] && echo --execute) \\
  ${VISUAL_DISTANCE:+--distance $VISUAL_DISTANCE}
EOF
  launch_cmd visual "$MAMBA" run -n ros_jazzy bash "$LOG_DIR/visual.sh"
  if ! wait_log visual 'visual follow ready' 30; then
    stop_stack 0
    die "纯视觉控制启动失败，见 $LOG_DIR/visual.log"
  fi
  log "纯视觉 READY: $TASK，镜头停车深度=${VISUAL_DISTANCE:-config/visual_follow.json} m"
fi

# ---- decision layers ----
if [[ "$DECISION" == "1" ]]; then
  EXEC_ARG="false"; [[ "$DECISION_EXECUTE" == "1" ]] && EXEC_ARG="true"
  PREVIEW_ARG="false"; [[ "$TASK" == "preview" || "$DECISION_EXECUTE" == "0" ]] && PREVIEW_ARG="true"
  cat >"$LOG_DIR/decision.sh" <<EOF
#!/usr/bin/env bash
set -e
source "\$CONDA_PREFIX/setup.bash"
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=1
export PERSON_APPROACH_ROOT="$ROOT"
exec ros2 launch robot_base_decision decision.launch.py \\
  "start_nav_decision:=$([[ $START_NAV_DEC == 1 ]] && echo true || echo false)" \\
  "start_voice_decision:=$([[ $START_VOICE_DEC == 1 ]] && echo true || echo false)" \\
  "start_vision_decision:=$([[ $START_VISION_DEC == 1 ]] && echo true || echo false)" \\
  "initial_task:=$TASK" "execute:=$EXEC_ARG" "preview:=$PREVIEW_ARG" \\
  "config:=$CONFIG"
EOF
  launch_cmd decision "$MAMBA" run -n ros_jazzy bash "$LOG_DIR/decision.sh"
  wait_log decision 'task coordinator ready' 30 || \
    log "WARN: 决策层未确认启动，见 $LOG_DIR/decision.log"
  log "决策层 READY"
fi

# --------------------------------------------------------------- summary
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
cat <<EOF
[robot] READY
  导航       ${NAV}${MAP_ARG:+ (map=$MAP_ARG)}${MOTOR:+ motor=$MOTOR}
  AI 语音    $([[ $AI == 1 ]] && echo on || echo off)
  视觉       ${VISION}（Astra=$([[ $VISION_ASTRA == 1 ]] && echo on || echo off)，手部=$([[ $VISION_HAND == 1 ]] && echo on || echo off)）
  决策层     $([[ $DECISION == 1 ]] && echo "nav=$START_NAV_DEC voice=$START_VOICE_DEC vision=$START_VISION_DEC execute=$DECISION_EXECUTE" || echo off)
  Foxglove   $([[ $FOXGLOVE == 1 ]] && echo "ws://${IP:-<pi>}:8765" || echo off)
  找人网页   $(if [[ $VISUAL_ONLY == 1 ]]; then echo "http://${IP:-<pi>}:8879"; elif [[ $START_VISION_DEC == 1 ]]; then echo "http://${IP:-<pi>}:8878"; else echo off; fi)
  日志       $LOG_DIR
  停止       scripts/robot.sh stop
EOF
