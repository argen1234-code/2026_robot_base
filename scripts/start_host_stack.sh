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
#
# 手部检测：/dev/video2 → MediaPipe 独立 venv → UDP → /perception/hand_pose
# Ctrl-C 会停止 AI、导航和手部检测（导航退出自动 DISARM）。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST_AI_WS="${HOST_AI_WS:-$HOME/host_ai_ws}"
MAP=""
MOTOR="false"
VIZ="none"
HAND="auto"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="${2:-}"; shift 2 ;;
    --motor) MOTOR="true"; shift ;;
    --viz) VIZ="${2:-none}"; shift 2 ;;
    --hand) HAND="true"; shift ;;
    --no-hand) HAND="false"; shift ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
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
for link in /dev/ydlidar /dev/stm32 /dev/mic-kws /dev/video0 /dev/video2; do
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
HAND_DEVICE="${HAND_DEVICE:-2}"
HAND_UDP="${HAND_UDP:-127.0.0.1:5005}"
HAND_ENABLE="false"
if [[ "$HAND" != "false" && -e "/dev/video$HAND_DEVICE" && -x "$HAND_VENV/bin/python" ]]; then
  HAND_ENABLE="true"
fi
if [[ "$HAND" == "true" && "$HAND_ENABLE" != "true" ]]; then
  die "--hand 需要 /dev/video$HAND_DEVICE 和 $HAND_VENV/bin/python"
fi
if [[ "$HAND_ENABLE" != "true" ]]; then
  warn "手部检测未启用（/dev/video$HAND_DEVICE 或 $HAND_VENV 缺失）"
fi

if pgrep -f "[r]os2 launch robot_base_bringup bringup.launch.py" >/dev/null; then
  die "已有导航 launch 在跑，先停掉再启动"
fi
if pgrep -f "[r]os2 launch largemodel largemodel_control.launch.py" >/dev/null; then
  die "已有 AI launch 在跑，先停掉再启动"
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
cat >"$LOGDIR/hand_det_cmd.sh" <<EOF
#!/usr/bin/env bash
set -e
ps -o pgid= -p \$\$ | tr -d ' ' > "$LOGDIR/hand_det.pgid"
exec env -u PYTHONPATH -u PYTHONHOME "$HAND_VENV/bin/python" \\
  "$ROOT/tools/hand_detector_host.py" --device "$HAND_DEVICE" --udp "$HAND_UDP"
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
chmod +x "$LOGDIR/nav_cmd.sh" "$LOGDIR/ai_cmd.sh"

export PATH="$HOME/.local/bin:$PATH"
export MAMBA_ROOT_PREFIX="$HOME/micromamba"

NAV_PG=""
AI_PG=""
HAND_DET_PG=""
HAND_BRIDGE_PG=""
cleanup() {
  set +e
  trap - EXIT INT TERM
  echo
  echo "[host] 正在停止..."
  [[ -n "$HAND_BRIDGE_PG" ]] && kill -INT -- -"$HAND_BRIDGE_PG" 2>/dev/null
  [[ -n "$HAND_DET_PG" ]] && kill -INT -- -"$HAND_DET_PG" 2>/dev/null
  [[ -n "$AI_PG" ]] && kill -INT -- -"$AI_PG" 2>/dev/null
  [[ -n "$NAV_PG" ]] && kill -INT -- -"$NAV_PG" 2>/dev/null
  sleep 3
  [[ -n "$HAND_BRIDGE_PG" ]] && kill -TERM -- -"$HAND_BRIDGE_PG" 2>/dev/null
  [[ -n "$HAND_DET_PG" ]] && kill -TERM -- -"$HAND_DET_PG" 2>/dev/null
  [[ -n "$AI_PG" ]] && kill -TERM -- -"$AI_PG" 2>/dev/null
  [[ -n "$NAV_PG" ]] && kill -TERM -- -"$NAV_PG" 2>/dev/null
  sleep 1
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
for _ in $(seq 1 60); do
  grep -q "Initialization completed" "$LOGDIR/ai.log" 2>/dev/null && break
  sleep 0.5
done
grep -q "Initialization completed" "$LOGDIR/ai.log" 2>/dev/null \
  || warn "AI 未确认初始化，见 $LOGDIR/ai.log"
grep -q "Registered 8 tools" "$LOGDIR/ai.log" 2>/dev/null \
  && echo "  AI tools=8" || warn "AI 工具数未确认"

if [[ "$HAND_ENABLE" == "true" ]]; then
  echo "[host] 启动手部检测（/dev/video$HAND_DEVICE → UDP $HAND_UDP → /perception/hand_pose）..."
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
    && echo "  手部相机 /dev/video$HAND_DEVICE opened" \
    || warn "手部检测未确认打开相机，见 $LOGDIR/hand_det.log"
  grep -q "hand bridge UDP" "$LOGDIR/hand_bridge.log" 2>/dev/null \
    && echo "  手部桥接 /perception/hand_pose" \
    || warn "手部桥接未确认启动，见 $LOGDIR/hand_bridge.log"
fi

cat <<EOF
[host] READY
  导航 ros_jazzy / AI ai_host / domain 1 / 日志: $LOGDIR
  /dev/mic-kws -> $(readlink /dev/mic-kws 2>/dev/null || echo missing)
  /dev/video0  -> $(if [ -e /dev/video0 ]; then readlink /dev/video0 2>/dev/null || echo present; else echo missing; fi)
  手部检测     -> $(if [[ "$HAND_ENABLE" == "true" ]]; then echo "/dev/video$HAND_DEVICE → /perception/hand_pose"; else echo "disabled"; fi)
  语音目标直接发 /goal_pose，无 MQTT/容器。
  Ctrl-C 统一停止并 DISARM。
EOF

set +e
wait "$NAV_RUN_PID"
RC=$?
set -e
echo "[host] 导航退出 rc=$RC"
exit "$RC"
