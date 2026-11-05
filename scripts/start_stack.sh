#!/usr/bin/env bash
# 一键启动：导航栈 + AI 混合容器 + MQTT 目标桥。
#
# 用法：
#   bash scripts/start_stack.sh                 # 最新 posegraph，干跑，不开电机
#   bash scripts/start_stack.sh --motor         # 允许 ARM/驱动电机（注意安全）
#   bash scripts/start_stack.sh --map /home/pi/maps/robot_map_20261105_054354
#   bash scripts/start_stack.sh --no-ai         # 只启动导航
#   bash scripts/start_stack.sh --no-mqtt       # AI 只做语音聊天，不桥接导航目标
#   bash scripts/start_stack.sh --viz none      # 不开 Foxglove
#
# Ctrl-C 会：停止 MQTT 桥、发送 DISARM/停止导航、删除 AI 容器。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP=""
MOTOR="false"
VIZ="foxglove"
WITH_AI="true"
WITH_MQTT="true"

usage() {
  sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="${2:-}"; shift 2 ;;
    --motor) MOTOR="true"; shift ;;
    --viz) VIZ="${2:-none}"; shift 2 ;;
    --no-ai) WITH_AI="false"; shift ;;
    --no-mqtt) WITH_MQTT="false"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown arg: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$MAP" ]]; then
  MAP="$(ls -t "$HOME"/maps/*.posegraph 2>/dev/null | head -1 || true)"
  MAP="${MAP%.posegraph}"
fi
[[ -n "$MAP" && -f "$MAP.posegraph" ]] || {
  echo "找不到 posegraph: ${MAP:-<empty>}"; exit 2; }

LOGDIR="${STACK_LOG_DIR:-$HOME/rosbags/stack_logs/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOGDIR"

warn() { echo "[stack] WARN: $*" >&2; }
die()  { echo "[stack] ERROR: $*" >&2; exit 1; }

echo "[stack] 外设检查"
for link in /dev/ydlidar /dev/stm32 /dev/mic-kws /dev/dap-vcp; do
  if [[ -e "$link" ]]; then
    printf '  %-16s -> %s\n' "$link" "$(readlink "$link")"
  else
    printf '  %-16s MISSING\n' "$link"
  fi
done
if arecord -l 2>/dev/null | grep -q "USB Audio"; then
  echo "  USB 音频       capture 可用"
else
  warn "没有 USB 音频 capture；语音输入不可用"
fi
[[ -e /dev/ydlidar ]] || die "/dev/ydlidar 缺失，导航无法启动"
if [[ "$MOTOR" == "true" && ! -e /dev/stm32 ]]; then
  die "--motor 需要 /dev/stm32（STM32 CDC）"
fi
if [[ "$WITH_AI" == "true" && ! -e /dev/mic-kws ]]; then
  warn "/dev/mic-kws 缺失：KWS 唤醒不可用，AI 仍可通过 /asr 文本或云端 ASR 测试"
fi
[[ -e /dev/video0 ]] || echo "  camera         未连接（视觉/寻人不可用）"
echo "  map            $MAP"

# [r]os2 技巧：避免 pgrep 匹配到自己的命令行（误报“已有 launch”）。
if pgrep -f "[r]os2 launch robot_base_bringup bringup.launch.py" >/dev/null; then
  echo "[stack] 已有导航 launch 在跑：" >&2
  pgrep -af "[r]os2 launch robot_base_bringup bringup.launch.py" >&2 || true
  die "先停掉已有导航（Ctrl-C 或 kill -INT 对应进程组）再启动"
fi
if [[ "$WITH_AI" == "true" ]]; then
  docker image inspect yahboom/ros-humble-ai:2026.11-mqtt >/dev/null 2>&1 \
    || die "AI 镜像 yahboom/ros-humble-ai:2026.11-mqtt 不存在"
  if [[ ! -f "$ROOT/.env" ]]; then
    warn ".env 不存在，AI 云端服务和 MQTT 凭据可能不可用"
  fi
  for k in NAV_MQTT_USERNAME NAV_MQTT_PASSWORD AI_MQTT_USERNAME AI_MQTT_PASSWORD; do
    grep -q "^${k}=" "$ROOT/.env" 2>/dev/null || {
      warn ".env 缺少 $k，禁用 MQTT 导航桥"; WITH_MQTT="false"; }
  done
  if ! grep -q '^AI_MQTT_CA_CERTS=' "$ROOT/.env" 2>/dev/null; then
    warn ".env 缺少 AI_MQTT_CA_CERTS"
  fi
fi

export PATH="$HOME/.local/bin:$PATH"
export MAMBA_ROOT_PREFIX="$HOME/micromamba"
set +u
eval "$(micromamba shell hook -s bash)"
micromamba activate ros_jazzy
source "$ROOT/robot_ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=1

if [[ "$WITH_MQTT" == "true" ]] && ! python3 -c "import paho.mqtt" 2>/dev/null; then
  warn "host 缺少 paho-mqtt，禁用 MQTT 导航桥"; WITH_MQTT="false"
fi

ING_PID=""
EGR_PID=""
NAV_PID=""

cleanup() {
  set +e
  trap - EXIT INT TERM
  echo
  echo "[stack] 正在停止..."
  [[ -n "$ING_PID" ]] && kill -INT "$ING_PID" 2>/dev/null
  [[ -n "$EGR_PID" ]] && kill -INT "$EGR_PID" 2>/dev/null
  if [[ -n "$NAV_PID" ]] && kill -0 "$NAV_PID" 2>/dev/null; then
    kill -INT -- -"$NAV_PID" 2>/dev/null
    sleep 3
    kill -TERM -- -"$NAV_PID" 2>/dev/null
    sleep 1
    kill -KILL -- -"$NAV_PID" 2>/dev/null
  fi
  if [[ "$WITH_AI" == "true" ]]; then
    docker rm -f ros_humble_ai_service >/dev/null 2>&1
  fi
  echo "[stack] 已停止（导航 Ctrl-C 时已发送 DISARM）"
}
trap cleanup EXIT INT TERM

if [[ "$WITH_AI" == "true" ]]; then
  echo "[stack] 启动 AI 混合容器（domain 2）..."
  ( cd "$ROOT" && AI_ROS_DOMAIN_ID=2 bash docker/run-ai-service.sh \
      >"$LOGDIR/ai_start.log" 2>&1 ) || warn "AI 容器启动失败"
  for _ in $(seq 1 40); do
    [[ "$(docker inspect -f '{{.State.Running}}' ros_humble_ai_service \
          2>/dev/null || true)" == "true" ]] && break
    sleep 0.5
  done
  if [[ "$(docker inspect -f '{{.State.Running}}' ros_humble_ai_service \
        2>/dev/null || true)" == "true" ]]; then
    echo "  AI 容器运行中（domain 2）"
  else
    warn "AI 容器没有正常运行，见 $LOGDIR/ai_start.log"
    WITH_AI="false"
  fi
fi

if [[ "$WITH_MQTT" == "true" && "$WITH_AI" == "true" ]]; then
  echo "[stack] 启动 MQTT 导航桥..."
  ROS_DOMAIN_ID=1 python3 "$ROOT/tools/mqtt_nav_ingress.py" \
    >"$LOGDIR/ingress.log" 2>&1 &
  ING_PID=$!
  # egress 必须跑在 AI 容器内（Humble）：主机 Jazzy 与容器 Humble
  # 在 domain 2 上互相发现不到，host egress 看不到 /nav_goal。
  docker exec -d ros_humble_ai_service bash -lc \
    'source /opt/ros/humble/setup.bash; source /root/yahboom_ws/install/setup.bash; exec python3 /root/yahboom_ws/src/largemodel/largemodel/mqtt_nav_egress.py > /tmp/mqtt_nav_egress.log 2>&1'
  for _ in $(seq 1 30); do
    grep -q "MQTT connected" "$LOGDIR/ingress.log" 2>/dev/null && \
    docker exec ros_humble_ai_service grep -q "MQTT connected" /tmp/mqtt_nav_egress.log \
      2>/dev/null && break
    sleep 0.5
  done
  grep -q "MQTT connected" "$LOGDIR/ingress.log" 2>/dev/null \
    && echo "  ingress domain 1 已连接" \
    || warn "ingress 未确认连接，见 $LOGDIR/ingress.log"
  docker exec ros_humble_ai_service grep -q "MQTT connected" \
      /tmp/mqtt_nav_egress.log 2>/dev/null \
    && echo "  egress  container domain 2 已连接" \
    || warn "容器内 egress 未确认连接，见容器 /tmp/mqtt_nav_egress.log"
fi

echo "[stack] 启动导航（domain 1, motor_enable=$MOTOR）..."
NAV_PIDFILE="$LOGDIR/nav.pid"
setsid bash -c 'echo $$ > "$1"; shift; exec "$@"' _ "$NAV_PIDFILE" \
  ros2 launch robot_base_bringup bringup.launch.py \
    slam:=false map:="$MAP" motor_enable:="$MOTOR" visualization:="$VIZ" \
  >"$LOGDIR/nav.log" 2>&1 &
for _ in $(seq 1 40); do
  [[ -s "$NAV_PIDFILE" ]] && break
  sleep 0.25
done
[[ -s "$NAV_PIDFILE" ]] || die "导航进程没有写 pid 文件"
NAV_PID="$(cat "$NAV_PIDFILE")"

for _ in $(seq 1 120); do
  grep -q "Managed nodes are active" "$LOGDIR/nav.log" 2>/dev/null && break
  kill -0 "$NAV_PID" 2>/dev/null || {
    tail -n 50 "$LOGDIR/nav.log" >&2; die "导航提前退出"; }
  sleep 0.5
done
grep -q "Managed nodes are active" "$LOGDIR/nav.log" 2>/dev/null \
  || { tail -n 80 "$LOGDIR/nav.log" >&2; die "导航 60 s 内未激活"; }

cat <<EOF
[stack] READY
  导航 domain 1 / AI domain 2 / 日志: $LOGDIR
  /dev/ydlidar -> $(readlink /dev/ydlidar 2>/dev/null || echo missing)
  /dev/stm32  -> $(readlink /dev/stm32 2>/dev/null || echo missing)
  /dev/mic-kws -> $(readlink /dev/mic-kws 2>/dev/null || echo missing)
  Ctrl-C 统一停止并 DISARM。
EOF

set +e
wait "$NAV_PID"
RC=$?
set -e
echo "[stack] 导航退出 rc=$RC"
exit "$RC"
