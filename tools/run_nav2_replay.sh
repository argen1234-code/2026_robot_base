#!/usr/bin/env bash
# 离线 Nav2 回放：同一个 bag、同一组目标，只跑 Nav2（不含 SLAM/EKF/底盘）。
#
# 用法：
#   bash tools/run_nav2_replay.sh --params <yaml> --out <dir> [--rate 1.0]
#
# 输入话题：/scan_filtered /odom /tf /tf_static /map /goal_pose
#   · /scan_filtered /odom /tf /goal_pose 由 relay 重打当前时间戳后转发；
#   · /tf_static /map 是 transient_local，直接回放。
# 输出：<dir>/metrics.json、nav2.log、relay.log、monitor.log、bag_play.log
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="/home/pi/rosbags/localization_20261105_055426"
PARAMS="$ROOT/robot_ros2_ws/src/robot_base_bringup/config/nav2_params_prior.yaml"
OUT=""
RATE="1.0"
DOMAIN="2"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --params) PARAMS="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

[[ -f "$PARAMS" ]] || { echo "params not found: $PARAMS" >&2; exit 2; }
[[ -d "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 2; }
if [[ -z "$OUT" ]]; then
  OUT="$ROOT/../rosbags/nav2_replay/$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUT"
rm -f "$OUT/metrics.json"
cp "$PARAMS" "$OUT/params.yaml"
cp "$BAG/metadata.yaml" "$OUT/bag_metadata.yaml" 2>/dev/null || true

export PATH="$HOME/.local/bin:$PATH"
export MAMBA_ROOT_PREFIX="$HOME/micromamba"
# micromamba 的 activate.d 与 ROS setup.bash 都会引用未定义变量
# （CONDA_BUILD / COLCON_TRACE），与本脚本的 set -u 冲突；
# 整个环境加载阶段临时关闭 nounset。
set +u
eval "$(micromamba shell hook -s bash)"
micromamba activate ros_jazzy
source "$ROOT/robot_ros2_ws/install/setup.bash"
set -u
# 强制使用独立 DDS 域，避免与主机 live 导航（ROS_DOMAIN_ID=1）串台。
export ROS_DOMAIN_ID="$DOMAIN"

if pgrep -af "ros2 launch robot_base_bringup nav2_navigation.launch.py" \
     | grep -v "bash -c" >/dev/null; then
  echo "已有 Nav2 回放 launch 在跑，先停掉再回放" >&2
  pgrep -af "ros2 launch robot_base_bringup nav2_navigation.launch.py" >&2 || true
  exit 3
fi

RELAY_PID=""
MON_PID=""
NAV_PID=""
BAG_PID=""

stop_all() {
  set +e
  if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null
  fi
  if [[ -n "$NAV_PID" ]] && kill -0 "$NAV_PID" 2>/dev/null; then
    kill -INT -- "-$NAV_PID" 2>/dev/null
  fi
  for pid in "$RELAY_PID" "$MON_PID"; do
    [[ -n "$pid" ]] && kill -INT "$pid" 2>/dev/null
  done
  sleep 2
  for pid in "$RELAY_PID" "$MON_PID"; do
    [[ -n "$pid" ]] && kill -TERM "$pid" 2>/dev/null
  done
  if [[ -n "$NAV_PID" ]] && kill -0 "$NAV_PID" 2>/dev/null; then
    kill -TERM -- "-$NAV_PID" 2>/dev/null
    sleep 1
    kill -KILL -- "-$NAV_PID" 2>/dev/null
  fi
  wait 2>/dev/null
  set -e
}
trap stop_all EXIT INT TERM

echo "[replay] out=$OUT domain=$ROS_DOMAIN_ID rate=$RATE"
echo "[replay] params=$PARAMS"

python3 "$ROOT/tools/bag_replay_relay.py" >"$OUT/relay.log" 2>&1 &
RELAY_PID=$!
sleep 1
python3 "$ROOT/tools/nav2_replay_monitor.py" --out "$OUT/metrics.json" \
  >"$OUT/monitor.log" 2>&1 &
MON_PID=$!

setsid ros2 launch robot_base_bringup nav2_navigation.launch.py \
  params_file:="$PARAMS" use_sim_time:=false >"$OUT/nav2.log" 2>&1 &
NAV_PID=$!

# ⚠️ 必须先开始回放：Nav2 的 controller/局部代价地图激活时要等
#    bag 里的 odom→base_footprint TF；先等 "active" 再回放会死锁。
sleep 1
set +e
ros2 bag play "$BAG" --rate "$RATE" --disable-keyboard-controls \
  --topics /scan_filtered /odom /tf /tf_static /map /goal_pose \
  --remap /scan_filtered:=/bag/scan_filtered \
          /odom:=/bag/odom \
          /tf:=/bag/tf \
          /goal_pose:=/bag/goal_pose \
  >"$OUT/bag_play.log" 2>&1 &
BAG_PID=$!
set -e

echo "[replay] waiting for Nav2 lifecycle active (bag is playing) ..."
# 只看 launch 日志里的激活行，不调用 ros2 CLI：
#   ROS 2 daemon 一旦无响应，`ros2 lifecycle get` 会无限挂起（R5 实测踩过），
#   而 lifecycle_manager 激活成功时一定会打印 "Managed nodes are active"。
ACTIVE=0
for _ in $(seq 1 160); do
  if grep -q "Managed nodes are active" "$OUT/nav2.log" 2>/dev/null; then
    ACTIVE=1
    break
  fi
  if ! kill -0 "$NAV_PID" 2>/dev/null; then
    echo "[replay] Nav2 launch exited early" >&2
    tail -n 60 "$OUT/nav2.log" >&2 || true
    exit 4
  fi
  sleep 0.5
done
if [[ "$ACTIVE" != 1 ]]; then
  echo "[replay] Nav2 did not become active in 80 s" >&2
  tail -n 80 "$OUT/nav2.log" >&2 || true
  exit 5
fi
echo "[replay] Nav2 active"

set +e
wait "$BAG_PID"
BAG_RC=$?
set -e
BAG_PID=""
echo "[replay] bag playback finished rc=$BAG_RC"

sleep 2
stop_all
trap - EXIT INT TERM

echo "[replay] done -> $OUT/metrics.json"
python3 - "$OUT/metrics.json" <<'PY'
import json, sys
p=sys.argv[1]
try:
    d=json.load(open(p))
except Exception as e:
    print(f'no metrics: {e}'); sys.exit(0)
print(json.dumps({
    'run': d.get('run'),
    'rates': d.get('rates'),
    'cmd_vel': d.get('cmd_vel'),
    'odom_path_m': d.get('odom_path_m'),
    'bt_failure_counts': d.get('bt_failure_counts'),
    'bt_success_counts': d.get('bt_success_counts'),
    'goal_durations': d.get('goal_durations'),
    'goal_pose_safe_count': d.get('goal_pose_safe_count'),
    'cpu': {k:v for k,v in d.get('cpu',{}).items() if k!='system'},
}, indent=2, ensure_ascii=False))
PY
