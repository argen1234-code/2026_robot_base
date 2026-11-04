#!/usr/bin/env bash
# 纯离线建图回放：不接雷达/MCU，不需要实车。
#
# 使用 bag 里的 /scan（原始）或 /scan_filtered，以及录制时的 /odom；
# 在独立 ROS_DOMAIN_ID 中启动：
#   robot_state_publisher（URDF 静态 TF）
#   scan_to_scan_filter_chain（可选，用当前 laser_filters.yaml）
#   odom_tf_replay（把 /odom 转成 odom->base_footprint TF）
#   slam_toolbox（用当前 slam_toolbox.yaml）
# 回放结束后保存地图，并可选录制 /map 供 map_stability.py 分析。
#
# 用法：
#   bash tools/replay_mapping.sh --bag /home/pi/rosbags/mapping_opt_xxx
#   SLAM_PARAMS=... bash tools/replay_mapping.sh --bag ... --out /tmp/run1
#   bash tools/replay_mapping.sh --bag ... --filtered --rate 2
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG=""
PARAMS="${SLAM_PARAMS:-$ROOT/robot_ros2_ws/src/robot_base_bringup/config/slam_toolbox.yaml}"
OUT="${OUT:-/tmp/replay_map}"
DOMAIN="${REPLAY_DOMAIN:-2}"
RATE="${RATE:-1}"
FILTER_CONFIG="$ROOT/robot_ros2_ws/src/robot_base_bringup/config/laser_filters.yaml"
USE_FILTERED=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --params) PARAMS="$2"; shift 2 ;;
    --filter-params) FILTER_CONFIG="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --filtered) USE_FILTERED=1; shift ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 1 ;;
  esac
done

[[ -n "$BAG" && -d "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 1; }
[[ -f "$PARAMS" ]] || { echo "params not found: $PARAMS" >&2; exit 1; }
[[ -f "$FILTER_CONFIG" ]] || { echo "filter params not found" >&2; exit 1; }
mkdir -p "$(dirname "$OUT")"

export PATH="$HOME/.local/bin:$PATH"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
if [ -x "$HOME/.local/bin/micromamba" ]; then
  eval "$("$HOME/.local/bin/micromamba" shell hook -s bash)"
  micromamba activate ros_jazzy
fi
# shellcheck disable=SC1091
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"

PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for p in "${PIDS[@]:-}"; do kill -INT -- -"$p" 2>/dev/null || true; done
  sleep 2
  for p in "${PIDS[@]:-}"; do kill -9 -- -"$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

echo "[replay] domain=$ROS_DOMAIN_ID bag=$BAG"
echo "[replay] params=$PARAMS out=$OUT"

# 1) URDF 静态 TF
RD="$(xacro "$ROOT/robot_ros2_ws/src/robot_base_description/urdf/robot_base.urdf.xacro")"
setsid ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$RD" -p use_sim_time:=true \
  > /tmp/replay_rsp.log 2>&1 &
PIDS+=($!)

# 2) /odom -> TF
setsid ros2 run robot_base_bringup odom_tf_replay.py --ros-args \
  -p odom_topic:=/odom -p use_sim_time:=true \
  > /tmp/replay_odom_tf.log 2>&1 &
PIDS+=($!)

# 3) 可选：原始 /scan -> 当前滤波链 -> /scan_filtered
if [[ "$USE_FILTERED" == 0 ]]; then
  setsid "$CONDA_PREFIX/lib/laser_filters/scan_to_scan_filter_chain" --ros-args \
    --params-file "$FILTER_CONFIG" \
    -r scan:=/scan -r scan_filtered:=/scan_filtered \
    > /tmp/replay_filter.log 2>&1 &
  PIDS+=($!)
fi

# 4) slam_toolbox
setsid ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$PARAMS" -p use_sim_time:=true \
  > /tmp/replay_slam.log 2>&1 &
PIDS+=($!)

sleep 3
# 6) 先让 bag 发布 /clock，再激活 SLAM；前 1~2 s 扫描会丢，属于正常预热。
setsid ros2 bag play "$BAG" --clock --rate "$RATE" \
  --topics /scan /odom $( [[ "$USE_FILTERED" == 1 ]] && echo /scan_filtered ) \
  > /tmp/replay_bag.log 2>&1 &
PIDS+=($!)
BAGPID=$!

sleep 2
ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1 || true
ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1 || true
wait "$BAGPID" 2>/dev/null || true
sleep 3

# 7) 保存最终地图
ros2 run nav2_map_server map_saver_cli -f "$OUT" \
  > /tmp/replay_mapsaver.log 2>&1 || true

# 生成 slam_toolbox 可复用的 posegraph，便于后续 localization/续建测试。
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: $OUT}" \
  > /tmp/replay_serialize.log 2>&1 || true

echo "[replay] map: ${OUT}.yaml/.pgm"
echo "[replay] posegraph: ${OUT}.posegraph/.data"
echo "[replay] 需要 /map 时序指标时，可在另一个终端单独 ros2 bag record --topics /map"
