#!/usr/bin/env bash
# =============================================================================
# 静态障碍 rosbag 调参：回放 bag -> 滤波 -> 独立代价地图
# =============================================================================
# 用法（先进入 ros_jazzy 环境）：
#   bash tools/run_costmap_tuner.sh --no-viewer       # 默认，无 GUI
#   bash tools/run_costmap_tuner.sh --rviz            # 尝试启动 RViz
#   bash tools/run_costmap_tuner.sh --bag /path --loop
#   bash tools/run_costmap_tuner.sh --legacy-tf       # 回放旧脉轮模型的 bag
#   bash tools/run_costmap_tuner.sh --no-viewer --duration 20
#
# 说明：
#   · 回放只取 bag 里的 /scan，TF 用固定的 odom->base_footprint，
#     避免 rf2o 静态漂移污染调参结果；
#   · 滤波输出改名为 /scan_filtered_tuned，不干扰已有 /scan_filtered；
#   · 代价地图节点名 costmap_tuner，发布 /costmap；
#   · ⚠️ 本机 RoboStack aarch64 的 rviz2 在 Ogre 初始化时必段错误，
#     默认不启动可视化；--rviz 仅用于换用可用 RViz 安装后的环境。
# =============================================================================
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${BAG:-/home/pi/rosbags/static_opt_segment_20261105_040149}"
COSTMAP_PARAMS="${COSTMAP_PARAMS:-$ROOT/robot_ros2_ws/src/robot_base_bringup/config/costmap_tune.yaml}"
FILTER_PARAMS="${FILTER_PARAMS:-$ROOT/robot_ros2_ws/src/robot_base_bringup/config/laser_filters.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$ROOT/robot_ros2_ws/src/robot_base_bringup/rviz/costmap_tune.rviz}"
LOOP=0
VIEW=none
DURATION=0
LEGACY_TF=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --params) COSTMAP_PARAMS="$2"; shift 2 ;;
    --filter-params) FILTER_PARAMS="$2"; shift 2 ;;
    --rviz-config) RVIZ_CONFIG="$2"; shift 2 ;;
    --loop) LOOP=1; shift ;;
    --rviz) VIEW=rviz; shift ;;
    --no-viewer|--no-rviz) VIEW=none; shift ;;
    --legacy-tf) LEGACY_TF=1; shift ;;
    --duration) DURATION="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 1 ;;
  esac
done

# 单实例保护：多个 costmap_tuner 同时发布 /costmap 会让稳定性指标失真。
if pgrep -f "nav2_costmap_2d.*costmap_tuner" >/dev/null 2>&1; then
  echo "ERROR: 已有 costmap_tuner 在运行，先停掉再回放（避免 /costmap 多发布者）" >&2
  exit 1
fi
[[ -d "$BAG" ]] || { echo "bag not found: $BAG" >&2; exit 1; }
[[ -f "$COSTMAP_PARAMS" ]] || { echo "params not found: $COSTMAP_PARAMS" >&2; exit 1; }
[[ -f "$FILTER_PARAMS" ]] || { echo "filter params not found: $FILTER_PARAMS" >&2; exit 1; }
if [[ "$BAG" == *static_tune_20261104_214418* && "$LEGACY_TF" != 1 ]]; then
  echo "[tuner] WARNING: 默认 bag 是旧脉轮模型录制的；" >&2
  echo "[tuner]          请换两轮模型的新 bag，或加 --legacy-tf 回放旧 bag。" >&2
fi

COLCON_TRACE=
source "$ROOT/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
C="$CONDA_PREFIX"
PIDS=()

cleanup() {
  trap - EXIT INT TERM
  for p in "${PIDS[@]:-}"; do kill -INT -- -"$p" 2>/dev/null || true; done
  sleep 2
  for p in "${PIDS[@]:-}"; do kill -9 -- -"$p" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

LOOP_ARG=()
[[ "$LOOP" == 1 ]] && LOOP_ARG=(--loop)

echo "[tuner] bag:    $BAG"
echo "[tuner] params: $COSTMAP_PARAMS"
echo "[tuner] filter: $FILTER_PARAMS"

setsid ros2 bag play "$BAG" --topics /scan "${LOOP_ARG[@]}" \
  > /tmp/costmap_tuner_bag.log 2>&1 &
PIDS+=($!)

# 静态 TF：odom -> base_footprint -> base_link -> laser_frame（与 URDF 一致）
# 两轮 CAD 模型（2026-11-04）：base_footprint 在两轮接地点中点。
# 旧 bag（static_tune_20261104_214418）是脉轮车录的，必须加 --legacy-tf。
if [[ "$LEGACY_TF" == 1 ]]; then
  BASE_X=0;      BASE_Z=0.069
  LASER_X=0.1716; LASER_Z=0.15
  LASER_YAW=-1.5707963267948966
else
  BASE_X=0.075;  BASE_Z=0.103
  LASER_X=0.1475; LASER_Z=-0.0168
  # 2026-11-05 修正：laser_frame yaw 从 -90° 改为 +90°，
  # raw -90° 才是车头；静态回放必须与 URDF 一致。
  LASER_YAW=+1.5707963267948966
fi
setsid "$C/lib/tf2_ros/static_transform_publisher" \
  --x 0 --y 0 --z 0 --frame-id odom --child-frame-id base_footprint \
  > /tmp/costmap_tuner_tf1.log 2>&1 &
PIDS+=($!)
setsid "$C/lib/tf2_ros/static_transform_publisher" \
  --x "$BASE_X" --y 0 --z "$BASE_Z" \
  --frame-id base_footprint --child-frame-id base_link \
  > /tmp/costmap_tuner_tf2.log 2>&1 &
PIDS+=($!)
setsid "$C/lib/tf2_ros/static_transform_publisher" \
  --x "$LASER_X" --y 0 --z "$LASER_Z" --yaw "$LASER_YAW" \
  --frame-id base_link --child-frame-id laser_frame \
  > /tmp/costmap_tuner_tf3.log 2>&1 &
PIDS+=($!)

sleep 2
setsid "$C/lib/laser_filters/scan_to_scan_filter_chain" --ros-args \
  --params-file "$FILTER_PARAMS" \
  -r scan:=/scan -r scan_filtered:=/scan_filtered_tuned \
  > /tmp/costmap_tuner_filter.log 2>&1 &
PIDS+=($!)
setsid "$C/lib/nav2_costmap_2d/nav2_costmap_2d" --ros-args \
  -r __node:=costmap_tuner --params-file "$COSTMAP_PARAMS" \
  > /tmp/costmap_tuner_costmap.log 2>&1 &
PIDS+=($!)

sleep 4
timeout 8 ros2 lifecycle set /costmap_tuner configure >/dev/null 2>&1 || true
timeout 8 ros2 lifecycle set /costmap_tuner activate >/dev/null 2>&1 || true

case "$VIEW" in
  rviz)
    DISPLAY="${DISPLAY:-:0}" setsid rviz2 -d "$RVIZ_CONFIG" \
      > /tmp/costmap_tuner_rviz.log 2>&1 &
    PIDS+=($!)
    echo "[tuner] RViz started on DISPLAY=${DISPLAY:-:0}"
    echo "[tuner] WARNING: RoboStack aarch64 rviz2 目前在 Ogre 初始化段错误；"
    echo "[tuner]          如果窗口没出来，用 --no-viewer 只跑数据面。"
    ;;
  none) ;;
esac

echo "[tuner] topics: /scan  /scan_filtered_tuned  /costmap"
echo "[tuner] Ctrl-C to stop"
if [[ "$DURATION" -gt 0 ]]; then
  sleep "$DURATION"
else
  wait "${PIDS[5]}" 2>/dev/null || true
fi
