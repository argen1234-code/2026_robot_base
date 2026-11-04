#!/usr/bin/env bash
# 保存 slam_toolbox 可复用的 posegraph（建图/定位/续建），
# 同时可选保存一份 2D PGM/YAML 供查看。
#
# 用法（建图栈运行中）：
#   bash tools/save_slam_map.sh /home/pi/maps/robot_map
# 生成：
#   /home/pi/maps/robot_map.posegraph
#   /home/pi/maps/robot_map.data
#   /home/pi/maps/robot_map.yaml/.pgm（2D，供查看/其他工具）
set -eo pipefail

NAME="${1:-/home/pi/maps/robot_map_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$NAME")"

export PATH="$HOME/.local/bin:$PATH"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
if [ -x "$HOME/.local/bin/micromamba" ]; then
  eval "$("$HOME/.local/bin/micromamba" shell hook -s bash)"
  micromamba activate ros_jazzy
fi
# shellcheck disable=SC1091
source "$HOME/2026_robot_base/robot_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

echo "[save] posegraph: $NAME"
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: $NAME}"

echo "[save] 2D map: $NAME"
ros2 run nav2_map_server map_saver_cli -f "$NAME"

echo "[save] 生成:"
ls -l "$NAME".posegraph "$NAME".data "$NAME".yaml "$NAME".pgm 2>/dev/null || true
