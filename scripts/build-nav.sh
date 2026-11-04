#!/usr/bin/env bash
# 在 RoboStack Jazzy 环境中编译 YDLidar SDK + robot_ros2_ws。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="${ENV_NAME:-ros_jazzy}"
MAMBA_BIN="$HOME/.local/bin/micromamba"
MAMBA_ROOT="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
SDK_DIR="$ROOT/third_party/ydlidar_sdk"

export MAMBA_ROOT_PREFIX="$MAMBA_ROOT"

if [ ! -d "$SDK_DIR" ]; then
  echo "Cloning YDLidar-SDK ..."
  git clone --depth 1 \
    https://ghfast.top/https://github.com/YDLidar/YDLidar-SDK.git "$SDK_DIR"
fi

echo "=== build YDLidar SDK (system cmake; conda cmake 4.x rejects old policies) ==="
"$MAMBA_BIN" run -n "$ENV_NAME" bash -c "
  set -e
  export CC=/usr/bin/gcc CXX=/usr/bin/g++
  cd '$SDK_DIR'
  rm -rf build && mkdir build && cd build
  /usr/bin/cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=\"\$CONDA_PREFIX\"
  make -j2
  make install
"

echo "=== build robot_ros2_ws (4GB address-space cap) ==="
"$MAMBA_BIN" run -n "$ENV_NAME" bash -c "
  set -e
  cd '$ROOT/robot_ros2_ws'
  ulimit -v 4194304
  MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
"

echo "Done. source 环境："
echo "  micromamba run -n $ENV_NAME bash"
echo "  source $ROOT/robot_ros2_ws/install/setup.bash"
