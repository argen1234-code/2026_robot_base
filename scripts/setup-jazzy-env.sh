#!/usr/bin/env bash
# 在 Debian 12 / aarch64 上创建 RoboStack Jazzy 环境。
# 说明：Debian 12 没有 ROS 2 官方二进制；RoboStack 提供 aarch64 conda 包。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="${ENV_NAME:-ros_jazzy}"
MAMBA_BIN="$HOME/.local/bin/micromamba"
MAMBA_ROOT="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
ROBOSTACK="https://conda.anaconda.org/robostack-jazzy"
CONDA_FORGE="https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge"

if [ ! -x "$MAMBA_BIN" ]; then
  echo "Installing micromamba ..."
  tmp="$(mktemp -d)"
  curl -fL --retry 3 -o "$tmp/mm.tar.bz2" \
    "https://ghfast.top/https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-aarch64.tar.bz2"
  mkdir -p "$HOME/.local/bin"
  tar -xjf "$tmp/mm.tar.bz2" -C "$tmp"
  install -m 0755 "$tmp/bin/micromamba" "$MAMBA_BIN"
  rm -rf "$tmp"
fi

export MAMBA_ROOT_PREFIX="$MAMBA_ROOT"
mkdir -p "$MAMBA_ROOT"

"$MAMBA_BIN" create -y -n "$ENV_NAME" \
  -c "$ROBOSTACK" -c "$CONDA_FORGE" --strict-channel-priority \
  ros-jazzy-ros-base \
  ros-jazzy-nav2-bringup \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-laser-filters \
  ros-jazzy-tf2-tools \
  ros-jazzy-rosidl-default-generators \
  colcon-common-extensions \
  cmake make compilers pkg-config \
  libboost-devel eigen yaml-cpp \
  python=3.12 pip pyserial

echo
echo "Done. 用法："
echo "  micromamba run -n $ENV_NAME bash"
echo "  # 或在交互 shell： micromamba activate $ENV_NAME"
echo "下一步： $ROOT/scripts/build-nav.sh"
