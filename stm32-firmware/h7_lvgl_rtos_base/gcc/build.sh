#!/usr/bin/env bash
# 在 Pi 上用 arm-none-eabi-gcc 编译 STM32H743 固件。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cmake -S "$HERE" -B "$HERE/build" \
  -DCMAKE_TOOLCHAIN_FILE="$HERE/toolchain-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$HERE/build" -j2

echo "输出：$HERE/build/firmware.elf / .hex / .bin"
