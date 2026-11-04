#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cmake -S "$HERE" -B "$HERE/build" \
  -DCMAKE_TOOLCHAIN_FILE="$HERE/toolchain-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$HERE/build" -j2

echo "输出：$HERE/build/firmware.elf / .hex / .bin"
