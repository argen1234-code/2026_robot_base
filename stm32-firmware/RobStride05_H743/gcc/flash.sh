#!/usr/bin/env bash
# DAPLink（CMSIS-DAP）烧录 RobStride05 H743 固件。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ELF="${1:-$HERE/build/firmware.elf}"
[ -f "$ELF" ] || { echo "未找到 $ELF，请先运行 $HERE/build.sh"; exit 1; }

exec openocd \
  -f interface/cmsis-dap.cfg \
  -f target/stm32h7x.cfg \
  -c "adapter speed 4000; program $ELF verify reset exit"
