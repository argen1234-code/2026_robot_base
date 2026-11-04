#!/usr/bin/env bash
# 用 DAPLink（CMSIS-DAP）通过 OpenOCD 烧录固件。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ELF="${1:-$HERE/build/firmware.elf}"

[ -f "$ELF" ] || { echo "未找到 $ELF，请先运行 $HERE/build.sh"; exit 1; }
command -v openocd >/dev/null || { echo "缺少 openocd"; exit 1; }

exec openocd \
  -f interface/cmsis-dap.cfg \
  -f target/stm32h7x.cfg \
  -c "adapter speed 4000; program $ELF verify reset exit"
