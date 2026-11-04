#!/usr/bin/env bash
# 启动 OpenOCD（GDB server），另开终端用 gdb-multiarch 连接。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ELF="${1:-$HERE/build/firmware.elf}"

[ -f "$ELF" ] || { echo "未找到 $ELF，请先运行 $HERE/build.sh"; exit 1; }

cat <<EOF
OpenOCD 正在启动（GDB server: localhost:3333）。
另开一个终端执行：

  gdb-multiarch "$ELF"
  (gdb) target extended-remote localhost:3333
  (gdb) monitor reset halt
  (gdb) load
  (gdb) monitor reset init
  (gdb) continue

EOF

exec openocd \
  -f interface/cmsis-dap.cfg \
  -f target/stm32h7x.cfg \
  -c "adapter speed 4000"
