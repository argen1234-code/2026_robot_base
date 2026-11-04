# STM32H743 GCC 构建

在 Linux/aarch64（树莓派）上用 `arm-none-eabi-gcc` 编译固件，不再依赖 Keil MDK。

## 依赖

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi cmake make \
                 openocd gdb-multiarch
```

DAPLink/CMSIS-DAP 探针的 udev 权限已由 `60-cmsis-dap.rules` 提供；用户需在 `plugdev` 组。

## 编译

```bash
./build.sh
```

产物在 `build/`：

- `firmware.elf` / `firmware.hex` / `firmware.bin`
- `firmware.map`

## 烧录（DAPLink v2）

```bash
./flash.sh
```

等价于：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "adapter speed 4000; program build/firmware.elf verify reset exit"
```

## 调试

```bash
./debug.sh          # 终端 1：OpenOCD/GDB server
gdb-multiarch build/firmware.elf   # 终端 2
(gdb) target extended-remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) monitor reset init
(gdb) continue
```

## 与 Keil 工程的差异

| 项 | 处理 |
|---|---|
| 源文件清单 | `sources.cmake` 由 `MDK-ARM/STM32H743.uvprojx` 自动生成 |
| 头文件路径 | `includes.cmake` 同上 |
| 启动文件 | 使用 CMSIS 的 GCC 版 `startup_stm32h743xx.s`；移除双核专用的 `ExitRun0Mode` 调用 |
| FreeRTOS | Keil 用 RVDS/ARM_CM4F 移植层；GCC 使用 `portable/GCC/ARM_CM7/r0p1`（V10.3.1） |
| 浮点 ABI | `-mfloat-abi=softfp`：`libneai.a` 是软浮点 ABI，hard-float 会链接失败 |
| 链接脚本 | `.data/.bss` 放在 AXI SRAM（0x24000000），避免 DTCM 不能被 DMA 访问 |
| 厂商库 | `Drivers/Hardware/NanoEdgeAI/libneai.a` 参与链接 |

## 内存占用（当前构建）

- FLASH: 约 458KB / 2MB
- RAM: 约 104KB / 512KB
