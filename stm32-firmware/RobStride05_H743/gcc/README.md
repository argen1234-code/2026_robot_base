# RobStride05 H743 GCC 构建

在 Linux/aarch64（树莓派）上用 `arm-none-eabi-gcc` 编译，不依赖 Keil。

## 依赖

```bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi cmake make \
                 openocd gdb-multiarch
```

DAPLink/CMSIS-DAP 需要 udev 权限与 `plugdev` 组。

## 编译

```bash
./build.sh
```

产物：`build/firmware.elf` / `.hex` / `.bin` / `.map`。

## 烧录 / 调试

```bash
./flash.sh      # DAPLink + OpenOCD 烧录
./debug.sh      # 启动 GDB server，另开终端 gdb-multiarch
```

## 与 Keil 工程的差异

| 项 | 处理 |
|---|---|
| 源文件/头文件 | `sources.cmake` / `includes.cmake` 由 `MDK-ARM/RobStride05_H743.uvprojx` 生成 |
| 启动文件 | CMSIS GCC 版 `startup_stm32h743xx.s`（保留 `ExitRun0Mode`，本工程在 `system_stm32h7xx.c` 中定义） |
| FreeRTOS | 使用 V10.6.2 的 `portable/GCC/ARM_CM7/r0p1`（Keil 用 RVDS 移植层） |
| 浮点 ABI | `-mfloat-abi=hard`（本工程无软浮点库依赖） |
| 内存布局 | 对照 `.sct`：RW/ZI 放 DTCM；`WS2812_DMA` 段 NOLOAD 放 D2 SRAM1 |
| WS2812 | `zero_init` 是 ARMCC 专有属性，源码中加了 GCC 条件分支 |

## 注意

- 本工程要求 H743 Rev.V、LDO/VOS0、480MHz；先用 DAPLink 确认目标板版本。
- 烧录前务必让电机悬空或断开动力电源，遵循原工程的安全说明。
