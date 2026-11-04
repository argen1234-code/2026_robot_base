# RobStride 05 双电机独立控制工程（FreeRTOS 版）

JDY31 蓝牙遥控已接入 USART1（PA9 TX / PA10 RX，9600 8N1）。手机与电脑共用 CRLF 文本指令，电脑入口在新版上位机的“JDY31 · 蓝牙遥控”。下载本次固件后使用，接线与指令见 [蓝牙遥控说明](../Docs/BLUETOOTH.md)。本次仅完成软件验证，未做实物蓝牙或车辆运动测试。

STM32H743IIT6 · STM32 HAL · FreeRTOS/CMSIS-RTOS2 · 经典 CAN · SSD1306 OLED · WS2812 灯带。

当前配置为 **Rev.V / LDO / VOS0，CPU 480 MHz、AHB 240 MHz、APB 120 MHz**；CAN 仍为 1 Mbps，OLED 和 16 颗灯带的通信时序已保持。ICache/DCache 继续关闭。已完成满频烧写、寄存器/时间基准核对、双电机运动与灯效回归，详见 [480 MHz 性能配置与实板回归](Docs/12_480MHz性能配置与实板回归.md)。本文后面的旧时钟与单电机记录属于历史背景，当前配置以第 10–12 篇为准。

WS2812 已支持上位机自定义 7 种灯效、颜色/亮度/速度及循环列表，使用方法见 [灯带控制说明](Docs/11_WS2812上位机自定义灯效.md)。

2026-10-26 已扩展为两台 RobStride 05 共用 CAN 总线，分别控制、分别反馈，OLED 同时显示两台。保留 FreeRTOS 和原 WS2812/RTOS 统计功能，已编译、烧写并通过两台实物正反转及 ±5 rad/s 同时运行验证。

**先看 [双电机使用与本轮实测](Docs/10_双电机独立控制与实测.md)。原电机 ID 127，新增电机 ID 126 已配置完成；正常使用无需再次改 ID。新版上位机为上级 `上位机/RobStrideConsole.exe`。**

下面单电机版的页面/验收记录保留作历史背景；当前双电机行为和新的构建集成方式以上面第 10 篇为准。

**I-Cache / D-Cache 在本工程中完全关闭且没有任何 Cache 维护代码**：没有 `SCB_EnableICache`、`SCB_DisableDCache`、clean/invalidate 调用；`SCB->CCR` 仅作只读记录。原因与验证见 [Docs/07_RTOS移植与上板验收.md](Docs/07_RTOS移植与上板验收.md)；机理与复现见 [Docs/08](Docs/08_Cache为什么会HardFault.md)；通用避坑见 [Docs/09](Docs/09_H7_Cache入门与避坑.md)。

**源码注释**：RTOS 专属文件（`main.c`、`freertos.c`、`stm32h7xx_it.c`、`app_tasks.*`、`app_main.c`、`app_display.*`、`app_rtos_stats.*`）与核心逻辑（`app_motor.c`、`bsp_can.c`）已补详尽中文注释；与 HAL 工程共享的文件注释只在本工程，双电机代码已在 RTOS 工程内独立扩展，HAL 兄弟工程保持原实现。

## 从这里开始

| 内容 | 入口 |
|---|---|
| Keil 工程 | [MDK-ARM/RobStride05_H743.uvprojx](MDK-ARM/RobStride05_H743.uvprojx) |
| CubeMX 配置 | [RobStride05_H743.ioc](RobStride05_H743.ioc) |
| 接线、协议与量程依据 | [Docs/01_硬件与协议.md](Docs/01_硬件与协议.md) |
| 任务划分、时钟、内存、Cache 与维护 | [Docs/02_架构与维护.md](Docs/02_架构与维护.md) |
| Keil 设置、`g_debug` 结构与命令提交 | [Docs/03_Keil调试.md](Docs/03_Keil调试.md) |
| 烧写与上板验收 | [Docs/04_上板验收.md](Docs/04_上板验收.md) |
| 本轮 Cache 代码移除、移植与实测记录 | [Docs/07_RTOS移植与上板验收.md](Docs/07_RTOS移植与上板验收.md) |
| Cache 为什么会让旧版一上电就 HardFault（含实物复现） | [Docs/08_Cache为什么会HardFault.md](Docs/08_Cache为什么会HardFault.md) |
| H7 Cache 通俗入门与避坑清单（含速查表、自检法） | [Docs/09_H7_Cache入门与避坑.md](Docs/09_H7_Cache入门与避坑.md) |
| 历史排查（已过时，保留追溯） | [Docs/05_验证记录.md](Docs/05_验证记录.md)、[Docs/06_RTOS启动排查.md](Docs/06_RTOS启动排查.md) |

## 构建、烧写与验证

```powershell
cd 'D:\竞赛\2026 AIC\RobStride05_H743'
# 1) 集成并重新编译（Keil ARMCC 5.06u7；先安装 Python 并加入 PATH）
pwsh -File .\Tools\build.ps1
# 2) 烧写（Keil 下载器；会备份当前 Flash 到 Tools/backup）
& 'C:\Keil_v5\UV4\UV4.exe' -f .\MDK-ARM\RobStride05_H743.uvprojx -j0 -o .\Tools\keil_flash.log
# 3) 只读验证：boot_stage/fatal/心跳/CCR/CAN/OLED/灯带
python .\Tools\flash_and_verify.py --no-flash
# 4) 回读 OLED 帧缓冲，把屏幕内容反解成文字（不需要看屏）
python .\Tools\read_oled_page.py
# 5) 满频时钟与外设时基核对（先断开上位机，释放 SWD）
python .\Tools\verify_clocks.py
```

`Tools/flash_and_verify.py` 只用 SWD 读 `g_debug` 与 `g_led`，**不下发电机命令**；使能/运行必须由上位机显式操作。

## 屏幕内容（RTOS 版）

0.96" 128×64 屏，8 行 × 21 字符，全部是 RTOS 信息；电机只保留在线状态、实测速度、设定速度：

```text
UP 00:00:08 SW 15763      ← 运行时长 + 上下文切换次数（约 1800-2000 次/秒）
HEAP 16.0K MIN 16.0K      ← 剩余堆 / 历史最小
MOTOR  BLK 1.2 S1.5K      ← 线程名 状态 CPU占用% 剩余栈
OLED   RUN 1.3 S1.9K
HEALTH BLK 0.0 S0.8K
LED    BLK 0.8 S1.3K
MOT ONLINE  V+0.043       ← 离线时显示 OFFLINE V  ---
SET +0.000 RAD/S DIS      ← 设定速度 + 应用状态(DIS/PRE/ENA/RUN/STP/FLT)
```

CPU 占用、状态、栈余量和切换次数都来自调度器自己的统计（`configGENERATE_RUN_TIME_STATS` + `DWT->CYCCNT` + `uxTaskGetSystemState` + `traceTASK_SWITCHED_IN`），实现在 `App/app_rtos_stats.c`。版面实现在 `App/app_display.c`（纯格式化，可用 `Tools/test_display.c` 在 PC 上编译检查），取数在 `App/app_main.c` 的 `draw()`，线程表在 `App/app_tasks.c` 的 `g_app_task[]`。屏面预览（由 `Tools/read_oled_page.py --png` 从实物帧缓冲渲染）：[Docs/oled_page_preview.png](Docs/oled_page_preview.png)。

Cache 为什么会让旧版一上电就 HardFault：见 [Docs/08_Cache为什么会HardFault.md](Docs/08_Cache为什么会HardFault.md)（含实物复现实验）。

## 上板运行状态（2026-10-22 实测）

- `boot_stage=9`、`fatal=0`、`vtor=0x08000000`、`ccr=0x40200`（IC/DC 均为 0）。
- 四个任务心跳持续增长；电机任务单次服务 10–30 µs；OLED 5 Hz 无失败；WS2812 50 fps 无错误。
- CAN：反馈持续在线、`tx_ok` 递增、无 `rx_drop`/`bus_off`。
- OLED 帧缓冲回读：`RS05 RTOS UP 00:00:28 / HEAP 16.0K MIN 16.0K / MOTOR BLK S1.5K H794 / OLED RUN S2.0K H794 / HEALTH BLK S0.8K H036 / LED BLK S1.2K H717 / MOT ONLINE V-0.051 / SET +0.000 RAD/S DIS`，无缺字形单元。
- 上位机 `RobStrideConsole.exe` 连接成功，自动识别本工程 AXF（Flash 全段核对）并显示实时数据。

## 固定范围

- PB9/PB8：FDCAN1 TX/RX，经典 CAN 1 Mbps、29 位扩展帧、8 字节。
- PB10/PB11：I²C2 SCL/SDA，SSD1306 128×64、7 位地址 0x3C，约 5 Hz 刷新。
- PA6：TIM3_CH1 + DMA1 Stream0 + DMAMUX1 ch0 驱动 WS2812，16 颗，50 fps。

2026-10-26 已将 `App/app_config.h` 的 `APP_LED_COUNT` 从 10 改为 16，并完成编译、下载与 Flash 核对。板上 `g_led.led_count=16`；常亮模式回读的 DMA 帧包含全部 16 颗数据（708 个半字、1416 字节），末尾复位低电平完整，发送无错误，随后恢复自动灯效。两台电机在线且失能、OLED 正常。这里验证的是板上数据与运行状态，灯珠实际发光需目视确认。证据：`Docs/ws2812_16_hardware.json`、`Docs/ws2812_16_frame_check.json`、`Docs/ws2812_16_console_check.json`。上位机 EXE 也已同步更新固件信息。
- 只实现速度模式。命令范围 33 rad/s / 11 A / 150 rad/s²，默认准备值 33 / 2.4 / 20。
- 上电只发送停止帧，不自动使能；必须经上位机「准备 → 使能 → 运行」。
- 无外部 SDRAM、LVGL、文件系统，仅首次隔离配置入口允许显式修改电机 CAN ID；不修改协议、机械零点或其它持久参数。

**首次使能前需人工核对：电机主电源独立供电、接线、负载固定。任何时刻都应保留可直接断电的手段。**

## 小车 WASD 更新（2026-11-02）

本工程已增加配套上位机的小车成对速度命令与模式超时处理，使用现有 SWD 调试连接。原独立双电机、OLED、WS2812、时钟及 Cache 配置保留。请将本次 AXF/HEX 与同仓库新版 EXE 配套使用；操作、软件测试结果及未做实车验证的范围见 [WASD 说明](../Docs/WASD.md)。
