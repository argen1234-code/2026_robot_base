# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32H743 四轮全向多功能小车，用于 2026 嵌入式竞赛。基于 FreeRTOS + LVGL，目前处于底盘控制与导航开发阶段（屏幕部分处于维护/待完善状态）。

## Build & Flash

- **IDE**: Keil MDK-ARM (uVision 5)
- **Project file**: `MDK-ARM/STM32H743.uvprojx` — 双击打开后点击 Build (F7) 编译
- **Toolchain**: **ARM Compiler V5.06** (实测 build 日志显示 V5.06 update 7, 非 ARMCLANG)
- **Debug/Flash**: 通过 SWD (PA13/PA14) 连接 ST-Link 或 J-Link，在 Keil 中 Download (F8)
- **Clean**: 运行工程根目录的 `keilkilll.bat` 清理中间文件

VSCode 仅用于代码编辑，不用于编译。VSCode IntelliSense 配置在 `.vscode/c_cpp_properties.json`，如果红波浪线过多，执行 `Ctrl+Shift+P` → `C/C++: Reset IntelliSense Database`。

## MCU & Clock

- **MCU**: STM32H743 (Cortex-M7)
- **主频**: 480MHz (HSE 25MHz → PLL)
- **SDRAM**: 通过 FMC 外挂，已配置 MPU 区域（Region1, 32MB, Cacheable）
- **LCD**: LTDC RGB 接口，时钟 33MHz
- **HAL tick**: TIM17 提供

## Architecture

### 分层结构

```
Core/                         HAL 生成层（main.c, 中断, 系统初始化）
Drivers/User/                 外设 MX 驱动（usart, tim, i2c, gpio, dma, lcd）
Drivers/Hardware/             BSP 硬件抽象（motor, encoder, gps, imu, bt, usb）
Drivers/Algorithm/            PID 控制器
Middware/App/                 应用层（底盘主控, 导航, 遥控, 语音）
Middware/LVGL/                LVGL 图形库
Middware/Third_Party/FreeRTOS/ FreeRTOS 内核 + CMSIS_OS_V2
Middware/ST/                  ST USB Device Library
USB_DEVICE/                   USB CDC 应用层
MDK-ARM/                      Keil 工程文件
```

### FreeRTOS 任务（3个）

| 任务 | 入口函数 | 优先级 | 栈 | 说明 |
|------|----------|--------|-----|------|
| `MyGuiTask` | `my_gui_task()` | High | 20KB | LVGL 界面 (`LVGL_TASK=1`) |
| `ChassisBoardTask` | `chassis_task()` | High | 1KB | 底盘主控 100Hz |
| `KeyTask` | `key_task()` | High | 512B | 按键测试 (`key_test=0` 默认关闭) |

任务创建在 `Drivers/User/Src/freertos.c` → `MX_FREERTOS_Init()`，开关宏定义在各自头文件中（`chassis_board_task`、`LVGL_TASK`、`key_test`）。

### 底盘主控 5 步循环（100Hz = 10ms 周期）

1. `chassis_mode_change()` — 蓝牙模式切换请求
2. `chassis_feedback_update()` — 传感器刷新（磁力计 QMC5883 + JY901S IMU + 编码器 + USB Jetson + GPS 定位解算）
3. `chassis_set_control()` — 控制量优先级调度（蓝牙 > 微信 > ROS室内 > GPS+ROS融合 > 纯GPS > 语音）
4. `chassis_control_loop()` — 麦克纳姆轮运动学分解 + 4路 PID 速度闭环
5. `chassis_send_cmd()` — PID 输出 → 电机 PWM + USB 遥测回传 Jetson

### 车辆工作模式（`CarMode_t`）

GPS导航 / 微信遥控(Jetson转发) / 室内ROS自主导航 / 蓝牙遥控 / WonderEcho语音控制

### 关键外设分配

| 外设 | 引脚 | 用途 |
|------|------|------|
| USART1 | PA9/PA10 | 蓝牙 (115200) |
| USART2 | PA2/PD6 | GPS (115200) |
| USART6 | PG14/PG9 | JY901S 九轴IMU (9600) |
| ~~UART4~~ | ~~PC10/PH14~~ | 已禁用，PC10 释放给 SDMMC1_D2 |
| SDMMC1 | PC8/9/10/11/12 + PD2 | TF 卡 4-bit 20MHz |
| USB CDC | PA11/PA12 | Jetson 通信 (12字节帧) |
| I2C1 | PB8/PB9 | QMC5883 磁力计 (0x0D) |
| TIM8 CH1-4 | PI5/PI6/PI7/PI2 | 4路电机PWM (~10kHz) |
| TIM2/3/4/5 | — | 4路编码器 (AB相) |
| Software I2C | PB10/PB11 | OLED |
| Software I2C | PG3/PG7 | Touch |

详细引脚分配见 `PCB_Pinout.md`。

### USB ↔ Jetson 协议

- **Jetson→STM32**: 12字节帧，解析为 `cmd_vel_t`（mode + vx + vz）
- **STM32→Jetson**: 遥测数据（目标方位角 + 当前经纬度）
- 超时判定: 500ms 无有效帧视为 Jetson 离线

### GPS 导航（`app_Navigation.c`）

- 支持多航点循环巡航（最多 `MAX_WAYPOINTS=10`）
- 算法: 距离闭环 + 航向闭环 PID → 底盘 `Vx/Wz` 合速度
- 航点到达判定 + 停留计时（`WAYPOINT_DWELL_MS=5000`）
- 融合模式: GPS + Jetson ROS 数据融合导航

## 注意事项

- 不要删除或移动 `keilkilll.bat`，Keil 编译产物可以由此清理
- `.vscode/c_cpp_properties.json` 中的 `limitSymbolsToIncludedHeaders` 已设为 `false`，`cStandard` 为 `gnu99`，这些是为了配合 Keil 编译器的 IntelliSense 设置
- 工程实际使用 **ARM Compiler V5.06**（非 ARMCLANG），不支持 UTF-8 源文件，不支持 C11/C99 for-loop 内声明
- 已启用 MicroLib（`fputc` 重定向到 USART1）
- `PB3/PB4` 同时是 JTAG 引脚，使用 SWD 不受影响，但不要启用完整 JTAG
- 磁力计 QMC5883 布局需远离电机和大电流铜皮

## SDMMC1 — TF 卡

- **模式**: SDIO 4-bit, 20MHz
- **引脚**: PC8(D0), PC9(D1), PC10(D2), PC11(D3), PC12(CK), PD2(CMD)
- **引脚冲突解决**: PC11/PC12 原为前右轮电机方向，已换到 PC4/PC5；PC10 原为 UART4_TX，UART4 已禁用
- **驱动文件**:
  - `Drivers/User/Src/sdmmc_sd.c/.h` — ST BSP SD 驱动（来自 SDK 例程）
  - `Drivers/Hardware/bsp_TF_card.c/.h` — BSP 封装
  - `Drivers/Hardware/bsp_fatfs_diskio.c` — FATFS 磁盘 I/O 适配
  - `Drivers/Hardware/bsp_tf_image_load.c/.h` — 图片从 TF 卡加载
- **HAL 模块**: `stm32h7xx_hal_sd.c` + `stm32h7xx_ll_sdmmc.c` 已加入工程
- **时钟**: PLL1_Q=480MHz, SDMMC_CK divider=12 → 20MHz
- **中断**: `SDMMC1_IRQHandler` 在 `stm32h7xx_it.c`，调用 `HAL_SD_IRQHandler(&hsd_sdmmc[0])`
- **D-Cache**: `disk_read` 中 DMA 写入 SDRAM 后须调用 `SCB_InvalidateDCache_by_Addr`（仅 SDRAM 地址范围 `>= 0xC0000000` 且 `< 0xC2000000`）

## FATFS

- **版本**: R0.12 (来自 SDK LCD 图片显示例程)
- **位置**: `Middware/Third_Party/FATFS/ff.c`
- **配置**: ASCII only, LFN=off, exFAT=off, 1 volume
- **Include path**: `..\Middware\Third_Party\FATFS`

## LVGL 图片从 TF 卡加载

- 启动流程: `lv_init → disp/indev → BSP_TF_Init → FAT32 mount → f_open/f_read → SDRAM → ui_init`
- 图片描述符非 const（运行时可写 `.data`）
- Flash 中图片数据数组被 `strip_image_data.pl` 缩减为占位符 `{0}`（~600B），实际像素从 TF 卡文件加载
- TF 卡文件: `icon.bin`, `bg.bin`（RGBA 原始像素，从 SquareLine C 数组提取）
- 168KB 以内小文件稳定加载；**大文件（>250KB）连续读取会报 FR_DISK_ERR**，原因未明（疑似 SDMMC 大批量传输超时或卡兼容性问题）
- 如需加载大图，考虑 QSPI Flash 或分小图拼接

## SquareLine Studio 导出的后处理

每次从 `C:\Users\Arjen\Desktop\screens` 更新代码后：

1. `perl strip_image_data.pl <image.c>` — 把图片数据缩为占位符
2. `perl cn_to_utf8_escapes.pl <file.c>` — 中文 → UTF-8 hex 转义
3. `ui.h`: 移除 `lv_i18n.h` include, `lvgl/lvgl.h` → `lvgl.h`, `LV_IMG_DECLARE` → `extern lv_img_dsc_t`
4. 图片 `.c` 文件中 `const lv_img_dsc_t` → `lv_img_dsc_t`
5. 用 Python 脚本提取 `.bin` 文件到桌面，复制到 TF 卡

## Keil 工程分组

| 组 | 新增文件 |
|---|---|
| **system** | `Middware/Third_Party/FATFS/ff.c` |
| **Application/User/Core** | `sdmmc_sd.c` |
| **Drivers/STM32H7xx_HAL_Driver** | `stm32h7xx_hal_sd.c`, `stm32h7xx_ll_sdmmc.c` |
| **Hardware** | `bsp_TF_card.c`, `bsp_fatfs_diskio.c`, `bsp_tf_image_load.c` |
| **BSP_LVGL** | `ui_img_1301672413.c`（当前仅此图片） |

## Keil 配置

- **Use MicroLIB**: 勾选（printf 重定向到 USART1）
- **Include Paths**: 含 `..\Middware\Third_Party\FATFS`
- **预定义宏**: `HAL_SD_MODULE_ENABLED`（在 `stm32h7xx_hal_conf.h` 中）
