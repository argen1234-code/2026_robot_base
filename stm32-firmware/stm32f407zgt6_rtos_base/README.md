# stm32f407zgt6_rtos_base

STM32F407ZGT6（LQFP144）下位机固件基础工程，用于后续按外设逐步扩展。

- 主频 168 MHz（HSE + PLL）
- FreeRTOS + CMSIS-RTOS v2，默认 `defaultTask`
- HAL 时基使用 TIM7（`Core/Src/stm32f4xx_hal_timebase_tim.c`）
- CAN1：PB8=RX / PB9=TX
- USB OTG FS：Device_Only，PA11=DM / PA12=DP
- 其余业务外设尚未配置

## 构建

GCC（Makefile）：

```bash
cd stm32-firmware/stm32f407zgt6_rtos_base
GCC_PATH="C:/Users/Arjen/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin" make -j8
```

输出：`build/stm32f407zgt6_rtos_base.elf/.hex/.bin`。

CMake / Keil：

- CubeMX 生成 `CMakeLists.txt`、`CMakePresets.json`
- Keil 工程：`MDK-ARM/stm32f407zgt6_rtos_base.uvprojx`

## 目录

```text
Core/        CubeMX 生成的初始化、中断、FreeRTOS 配置
Drivers/     STM32F4 HAL + CMSIS
Middlewares/ FreeRTOS
MDK-ARM/     Keil 工程
cmake/       工具链文件
```
