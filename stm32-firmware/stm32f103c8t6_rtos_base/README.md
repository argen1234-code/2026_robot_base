# stm32f103c8t6_rtos_base

STM32F103C8T6（LQFP48）CubeMX 空工程基线，用于后续按外设逐步扩展。

- 主频 72 MHz（HSE + PLL ×9）
- FreeRTOS + CMSIS-RTOS v2，默认 `defaultTask`
- HAL 时基使用 TIM4（`Core/Src/stm32f1xx_hal_timebase_tim.c`）
- 当前只初始化 GPIO，未配置 UART/CAN/SPI/I2C/PWM 等业务外设

## 构建

GCC（Makefile）：

```bash
cd stm32-firmware/stm32f103c8t6_rtos_base
GCC_PATH="C:/Users/Arjen/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin" make -j8
```

输出：`build/stm32f103c8t6_rtos_base.elf/.hex/.bin`。

Keil：打开 `MDK-ARM/stm32f103c8t6_rtos_base.uvprojx`。

## 目录

```text
Core/        CubeMX 生成的初始化、中断、FreeRTOS 配置
Drivers/     STM32F1 HAL + CMSIS
Middlewares/ FreeRTOS
MDK-ARM/     Keil 工程
```
