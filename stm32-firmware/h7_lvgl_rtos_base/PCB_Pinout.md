# PCB Pinout

本文档用于画 PCB 时核对接口和引脚。内容按当前工程代码整理：

- 串口配置：`Drivers/User/Src/usart.c`
- I2C 配置：`Drivers/User/Src/i2c.c`
- PWM / 编码器：`Drivers/User/Src/tim.c`
- 电机方向脚：`Drivers/User/Src/gpio.c`、`Drivers/Hardware/bsp_motor.c`

## 接线注意

- 串口必须交叉：`MCU_TX -> 模块_RX`，`MCU_RX <- 模块_TX`。
- 所有模块必须共地。
- `PD6` 在 `main.h` 里的宏名有点误导，实际功能是 `USART2_RX`。
- `PB8/PB9` 的 I2C1 建议预留 `4.7k` 上拉到 `3.3V`。
- 若不做屏幕，`Touch / LCD / SDRAM` 部分可以忽略。

## 基础引脚

| 功能 | MCU 引脚 | 说明 |
| --- | --- | --- |
| SWDIO | PA13 | 调试下载 |
| SWCLK | PA14 | 调试下载 |
| NRST | NRST | 建议引出 |
| HSE OSC_IN | PH0 | 外部晶振输入 |
| HSE OSC_OUT | PH1 | 外部晶振输出 |
| LED1 | PC13 | 低电平点亮 |
| KEY | PA15 | 输入上拉，按键接 GND |

## 串口接口

### USART1 - Bluetooth

波特率：`115200`

| MCU 信号 | MCU 引脚 | 接模块 |
| --- | --- | --- |
| USART1_TX | PA9 | Bluetooth RX |
| USART1_RX | PA10 | Bluetooth TX |
| GND | GND | Bluetooth GND |
| VCC | 按模块要求 | 常见 3.3V / 5V |

### USART2 - GPS

波特率：`115200`

| MCU 信号 | MCU 引脚 / 板端 | 接模块 |
| --- | --- | --- |
| USART2_TX | PA2 | GPS RX |
| USART2_RX | PD6 | GPS TX |
| GND | TB6612-GND | GPS GND |
| VCC | TB6612-5V | GPS VCC |

### USART6 - JY901S

波特率：`9600`

| MCU 信号 | MCU 引脚 / 板端 | 接模块 |
| --- | --- | --- |
| USART6_TX | PG14 | JY901S RX |
| USART6_RX | PG9 | JY901S TX |
| GND | TB6612-GND | JY901S GND |
| VCC | TB6612-3.3V | JY901S VCC |

### UART4 - Disabled (pins freed for SDMMC1)

UART4 已禁用，`MX_UART4_Init()` 已注释。引脚 `PC10` 释放给 SDMMC1_D2。

## SDMMC1 - TF Card

SDIO 4-bit 模式，时钟 20MHz。

| SDMMC1 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| SDMMC1_CK | PC12 | 时钟（原 Motor DIR2，引脚已换到 PC5） |
| SDMMC1_CMD | PD2 | 命令 |
| SDMMC1_D0 | PC8 | 数据 0 |
| SDMMC1_D1 | PC9 | 数据 1 |
| SDMMC1_D2 | PC10 | 数据 2（原 UART4_TX，已禁用） |
| SDMMC1_D3 | PC11 | 数据 3（原 Motor DIR1，引脚已换到 PC4） |

驱动文件：`Drivers/User/Src/sdmmc_sd.c` / `.h`（来自 SDK 例程 `05.SDIO-基本数据读写`）。

封装文件：`Drivers/Hardware/bsp_TF_card.c` / `.h`

图片加载：`Drivers/Hardware/bsp_tf_image_load.c` / `.h`
- TF 卡 FAT32 格式化，复制 `icon.bin` / `bg.bin` 文件
- 启动时 f_read 读取到 SDRAM，patches lv_img_dsc_t 描述符
- SquareLine 更新后用 Python 提取 .bin 文件替换 TF 卡上的即可
- diskio 层用 `__disable_irq()` 保护多扇区传输不被 FreeRTOS 打断

文件管理：`Drivers/Hardware/bsp_fatfs_ex.c` / `.h`
- 文件类型识别、磁盘空间查询、文件/文件夹复制（带进度回调）

## USB CDC

| 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| USB_OTG_FS_DM | PA11 | USB D- |
| USB_OTG_FS_DP | PA12 | USB D+ |
| GND | GND | USB GND |
| VBUS | USB 5V | 当前代码关闭 VBUS sensing |

PCB 建议：`D+ / D-` 尽量等长，按差分走线，接口附近放 ESD。

## I2C1 - QMC5883 磁力计

QMC5883 地址：

- 7 位地址：`0x0D`
- HAL 8 位地址：`0x1A`

| MCU 信号 | MCU 引脚 / 板端 | 接模块 |
| --- | --- | --- |
| I2C1_SCL | PB8 | QMC5883 SCL |
| I2C1_SDA | PB9 | QMC5883 SDA |
| GND | STM32H7 GND | QMC5883 GND |
| VCC | STM32H7 3.3V | QMC5883 VCC |

磁力计布局建议：远离电机、电源线、大电流铜皮、磁铁和铁件，最好放在车体边缘或做成外接小板。

## OLED - Software I2C

OLED 地址：`0x78` 或 `0x7A`，以实际模块为准。

| 信号 | MCU 引脚 | 接模块 |
| --- | --- | --- |
| OLED_SCL | PB10 | OLED SCL |
| OLED_SDA | PB11 | OLED SDA |
| GND | GND | OLED GND |
| VCC | 按模块要求 | 常见 3.3V / 5V |

备注：这是软件 I2C，不是 I2C1。

## TB6612 电机驱动

`TB6612-STBY` 接 `TB6612-3V3`。

### PWM 与方向

| 车轮 | PWM | 方向 1 | 方向 2 |
| --- | --- | --- | --- |
| Front Left | PI5 / TIM8_CH1 / TB6612-PWMA | PA6 / TB6612-AIN1 | PA7 / TB6612-AIN2 |
| Front Right | PI6 / TIM8_CH2 / TB6612-PWMB | PC4 / TB6612-BIN1 | PC5 / TB6612-BIN2 |
| Rear Left | PI7 / TIM8_CH3 / TB6612-PWMC | PB12 / TB6612-CIN1 | PB13 / TB6612-CIN2 |
| Rear Right | PI2 / TIM8_CH4 / TB6612-PWMD | PB14 / TB6612-DIN1 | PB15 / TB6612-DIN2 |

### 方向逻辑

| 车轮 | PWM >= 0 | PWM < 0 |
| --- | --- | --- |
| Front Left | PA6=1, PA7=0 | PA6=0, PA7=1 |
| Front Right | PC4=1, PC5=0 | PC4=0, PC5=1 |
| Rear Left | PB12=1, PB13=0 | PB12=0, PB13=1 |
| Rear Right | PB14=1, PB15=0 | PB14=0, PB15=1 |

备注：`TIM8` 当前配置约为 `10 kHz PWM`。

## 编码器接口

| 车轮 | 定时器 | A 相 / CH1 | B 相 / CH2 |
| --- | --- | --- | --- |
| Front Left | TIM2 | PA5 / TB6612-E1B | PB3 / TB6612-E1A |
| Front Right | TIM3 | PB4 / TB6612-E2A | PB5 / TB6612-E2B |
| Rear Left | TIM4 | PB6 / TB6612-E3B | PB7 / TB6612-E3A |
| Rear Right | TIM5 | PH10 / TB6612-E4A | PH11 / TB6612-E4B |

备注：

- `PB3` 同时是 `JTDO/TRACESWO`。
- `PB4` 同时是 `NJTRST`。
- 使用 SWD 下载调试一般不影响，但不要再启用完整 JTAG。
- 如果速度符号和实际相反，可以交换 A/B 相，或在软件里调整符号。

## Touch 触摸屏

软件 I2C，由 `touch_iic.c` 初始化。

| 信号 | MCU 引脚 | 说明 |
| --- | --- | --- |
| TOUCH_SCL | PG3 | Touch software I2C SCL |
| TOUCH_SDA | PG7 | Touch software I2C SDA |
| TOUCH_RST | PI10 | Touch reset |
| TOUCH_INT | PI11 | Touch interrupt / data |

## LCD RGB LTDC

| LCD 信号 | MCU 引脚 |
| --- | --- |
| LTDC_R0 | PI15 |
| LTDC_R1 | PJ0 |
| LTDC_R2 | PJ1 |
| LTDC_R3 | PJ2 |
| LTDC_R4 | PJ3 |
| LTDC_R5 | PJ4 |
| LTDC_R6 | PJ5 |
| LTDC_R7 | PJ6 |
| LTDC_G0 | PJ7 |
| LTDC_G1 | PJ8 |
| LTDC_G2 | PJ9 |
| LTDC_G3 | PG10 |
| LTDC_G4 | PH15 |
| LTDC_G5 | PH4 |
| LTDC_G6 | PK1 |
| LTDC_G7 | PK2 |
| LTDC_B0 | PJ12 |
| LTDC_B1 | PJ13 |
| LTDC_B2 | PJ14 |
| LTDC_B3 | PJ15 |
| LTDC_B4 | PK3 |
| LTDC_B5 | PK4 |
| LTDC_B6 | PK5 |
| LTDC_B7 | PK6 |
| LTDC_HSYNC | PI12 |
| LTDC_VSYNC | PI13 |
| LTDC_CLK | PI14 |
| LTDC_DE | PK7 |
| LCD_BL | PH6 |

## SDRAM FMC

### 地址线

| 信号 | MCU 引脚 | 信号 | MCU 引脚 |
| --- | --- | --- | --- |
| FMC_A0 | PF0 | FMC_A7 | PF13 |
| FMC_A1 | PF1 | FMC_A8 | PF14 |
| FMC_A2 | PF2 | FMC_A9 | PF15 |
| FMC_A3 | PF3 | FMC_A10 | PG0 |
| FMC_A4 | PF4 | FMC_A11 | PG1 |
| FMC_A5 | PF5 | FMC_A12 | PG2 |
| FMC_A6 | PF12 |  |  |

### 数据线

| 信号 | MCU 引脚 | 信号 | MCU 引脚 |
| --- | --- | --- | --- |
| FMC_D0 | PD14 | FMC_D8 | PE11 |
| FMC_D1 | PD15 | FMC_D9 | PE12 |
| FMC_D2 | PD0 | FMC_D10 | PE13 |
| FMC_D3 | PD1 | FMC_D11 | PE14 |
| FMC_D4 | PE7 | FMC_D12 | PE15 |
| FMC_D5 | PE8 | FMC_D13 | PD8 |
| FMC_D6 | PE9 | FMC_D14 | PD9 |
| FMC_D7 | PE10 | FMC_D15 | PD10 |

### 控制信号

| 信号 | MCU 引脚 | 信号 | MCU 引脚 |
| --- | --- | --- | --- |
| FMC_BA0 | PG4 | FMC_BA1 | PG5 |
| FMC_NBL0 | PE0 | FMC_NBL1 | PE1 |
| FMC_SDCLK | PG8 | FMC_SDCKE0 | PH2 |
| FMC_SDNE0 | PH3 | FMC_SDNCAS | PG15 |
| FMC_SDNRAS | PF11 | FMC_SDNWE | PC0 |
