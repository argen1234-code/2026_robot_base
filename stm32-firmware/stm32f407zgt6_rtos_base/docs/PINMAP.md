# STM32F407ZGT6 开发板引脚 ↔ PCB 网络对照

> 来源：`Netlist_F407开发板_1_2026-11-16.tel`（Protel/Telesis 网表）。
> 本文按**网络连接关系**整理，不是 PCB 坐标图；器件位号、封装来自网表 `$PACKAGES` 段。
> MCU 为 **U1：STM32F407ZGT6，LQFP-144**，下表 1–144 为封装引脚号。

## 板级资源速览

| 位号 | 类型/封装 | 主要连接与作用 |
|---|---|---|
| U1 | LQFP-144 | STM32F407ZGT6 主控 |
| X1 / U2 | 8MHz 晶振 / 32.768kHz 振荡器 | PH0/PH1 HSE；PC14/PC15 LSE |
| IC5 | SOP8 | CAN 收发器；PB8=RX、PB9=TX，CANH/CANL 到 FPC7 |
| FPC7 | 2P + 2 屏蔽脚 | CAN 接口；R33 120Ω 终端、D172/D173 TVS、C236/C237 |
| FPC1 | 10P + 2 屏蔽脚 | SPI LCD；PB3=SCK、PB5=MOSI、PD11=CS、PD12=DC、PD13=BL |
| USB1 | USB-C 6P×2 | PA11=D-、PA12=D+；R7/R8 5.1k CC 下拉，R9 1.5k D+ 上拉 |
| H16 | 4P 排针 | 5V/GND/PD5/PD6（USART2） |
| H17 | 4P 排针 | 5V/GND/PB10/PB11（USART3） |
| H18 | 4P 排针 | 5V/GND/PB6/PB7（I2C1/USART1 重映射） |
| H19 | 4P 排针 | 5V/GND/PA9/PA10（USART1） |
| H21 | 8P 排针 | SWD + 串口：SWDIO/SWCLK/NRST/3V3/5V/GND/PA9/PA10 |
| CN3 | 5P 连接器 | 5V/GND/PC6/PC7（USART6；丝印 RX/TX） |
| U30 | 5P 连接器 | GND/PC10/PC11/PC12/PD2（SDIO 信号） |
| H26 | 4P 排针 | PE9/PE11/PE13/PE14（TIM1_CH1–CH4） |
| H27 | 4P 排针 | PD15/PG2/PG3/PG4 |
| H28 | 4P 排针 | GND/5V/PF0(SDA)/PF1(SCL)（I2C2） |
| CN7 | 3P 连接器 | 5V/GND/PA5（ADC1_IN5） |
| H29–H32 | 4×3P 排针 | 6V/GND/PWM，经 U31/U32/U33/U36 光耦隔离到 PF8/PF9/PF7/PF10 |
| SW1 / SW2 / SW3 / SW14 | 轻触/拨动开关 | PA15 按键 / 复位 / BOOT0 / PD14 按键 |
| LED1/2/12/14/15/16/17/18 | 0603 LED | 3V3、5V、6V、VBATA 指示；LED2=PC13，LED15=PG0 |
| CN8 / CN9 | XT30 | 电池输入（CN8）/ 经 Q7 切换的电池输出（CN9） |
| U34 / U35 | 降压芯片 + L55/L56 | VCCBAT→5V / VCCBAT→6V |
| U3 | SOT-23-5 + L2 | 5V→3V3 |
| Q6/Q7 + SW15/SW16 | TO-252 + 开关 | Q6：VBATA→VCCBAT 主电源；Q7：VBATA→CN9 输出 |
| U31/U32/U33/U36 | 4P 光耦 | PF7–PF10 → H29–H32 隔离 PWM |

## 电源树

```text
CN8 (XT30) ── VBATA
   ├─ Q6（SW15/SW16 控制）── VCCBAT
   │     ├─ U34 + L55 ── 5V ── U3 + L2 ── 3V3
   │     └─ U35 + L56 ── 6V
   └─ Q7（SW16 控制）── CN9 (XT30) 电池输出

3V3：MCU、LCD、光耦、FPC1、H21 等
5V ：CAN 收发器 IC5、USB、H16–H19、H21、H28、CN3、CN7、LED14/17
6V ：H29–H32 隔离 PWM 接口、LED12/18
```

## 完整引脚表（U1，1–144）

| 引脚 | MCU 引脚 | 网络 | PCB 连接 / 功能 |
|---:|---|---|---|
| 1 | PE2 | `PE2` | 未连接（仅 MCU） |
| 2 | PE3 | `PE3` | 未连接（仅 MCU） |
| 3 | PE4 | `PE4` | 未连接（仅 MCU） |
| 4 | PE5 | `PE5` | 未连接（仅 MCU） |
| 5 | PE6 | `PE6` | 未连接（仅 MCU） |
| 6 | VBAT | `VBAT` | 仅 MCU；未接备份电池/3V3（见注意事项） |
| 7 | PC13 / ANTI_TAMP | `LED_PC13` | LED2（绿）→R2 2k→3V3；PC13 低电平点亮 |
| 8 | PC14 / OSC32_IN | `OS_32K_IN` | 32.768kHz 晶振 U2.2 + C3 10pF |
| 9 | PC15 / OSC32_OUT | `OS_32K_OUT` | 32.768kHz 晶振 U2.1 + C4 10pF |
| 10 | PF0 | `PF0` | H28.SDA；I2C2_SDA |
| 11 | PF1 | `PF1` | H28.SCL；I2C2_SCL |
| 12 | PF2 | `PF2` | 未连接（仅 MCU） |
| 13 | PF3 | `PF3` | 未连接（仅 MCU） |
| 14 | PF4 | `PF4` | 未连接（仅 MCU） |
| 15 | PF5 | `PF5` | 未连接（仅 MCU） |
| 16 | VSS | `GND` | GND 网络 |
| 17 | VDD | `3V3` | 3V3 电源网络 |
| 18 | PF6 | `PF6` | 未连接（仅 MCU） |
| 19 | PF7 | `PF7` | R59→U33 光耦→H31.PWM（隔离 PWM 输出） |
| 20 | PF8 | `PF8` | R55→U31 光耦→H29.PWM（隔离 PWM 输出） |
| 21 | PF9 | `PF9` | R57→U32 光耦→H30.PWM（隔离 PWM 输出） |
| 22 | PF10 | `PF10` | R78→U36 光耦→H32.PWM（隔离 PWM 输出） |
| 23 | PH0 / OSC_IN | `OS_8M_IN` | 8MHz 晶振 X1.1 + C12 10pF |
| 24 | PH1 / OSC_OUT | `OS_8M_OUT` | 8MHz 晶振 X1.3 + C13 10pF |
| 25 | NRST | `RST` | NRST；R4 10k 上拉、C21 100nF、SW2 复位、R6 1k→H21.5 |
| 26 | PC0 | `PC0` | 未连接（仅 MCU） |
| 27 | PC1 | `PC1` | 未连接（仅 MCU） |
| 28 | PC2 | `PC2` | 未连接（仅 MCU） |
| 29 | PC3 | `PC3` | 未连接（仅 MCU） |
| 30 | VDD | `IO30` | VDD；经 R23 0Ω 接 3V3 |
| 31 | VSSA | `GND` | GND 网络 |
| 32 | VREF+ | `VDDA` | VDDA / VREF+；L1 磁珠 + C5/C6 10uF 滤波 |
| 33 | VDDA | `VDDA` | VDDA / VREF+；L1 磁珠 + C5/C6 10uF 滤波 |
| 34 | PA0 / WKUP | `PA0` | 未连接（仅 MCU） |
| 35 | PA1 | `PA1` | 未连接（仅 MCU） |
| 36 | PA2 | `PA2` | 未连接（仅 MCU） |
| 37 | PA3 | `PA3` | 未连接（仅 MCU） |
| 38 | VSS | `GND` | GND 网络 |
| 39 | VDD | `3V3` | 3V3 电源网络 |
| 40 | PA4 | `PA4` | 未连接（仅 MCU） |
| 41 | PA5 | `PA5` | CN7.3；ADC1_IN5（模拟输入/电位器） |
| 42 | PA6 | `PA6` | 未连接（仅 MCU） |
| 43 | PA7 | `PA7` | 未连接（仅 MCU） |
| 44 | PC4 | `PC4` | 未连接（仅 MCU） |
| 45 | PC5 | `PC5` | 未连接（仅 MCU） |
| 46 | PB0 | `PB0` | 未连接（仅 MCU） |
| 47 | PB1 | `PB1` | 未连接（仅 MCU） |
| 48 | PB2 | `BOOT1` | PB2/BOOT1；R16 10k 下拉 |
| 49 | PF11 | `PF11` | 未连接（仅 MCU） |
| 50 | PF12 | `PF12` | 未连接（仅 MCU） |
| 51 | VSS | `GND` | GND 网络 |
| 52 | VDD | `3V3` | 3V3 电源网络 |
| 53 | PF13 | `PF13` | 未连接（仅 MCU） |
| 54 | PF14 | `PF14` | 未连接（仅 MCU） |
| 55 | PF15 | `PF15` | 未连接（仅 MCU） |
| 56 | PG0 | `PG0` | R65 1k→LED15（红）→GND；高电平点亮 |
| 57 | PG1 | `PG1` | 未连接（仅 MCU） |
| 58 | PE7 | `PE7` | 未连接（仅 MCU） |
| 59 | PE8 | `PE8` | 未连接（仅 MCU） |
| 60 | PE9 | `PE9` | H26.1；TIM1_CH1 |
| 61 | VSS | `GND` | GND 网络 |
| 62 | VDD | `3V3` | 3V3 电源网络 |
| 63 | PE10 | `PE10` | 未连接（仅 MCU） |
| 64 | PE11 | `PE11` | H26.2；TIM1_CH2 |
| 65 | PE12 | `PE12` | 未连接（仅 MCU） |
| 66 | PE13 | `PE13` | H26.3；TIM1_CH3 |
| 67 | PE14 | `PE14` | H26.4；TIM1_CH4 |
| 68 | PE15 | `PE15` | 未连接（仅 MCU） |
| 69 | PB10 | `PB10` | H17.3；USART3_TX / I2C2_SCL |
| 70 | PB11 | `PB11` | H17.4；USART3_RX / I2C2_SDA |
| 71 | VCAP_1 | VCAP_1 (`$1N2479`) | VCAP_1；C28 2.2uF→GND |
| 72 | VDD | `3V3` | 3V3 电源网络 |
| 73 | PB12 | `PB12` | 未连接（仅 MCU） |
| 74 | PB13 | `PB13` | 未连接（仅 MCU） |
| 75 | PB14 | `PB14` | 未连接（仅 MCU） |
| 76 | PB15 | `PB15` | 未连接（仅 MCU） |
| 77 | PD8 | `PD8` | 未连接（仅 MCU） |
| 78 | PD9 | `PD9` | 未连接（仅 MCU） |
| 79 | PD10 | `PD10` | 未连接（仅 MCU） |
| 80 | PD11 | `LCD_CS-PD11` | FPC1.6 LCD_CS |
| 81 | PD12 | `LCD_DC-PD12` | FPC1.5 LCD_DC |
| 82 | PD13 | `LCD_BL-PD13` | FPC1.4 LCD_BL |
| 83 | VSS | `GND` | GND 网络 |
| 84 | VDD | `3V3` | 3V3 电源网络 |
| 85 | PD14 | `PD14` | R54 100Ω→SW14→GND；PD14 按键 |
| 86 | PD15 | `PD15` | H27.1 |
| 87 | PG2 | `PG2` | H27.2 |
| 88 | PG3 | `PG3` | H27.3 |
| 89 | PG4 | `PG4` | H27.4 |
| 90 | PG5 | `PG5` | 未连接（仅 MCU） |
| 91 | PG6 | `PG6` | 未连接（仅 MCU） |
| 92 | PG7 | `PG7` | 未连接（仅 MCU） |
| 93 | PG8 | `PG8` | 未连接（仅 MCU） |
| 94 | VSS | `GND` | GND 网络 |
| 95 | VDD | `3V3` | 3V3 电源网络 |
| 96 | PC6 | `PC6` | CN3.RX（网络名）；USART6_TX |
| 97 | PC7 | `PC7` | CN3.TX（网络名）；USART6_RX |
| 98 | PC8 | `PC8` | 未连接（仅 MCU） |
| 99 | PC9 | `PC9` | 未连接（仅 MCU） |
| 100 | PA8 | `PA8` | 未连接（仅 MCU） |
| 101 | PA9 | `USART1-TX_PA9` | H19.3、H21.3；USART1_TX |
| 102 | PA10 | `USART1-RX_PA10` | H19.4、H21.1；USART1_RX |
| 103 | PA11 | `USB_D-` | USB1 D-（PA11） |
| 104 | PA12 | `USB_D+` | USB1 D+（PA12）；R9 1.5k 上拉到 3V3 |
| 105 | PA13 | `SWDIO_PA13` | H21.4 SWDIO |
| 106 | VCAP_2 | VCAP_2 (`$1N2482`) | VCAP_2；C29 2.2uF→GND |
| 107 | VSS | `GND` | GND 网络 |
| 108 | VDD | `3V3` | 3V3 电源网络 |
| 109 | PA14 | `SWCLK_PA14` | H21.2 SWCLK |
| 110 | PA15 | `KEY_PA15` | SW1 按键→GND；PA15 低有效 |
| 111 | PC10 | `PC10` | U30.2 SDIO_D2 |
| 112 | PC11 | `PC11` | U30.3 SDIO_D3 |
| 113 | PC12 | `PC12` | U30.4 SDIO_CK |
| 114 | PD0 | `PD0` | 未连接（仅 MCU） |
| 115 | PD1 | `PD1` | 未连接（仅 MCU） |
| 116 | PD2 | `PD2` | U30.5 SDIO_CMD |
| 117 | PD3 | `PD3` | 未连接（仅 MCU） |
| 118 | PD4 | `PD4` | 未连接（仅 MCU） |
| 119 | PD5 | `PD5` | H16.3；USART2_TX |
| 120 | VSS | `GND` | GND 网络 |
| 121 | VDD | `3V3` | 3V3 电源网络 |
| 122 | PD6 | `PD6` | H16.4；USART2_RX |
| 123 | PD7 | `PD7` | 未连接（仅 MCU） |
| 124 | PG9 | `PG9` | 未连接（仅 MCU） |
| 125 | PG10 | `PG10` | 未连接（仅 MCU） |
| 126 | PG11 | `PG11` | 未连接（仅 MCU） |
| 127 | PG12 | `PG12` | 未连接（仅 MCU） |
| 128 | PG13 | `PG13` | 未连接（仅 MCU） |
| 129 | PG14 | `PG14` | 未连接（仅 MCU） |
| 130 | VSS | `GND` | GND 网络 |
| 131 | VDD | `3V3` | 3V3 电源网络 |
| 132 | PG15 | `PG15` | 未连接（仅 MCU） |
| 133 | PB3 | `SPI3_SCK-PB3` | FPC1.2 LCD_SPI_SCK |
| 134 | PB4 | `PB4` | 未连接（仅 MCU） |
| 135 | PB5 | `SPI3_MOSI-PB5` | FPC1.7 LCD_SPI_MOSI（无 MISO） |
| 136 | PB6 | `PB6` | H18.3；I2C1_SCL / USART1_TX(重映射) |
| 137 | PB7 | `PB7` | H18.4；I2C1_SDA / USART1_RX(重映射) |
| 138 | BOOT0 | `BOOT0` | BOOT0；R5 10k 下拉、SW3 拨到 3V3 |
| 139 | PB8 | `PB8` | IC5.4 CAN_RX；CAN1_RX |
| 140 | PB9 | `PB9` | IC5.1 CAN_TX；CAN1_TX |
| 141 | PE0 | `PE0` | 未连接（仅 MCU） |
| 142 | PE1 | `PE1` | 未连接（仅 MCU） |
| 143 | PDR_ON | `3V3` | 3V3 电源网络 |
| 144 | VDD | `3V3` | 3V3 电源网络 |

## 功能分组

### 时钟、复位、启动
- **HSE 8MHz**：X1 + C12/C13，PH0/PH1（23/24）
- **LSE 32.768kHz**：U2 + C3/C4，PC14/PC15（8/9）
- **NRST**：25，R4 10k 上拉、C21 100nF、SW2 复位、R6 1k 到 H21.5
- **BOOT0**：138，R5 10k 下拉、SW3 选择 3V3；**BOOT1/PB2**：48，R16 10k 下拉

### 调试与串口
- **SWD**：PA13=SWDIO(105)、PA14=SWCLK(109)，引到 H21
- **USART1**：PA9=TX(101)、PA10=RX(102)，引到 H19 和 H21
- **USART2**：PD5=TX(119)、PD6=RX(122)，引到 H16
- **USART3**：PB10=TX(69)、PB11=RX(70)，引到 H17
- **USART6**：PC6=TX(96)、PC7=RX(97)，引到 CN3（丝印 RX/TX）
- **I2C1**：PB6=SCL(136)、PB7=SDA(137)，引到 H18
- **I2C2**：PF0=SDA(10)、PF1=SCL(11)，引到 H28

### USB
- **USB OTG FS**：PA11=D-(103)、PA12=D+(104)；USB1 为 USB-C
- CC1/CC2 经 R8/R7 5.1k 下拉；D+ 经 R9 1.5k 上拉到 3V3（全速设备）

### CAN
- **CAN1**：PB8=RX(139)→IC5.4、PB9=TX(140)→IC5.1
- IC5 CANH/CANL → FPC7；R33 120Ω 终端，D172/D173 TVS，C236/C237 10pF

### LCD（SPI）
- FPC1：PB3=SCK(133)、PB5=MOSI(135)、PD11=CS(80)、PD12=DC(81)、PD13=BL(82)
- 只有写方向，没有 MISO；3V3/GND 由 FPC1 供电

### SDIO / 存储接口
- U30 5P：PC10=D2(111)、PC11=D3(112)、PC12=CK(113)、PD2=CMD(116)，pin1=GND
- 网表中没有 D0/D1；若接 4-bit SD 卡需核对模块实际引脚

### 隔离 PWM 输出
- PF8(20)→R55→U31→R56→H29.PWM
- PF9(21)→R57→U32→R58→H30.PWM
- PF7(19)→R59→U33→R60→H31.PWM
- PF10(22)→R78→U36→R79→H32.PWM
- H29–H32 均为 6V/GND/PWM 三脚接口；光耦副边由 3V3 供电

### 按键与 LED
- SW1：PA15(110) 到 GND；SW14：PD14(85) 经 R54 100Ω 到 GND；SW2：NRST 复位；SW3：BOOT0
- LED2（绿）：PC13(7) 低电平点亮（3V3→R2 2k→LED2→PC13）
- LED15（红）：PG0(56) 高电平点亮（PG0→R65 1k→LED15→GND）
- 其余 LED 为电源指示：LED1=3V3、LED14/17=5V、LED12/18=6V、LED16=VBATA

### 扩展排针
- H26：PE9(60)=TIM1_CH1、PE11(64)=TIM1_CH2、PE13(66)=TIM1_CH3、PE14(67)=TIM1_CH4
- H27：PD15(86)、PG2(87)、PG3(88)、PG4(89)
- CN7：PA5(41)=ADC1_IN5

## 未连接/未引出引脚

- **PA**：PA0(34)、PA1(35)、PA2(36)、PA3(37)、PA4(40)、PA6(42)、PA7(43)、PA8(100)
- **PB**：PB0(46)、PB1(47)、PB12(73)、PB13(74)、PB14(75)、PB15(76)、PB4(134)
- **PC**：PC0(26)、PC1(27)、PC2(28)、PC3(29)、PC4(44)、PC5(45)、PC8(98)、PC9(99)
- **PD**：PD8(77)、PD9(78)、PD10(79)、PD0(114)、PD1(115)、PD3(117)、PD4(118)、PD7(123)
- **PE**：PE2(1)、PE3(2)、PE4(3)、PE5(4)、PE6(5)、PE7(58)、PE8(59)、PE10(63)、PE12(65)、PE15(68)、PE0(141)、PE1(142)
- **PF**：PF2(12)、PF3(13)、PF4(14)、PF5(15)、PF6(18)、PF11(49)、PF12(50)、PF13(53)、PF14(54)、PF15(55)
- **PG**：PG1(57)、PG5(90)、PG6(91)、PG7(92)、PG8(93)、PG9(124)、PG10(125)、PG11(126)、PG12(127)、PG13(128)、PG14(129)、PG15(132)

## 硬件注意事项

1. **VBAT（6）在网表中只连 U1**，没有接 3V3 或备份电池；若不使用 RTC 备份域，建议硬件上把 VBAT 接到 3V3，避免备份域悬空。
2. **VDD 引脚 30** 的网络名为 `IO30`，实际经 R23 0Ω 接 3V3；不是普通 GPIO。
3. **PC13 LED 低有效**，PG0 LED 高有效，软件点灯极性不要写反。
4. **PF7–PF10 不是直接排针**，都经过光耦；H29–H32 的 PWM 方向是 MCU→光耦→外部接口。
5. **LCD 是单向 SPI**（SCK/MOSI/CS/DC/BL），没有 MISO，读取 LCD ID 需要另外的方案。
6. **CAN 终端电阻 R33 120Ω 固定焊在 FPC7 侧**；如果 FPC7 外接的线束末端还有 120Ω，需要确认是否重复终端。
7. **USB-C 是设备口**：CC1/CC2 各 5.1k 下拉，D+ 1.5k 上拉；不要当 Host 口使用。
8. **U30 只引出了 SDIO 的 D2/D3/CK/CMD**，没有 D0/D1；若接 SD 卡请先核对座子定义，必要时改用 1-bit 模式并确认 D0 的走向。
9. **CN3 的 RX/TX 是网络名**，对应 MCU 的 PC6=USART6_TX、PC7=USART6_RX；接线时以底板丝印和实际交叉为准。
10. **电源域**：6V 只给 H29–H32 和 LED12/18；5V 给 CAN/USB/排针；3V3 由 5V 再降压得到。外接大电流负载前先核对 U34/U35/U3 的额定电流。
11. **PDR_ON（143）接 3V3**，使用内部上电复位；如果改成低电平需要外加复位电路，不要直接悬空。
12. **CN9 的 pin1 经 FB2 到 GND、pin2 经 Q7 到 VBATA**，是受开关控制的电池输出；CN8 才是电池输入。

---

*本表由网表 `Netlist_F407开发板_1_2026-11-16.tel` 与 STM32CubeMX `STM32F407Z(E-G)Tx` 引脚库交叉生成；未连接的 GPIO 仅代表网表中没有其它器件，不代表 PCB 上没有测试点/焊盘。*
