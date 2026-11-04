# H7 的 Cache：通俗入门 + 避坑清单

> 这份文档写给"被 Cache 坑过、但没时间读 1000 页手册"的人。
> 全部用大白话 + 本项目踩过的真实例子；文末有一页速查表和一个 5 分钟自检法。

---

## 1. 为什么会有 Cache

CPU 跑 200 MHz（一个周期 5 ns），但 Flash 和 SRAM 的响应要几十纳秒。如果每条指令、
每次读变量都直接去内存拿，CPU 大部分时间都在等。

Cache 就是"放在 CPU 旁边的小书架"：读一次内存，顺手把**周围 32 字节**都搬到书架上；
下次再读同一片区域，直接从书架拿（1~2 个周期），不用下楼去仓库。

- **I-Cache**（指令缓存）：放"代码"。取指令命中就不用等 Flash。
- **D-Cache**（数据缓存）：放"变量/数组"。读变量命中就不用等 SRAM。

H7 上两个都是 **16 KB、4 路组相联、32 字节一行**（16 KB ÷ (32×4) = 128 组）。

## 2. 三个必须记住的词

| 词 | 大白话 |
|---|---|
| **cache line（行）** | 书架的最小格子，32 字节。你只读 1 字节，它也会搬 32 字节上来 |
| **命中 / 缺失** | 要的东西在书架上 / 不在，得下楼拿 |
| **脏（dirty）** | 你在草稿（cache）上改了数据，仓库（内存）里还是旧的 |

**关键事实：CPU 读写的是"书架"，不是"仓库"。** 只要 D-Cache 开着，
CPU 看到的和内存里的就可能不一致 —— 这就是所有 Cache 坑的根源。

## 3. H7 上哪些内存会被缓存

| 区域 | 地址 | 会被 D-Cache 缓存吗 | 能被 DMA 访问吗 | 本项目用途 |
|---|---|---|---|---|
| ITCM / DTCM | `0x00000000` / `0x20000000` | **不会**（TCM 不经过 Cache） | **不能** | 栈、堆、`g_debug`、所有变量 |
| AXI SRAM (D1) | `0x24000000` | 会（默认映射是 Normal 可缓存） | 能 | 未使用 |
| D2 SRAM1/2/3 | `0x30000000` 起 | 一般不经过 D-Cache | 能 | WS2812 波形缓冲 |
| 外设 | `0x40000000` 以上 | **不会**（Device 属性） | — | FDCAN/I²C/TIM/DMA 寄存器 |
| 外部 SDRAM/FMC | `0xC0000000` 起 | 取决于 FMC/MPU 配置 | 能 | 未使用 |

两条铁律：

1. **TCM 永远不会被 Cache 污染**（因为访问根本不进 Cache），但它也永远不能被 DMA 访问。
   所以"DMA 缓冲放 DTCM"是错的，"DMA 缓冲放 D2 SRAM1"是本项目的选择。
2. **外设寄存器区永远不被缓存**，所以写寄存器不用担心 Cache。

## 4. D-Cache 是怎么骗你的

### 4.1 写：CPU 写进 cache，内存还是旧的

```
CPU: buf[0] = 0x55      →  写进 cache（脏行），内存里还是 0x00
DMA: 从内存读 buf        →  读到 0x00（旧数据）
```

**症状**：串口发出去的是旧数据、灯带颜色不对、CAN 发出去的帧内容错。
**修法**：DMA 启动**之前**把这块缓冲"刷回内存" → `SCB_CleanDCache_by_Addr(buf, len)`。

### 4.2 读：DMA 写了内存，CPU 读到旧副本

```
DMA: 把收到的数据写进 buf（内存里是新的）
CPU: 读 buf             →  如果这 32 字节之前被缓存过，读到的是旧副本
```

**症状**：串口收到的全是上一次的内容、ADC 采样值不变、网卡收包错乱。
**修法**：DMA 完成**之后**把这块缓冲"作废" → `SCB_InvalidateDCache_by_Addr(buf, len)`。

### 4.3 一个必须知道的前提

`*_by_Addr()` 系列要求 **地址 32 字节对齐、长度向上取整到 32 字节**，
否则会误伤相邻数据（把别人的脏数据丢掉/把别人的新数据覆盖）。
最稳的做法：给 DMA 缓冲加 `__attribute__((aligned(32)))`，长度也凑成 32 的倍数。

## 5. 为什么"关掉 Cache"也会崩（本项目的真实事故）

老版本 FreeRTOS 一上电就 HardFault，`cfsr=0x400`（IMPRECISERR），`tick=0`。
凶手是这行 CMSIS 代码（`Drivers/CMSIS/Include/cachel1_armv7.h`）：

```c
SCB->DCCIMVAC = (uint32_t)&locals;   /* locals 是栈上的局部变量 */
```

它在"关 D-Cache"的过程里，顺手对**栈地址**做一次"按地址 clean+invalidate"。
而本工程的栈在 **DTCM**（`0x2000xxxx`），TCM 地址不是可缓存区域 ——
**对不可缓存地址做按地址维护，ARM 规定是未定义行为**，这颗芯片上的表现就是
写缓冲里冒出一个总线错误，然后升级成 HardFault。

三个条件凑齐才炸，缺一个都没事：

1. 调用了 `SCB_DisableDCache()`（很多工程根本不调，因为复位后本来就是关的）；
2. 用的是 ARMCC（它不定义 `__OPTIMIZE__`，那句"仅无优化时编译"的分支永远为真）；
3. 栈在 DTCM（如果栈在 AXI SRAM，同一句只是白做，不报错）。

**教训：Cache 维护不是"多刷一次总没坏处"，刷错地址会直接打死 CPU。**

## 6. 三种正确姿势（按需求选）

### 姿势 A：全关（本项目）

```c
/* 什么都不做：复位后 CCR.IC = CCR.DC = 0 */
```
- 优点：绝对不会有 Cache 一致性/维护地址问题；DMA 缓冲随便放（只要 DMA 能访问）。
- 代价：CPU 跑 200 MHz 但从 Flash 取指要等，实测性能大约损失 30%~50%（视代码）。
- 适合：控制类、实时性靠"确定性"而不是"平均速度"、代码量不大、DMA 多。

### 姿势 B：局部 non-cacheable（推荐给"想开又怕坑"的人）

用 MPU 把 **DMA 缓冲区**所在的那一段设成 non-cacheable，其余照常缓存。
这样 DMA 缓冲天然一致，不需要任何 `*_by_Addr` 调用，也没有"忘记刷"的风险。

```c
/* 例：把 D2 SRAM1 的一段（0x30000000，16 KB）设为 non-cacheable */
MPU->RBAR = 0x30000000 | MPU_RBAR_VALID_Msk | 0;   /* region 0 */
MPU->RASR = MPU_RASR_ENABLE_Msk | (0x0D << MPU_RASR_SIZE_Pos) /* 16KB */
          | MPU_RASR_AP_FULL_ACCESS | /* TEX/S/C/B 全 0 = non-cacheable, shareable */
            MPU_RASR_XN_Msk; /* 可选：不许执行 */
```

### 姿势 C：全开 + 严格维护

- 所有 DMA 缓冲必须 `aligned(32)`；
- 发送前 clean、接收后 invalidate；
- 中途改缓冲内容要重新 clean；
- 中断里改的缓冲、DMA 正在读的缓冲，更要小心（最好用双缓冲 + 只在切换点维护）。

姿势 C 最快，也最容易出"偶发、难复现"的问题。比赛项目里如果没有实测需求，建议 A 或 B。

## 7. 一页速查表

| 现象 | 最可能的原因 | 怎么查 / 怎么改 |
|---|---|---|
| 一上电就 HardFault，`cfsr` 有 0x400（IMPRECISERR） | 对 TCM/外设做了按地址 cache 维护 | 搜 `SCB_.*DCache`、`DCCMVAC/DCCIMVAC/DCIMVAC`，删掉或改成只对可缓存区 |
| DMA 发出去的数据是旧的 | 写缓冲还在 cache 里 | 发之前 `SCB_CleanDCache_by_Addr()` |
| DMA 收的数据 CPU 读到旧的 | cache 里有旧副本 | 收完 `SCB_InvalidateDCache_by_Addr()` |
| 改一行无关代码，问题消失/出现 | 时序/对齐变了 | 先怀疑 cache line 对齐与临界区，而不是"玄学" |
| 用调试器 Watch 看到的变量和串口打印的不一样 | 调试器读的是内存，CPU 读的是 cache | 加了 cache 就要意识到这一点；以 CPU 视角（串口/内存屏障后读）为准 |
| 在线升级/写 Flash 后跑飞 | I-Cache 里是旧指令 | 写完 `SCB_InvalidateICache()`，必要时再 clean D-Cache |
| 从 Bootloader 跳到 App 后异常 | 跳转前没清 cache/没设 VTOR | 跳转前 `SCB_CleanInvalidateDCache()` + 关 cache + 设 VTOR |
| 开了 cache 后偶发花屏/错帧 | DMA 缓冲与 cache 打架 | 姿势 B：把 DMA 缓冲设 non-cacheable |

## 8. H7 特有的坑（和 cache 相关但不完全是 cache）

1. **DTCM 不能被 DMA 访问**：把 DMA 缓冲放 `0x20000000` 会得到"永远不变的缓冲"或总线错误。
2. **D2 SRAM1/2/3 的时钟默认关闭**：链接脚本里若把它当普通 RAM 用，CRT 在时钟使能前清零会 HardFault。
   本项目的做法：scatter 里 `RW_LED_DMA ... UNINIT`，`BspWs2812_Init()` 里先开时钟再写数据。
3. **AXI SRAM 有 ECC**：上电后未初始化的 ECC 会在第一次读时产生错误，需要先整段写一遍。
4. **地址 0 是 ITCM，不是 Flash 别名**（和 F4 不同）：`VTOR` 不设就会在第一个中断时跑飞。
5. **调试器读写内存绕过 D-Cache**：Keil 的 Watch/Memory 看到的是内存真值，不是 CPU 当前看到的。
6. **`SCB_DisableDCache()` 在 ARMCC 下永远包含那句 DCCIMVAC**（见第 5 节），
   所以"复位后先关一次 cache 图个安心"是危险动作 —— 复位后本来就是关的，不需要关。
7. **DWT->CYCCNT 与 cache 无关**：本项目用它做 CPU 占用统计，开不开 cache 都准。

## 9. 5 分钟自检法（换新板/新工程时值得做一次）

1. **看复位后的 CCR**：在 `main` 最前面读 `SCB->CCR`，`bit16(DC)/bit17(IC)` 应该是 0。
2. **搜代码**：全工程 grep `DCache|ICache|DCCMVAC|DCIMVAC|DCMVAC|ICIALLU`。
   只要出现 `SCB_DisableDCache()`，先确认它对栈地址做了什么（第 5 节）。
3. **看链接脚本**：RW/ZI 在 DTCM（`0x20000000`）还是 AXI SRAM（`0x24000000`）？
   栈在 DTCM 时，任何"按地址维护"都别做。
4. **看 DMA 缓冲**：用 `grep -n "DMA"` 找到缓冲定义，确认它在 D2 SRAM 或已被 MPU 设 non-cacheable。
5. **做一次"关 cache / 开 cache"的对比测试**：同样的数据通路跑两遍，
   数据错 → cache 一致性问题；数据对但偶发 HardFault → 维护地址问题。

## 10. 本项目现在的做法（可以直接抄）

- I-Cache / D-Cache **都不开**，源码里**零** cache 维护调用；反汇编里 `MCR/MCRR` 指令数为 0。
- `SCB->CCR` 只做只读记录：`g_debug.system.ccr` 正常是 `0x40200`（只有 STKALIGN 和 BP），
  上位机连上就能看到"确实没开"。
- WS2812 波形缓冲放 `RW_LED_DMA 0x30000000`，`BSP/bsp_ws2812.c` 里先检查地址范围、
  先开 SRAM 时钟、再写数据，避免"清零时 HardFault"。
- 如果哪天真要开 D-Cache：按第 6 节姿势 B 做（MPU 把 DMA 区设 non-cacheable），
  并且在 `Docs/02_架构与维护.md` 的"Cache"条目里同步更新，别让下一个人踩回头路。

---

## 附：一句话总结

> **Cache 的坑只有两个：①"以为改了内存，其实只改了书架"（一致性）；
> ②"对着书架外的地址做维护"（未定义行为，会 HardFault）。
> 本工程选择"不开 cache"，就是为了从根上不碰这两件事。**
