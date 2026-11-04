# Cache 为什么会让这个工程 HardFault（含实物复现实验）

## 1. 历史现象

改造前的 FreeRTOS 版一上电就停：`stage=1`、`error=0x48415244`（HardFault）、`tick=0`、`cfsr=0x400`、`hfsr=0x40000000`。当时只能推断"故障在最早期 Cache/HAL 初始化段"，没有拿到 PC/FAR，所以无法确定是哪一条指令。

## 2. 本轮实测复现（2026-10-22 12:37）

在**当前已能正常运行的工程**里，只加回一行旧代码（`Core/Src/main.c`，VTOR 设置之后、HAL_Init 之前）：

```c
SCB_DisableDCache();
```

重新编译、Keil 下载、SWD 读 `g_debug.system`：

| 字段 | 正常固件 | 加回 `SCB_DisableDCache()` |
|---|---|---|
| `boot_stage` | 9 | **1**（停在这一段） |
| `fatal` | 0 | **0x9**（`APP_FATAL_HARDFAULT`） |
| `cfsr` | 0 | **0x400**（IMPRECISERR，非精确数据总线错误） |
| `hfsr` | 0 | **0x40000000**（FORCED） |
| `exception` | 0 | **3**（HardFault） |
| `tick_ms` / `loop_heartbeat` | 持续增长 | **0 / 0** |
| `vtor` | `0x08000000` | `0x08000000`（正确，排除向量表因素） |
| `ccr` | `0x40200` | `0x40200`（**两个 Cache 本来就都是关的**） |

结论：**故障就是这一行引起的**，与"Cache 有没有开"无关——恰恰是在"关 Cache"的过程中炸的，而且 VTOR 已经正确。

## 3. 机理：为什么"关缓存"会打死自己

项目里的 CMSIS 实现在 `Drivers/CMSIS/Include/cachel1_armv7.h`：

```c
__STATIC_FORCEINLINE void SCB_DisableDCache (void)
{
  struct { uint32_t ccsidr, sets, ways; } locals;   /* ← 栈上局部变量 */
  SCB->CSSELR = 0U;
  __DSB();
  SCB->CCR &= ~(uint32_t)SCB_CCR_DC_Msk;            /* 关 D-Cache */
  __DSB();
  #if !defined(__OPTIMIZE__)
    /* ARM-software/CMSIS_5 issue #620：-O0 下局部变量在栈上，
       关缓存后循环变量可能读到旧值，所以先把 locals 所在 cache line 清掉 */
    SCB->DCCIMVAC = (uint32_t)&locals;              /* ← 出事的就是这一句 */
    __DSB(); __ISB();
  #endif
  ...按 set/way 清整个 D-Cache...
}
```

三个条件同时成立就出事：

1. **-O0 编译**：Keil 工程是 `<Optim>1</Optim>`（= -O0）。该分支的判据是 `!defined(__OPTIMIZE__)`，本机用 ARMCC 5.06 实测：`-O0/-O1/-O2/-O3` 四个等级**都不定义 `__OPTIMIZE__`**（ARMCC 没有 GCC 的这个宏），所以这段代码在这套工具链下**永远被编进去**，不存在"换个优化等级就没了"。
2. **栈在 DTCM**：scatter 把 RW/ZI 全部放在 `0x20000000`（DTCM），`&locals` 就是 `0x2000xxxx`。
3. **对 TCM 地址做按地址维护**：`DCCIMVAC` = "Data Cache Clean and Invalidate by MVA"，它把地址当参数，去查这个地址是否在可缓存区域。Cortex-M7 的 D-Cache 只覆盖"可缓存的 normal memory"（由 MPU/默认映射决定），**TCM 访问根本不经过 D-Cache**。对不可缓存地址做按地址的 cache 维护，ARM 架构不保证结果；在这颗 H7 上表现为写缓冲里出现一次总线错误 → `CFSR.IMPRECISERR`（0x400）。非精确总线错误无法定位到具体指令，硬件直接升级成 HardFault（`HFSR.FORCED`）。

一句话：**这段代码假设"栈所在的内存是可缓存的"，而本工程的栈在 TCM 上，于是"给栈变量洗 cache line"变成了一次非法维护操作。**

同样的代码如果栈在 AXI SRAM（`0x24000000`，可缓存区）上，这一句只是白做一次 cache 维护、不会报错——所以这个坑**只对"栈在 DTCM + 调用 SCB_DisableDCache"的组合成立**，本工程恰好全中。

## 4. 为什么你以前"加一句清缓存就好了"

缓存害人的方式有三种，症状完全不同，别混：

### 4.1 一致性（最常见，"加一句清缓存"治的就是它）

CPU 写数据先进 D-Cache，内存里还是旧值；DMA（WS2812、CAN、UART、ADC、SPI）直接读内存 → 读到旧数据。反过来 DMA 写入新数据，CPU 读到的却是 Cache 里的旧副本。

- 发送前：`SCB_CleanDCache_by_Addr(addr, len)`（把脏行写回内存）
- 接收后：`SCB_InvalidateDCache_by_Addr(addr, len)`（丢掉旧副本）
- 地址和长度必须 32 字节对齐（M7 cache line = 32 B），且**只能对可缓存区域**做。
- 本工程 WS2812 的 `waveform[]` 在 D2 SRAM1（`0x30000000`），Cortex-M7 的 D-Cache 不覆盖 D2 域，所以它天然不会被缓存；但一旦有人把缓冲挪到 DTCM/AXI SRAM 并开着 D-Cache，就必须做维护（DTCM 则根本无法被 DMA 访问，见 `bsp_ws2812.c` 的地址检查）。

### 4.2 在错误的地方做维护（本工程的故障）

对 TCM（`0x00000000` / `0x20000000` 段）、外设区（`0x40000000` 以上）做 `*MVAC/*MVAC` 这类按地址维护，就是第 3 节那种 HardFault。**"多清一次总没坏处"是错的。**

### 4.3 I-Cache

I-Cache 只在"运行中改变指令"时才需要维护：把函数拷到 RAM 执行、在线升级写 Flash、自修改代码。本工程没有这类操作，所以 `SCB_EnableICache()` 本身并不危险（`ICIALLU` 是整 cache 操作、不带地址）。真正危险的是同一段初始化里的 **D-Cache 关闭流程**。

## 5. 本工程现在的规则

- I-Cache、D-Cache 都保持在复位默认的**关闭**状态；源码中没有任何使能/关闭/clean/invalidate 调用（最终镜像反汇编 `MCR/MCRR` 指令数 = 0）。
- `SCB->CCR` 只做**只读**记录：`g_debug.system.ccr`、`g_debug.motor.cpu_dcache_enabled`、`bsp_ws2812.c` 的初始化前置检查。它们不会改变 Cache 状态。
- WS2812 的 DMA 缓冲放在 `RW_LED_DMA 0x30000000 UNINIT`，既避开 TCM 的 DMA 不可访问问题，也不受 D-Cache 影响。

如果以后确实要开 D-Cache（例如为了性能），必须同时做到：

1. 用 MPU 把 DTCM/ITCM 和外设区配成 non-cacheable，把 DMA 缓冲放到 non-cacheable 区域（或每帧做 by_Addr 维护）；
2. 开启前在 Cache 关闭状态下按 set/way 清整个 cache；
3. 绝不对 `0x20000000`、`0x5C000000`、`0x40000000` 等区域做按地址维护；
4. 重新验证 CAN/WS2812/OLED 三类 DMA/中断缓冲的一致性。

## 6. 复现步骤（可重复）

```powershell
cd 'D:\竞赛\2026 AIC\RobStride05_H743'
# 在 Core/Src/main.c 的 `g_debug.system.ccr = SCB->CCR;` 之后加一行：
#   SCB_DisableDCache();
& 'C:\Keil_v5\UV4\UV4.exe' -b .\MDK-ARM\RobStride05_H743.uvprojx -j0 -o .\MDK-ARM\build.log
& 'C:\Keil_v5\UV4\UV4.exe' -f .\MDK-ARM\RobStride05_H743.uvprojx -j0 -o .\Tools\keil_flash.log
python .\Tools\flash_and_verify.py --no-flash   # 会报 boot_stage/fatal/ccr 失败
# 读回现场：fatal=0x9(HardFault)、cfsr=0x400(IMPRECISERR)、hfsr=0x40000000(FORCED)
# 验证完把这行删掉，重新编译下载即可恢复
```

注意：崩溃固件只是停在 `App_Fatal`（关中断死循环），不会使能电机，也不会写 Flash；实验结束后必须重新烧回正常固件。
