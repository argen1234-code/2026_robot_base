# RTOS 移植、Cache 代码移除与上板验收（2026-10-22）

## 1. 结论

`RobStride05_H743`（FreeRTOS）已从“上电即 HardFault”修到**在实物板上持续运行**，并实现了 `RobStride05_H743_HAL` 单线程工程的全部功能（电机速度环、OLED、WS2812 灯带、CAN 保护、`g_debug` 调试 ABI），可直接由 `上位机\RobStrideConsole.exe` 连接控制。

本轮没有改动任何硬件接线：引脚与 HAL 工程一致（PB9/PB8 FDCAN1、PB10/PB11 I²C2、PA6 WS2812、PA13/PA14 SWD）。

## 2. 原故障与本轮判据

历史现场（`Docs/06_RTOS启动排查.md`）：`stage=1`、`error=0x48415244`（HardFault）、`tick=0`、`cfsr=0x400`（IMPRECISERR 非精确总线写）、`hfsr=0x40000000`（FORCED）。

判据：故障发生在最早期 Cache 初始化段。CMSIS 的 `SCB_DisableDCache()` 在 ARMCC -O0 下会对**位于 DTCM 的栈局部变量**按地址做 clean/invalidate；对非可缓存地址做 MVA 维护属于架构未定义行为，可产生总线错误。因此本轮按用户要求**彻底移除所有 Cache 使能/关闭/维护代码**，而不是继续给条件分支打补丁。

## 3. 移除清单（逐文件）

| 文件 | 处理 |
|---|---|
| `Core/Src/main.c` | 删除 `SCB_DisableDCache()`、`SCB_EnableICache()` 及 `if (SCB->CCR & SCB_CCR_DC_Msk)` 分支；删除 `BootStatus_t` 启动遥测；只保留 `g_debug.system.ccr = SCB->CCR;` 这一处**只读**采样 |
| `Core/Inc/main.h` | 删除 `BootStatus_t` / `g_boot_status` / `Boot_CaptureFault` 声明 |
| `Core/Src/stm32h7xx_it.c` | 异常处理改为 `App_Fatal(APP_FATAL_*)`，写入 `g_debug.system` |
| `Core/Inc/FreeRTOSConfig.h` | `configASSERT` → `App_AssertFailed(__FILE__,__LINE__)`（记录位置后 `App_Fatal`） |
| `RobStride05_H743.ioc` | `CORTEX_M7.CPU_ICache=Enabled` → `Disabled`；`CPU_DCache` 保持 `Disabled` |
| `BSP/bsp_ws2812.c`、`App/app_motor.c`、`App/app_main.c` | 仅保留**只读**的 `SCB->CCR` 采样（`LED_ERR_CACHE` 前置检查、`cpu_dcache_enabled` 字段、`fatal` 现场记录），没有任何写 CCR 或维护指令 |

保留只读采样的原因：`cpu_dcache_enabled` 与 `system.ccr` 是上位机 ABI 字段，用于事后证明“运行时确实没有开 Cache”；它们不会改变 Cache 状态。

**证据**：最终 AXF 反汇编（`Docs/rtos_disassembly.txt`）中不存在任何 `MCR`/`MCRR` 协处理器指令，即镜像里没有 Cache 维护指令。运行时 `g_debug.system.ccr = 0x40200`，即 STKALIGN + BP，IC(bit17)/DC(bit16) 均为 0。

## 4. 移植清单

**与 HAL 工程代码逻辑相同**（2026-10-22 补中文注释前的 SHA256 完全一致；补注释后去注释再比对仍逐字一致）：`App/app_config.h`、`app_debug.*`、`app_motor.*`、`app_led.*`、`BSP/bsp_can.*`、`bsp_i2c.*`、`bsp_time.*`、`bsp_ws2812.*`、`Devices/oled_ssd1306.*`、`robstride05.*`、`oled_font.h`、`Core/Src/gpio.c`、`stm32h7xx_hal_msp.c`、`system_stm32h7xx.c`、`MDK-ARM/startup_stm32h743xx.s`。中文注释只在本工程，同步逻辑改动时以代码为准。

**RTOS 侧重写**：

| 文件 | 作用 |
|---|---|
| `Core/Src/main.c` | VTOR → HAL_Init → 时钟 → GPIO/FDCAN/I²C → `App_Init()` → `boot_stage=9` → `osKernelInitialize` → `MX_FREERTOS_Init` → `osKernelStart`；定义 TIM6 的 `HAL_TIM_PeriodElapsedCallback`（否则 `HAL_GetTick` 停摆） |
| `Core/Src/freertos.c` | 只保留 `MX_FREERTOS_Init()`（调用 `AppTasks_Start`）与栈溢出/堆失败钩子；删除 CubeMX 的 `defaultTask` |
| `App/app_tasks.c/.h` | 四个周期任务：motor 2 ms / AboveNormal / 2048 B，oled 2 ms / Normal / 3072 B，health 100 ms / BelowNormal / 1024 B，led 5 ms / Low / 1536 B；`osDelayUntil` 绝对周期，超期不补跑 |
| `App/app_main.c/.h` | 把 HAL 的 `App_Service()` 拆成 `AppMotor_Loop`/`AppOled_Loop`/`AppLed_Loop`/`AppHealth_Loop`；OLED 绘制逻辑与 HAL 相同，新增基于 `snapshot_seq` 的稳定快照读取；`App_Fatal`/`APP_FATAL_*` 统一故障码 |

**配置变更**：

- `MDK-ARM/RobStride05_H743.sct`：新增 `RW_LED_DMA 0x30000000 UNINIT 0x1000 { *.o (WS2812_DMA) }`，与 HAL 相同；DTCM 仍承载 RW/ZI、24 KB heap、任务栈与 `g_debug`。
- NVIC：FDCAN1_IT0、I²C2 EV/ER 统一为 5（= `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`），可被 RTOS 临界区屏蔽；ISR 内不调用 RTOS 接口。HAL 时基 TIM6 保持 `TICK_INT_PRIORITY=15`。
- `uvprojx`：补入 `app_debug.c`、`app_led.c`、`app_main.c`、`bsp_ws2812.c` 及对应头文件。
- 删除 `Tests/test_control.c`：该文件针对已废弃的 `g_motor_status`/租约状态机，无法编译；PC 侧状态机检查已由上位机 `tests/test_firmware.py` 覆盖。

## 5. 构建与静态核对

- Keil ARMCC 5.06u7：`0 Error(s), 0 Warning(s)`（下列为 12:40 含调度统计页面的版本）。
- Program Size：Code=52248、RO-data=1564、RW-data=980、ZI-data=36916。
- AXF 1 949 008 B，SHA256 `9B4D939A2AC480FEBF54F40DABAB60C857C67284BEDA55D49CC8E6FD7016158A`。
- HEX 151 685 B，SHA256 `33D57D61B8AD575C5B4B8DB27856C998389CE597091C2DAF9EDE59CB6C1758DF`。
- 链接 map：`g_debug=0x2000009C`(396 B)、`g_led=0x20000240`(80 B)、`ucHeap=0x200011F0`(24 576 B)、OLED 帧缓冲 `pixels=0x20007940`(1024 B)、`waveform=0x30000000`(1128 B，`RW_LED_DMA` UNINIT)；`RW_DTCM` 用量 0x8Fxx / 128 KB。
- `g_debug` ABI 与 HAL 工程逐字段相同（上位机 `profile.py` 解析两版 AXF：`motor` 280 B @28、`can` @148、`oled` 40 B @308、`system` 48 B @348，`cmd` 28 B @0），仅地址不同（RTOS `0x20000094` / HAL `0x20000014`）。

## 6. 上板实测（烧写后只读读取，未下发任何使能/运行命令）

| 检查项 | 实测 |
|---|---|
| `system.boot_stage` / `fatal` | 9 / 0 |
| `system.vtor` / `system.ccr` | `0x08000000` / `0x40200`（IC/DC=0） |
| `loop_heartbeat`（2 ms 任务） | 2103 → 2406 → 2708，持续增长 |
| `tick_ms`（TIM6） | 4206 → 4810 → 5414，与心跳同步 |
| `health_heartbeat`（100 ms） | 55（≈ tick/100） |
| 电机任务单次耗时 | `last_service_us=14`，`max_service_us=30` |
| 电机反馈 | `online=1`，`feedback_age_ms=8`，`motor_state=0`（Reset，未使能） |
| CAN | `rx=606`、`tx_ok=64`、`tx_fail=0`、`rx_drop=0`、`bus_off=0`（STOP 帧 10 Hz + 反馈） |
| OLED | `ready=1`、`refresh_ok=26`、`refresh_fail=0`（5 Hz） |
| WS2812 | `ready=1`、`active_mode=1`、`frames_started=frames_done=271`、`errors=0`（≈50 fps） |
| 快照一致性 | 三次采样 `coherent=true`，`snapshot_seq` 偶数且前后一致 |

补充：`SYSRESETREQ` 复位后复测 → `boot_stage=9`、`fatal=0`、`ccr=0x40200`、心跳重新递增，说明启动路径可重复。

## 6.1 续：OLED 改为 RTOS 页面（09:50 起，12:40 升级为调度统计页面）

原 RTOS 版沿用 HAL 的电机页面。现在改为 RTOS 专用页面：`App/app_display.c`（纯格式化，可 PC 编译测试）+ `App/app_rtos_stats.c`（调度统计）+ `App/app_tasks.c` 的 `g_app_task[]`（线程表），`App/app_main.c` 的 `draw()` 只负责取数。

版面（0.96"，21 字符 × 8 行，全部大写——字形表没有小写和 `%`）：

```text
UP 00:00:08 SW 15763
HEAP 16.0K MIN 16.0K
MOTOR  BLK 1.2 S1.5K
OLED   RUN 1.3 S1.9K
HEALTH BLK 0.0 S0.8K
LED    BLK 0.8 S1.3K
MOT ONLINE  V+0.043
SET +0.000 RAD/S DIS
```

电机相关只保留在线状态、实测速度、设定速度（另加 3 字符应用状态，便于看故障）；其余全部是 RTOS 信息：

- `SW`：上电以来上下文切换次数（低 5 位）。实测 ≈1800–2000 次/秒，每帧跳约 360，画面里"一直在数"。
- `MOTOR/OLED/HEALTH/LED` 四行：线程名、当前调度状态、**CPU 占用百分比**、剩余栈。
- 统计来源：`configGENERATE_RUN_TIME_STATS=1` + `portGET_RUN_TIME_COUNTER_VALUE()=DWT->CYCCNT`（200 MHz）+ `uxTaskGetSystemState()`；切换次数来自 `traceTASK_SWITCHED_IN()` 宏。占用 = 本任务窗口增量 / 全部任务增量之和，所以四行之和 ≈ 100 − IDLE。
- 为什么不是"当前正在跑的线程名"：采样者本身在跑，直接读当前 TCB 只会得到 OLED 自己；看"谁在跑"要看调度器分配的时间片和状态。

验证方式（不靠肉眼）：

- PC：`Tools/test_display.c` 用真实 `app_display.c` + 假 OLED 传输，检查每行 ≤21 字符、每个字符都存在于 `Devices/oled_font.h`，并对在线/离线/±33/堆空/极值各跑一遍；结果 `PASS: 8 rows, 21 columns, all glyphs present`。
- 上板：`Tools/read_oled_page.py` 通过 DWARF 定位 `pixels`（本版 `0x20007940`，1024 B），读回帧缓冲反解。实测：`SW 15763`（8 s 时，≈1970/s）、`MOTOR 1.2% / OLED 1.3% / HEALTH 0.0% / LED 0.8%`、栈 `1.5K/1.9K/0.8K/1.3K`、堆 `16.0K` 与历史最小相同、`V+0.043`、状态 `DIS`；间隔 3 s 两次采样数值持续变化，无缺字形单元。
- 本轮重新编译烧写后的整体复测：`boot_stage=9`、`fatal=0`、`ccr=0x40200`、心跳持续增长、CAN 无错误、OLED 无失败、灯带无错误，脚本 `PASSED`。

**另外本轮把旧的 Cache 故障做了实物复现**：在正常工程里加回一行 `SCB_DisableDCache()` 就 100% 复现 `stage=1 / fatal=9 / cfsr=0x400 / hfsr=0x40000000 / tick=0`，机理与证据见 [08_Cache为什么会HardFault.md](08_Cache为什么会HardFault.md)。

注意：这段时间另一个会话把 HAL 单线程固件刷到过板上（09:15–09:47）。本轮已重新刷回 RTOS 版并复测；两个工程的 `g_debug` 地址不同且随链接变化（本版 RTOS `0x2000009C`），上位机按 AXF + 整段 Flash 自动识别，不会读错。

## 7. 上位机联调

- 上位机 `server.py` / `bridge.py` 改为**同时支持两个工程**：启动时列出附近所有候选 AXF（按构建时间排序），连接时先用首段前缀快速排除，再对命中的 AXF 做**整段 Flash 比对**，只有完全一致才允许控制。`build-singlefile.ps1` 打包的 `firmware_profile.json` 也改为取最新构建的 AXF。
- 重新打包并替换 `上位机\RobStrideConsole.exe`（旧版备份在 `back_up\RobStrideConsole_20261022_0847_old.exe`）。
- 实机联调结果：探针 `d9381b44` → 连接成功 → 自动选中 `RobStride05_H743`（地址由 AXF 解析，当次为 `0x20000094`），日志 `Flash 一致 · RobStride05_H743 · H743 正在运行`；页面读到 `state=0`、`online=1`、`feedback_age_ms=3`、`velocity=-0.053 rad/s`、`position=1.052 rad`、`bus=25.5 V`、`temp=28 ℃`，`continuous=false`，采样延迟 10 ms。
- 上位机单元测试 15 项全部通过（含用本工程 `app_motor.c` 编译的产线状态机仿真）。

## 8. 未覆盖 / 限制

- **未下发使能/运行命令**：电机使能、实际转动、转速/电流闭环表现必须由操作者在现场按 `Docs/04_上板验收.md` 执行。所有故障保护逻辑在 PC 仿真与只读遥测上验证，不等于实物运动安全验收。
- **冷启动（断电再上电）未远程验证**：本轮用的是 Keil 下载后的复位与 `SYSRESETREQ`。由于镜像内不存在任何 Cache 维护指令，冷复位与热复位的启动路径相同，但仍建议现场第一次断电重启时确认一次 `boot_stage=9` 和屏幕内容。
- **灯带颜色/亮度未目视确认**：只验证了 DMA 帧计数与无错误；灯带型号修订、颜色顺序需目视。
- **OLED 像素内容已回读核对，但未目视**：`Tools/read_oled_page.py` 从帧缓冲反解出的 8 行文字与设计一致，字符均有字形；亮度、对比度与面板个体差异仍需看一眼屏幕。
- 本电机明确拒绝对 0x7028/0x200C 的写入（上位机 `timeout_motor_supported=0`），电机侧 CAN 断线自停未被验证，必须保留可直接断电的手段。
- 探针同一时间只能被一个进程占用：Keil、上位机、`Tools/flash_and_verify.py` 不要同时使用。

## 9. 复现与回滚

```powershell
cd 'D:\竞赛\2026 AIC\RobStride05_H743'
& 'C:\Keil_v5\UV4\UV4.exe' -b .\MDK-ARM\RobStride05_H743.uvprojx -j0 -o .\MDK-ARM\build.log
& 'C:\Keil_v5\UV4\UV4.exe' -f .\MDK-ARM\RobStride05_H743.uvprojx -j0 -o .\Tools\keil_flash.log
python .\Tools\flash_and_verify.py --no-flash
```

- 烧写前现场已备份到 `Tools/backup/flash_before_*.bin`（`0x08000000` 起 128 KB），需要回到旧固件时按同基址写回。
- 改造前的完整源码备份：`back_up\RobStride05_H743_pre_rtos_port_20261022-090154`。

## 10. 与 HAL 工程的同步规则

`App/`、`BSP/`、`Devices/` 是两工程共享源码，改动应同时落到两边（或只改一边后立刻同步），否则“功能等价”不再成立。本轮同步记录：HAL 工程在 09:05 把 WS2812 `T1H` 由 66 tick(0.66 µs) 改为 85 tick(0.85 µs)（WS2812B 手册 T1H=0.8±0.15 µs），本工程已按同值重新编译、烧写并复测通过。
