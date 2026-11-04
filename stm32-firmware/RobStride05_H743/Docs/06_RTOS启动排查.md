# FreeRTOS 启动排查与本轮验收

> **历史文档（2026-10-21）**：这里的 `g_boot_status`、`stage` 表与条件式 D-Cache 规避方案都已被 2026-10-22 的方案取代——**Cache 代码已全部移除**（不再有条件式 `SCB_DisableDCache`，也没有 `SCB_EnableICache`），启动遥测改用上位机 `g_debug`。当前结论、移除清单与实测数据见 [07_RTOS移植与上板验收.md](07_RTOS移植与上板验收.md)。本页仅保留故障定位过程作为追溯。

2026-10-21，上海时间。用户提供的 Keil 全速运行截图中，电机/OLED 的发布序号、心跳和年龄全部为 0，命令结构中的初始化浮点值存在。截图本身不能区分启动停机和 Watch 旧值；尚未取得暂停后的 PC、VTOR 或异常寄存器。

通过 antigravity-cli-bridge 调用 `Gemini 3.8 Flash (High)` 做启动分析和修改方案，Codex 复核、决定并应用修复。Gemini 原始输出属于待核实建议，不能作为硬件验收记录。

## 用户上板反馈与第二轮修复

用户已经下载第一轮修复版，截图确认 `build_marker=0x524F4231`、`stage=1`、`error=0x48415244`（HardFault）、`tick=0`、`vtor=0x08000000`、`cfsr=0x400`（IMPRECISERR，非精确总线写错误）、`hfsr=0x40000000`（FORCED）。这证明仅设置 VTOR 没有解决实物故障，故障仍在 Cache/HAL 的最早初始化段；不再把 VTOR 缺口当作唯一根因。

CMSIS 的 `SCB_DisableDCache()` 即使 DCache 已关闭仍执行清理/无效化；当前 O0 版本还对 DTCM 栈局部变量执行按地址维护。Gemini 认为冷启动下的无条件清理可能触发总线写错误。未取得原始故障 PC/ABFSR，不能只凭 CFSR 就确定是哪一条维护指令或哪一种 Cache 机制。

第二轮（当前产物）标记为 `0x524F4232`：仅当入口 CCR.DC 已置位才执行完整 clean-and-disable；已关闭时直接跳过多余的维护。记录 `entry_ccr`，并在 DCache、ICache、HAL_Init 前分别标记 `0x101/0x102/0x103`。故障快照增加 `SCB->ABFSR`，其实际地址为 **0xE000EFA8**；Gemini 建议中的 0xE000ED64 已核对为错误，未采用。

第二轮已 Keil 重建 0 错误、0 警告。尚未取得第二轮上板心跳结果，故障是否消除仍待验证。

Gemini 对实际第二轮代码复审返回 `SHIP WITH NITS / Blocking: none`，原文保存为 `Debug_20261021/gemini-cache-final.txt`。验收结论限于源码与构建：冷复位 CCR.DC=0 时已避免原来的无条件维护；若从已开启 DCache 的引导程序跳入，则仍会执行完整清理，不能由本次检查保证该路径的实物行为。保留 TIM6 优先级 4，因为它服务独立 HAL 时基，后续 HAL 重新初始化时基所用的默认优先级并不等同于该值。

## 已修复与依据

原 `system_stm32h7xx.c` 只在 `USER_VECT_TAB_ADDRESS` 定义时写入 VTOR，实际工程没有这个定义。ELF 的 `__Vectors` 在 `0x08000000`，芯片头文件将 `0x00000000` 定义为 ITCM；CM7 FreeRTOS 端口从 VTOR 所指向量表读取初始 MSP，再以 SVC 启动任务。若实际 VTOR 没有指向本固件向量表，TIM6 中断及调度器启动可能失败。这是优先修复的源码缺口，尚不能宣称已证明是本次实物停机的唯一原因。

`main()` 在 HAL 初始化和 TIM6 中断启动之前，使用链接符号 `__Vectors` 设置 VTOR，随后执行 DSB/ISB。增加 `g_boot_status`，检查 HAL 初始化、电源配置、内核初始化、默认任务创建和调度器返回；HAL 失败、异常和 FreeRTOS 断言记录现场后停止。原有 malloc/栈溢出与应用致命错误仍读取 `g_motor_status.rtos_fatal`。

维护脚本同步保留修改；重复整合和用原始源码重新整合均已检查。没有改变时钟、引脚、RTOS 端口、任务栈、堆大小或电机命令流程。

## 本轮软件验收

- Keil ARMCC 5.06u7 重建：0 错误、0 警告。
- 原有生产状态机/协议 PC 检查：11 组全部 PASS，包括启动不使能。
- 最终 AXF 反汇编：`main` 入口写 VTOR=`0x08000000`，DSB、ISB 均在 `HAL_Init` 调用之前。
- 最终 ELF 有强定义的 SVC、PendSV、SysTick、TIM6 中断函数及 `g_boot_status`。
- 整合幂等；临时目录内恢复原始启动源码再运行整合，输出与当前生产源码一致。未实际再次运行 CubeMX GUI。
- 最新 AXF/HEX 大小与 SHA256 见 `delivery_audit.json`，日志见 `build_run.log`、`host_checks.log` 和 `Debug_20261021/startup_checks.txt`。

后续通过 computer-use 实际读取到 Keil 工程窗口、CMSIS-DAP 调试器标识，以及用户下载记录（Programming Done、Verify OK，16:07:20）。读取期间调试会话已退出，未取得暂停 PC。随后检测到用户物理 Esc，已停止界面操作。工具没有主动下载第二轮固件或操作电机。**上板启动验收仍待完成。**

## 下一次 Keil 操作

1. 结束旧调试会话，重新加载当前工程和符号，F7 后下载新固件。仅重新编译后按 Run 不能保证板上代码已更新。
2. Watch 添加 `g_boot_status`、`uwTick`，保留 `g_motor_status`、`g_oled_status`，开启 Periodic Window Update。
3. 运行，不提交使能/运行命令。正常应为 `build_marker=0x524F4232`、`vtor=0x08000000`、`stage=13`、`error=0`，`uwTick` 和原有三个任务心跳持续增加。离线电机和缺失 OLED 可以导致通信错误，但不应让所有任务心跳一直为 0。
4. 若仍异常，暂停一次，记录当前函数/PC，以及整个 `g_boot_status`、`g_motor_status.rtos_fatal` 和 `uwTick`。阶段只是最后到达位置；`stage=13` 只证明健康任务曾进入，仍须连续心跳证明持续运行。
5. `build_marker` 一直是 0 时，先排查新固件/符号是否加载、是否进入 main、Watch 是否刷新；此时不能仅用全零结构推断 HAL/RTOS 故障。

### stage：当前进入的启动阶段

| 值 | 阶段 |
|---|---|
| 0 | 尚未执行 main 中的诊断标记，或显示的是旧内存 |
| 1 | 仅第一轮旧固件使用：Cache/HAL 初始化段 |
| 257（0x101） | 第二轮：检查入口 CCR、按需关闭 DCache |
| 258（0x102） | 第二轮：开启 ICache |
| 259（0x103） | 第二轮：HAL_Init（含 TIM6 时基） |
| 2 | LDO 供电配置 |
| 3 | VOS1 配置与 VOSRDY 等待 |
| 4 | HSE/HSI/PLL 振荡器配置 |
| 5 | 系统/总线时钟切换 |
| 6 | GPIO 初始化 |
| 7 | FDCAN 初始化 |
| 8 | I2C2 初始化 |
| 9 | osKernelInitialize |
| 10 | 默认/电机/OLED 任务及状态锁创建 |
| 11 | 任务创建函数已返回，调度器即将启动（瞬态） |
| 12 | osKernelStart / 第一个任务启动过程 |
| 13 | 默认健康任务已进入 |

### error：十六进制读取

| 值 | 含义 |
|---|---|
| 0 | 没有完成故障现场记录；不等于任务必然正常 |
| 0x1001 | HAL_Init 失败 |
| 0x1002 | HAL_PWREx_ConfigSupply 失败 |
| 0x1009 | osKernelInitialize 失败 |
| 0x100A | 默认任务创建返回 NULL |
| 0x100C | osKernelStart 意外返回（任何返回值都视为失败） |
| 0x48414C46 | HAL 错误或 VOS 超时，结合 stage、file、line 定位 |
| 0x48414C41 | HAL 参数断言（仅启用 USE_FULL_ASSERT 时） |
| 0x45525248 | 通用 Error_Handler |
| 0x41535354 | FreeRTOS configASSERT |
| 0x48415244 | HardFault |
| 0x4D454D46 | MemManage |
| 0x42555346 | BusFault |
| 0x55534746 | UsageFault |
| 0x4E4D4921 | NMI |

`error` 最后写入，非零时 `tick/vtor/cfsr/hfsr/mmfar/bfar/icsr/primask/basepri/abfsr` 是故障快照；正常运行时 vtor、entry_ccr 在启动时写入，其余故障快照保持 0，并非实时寄存器镜像。`primask/basepri` 在捕获函数关中断前读取。仅在 CFSR 的 MMARVALID/BFARVALID 置位时解释相应地址。

捕获是尽力而为，需要可用栈和可写 DTCM，不能保证记录严重栈损坏、Lockup 或进入 main 之前的错误；没有保存异常堆栈中的原始 PC。此类情况仍需调试器的 PC/LR/MSP/PSP、`SCB->VTOR/CFSR/HFSR`。

## 未采纳的 Gemini 建议

- 把 TIM6 从优先级 4 降到 15：当前回调只调用 HAL_IncTick，不调用 RTOS API，允许它抢占 BASEPRI=5 临界区不构成 RTOS API 违规。
- 因 `configENABLE_FPU=0` 修改 FPU：当前非 MPU CM7/r0p1 端口自行启用 VFP，并按 EXC_RETURN 保存/恢复浮点寄存器，该宏不是已证实故障原因。
- 增大主频到 480 MHz 或直接扩大堆：没有对应故障证据；当前 200 MHz 时钟计划有效，24 KB 堆并不能仅凭 10 KB 应用任务栈就推断耗尽。

后续只依据新固件的实际 stage、异常现场和连续心跳继续决定修复。
