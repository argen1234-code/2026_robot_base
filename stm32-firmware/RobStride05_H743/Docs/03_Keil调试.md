# Keil 调试与 `g_debug` 使用

## 编译与下载

1. 打开 `MDK-ARM/RobStride05_H743.uvprojx`，F7 编译（ARMCC 5.06u7）。正常输出 `0 Error(s), 0 Warning(s)`。
2. 下载前确认没有别的工具占用探针（上位机、pyocd、另一个 Keil 会话）。
3. Keil → Flash → Download（或命令行 `UV4 -f ...uvprojx -o flash.log`），日志应出现 `Erase Done. Programming Done. Verify OK. Application running ...`。
4. 也可以只读验收：`python Tools/flash_and_verify.py --no-flash`。

日志放在 `MDK-ARM/build.log`、`Tools/keil_flash.log`。**不要用“只编译后按 Run”代替重新下载**，板上仍是旧固件。

## Watch 变量

| 变量 | 用途 |
|---|---|
| `g_debug` | 唯一根结构，展开 `cmd / motor / oled / system`；字段含义见 `Docs/02_架构与维护.md` |
| `g_debug.system.boot_stage` | 启动阶段，正常运行应为 9 |
| `g_debug.system.fatal` | 0 正常；非 0 见 `App/app_main.h` 的 `APP_FATAL_*` |
| `g_debug.system.ccr` | 只读快照，正常 `0x40200`（IC/DC 均为 0） |
| `g_debug.motor.snapshot_seq` | 偶数且读前后相同才是同一发布快照 |
| `g_led` | WS2812 状态：`ready/busy/active_mode/frames_started/errors` |
| `uwTick` | HAL 1 ms tick（TIM6），应与 `g_debug.system.tick_ms` 同步增长 |
| `g_assert_file` / `g_assert_line` | `configASSERT` 或 HAL 断言命中位置，`fatal=7` 时看这两个 |

打开 View → Periodic Window Update；支持的探针可用 Live Watch。普通 Watch 读取不是原子的，判断一致性请看 `snapshot_seq`。

## 提交电机命令

**推荐用上位机**（`上位机\RobStrideConsole.exe`）：它完成序号递增、写入校验、结果等待和状态门控，并拒绝非法值。

手工在 Keil 里改 `g_debug.cmd` 时遵守同一协议：

1. CPU 保持 Run。
2. `seq = 旧值 + 1`（奇数）→ 写 `command`、`confirm_key=0x52533035`、`target_rad_s`、`speed_limit_rad_s`、`current_limit_a`、`accel_rad_s2`。
3. 最后写新序号（旧值 + 2，偶数，非 0）。
4. 固件要求在**两个不同服务周期**读到完全相同的一份命令才执行，因此改完不要立刻停 CPU。
5. 用 `motor.executed_seq` / `motor.result` 确认结果（0 成功、16 正在执行、其余为拒绝码）。

使能顺序固定：1 准备 → 2 使能 → 3 运行；4 减速停止、5 立即失能、6 清错。准备与使能要求 `target_rad_s=0` 且限制值与准备时一致，所以改限制必须先失能再重新准备。

## 调试器暂停的影响

CPU 被暂停时固件不执行任何保护；恢复后 TIM2 服务间隔 >100 ms 会立即撤销命令与目标并 trip `RESULT_SERVICE_GAP`（使能态会同时发停止帧）。因此：

- 需要长时间停在断点时，先在上位机「减速停止/立即失能」。
- 本工程没有看门狗，也没有“复位后自动继续”的机制；异常只能靠 `fatal`、心跳和现场读取定位。

## 不要做的事

- 不要在 Watch 里写 `g_debug.motor/oled/system`，那是固件发布区。
- 不要为了“跑得快”打开 I-Cache/D-Cache：WS2812 的 DMA 缓冲在 D2 SRAM1，Cache 打开后没有维护代码会直接花屏/错帧；原故障也正是 Cache 维护指令引起的。
- 不要在 Keil 里同时用另一个 Watch 窗口改 `cmd`，会与上位机冲突。
