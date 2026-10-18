# dm_h7_rtos_base — 4×DM-H6215 麦轮底盘

STM32H723VGT6 + FreeRTOS（CMSIS-RTOS v2）+ 4× 达妙 DM-H6215 轮毂电机。
当前版本只包含：**5 ms 底盘任务、FDCAN1 MIT 力矩驱动、蓝牙遥控、USART10 VOFA+ 遥测**。
旧 H743 工程的 LVGL、GPS、IMU、OLED、TF、直流电机/编码器代码已清理，只保留 `Algorithm/pid.c/h`。

## 外设分配

| 外设 | 引脚 | 用途 | 参数 |
|---|---|---|---|
| FDCAN1 | PD0=RX / PD1=TX | 4× DM-H6215 | 经典 CAN，1 Mbps，标准 ID |
| USART1 | PA9=TX / PA10=RX | 蓝牙模块 | 115200 8N1，RX DMA 循环，TX DMA |
| USART10 | PE3=TX / PE2=RX | VOFA+ 遥测 | 115200 8N1，IT 发送，20 Hz |
| PC13 | Power_OUT1_EN | XT30 第 1 路 VCC_OUT + CAN1 | 高有效 |
| PC14 | Power_OUT2_EN | XT30 第 2 路 VCC_OUT + CAN2 | 高有效 |
| PC15 | PWR_EN_5V | 板载/对外 5 V（CAN 收发器、蓝牙） | 高有效 |

`PC13/PC14` 在 `main.c` 的 `USER CODE 2` 中置高，`PC15` 在 `gpio.c` 中置高。
电机电源来自 XT30 的 `VCC_OUT1/2`，请确认 `VCC_INPUT` 为电机额定 24 V；CAN 总线最远端加 120 Ω 终端电阻。

## 目录结构

```text
App/
  app_chassis_board.c/h   五步循环、模式/故障、运动学、PID、chassis_task
  app_motor_config.c/h    上电探测/顺序使能、蓝牙配置命令
  app_remote_control.c/h  蓝牙按键 → Vx/Vy/Wz
  app_telemetry.c/h       VOFA+ JustFloat
Bsp/
  bsp_fdcan.c/h           FDCAN1 过滤器/启动/中断/FIFO 排空/TX/bus-off
  bsp_dm6215.c/h          MIT 打包解包、使能/失能/清错、参数读写/保存
  bsp_uart.c/h            USART1 循环 DMA 收 + DMA 发；USART10 IT 发
  bsp_bluetooth.c/h       @...\r\n 解析、按键锁存、配置请求
Algorithm/
  pid.c/h                 位置式 PID（保留旧工程变量/函数风格）
```

## 构建与烧录

GCC 构建：

```bash
cd stm32-firmware/dm_h7_rtos_base
GCC_PATH="C:/Users/Arjen/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin" make -j8
```

### 烧录与调试（本机实测）

系统 PATH 里的 OpenOCD 0.12.0（2023 版）与 Horco CMSIS-DAP v2 不兼容，会在 `init` 报
`CMSIS-DAP command CMD_INFO failed`。已换用可用的 xPack OpenOCD 0.12.0-7（2025-10-04）：

```text
C:/00_git_code/tools/openocd-0.12.0-7/bin/openocd.exe
```

**OpenOCD 烧录（已验证）：**

```bash
cd stm32-firmware/dm_h7_rtos_base
C:/00_git_code/tools/openocd-0.12.0-7/bin/openocd.exe \
  -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "adapter speed 4000" \
  -c "program build/dm_h7_rtos_base.elf verify reset exit"
```

期望看到 `** Programming Finished **`、`** Verify Started **`、`** Verified OK **`。

**OpenOCD + GDB 调试（已验证）：**

终端 1（保持运行）：

```bash
C:/00_git_code/tools/openocd-0.12.0-7/bin/openocd.exe \
  -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
  -c "adapter speed 4000"
```

终端 2：

```bash
arm-none-eabi-gdb build/dm_h7_rtos_base.elf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) break chassis_send_cmd
(gdb) continue
(gdb) print (int)chassis_move.mode
(gdb) print (int)chassis_move.fault_flags
(gdb) monitor reset run
(gdb) detach
(gdb) quit
```

实测断点命中后读到 `MODE=3`（FAULT）、`FAULT=0x20`（MOTORS_NOT_READY）、`READY=0`，
与电机尚未配置 ID 的预期一致。

**单电机台架测试（GDB，已验证）：**

固件内置 `g_bench_*` 调试变量，绕过“四电机全部就绪”检查，直接对指定 ESC_ID 发 MIT 力矩，
到时自动停止。只在台架上用，务必让轮子悬空/固定。

```gdb
(gdb) target extended-remote :3333
(gdb) monitor halt
(gdb) set g_bench_done=0
(gdb) set g_bench_enable=0
(gdb) set g_bench_esc_id=3        # RL 电机
(gdb) set g_bench_torque=0.3
(gdb) set g_bench_duration_ms=1000
(gdb) set g_bench_enable=1
(gdb) break chassis_send_cmd if g_bench_done != 0
(gdb) continue
(gdb) print g_bench_max_state
(gdb) print g_bench_max_speed
```

断点命中后应看到 `DONE=1 ENABLE=0 TORQUE=0.000000`。实测 RL 电机（ESC 3/MST 0x53）：

- 0.1 N·m 空载不转（力矩偏小）；
- **0.3 N·m、1 s**：`MAX_STATE=1`、`MAX_SPEED≈35.24 rad/s`（约 336 rpm）、
  `rx_count=144`、`id_mismatch=0`、`tx_fail=0`、`bus_off=0`，电机正常旋转。

台架启动时会先发 `Disable + 写 RID10=1(MIT) + ClearError + Enable`；MIT 模式需要另发
`DM_WriteParam(3, 10, 1)` + `DM_SaveParams(3)` 才会断电保存。条件断点会拖慢循环，所以
命令数低于 200 Hz；正常无调试器时是 5 ms 周期。

Keil CMSIS-DAP 仍可作为备用烧录方式：

```bash
cd MDK-ARM
C:/Keil_v5/UV4/UV4.exe -b dm_h7_rtos_base.uvprojx -o keil_build.log
C:/Keil_v5/UV4/UV4.exe -f dm_h7_rtos_base.uvprojx -o keil_flash.log
```

期望 `Erase Done.Programming Done.Verify OK.`。Keil 当前 `pMon = BIN\CMSIS_AGDI.dll`；
`UL2CM3`/`ST-LINKIII-KEIL_SWO` 只是 registry 里的备选配置。

VS Code 任务：

- `Build (make)`：GCC 构建
- `Clean`：清理 GCC 构建
- `Build (Keil AC5)`：Keil AC5 构建（UV4 有 warning 时可能返回 1，先看日志）
- `Flash (Keil CMSIS-DAP)`：Keil CLI 烧录当前 `.axf`
- `Flash (OpenOCD)`：用 0.12.0-7 烧录 GCC ELF
- `OpenOCD GDB Server`：启动 GDB server（3333）
- `GDB (connect :3333)`：连接 GDB，交互调试

Keil 构建使用 `at(0x24000000/0x24001000)` 放置 USART1 DMA 缓冲；GCC 构建使用链接脚本的
`.dma_buffer` 段，两者互不影响。系统 PATH 里的 `openocd` 仍是旧的 0.12.0，日常请用上面的
绝对路径或把 `C:/00_git_code/tools/openocd-0.12.0-7/bin` 加到 PATH 前面。

## 控制架构

`chassis_task` 每 **5 ms** 执行一次，严格保留旧工程五步：

1. `chassis_feedback_update()` — 蓝牙、配置请求、DM 反馈快照、在线/故障检测
2. `chassis_mode_change()` — 急停 > 配置 > 故障 > 蓝牙 INDOOR > IDLE
3. `chassis_set_control()` — 蓝牙键位 → Vx/Vy/Wz，限幅 + 斜坡
4. `chassis_control_loop()` — 麦轮逆运动学 + 4× 速度 PID → 力矩 N·m
5. `chassis_send_cmd()` — MIT 力矩帧、遥测、蓝牙应答

电机使用 **MIT 模式**，`Kp=0/Kd=0`，主机 PID 输出力矩。反馈是“发一帧命令才回一帧”，
因此 IDLE/FAULT 时也会每 5 ms 发 0 力矩帧维持反馈；未配置/未就绪时 **不发 MIT/使能帧**。

## 电机 ID 配置

出厂默认四台都是 `ESC_ID=1 / MST_ID=0`，同时上总线会冲突。一次只接一台，用蓝牙发：

```text
@cfgselect 1 1       # 轮位 1=FL，当前 ESC_ID=1
@cfgmst 0x51         # MST_ID
@cfgmode 1           # MIT 模式（已是 1 可跳过）
@cfgsave             # 自动先失能再保存
断电重启
@cfgread             # 核对 esc/mst/mode/P10/V10/T10
```

FR/RL/RR 依次：

```text
@cfgselect 2 1
@cfgid 2
@cfgmst 0x52
@cfgsave
```

**没有蓝牙模块时，可以用 GDB 直接调 DM 参数接口。** 以一台默认 ESC 1/MST 0 的电机改成
RL（ESC 3/MST 0x53）为例：

```gdb
(gdb) target extended-remote :3333
(gdb) monitor halt
(gdb) call DM_SendDisable(1)
(gdb) call DM_WriteParam(1, 8, 3)      # ESC_ID 1 -> 3
(gdb) monitor resume
(gdb) detach
```

等约 1 s 再连：

```gdb
(gdb) target extended-remote :3333
(gdb) monitor halt
(gdb) call DM_WriteParam(3, 7, 0x53)   # MST_ID -> 0x53
(gdb) call DM_SaveParams(3)            # 保存（电机必须已失能）
(gdb) monitor resume
(gdb) detach
```

断电重启电机后，用台架模式把 `g_bench_esc_id` 设为 3 验证；反馈会进入 RL 槽（slot 2）。
四台全部接回、重新上电，固件会自动探测并顺序使能。忘记 ID 时只接一台后 `@cfgscan`。
`@cfgread` 中 `V10` 应为 `450`（V_MAX=45 rad/s）；若被上位机改过，需要同步修改
`bsp_dm6215.h` 的 `DM_V_MAX_RAD_S`。

## 蓝牙命令

| 命令 | 作用 |
|---|---|
| `@f` / `@b` | 前进 / 后退（Vx，按一次锁存，再按取消） |
| `@l` / `@r` | 左 / 右横移（Vy） |
| `@q` / `@e` | 逆时针 / 顺时针原地转（Wz） |
| `@s` / `@stop` | 清全部按键 |
| `@!` | 急停：清键、锁存、失能电机 |
| `@~` / `@resume` | 清故障、重新探测/使能，不恢复旧运动 |
| `@i` / `@indoor` | 进入蓝牙模式 |
| `@cfg*` | 电机 ID/模式/参数配置（见上） |

蓝牙模块必须是 **115200 8N1**。旧 App 没有心跳，默认 `BT_REQUIRE_HEARTBEAT=0`：
有锁存键时链路一直视为在线；置 1 后必须周期发帧，超时清键停车。

## 标定顺序

1. 架空，`@f/@b/@l/@r/@q/@e` 逐个确认轮子方向；装反的轮子改 `MOTOR_DIR_*`。
2. 横移整体反了改 `CHASSIS_KIN_VY_SIGN`；旋转整体反了改 `CHASSIS_KIN_WZ_SIGN`。
3. 实测麦轮半径、半轴距、半轮距，修改 `app_chassis_board.h` 中的三个常量。
4. PID 起步：`Kp=0.35, Ki=0.01, Kd=0`，先空载调 Kp，再加 Ki；`OutMax=2 N·m` 为峰值限幅。
5. 落地低速验证前进/横移/旋转，再按地面摩擦补 Ki 或开启静摩擦前馈。

## 遥测（VOFA+ JustFloat）

USART10，32 通道，20 Hz：

`loop_count, mode, fault_flags, bt_keys, Vx/Vy/Wz_set, Vx/Vy/Wz_cmd,
四轮目标 rad/s, 四轮实测 rad/s, 四轮力矩 N·m, 四轮 T_MOS, 四轮 T_Rotor,
state 半字节, loop_dt_ms`

VOFA+ 选择 JustFloat、115200、32 通道。

## 故障保护

- 单电机反馈超时 100 ms → 锁存 `MOTOR_TIMEOUT`，全部力矩置零，已掉线电机不再发帧。
- 电机反馈 state ≥ 8 → `MOTOR_STATE`。
- CAN 连续发送失败 20 次 → `CAN_TX`；bus-off → 强制故障并重启 FDCAN，需 `@~` 恢复。
- `@!` → 0 力矩 + 失能 + 锁存；`@~` → 重新探测/使能。
- 未配置/未就绪时只发 0x7FF 只读探测，不发 MIT/使能。

## 已知限制

- 旧蓝牙协议无心跳，断链时锁存运动无法自动停止；实车保留 `@!`/`@s` 和物理急停。
- 轮径/轴距目前是占位值，必须实测。
- 本轮未接入 IMU、ROS/USB 遥测、GPS、语音；五步循环留好了扩展位置。
- 用 CubeMX 重新生成代码会覆盖 Makefile 和链接脚本中的 `.dma_buffer` 段，需要重新加回。
