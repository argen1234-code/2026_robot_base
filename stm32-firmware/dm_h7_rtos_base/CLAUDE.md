# CLAUDE.md — dm_h7_rtos_base

STM32H723VGT6 四轮麦轮底盘。4× 达妙 DM-H6215（FDCAN1，MIT 力矩模式）+ 蓝牙（USART1）
+ VOFA+（USART10）。FreeRTOS 10.6.2 / CMSIS-RTOS v2，5 ms 底盘任务。
硬件与 DM-balanceV1 使用同一块 DM-MC02/CtrlBoard-H7 开发板。

## Build / Flash

```bash
cd stm32-firmware/dm_h7_rtos_base
GCC_PATH="C:/Users/Arjen/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin" make -j8
```

- 必须用 GCC 13.3.1 的 `GCC_PATH`；PATH 里的 GCC 10.3.1 不支持 ld 脚本里的 `(READONLY)`。
- **OpenOCD**：系统 PATH 里的 0.12.0（2023 版）与 Horco CMSIS-DAP v2 不兼容
  （`CMD_INFO failed`）。已验证可用的是便携版 xPack **0.12.0-7（2025-10-04）**：
  `C:/00_git_code/tools/openocd-0.12.0-7/bin/openocd.exe`
  ```bash
  C:/00_git_code/tools/openocd-0.12.0-7/bin/openocd.exe \
    -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
    -c "adapter speed 4000" \
    -c "program build/dm_h7_rtos_base.elf verify reset exit"
  ```
  成功标志：`** Verified OK **`。
- **OpenOCD + GDB 调试**（已验证）：先启动 OpenOCD（不加 program），再
  `arm-none-eabi-gdb build/dm_h7_rtos_base.elf` →
  `target extended-remote :3333` → `monitor reset halt` → `break chassis_send_cmd` →
  `continue`。实测断点命中，`MODE=3 FAULT=0x20 READY=0`（未配置电机的预期状态）。
- **单电机台架测试**：`app_chassis_board.c` 的 `g_bench_enable/g_bench_esc_id/
  g_bench_torque/g_bench_duration_ms/g_bench_done`，GDB 写变量后绕过四电机就绪检查，
  直接对指定 ESC_ID 发 MIT 力矩；启动时先 Disable + 写 RID10=1(MIT) + ClearError + Enable，
  到时自动停止，仅台架使用。已验证 RL（ESC 3/MST 0x53）1 s/0.3 N·m：
  `MAX_STATE=1`、`MAX_SPEED≈35.2 rad/s`、`rx_count=144`、`tx_fail=0`、`bus_off=0`。
- **Keil CMSIS-DAP 备用烧录**：
  ```bash
  cd MDK-ARM
  C:/Keil_v5/UV4/UV4.exe -b dm_h7_rtos_base.uvprojx -o keil_build.log
  C:/Keil_v5/UV4/UV4.exe -f dm_h7_rtos_base.uvprojx -o keil_flash.log
  ```
  成功标志：`Erase Done.Programming Done.Verify OK.`；Keil 当前 `pMon = BIN\CMSIS_AGDI.dll`。
- VS Code 任务：`Build (make)`、`Clean`、`Build (Keil AC5)`、`Flash (Keil CMSIS-DAP)`、
  `Flash (OpenOCD)`、`OpenOCD GDB Server`、`GDB (connect :3333)`。
- Keil 工程分组已包含新文件；主构建仍是 GCC Makefile，烧录用 Keil 的 `.axf` 时需要先跑 Keil 构建。

## 架构（保持旧工程五步，不要随意改名）

```
App/app_chassis_board.c     chassis_task 5 ms
  chassis_feedback_update → chassis_mode_change → chassis_set_control
  → chassis_control_loop → chassis_send_cmd
App/app_motor_config.c      上电探测/顺序使能 + 蓝牙配置命令（阻塞，仅 IDLE/CONFIG）
App/app_remote_control.c    蓝牙键位 → Vx/Vy/Wz（米制）
App/app_telemetry.c         USART10 JustFloat，20 Hz
Bsp/bsp_fdcan.c             FDCAN1 过滤器/启动/FIFO 排空/TX/bus-off
Bsp/bsp_dm6215.c            MIT 打包解包、反馈 seqlock、参数读写
Bsp/bsp_uart.c              USART1 循环 DMA 收 + DMA 发；USART10 IT 发
Bsp/bsp_bluetooth.c         @...\r\n 解析、锁存键、配置请求
Algorithm/pid.c             旧位置式 PID（double，无 dt），增加 PID_Reset()
```

## 关键常量

- `app_chassis_board.h`：轮径/半轴距/半轮距、`MOTOR_DIR_*`、`CHASSIS_KIN_VY/WZ_SIGN`、
  PID 起步参数、蓝牙默认速度、故障位。
- `bsp_dm6215.h`：P/V/T 映射范围、RID、MIT/参数帧常量。
- 电机 ID：FL=1/0x51，FR=2/0x52，RL=3/0x53，RR=4/0x54（`app_motor_config.c`）。

## 必须知道的坑

1. **DTCM 不能做 DMA**：USART1 RX/TX DMA 缓冲在 `RAM_D1 0x24000000`，
   通过 `.dma_buffer` 段；不要改成普通 static `.bss`。
2. **USART1 RX DMA 是 CIRCULAR**：`HAL_UARTEx_RxEventCallback` 的 `Size` 是绝对写位置，
   按 last/回卷处理；正常回调不要重新 arm。
3. **FDCAN DLC**：新 HAL `FDCAN_DLC_BYTES_8 == 8`，不要用旧工程 `len<<16`。
4. **DM 反馈是命令驱动**：正常/IDLE 都要每 5 ms 发命令；未配置/未就绪时不要发 MIT/使能。
5. **PID 参数顺序**：`PID_Calculate(pid, 实测, 目标)`。
6. **板级电源**：PC15(5V) 在 `gpio.c` 置高；PC13/PC14(Power_OUT1/2) 在 `main.c USER CODE 2` 置高。
   电机从 XT30 的 `VCC_OUT1/2` 取电。
7. **默认电机 ID 是 1/MST 0**：四台同时上电会冲突，必须一次接一台用 `@cfg*` 配置；
   没有蓝牙时可用 GDB 调 `DM_SendDisable/DM_WriteParam/DM_SaveParams`（先写 RID8=ESC，
   再写 RID7=MST，最后 0xAA 保存并断电重启）。
8. **旧蓝牙协议无心跳**：默认有锁存键即在线；`BT_REQUIRE_HEARTBEAT=1` 才按超时清键。
9. **CubeMX 重新生成**会覆盖 Makefile 和链接脚本；`.dma_buffer` 段和 App/Bsp 源列表需要重新加。

## 常用调试

- 串口助手接 USART1（115200）：`@cfgstatus`、`@cfgread`、`@cfgscan`、`@!`、`@~`。
- VOFA+ 接 USART10（115200，JustFloat，32 通道）看目标/实测/力矩/温度/loop_dt。
- 上电未配置时的预期：`mode=FAULT`、`fault_flags` 含 `MOTORS_NOT_READY`，总线上只有
  0x7FF 只读探测，没有 MIT/使能帧。
- ID 配置完成后预期：四路 state=1，`loop_dt_ms≈5`，IDLE 下力矩 0。

## 验证清单

1. `make` 通过；map 中 `s_bt_rx_dma/s_bt_tx_dma` 在 `0x2400xxxx`。
2. 逐个配置 ID，断电重启后 `@cfgread` 核对 ESC/MST/mode/P10/V10/T10。
3. 架空方向测试：`@f/@l/@q`；修正 `MOTOR_DIR_*`/`KIN_*_SIGN`。
4. 空载调 PID，再落地低速验证。
5. 失效测试：拔电机、断蓝牙、`@!`、`@~`、CAN bus-off。
