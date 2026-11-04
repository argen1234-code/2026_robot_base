/* ============================================================================
 * app_config.h —— 全部可调参数集中在这里
 *
 * 改限值/ID/超时只改这个文件；上位机 RobStride_Console/server.py 里的
 * LIMITS / DEFAULTS 必须同步，否则"准备"会因参数不一致被拒。
 *
 * 数据来源：RS05 使用说明书 261009（CAN 2.0 私有协议，不是 CAN-FD 版）。
 *   · 速度模式 0x700A 的指令范围是 ±33 rad/s（第 37 页）；反馈编码是 ±50。
 *   · 相电流：额定 2.4 A、最大 11 A（第 6/9 页）。
 *   · 加速度默认 20 rad/s²（第 37 页），参数表上限 150（第 17 页）。
 * 注意：指令范围不等于"任何供电/负载下都能达到这个速度"。
 * ==========================================================================*/
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---- CAN 地址 -------------------------------------------------------------
 * APP_MOTOR_ID：电机的 CAN ID，官方示例默认 0x7F(127)。这里只改 MCU 侧
 *   的收发地址，**绝不下发"改电机 ID"的指令**，不会动电机的永久配置。
 * APP_HOST_ID ：主机（本板）地址 0xFD，协议里回复帧的目的地址就是它。 */
#define APP_MOTOR_ID 0x7FU
#define APP_MOTOR2_ID 0x7EU
#define APP_MOTOR_COUNT 2U
#define APP_ISOLATED_KEY 0x49534F4CUL /* ISOL: operator confirms only ONE motor on bus. */
#define APP_HOST_ID 0xFDU

/* ---- 命令范围（硬上限，超出直接拒绝，不会裁剪后继续执行）-------------------
 * 速度上限 33 rad/s、相电流上限 11 A、加速度上限 150 rad/s²。 */
#define APP_SPEED_MAX_RAD_S 33.0f
#define APP_CURRENT_MAX_A 11.0f
#define APP_ACCEL_MAX_RAD_S2 150.0f

/* ---- "准备"时的默认值 -----------------------------------------------------
 * 速度默认取上限 33（上位机默认也一样）、电流取额定 2.4 A、加速度取手册默认 20。
 * 2.4 A 是额定值不是最大值，空载联调够用；要更大扭矩由操作者显式改。 */
#define APP_DEFAULT_SPEED_CAP APP_SPEED_MAX_RAD_S
/* 2.4 A 是额定值；原地大角度旋转时实测驱动轮打滑约 3 倍（指令 238° /
 * 实际 80°），先提高到 4 A 以增加克服脚轮阻力的扭矩。电机最大 11 A，
 * 4 A 只影响大负载/起转时的峰值，正常直行电流仍远低于此值。 */
#define APP_DEFAULT_CURRENT_A 4.0f
#define APP_DEFAULT_ACCEL 20.0f

/* ---- 各类超时 -------------------------------------------------------------
 * APP_MOTOR_TIMEOUT_UNITS：0x7028 的单位是 50 µs，20000=1 s，这里写 5000=250 ms。
 *   注意：实测这颗电机拒绝对 0x7028 写入，所以这个"电机侧断线自停"并未生效，
 *   上位机会提示 timeout_motor_supported=0。
 * APP_FEEDBACK_TIMEOUT_MS：超过 150 ms 没有新反馈就判离线（会触发保护）。
 * APP_PARAM_TIMEOUT_MS   ：参数读回/状态确认的单项超时 100 ms。
 * APP_CAN_TX_TIMEOUT_MS  ：一帧最多等 5 ms 的 ACK，超时 abort 并报 TX 失败。
 * APP_SERVICE_GAP_US     ：两次电机服务间隔超过 100 ms（例如调试暂停后恢复）
 *                          就撤销命令与目标 —— 这是"恢复后的检查"，不是停机期间的保护。 */
#define APP_MOTOR_TIMEOUT_UNITS 5000U
#define APP_FEEDBACK_TIMEOUT_MS 150U
#define APP_PARAM_TIMEOUT_MS 100U
#define APP_CAN_TX_TIMEOUT_MS 5U
#define APP_SERVICE_GAP_US 100000U

/* JDY31: USART1 PA9 TX / PA10 RX, same actual baud as the reference. */
#define APP_BT_BAUD 9600U
#define APP_BT_TIMEOUT_MS 600U

/* "危险命令"确认键：准备/使能/运行都必须带这个 magic，防止误把结构体写坏当命令。
 * 数值可以理解为 "RS05" 的 ASCII。 */
#define APP_ENABLE_KEY 0x52533035UL

/* ---- OLED ---------------------------------------------------------------
 * 7 位地址 0x3C（模块背面常见默认）。I²C 的 ACK 只能说明"有器件应答"，
 * 不能证明屏真的亮了、方向对、像素内容对 —— 那些只能看屏或回读帧缓冲。 */
#define APP_OLED_ADDRESS_7BIT 0x3CU
#define APP_OLED_PERIOD_MS 200U /* 5 Hz 刷整屏 */

/* ---- WS2812 灯带 ---------------------------------------------------------
 * 独立占用 PA6 / TIM3_CH1 / DMA1 Stream0 / DMAMUX1 ch0，与本工程其它外设无关。
 * 波形缓冲放在内部 D2 SRAM1（0x30000000），因为 DTCM 不能被 DMA 访问，
 * 而 D2 SRAM1 既能让 DMA 读、又不受 CPU Cache 影响（本工程 Cache 全关）。
 * APP_LED_MODE：0=自动轮播；1..6=固定特效；7=常亮。 */
#define APP_LED_FEATURE_ENABLED 1U
#define APP_LED_COUNT 16U             /* Physical strip: all 16 pixels. */
#define APP_LED_DEFAULT_ENABLED 1U
#define APP_LED_DEFAULT_MODE 0U       /* 0 auto, 1..6 effects, 7 solid. */
#define APP_LED_DEFAULT_BRIGHTNESS 64U /* 0..255，gamma 之后再乘 */
#define APP_LED_FRAME_MS 20U          /* 50 帧/秒 */
#define APP_LED_AUTO_PERIOD_MS 8000U  /* 自动模式每 8 s 换一个特效 */
#define APP_LED_GAMMA_ENABLED 1U      /* 做一次平方 gamma，低亮度更细腻 */

/* ---- USB CDC 桥（树莓派导航 <-> 底盘） -----------------------------------
 * 2026-11-04 两轮 CAD 模型实测：
 *   轮半径 0.041 m；左右轮接地点中心 y=±0.074 m -> 有效轮距 0.148 m。
 * 必须与 robot_base.urdf.xacro、controllers.yaml 同步；否则下发的轮速与
 * 实际行走距离不一致。速度上限与 chassis_bridge.py 保持一致。 */
/* 几何半径 0.041 m；2026-11-05 卷尺/编码器/雷达标定有效滚动半径 0.041713 m。 */
#define APP_WHEEL_RADIUS_M       0.041713f
#define APP_TRACK_WIDTH_M        0.148f
#define APP_USB_MAX_LINEAR_M_S   0.24f
#define APP_USB_MAX_ANGULAR_RAD_S 0.55f
#define APP_USB_CMD_TIMEOUT_MS   500U   /* 超过 500 ms 没有合法速度帧 -> 目标清零 */
#define APP_USB_TELEM_PERIOD_MS  20U    /* 上行遥测 50 Hz */
#define APP_USB_ARM_TIMEOUT_MS   4000U  /* 准备+使能事务的最长总时长 */
#endif
