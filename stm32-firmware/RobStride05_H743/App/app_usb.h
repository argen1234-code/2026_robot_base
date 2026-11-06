/* ============================================================================
 * app_usb.h —— 树莓派 ↔ STM32 的 USB CDC 虚拟串口桥
 *
 * 下行（Pi -> MCU）
 *   AA 55 | mode u8 | vx f32 | wz f32 | xor         12 字节，现有速度帧
 *   BB 55 | cmd u8  | xor                           4 字节，服务帧
 *          cmd: 1 = ARM（准备+使能双电机），2 = DISARM/STOP
 *
 * 上行（MCU -> Pi），50 Hz
 *   CC 55 | ver=1 | flags | mode | seq u16 | tick_ms u32
 *         | cmd_vx f32 | cmd_vz f32
 *         | left_speed f32 | right_speed f32
 *         | left_target f32 | right_target f32
 *         | left_current f32 | right_current f32
 *         | roll f32 | pitch f32 | yaw f32
 *         | gyro_x f32 | gyro_y f32 | gyro_z f32
 *         | acc_x f32 | acc_y f32 | acc_z f32
 *         | req_left f32 | req_right f32   ← 下行换算出的轮速（dry-run）
 *         | last_result u32 | rx_frames u32 | rx_bad u32 | tx_busy u32
 *         | xor                                      104 字节
 *
 * 坐标系约定（与 chassis_protocol.py 一致）：
 *   下行 vx < 0 表示前进；wz < 0 表示左转（ROS +Z 逆时针）。
 *   上行 left/right_speed、target、current 统一为"车辆前进为正"的轮速。
 *   角度单位：roll/pitch/yaw = 度；gyro = 度/秒；acc = g。
 *
 * ⚠️ AppUsb_Service() 必须在 MOTOR 任务上下文调用（和 AppMotor_RemoteTargets、
 *    AppMotor_LocalCommand 的契约一致），见 AppMotor_Service() 末尾。
 * ==========================================================================*/
#ifndef APP_USB_H
#define APP_USB_H

#include <stdint.h>

/* ---- 帧头 / 版本 ---------------------------------------------------------- */
#define APP_USB_DOWN_CMD_VEL   0xAAU
#define APP_USB_DOWN_SERVICE   0xBBU
#define APP_USB_UP_TELEMETRY   0xCCU
#define APP_USB_MAGIC          0x55U
#define APP_USB_TELEMETRY_VER  2U
#define APP_USB_TELEMETRY_SIZE 104U

/* ---- 服务帧命令 ----------------------------------------------------------- */
enum { APP_USB_CMD_ARM = 1, APP_USB_CMD_DISARM = 2 };

/* ---- 上行 flags ----------------------------------------------------------- */
#define APP_USB_FLAG_IMU_READY   (1U << 0)
#define APP_USB_FLAG_IMU_ONLINE  (1U << 1)
#define APP_USB_FLAG_MOTORS_READY (1U << 2)
#define APP_USB_FLAG_ARMED       (1U << 3)
#define APP_USB_FLAG_CMD_FRESH   (1U << 4)
#define APP_USB_FLAG_FATAL       (1U << 5)
#define APP_USB_FLAG_RX_SEEN     (1U << 6)
#define APP_USB_FLAG_CMD_TIMEOUT (1U << 7)

/* 独立于 Jy901sData 的 IMU 快照。
 * 不改 Devices/jy901s.h 的结构体布局：g_debug 的偏移量被上位机
 * firmware_profile.json 固定读取，动它会让 SWD 界面解析错位。 */
typedef struct {
  uint32_t ready;
  uint32_t angle_tick_ms;
  uint32_t gyro_frames;
  uint32_t accel_frames;
  uint32_t angle_frames;
  uint32_t checksum_errors;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;      /* 连续展开，可超过 ±180° */
  float yaw_raw_deg;
  float gyro_deg_s[3];
  float accel_g[3];
} AppUsbImu;

/* 仅观察用，不放进 g_debug（保持 SWD ABI 不变）。 */
typedef struct {
  uint32_t rx_frames;
  uint32_t rx_bad;
  uint32_t service_frames;
  uint32_t telemetry_frames;
  uint32_t tx_busy;
  uint32_t rx_overflow;
  uint32_t armed;
  uint32_t arm_phase;
  uint32_t cmd_timeout;
  uint32_t last_result;
  float cmd_vx;
  float cmd_vz;
} AppUsbDebug;

extern volatile AppUsbDebug g_usb_debug;

void AppUsb_Init(void);
void AppUsb_Service(uint32_t now);
void AppUsb_ImuByte(uint8_t byte, uint32_t now);
uint32_t AppUsb_OwnsMotors(void);
/* Relinquish USB motion ownership and zero its effect. Clears only this
 * module's state; never calls AppMotor_StopAll (avoids mutual recursion and
 * lets a Bluetooth takeover keep motors enabled at zero). */
void AppUsb_Cancel(void);

#endif
