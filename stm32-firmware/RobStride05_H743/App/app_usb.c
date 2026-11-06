/* ============================================================================
 * app_usb.c —— 树莓派 ↔ STM32 USB CDC 桥
 *
 * 职责：
 *   1. 从 CDC 环形缓冲解析下行速度帧/服务帧（AA/BB）。
 *   2. 收到 ARM 后，按与蓝牙相同的四步事务准备并使能双电机。
 *   3. 把 vx/wz 换算成左右轮目标 rad/s，交给 AppMotor_RemoteTargets。
 *   4. 50 Hz 回传轮速、目标、电流和 JY901S 的 roll/pitch/yaw、陀螺、加速度。
 *
 * 安全：
 *   · 未 ARM 时只记录指令，绝不驱动电机。
 *   · ARM 中任一电机不在线/故障 → AppMotor_StopAll() 并清 arm 状态。
 *   · 连续 APP_USB_CMD_TIMEOUT_MS 没有合法速度帧 → 目标清零（保持 ARM，
 *     便于上位机重连后继续，不需要重新使能）。
 *   · 与蓝牙/SWD/机载航向控制互斥：AppUsb_OwnsMotors() 参与仲裁。
 *
 * 上下文：全部函数在 MOTOR 任务里调用（见 AppMotor_Service 末尾），
 * 因此可以直接使用 AppMotor_LocalCommand/RemoteTargets 的本地传输接口。
 * ==========================================================================*/
#include "app_usb.h"
#include "app_bluetooth.h"
#include "app_chassis.h"
#include "app_config.h"
#include "app_debug.h"
#include "app_motor.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

volatile AppUsbDebug g_usb_debug;

/* ---- IMU：独立解析器，避免改动 Jy901sData 的 SWD ABI --------------------- */
static AppUsbImu imu;
static uint8_t imu_frame[11];
static uint8_t imu_used;
static float imu_previous_yaw;
static uint8_t imu_previous_valid;

/* ---- 下行解析状态机 ------------------------------------------------------ */
static uint8_t down[12];
static uint8_t down_idx;
static uint8_t down_type;
static uint8_t down_expected;
static float cmd_vx;
static float cmd_vz;
static uint8_t cmd_mode;
static uint32_t cmd_tick;
static uint32_t cmd_frames;

/* ---- ARM 事务状态机（与 app_bluetooth.c 一致） --------------------------- */
static uint32_t armed;
static uint32_t arm_phase;
static uint32_t arm_waiting;
static uint32_t arm_tick;
static uint32_t authorizing;
static MotorCommand limits;

/* ---- 上行遥测 ------------------------------------------------------------ */
static uint8_t telem[APP_USB_TELEMETRY_SIZE];
static uint32_t telem_seq;
static uint32_t telem_tick;
static uint32_t tx_busy_count;

static float clampf(float value, float limit)
{
  return fmaxf(-limit, fminf(limit, value));
}

static int16_t word(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_u8(uint8_t **p, uint8_t value)
{
  **p = value;
  *p += 1;
}

static void put_u16(uint8_t **p, uint16_t value)
{
  memcpy(*p, &value, sizeof(value));
  *p += sizeof(value);
}

static void put_u32(uint8_t **p, uint32_t value)
{
  memcpy(*p, &value, sizeof(value));
  *p += sizeof(value);
}

static void put_f32(uint8_t **p, float value)
{
  memcpy(*p, &value, sizeof(value));
  *p += sizeof(value);
}

/* ============================================================================
 * IMU 原始字节解析（USART6 的每个字节由 app_chassis.c 额外送进来一份）
 * ==========================================================================*/
void AppUsb_ImuByte(uint8_t byte, uint32_t now)
{
  uint8_t i, sum = 0U;

  if (!imu_used && byte != 0x55U) {
    return;
  }
  imu_frame[imu_used++] = byte;
  if (imu_used < 11U) {
    return;
  }

  for (i = 0U; i < 10U; i++) {
    sum = (uint8_t)(sum + imu_frame[i]);
  }
  if (sum != imu_frame[10]) {
    imu.checksum_errors++;
    for (i = 1U; i < 11U && imu_frame[i] != 0x55U; i++) {
    }
    imu_used = (uint8_t)(11U - i);
    memmove(imu_frame, imu_frame + i, imu_used);
    return;
  }
  imu_used = 0U;

  if (imu_frame[1] == 0x51U) {
    /* 加速度：±16 g，16 位有符号，16/32768 g/LSB。 */
    imu.accel_g[0] = word(imu_frame + 2) * (16.0f / 32768.0f);
    imu.accel_g[1] = word(imu_frame + 4) * (16.0f / 32768.0f);
    imu.accel_g[2] = word(imu_frame + 6) * (16.0f / 32768.0f);
    imu.accel_frames++;
  } else if (imu_frame[1] == 0x52U) {
    /* 角速度：±2000 °/s。 */
    imu.gyro_deg_s[0] = word(imu_frame + 2) * (2000.0f / 32768.0f);
    imu.gyro_deg_s[1] = word(imu_frame + 4) * (2000.0f / 32768.0f);
    imu.gyro_deg_s[2] = word(imu_frame + 6) * (2000.0f / 32768.0f);
    imu.gyro_frames++;
  } else if (imu_frame[1] == 0x53U) {
    float yaw, delta;
    imu.roll_deg = word(imu_frame + 2) * (180.0f / 32768.0f);
    imu.pitch_deg = word(imu_frame + 4) * (180.0f / 32768.0f);
    yaw = word(imu_frame + 6) * (180.0f / 32768.0f);
    if (imu_previous_valid) {
      delta = yaw - imu_previous_yaw;
      if (delta > 180.0f) {
        delta -= 360.0f;
      }
      if (delta < -180.0f) {
        delta += 360.0f;
      }
      imu.yaw_deg += delta;
    }
    imu_previous_yaw = yaw;
    imu_previous_valid = 1U;
    imu.yaw_raw_deg = yaw;
    imu.ready = 1U;
    imu.angle_tick_ms = now;
    imu.angle_frames++;
  }
}

static void imu_reset(void)
{
  memset(&imu, 0, sizeof(imu));
  imu_used = 0U;
  imu_previous_yaw = 0.0f;
  imu_previous_valid = 0U;
}

/* ============================================================================
 * ARM / DISARM
 * ==========================================================================*/
/* Clear only the USB bridge's own authorization and pending command. No motor
 * stop call: a Bluetooth takeover may keep the enabled pair and simply drive
 * it to zero through the normal ramp. */
static void usb_clear_state(void)
{
  armed = 0U;
  arm_phase = 0U;
  arm_waiting = 0U;
  authorizing = 0U;
  cmd_tick = 0U;               /* next ARM must receive a fresh velocity frame */
  g_usb_debug.armed = 0U;
  g_usb_debug.arm_phase = 0U;
  g_usb_debug.cmd_timeout = 0U;
}

void AppUsb_Cancel(void)
{
  usb_clear_state();
  AppMotor_PairTargets(0.0f, 0.0f);
}

static void arm_abort(uint32_t result)
{
  AppMotor_StopAll();          /* stop_all also clears USB ownership */
  usb_clear_state();
  g_usb_debug.last_result = result;
}

static uint32_t arm_start(uint32_t now)
{
  uint32_t result;

  if (armed) {
    return RESULT_OK;
  }
  if (arm_phase) {
    return RESULT_PENDING;
  }
  if (g_debug.drive.active || AppChassis_Active() || AppBluetooth_BlocksUsb()) {
    g_usb_debug.last_result = RESULT_BUSY;
    return RESULT_BUSY;
  }

  result = AppMotor_PairReady(2U);
  if (result == RESULT_OK) {
    armed = 1U;
    cmd_tick = 0U;             /* 旧帧不得在使能瞬间变成运动命令 */
    AppMotor_PairTargets(0.0f, 0.0f);
    g_usb_debug.armed = 1U;
    g_usb_debug.last_result = RESULT_OK;
    return RESULT_OK;
  }

  result = AppMotor_AuthorizePair();
  if (result != RESULT_OK && result != RESULT_OFFLINE) {
    arm_abort(result);
    return result;
  }

  /* OFFLINE 只表示电机身份/反馈还没到，授权事务会在下面重试到超时。 */
  authorizing = (result == RESULT_OFFLINE) ? 1U : 0U;
  arm_phase = 1U;
  arm_waiting = 0U;
  arm_tick = now;
  g_usb_debug.arm_phase = arm_phase;
  g_usb_debug.last_result = RESULT_PENDING;
  return RESULT_PENDING;
}

static void arm_service(uint32_t now)
{
  uint32_t slot, result;

  /* A takeover may cancel USB while its transaction is mid-flight. Drop only
   * our own authorization; the new owner decides what happens to motors. */
  if (AppBluetooth_BlocksUsb()) {
    usb_clear_state();
    return;
  }

  if (!arm_phase) {
    return;
  }

  if (authorizing) {
    result = AppMotor_AuthorizePair();
    if (result == RESULT_OK) {
      authorizing = 0U;
    } else {
      if (result != RESULT_OFFLINE || now - arm_tick >= APP_USB_ARM_TIMEOUT_MS) {
        arm_abort(result);
      }
      return; /* never start PREPARE while identity authorization is pending */
    }
  }

  if (!arm_phase) {
    return;
  }

  slot = (arm_phase - 1U) & 1U;
  result = arm_waiting ? AppMotor_LocalResult(slot) : RESULT_OK;
  if (now - arm_tick >= APP_USB_ARM_TIMEOUT_MS ||
      (result != RESULT_OK && result != RESULT_PENDING)) {
    arm_abort((result == RESULT_OK || result == RESULT_PENDING) ? RESULT_TIMEOUT : result);
    return;
  }
  if (result != RESULT_OK) {
    return;
  }

  if (arm_waiting) {
    arm_phase++;
    arm_waiting = 0U;
    g_usb_debug.arm_phase = arm_phase;
  }
  if (arm_phase == 5U) {
    arm_phase = 0U;
    armed = 1U;
    cmd_tick = 0U;
    AppMotor_PairTargets(0.0f, 0.0f);
    g_usb_debug.arm_phase = 0U;
    g_usb_debug.armed = 1U;
    g_usb_debug.last_result = RESULT_OK;
    return;
  }

  /* 相位 1/2 = 两个电机 PREPARE，3/4 = 两个电机 ENABLE。 */
  slot = (arm_phase - 1U) & 1U;
  limits.command = (arm_phase <= 2U) ? CMD_PREPARE : CMD_ENABLE;
  result = AppMotor_LocalCommand(slot, &limits);
  arm_waiting = 1U;
  if (result != RESULT_OK && result != RESULT_PENDING) {
    arm_abort(result);
  }
}

static void disarm(void)
{
  AppMotor_StopAll();          /* stop_all also clears USB ownership */
  usb_clear_state();
  g_usb_debug.last_result = RESULT_OK;
}

/* ============================================================================
 * 速度指令 -> 左右轮目标
 *
 * 下行约定：vx < 0 = 前进，wz < 0 = 左转（ROS +Z 逆时针）。
 * 车辆前进坐标：forward = -vx，ccw = -wz。
 *   left  = (forward - ccw * track/2) / radius
 *   right = (forward + ccw * track/2) / radius
 * 电机槽位映射沿用 app_config/蓝牙：left_slot=1、left_sign=-1、right_sign=+1。
 * ==========================================================================*/
static void wheel_targets(float vx, float wz, float *left, float *right)
{
  float forward = clampf(-vx, APP_USB_MAX_LINEAR_M_S);
  float ccw = clampf(-wz, APP_USB_MAX_ANGULAR_RAD_S);
  *left = (forward - ccw * APP_TRACK_WIDTH_M * 0.5f) / APP_WHEEL_RADIUS_M;
  *right = (forward + ccw * APP_TRACK_WIDTH_M * 0.5f) / APP_WHEEL_RADIUS_M;
}

static void apply_drive(uint32_t now)
{
  float left, right;
  uint32_t result;

  /* Bluetooth priority: while Bluetooth owns the motors or latches a manual
   * hold, USB velocity frames must not touch targets or state. Telemetry keeps
   * running independently in send_telemetry(). */
  if (AppBluetooth_BlocksUsb() || !armed) {
    return;
  }

  result = AppMotor_PairReady(2U);
  if (result != RESULT_OK) {
    arm_abort(result);
    return;
  }

  if (!cmd_tick || now - cmd_tick > APP_USB_CMD_TIMEOUT_MS) {
    AppMotor_PairTargets(0.0f, 0.0f);
    g_usb_debug.cmd_timeout = 1U;
    return;
  }
  g_usb_debug.cmd_timeout = 0U;

  wheel_targets(cmd_vx, cmd_vz, &left, &right);

  /* 左电机槽位 1、负号；右电机槽位 0、正号。 */
  result = AppMotor_RemoteTargets(right, -left);
  if (result != RESULT_OK) {
    AppMotor_PairTargets(0.0f, 0.0f);
    g_usb_debug.last_result = result;
  }
}

/* ============================================================================
 * 下行帧解析
 * ==========================================================================*/
static void down_start(uint8_t header)
{
  down_type = header;
  down[0] = header;
  down_idx = 1U;
  down_expected = (header == APP_USB_DOWN_CMD_VEL) ? 12U : 4U;
}

static void down_complete(uint32_t now)
{
  uint8_t i, checksum = 0U;

  if (down_type == APP_USB_DOWN_CMD_VEL) {
    for (i = 2U; i < 11U; i++) {
      checksum ^= down[i];
    }
    if (checksum != down[11] || (down[2] != 2U && down[2] != 3U)) {
      g_usb_debug.rx_bad++;
      return;
    }
    memcpy(&cmd_vx, &down[3], sizeof(cmd_vx));
    memcpy(&cmd_vz, &down[7], sizeof(cmd_vz));
    if (!isfinite(cmd_vx) || !isfinite(cmd_vz) ||
        fabsf(cmd_vx) > 1.0f || fabsf(cmd_vz) > 2.0f) {
      g_usb_debug.rx_bad++;
      return;
    }
    cmd_mode = down[2];
    cmd_tick = now;
    cmd_frames++;
    g_usb_debug.rx_frames = cmd_frames;
    g_usb_debug.cmd_vx = cmd_vx;
    g_usb_debug.cmd_vz = cmd_vz;
    return;
  }

  /* 服务帧：BB 55 cmd xor，xor 就是 cmd。 */
  checksum = down[2];
  if (down[3] != checksum) {
    g_usb_debug.rx_bad++;
    return;
  }
  g_usb_debug.service_frames++;
  if (down[2] == APP_USB_CMD_ARM) {
    uint32_t result = arm_start(now);
    /* Report BUSY verbatim when Bluetooth owns the motors or holds them. */
    g_usb_debug.last_result = result;
  } else if (down[2] == APP_USB_CMD_DISARM) {
    /* Never let USB DISARM cancel a Bluetooth-held/owned motors. */
    if (AppBluetooth_BlocksUsb()) {
      g_usb_debug.last_result = RESULT_BUSY;
    } else {
      disarm();
    }
  } else {
    g_usb_debug.last_result = RESULT_BAD_COMMAND;
  }
}

static void down_byte(uint8_t byte, uint32_t now)
{
  if (down_idx == 0U) {
    if (byte == APP_USB_DOWN_CMD_VEL || byte == APP_USB_DOWN_SERVICE) {
      down_start(byte);
    }
    return;
  }

  if (down_idx == 1U) {
    if (byte != APP_USB_MAGIC) {
      down_idx = 0U;
      if (byte == APP_USB_DOWN_CMD_VEL || byte == APP_USB_DOWN_SERVICE) {
        down_start(byte);
      }
      return;
    }
  }

  down[down_idx++] = byte;
  if (down_idx == down_expected) {
    down_idx = 0U;
    down_complete(now);
  }
}

/* ============================================================================
 * 上行遥测
 * ==========================================================================*/
static void send_telemetry(uint32_t now)
{
  uint8_t *p = telem;
  uint8_t flags = 0U;
  uint32_t i, checksum = 0U;
  uint32_t motors_ready, imu_online, fresh;
  float req_left, req_right;

  /* 未枚举/未配置时 pClassData 为 NULL，CDC_Transmit_FS 会解引用它。 */
  if (hUsbDeviceFS.pClassData == NULL ||
      hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
    return;
  }

  motors_ready = (AppMotor_PairReady(2U) == RESULT_OK) ? 1U : 0U;
  imu_online = (imu.ready && now - imu.angle_tick_ms <= 250U) ? 1U : 0U;
  fresh = (cmd_tick && now - cmd_tick <= APP_USB_CMD_TIMEOUT_MS) ? 1U : 0U;

  if (imu.ready) {
    flags |= APP_USB_FLAG_IMU_READY;
  }
  if (imu_online) {
    flags |= APP_USB_FLAG_IMU_ONLINE;
  }
  if (motors_ready) {
    flags |= APP_USB_FLAG_MOTORS_READY;
  }
  if (armed) {
    flags |= APP_USB_FLAG_ARMED;
  }
  if (fresh) {
    flags |= APP_USB_FLAG_CMD_FRESH;
  }
  if (g_debug.system.fatal) {
    flags |= APP_USB_FLAG_FATAL;
  }
  if (cmd_frames) {
    flags |= APP_USB_FLAG_RX_SEEN;
  }
  if (g_usb_debug.cmd_timeout) {
    flags |= APP_USB_FLAG_CMD_TIMEOUT;
  }

  put_u8(&p, APP_USB_UP_TELEMETRY);
  put_u8(&p, APP_USB_MAGIC);
  put_u8(&p, APP_USB_TELEMETRY_VER);
  put_u8(&p, flags);
  put_u8(&p, cmd_mode);
  put_u16(&p, (uint16_t)telem_seq++);
  put_u32(&p, now);

  put_f32(&p, cmd_vx);
  put_f32(&p, cmd_vz);
  put_f32(&p, -AppMotor_FeedbackSpeed(1U));   /* left, 前进为正 */
  put_f32(&p, AppMotor_FeedbackSpeed(0U));    /* right */
  put_f32(&p, -AppMotor_TargetSpeed(1U));     /* left target */
  put_f32(&p, AppMotor_TargetSpeed(0U));      /* right target */
  put_f32(&p, -AppMotor_FeedbackCurrent(1U)); /* left Iq */
  put_f32(&p, AppMotor_FeedbackCurrent(0U));  /* right Iq */

  put_f32(&p, imu.roll_deg);
  put_f32(&p, imu.pitch_deg);
  put_f32(&p, imu.yaw_deg);
  put_f32(&p, imu.gyro_deg_s[0]);
  put_f32(&p, imu.gyro_deg_s[1]);
  put_f32(&p, imu.gyro_deg_s[2]);
  put_f32(&p, imu.accel_g[0]);
  put_f32(&p, imu.accel_g[1]);
  put_f32(&p, imu.accel_g[2]);

  /* dry-run：即使未 ARM，也把当前下行指令换算出的轮速报上去，
   * 便于上位机在不驱动电机的情况下核对符号和比例。 */
  wheel_targets(cmd_vx, cmd_vz, &req_left, &req_right);
  put_f32(&p, req_left);
  put_f32(&p, req_right);

  put_u32(&p, g_usb_debug.last_result);
  put_u32(&p, cmd_frames);
  put_u32(&p, g_usb_debug.rx_bad);
  put_u32(&p, tx_busy_count);

  if ((uint32_t)(p - telem) != APP_USB_TELEMETRY_SIZE - 1U) {
    return; /* 组帧长度错位：宁可整帧不发，也不发错位数据。 */
  }
  for (i = 2U; i < APP_USB_TELEMETRY_SIZE - 1U; i++) {
    checksum ^= telem[i];
  }
  telem[APP_USB_TELEMETRY_SIZE - 1U] = (uint8_t)checksum;

  if (CDC_Transmit_FS(telem, APP_USB_TELEMETRY_SIZE) == USBD_BUSY) {
    tx_busy_count++;
  }
  g_usb_debug.telemetry_frames++;
  g_usb_debug.tx_busy = tx_busy_count;
}

/* ============================================================================
 * 对外接口
 * ==========================================================================*/
void AppUsb_Init(void)
{
  memset((void *)&g_usb_debug, 0, sizeof(g_usb_debug));
  imu_reset();

  memset(down, 0, sizeof(down));
  down_idx = 0U;
  down_type = 0U;
  down_expected = 0U;
  cmd_vx = 0.0f;
  cmd_vz = 0.0f;
  cmd_mode = 0U;
  cmd_tick = 0U;
  cmd_frames = 0U;

  armed = 0U;
  arm_phase = 0U;
  arm_waiting = 0U;
  arm_tick = 0U;
  authorizing = 0U;

  memset(&limits, 0, sizeof(limits));
  limits.confirm_key = APP_ENABLE_KEY;
  limits.speed_limit_rad_s = APP_DEFAULT_SPEED_CAP;
  limits.current_limit_a = APP_DEFAULT_CURRENT_A;
  limits.accel_rad_s2 = APP_DEFAULT_ACCEL;

  telem_seq = 0U;
  telem_tick = 0U;
  tx_busy_count = 0U;
}

uint32_t AppUsb_OwnsMotors(void)
{
  return (armed || arm_phase) ? 1U : 0U;
}

void AppUsb_Service(uint32_t now)
{
  uint8_t chunk[64];
  uint32_t budget = 4U;
  uint32_t length, i;

  /* 每 2 ms 最多取 4×64 字节；主机 50 Hz 只发 12 字节/帧，余量足够。 */
  while (budget-- && (length = CDC_ReadRxData(chunk, sizeof(chunk))) > 0U) {
    for (i = 0U; i < length; i++) {
      down_byte(chunk[i], now);
    }
  }

  arm_service(now);
  apply_drive(now);

  if ((int32_t)(now - telem_tick) >= (int32_t)APP_USB_TELEM_PERIOD_MS) {
    telem_tick = now;
    send_telemetry(now);
  }

  g_usb_debug.armed = armed;
  g_usb_debug.arm_phase = arm_phase;
  g_usb_debug.rx_overflow = CDC_GetRxOverflowCount();
}
