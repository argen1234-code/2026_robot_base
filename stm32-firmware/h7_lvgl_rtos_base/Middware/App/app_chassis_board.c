#include "app_chassis_board.h"
#include "app_Navigation.h"
#include "app_remote_control.h"
#include "app_Voice_Recognition.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_uart.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_GPS.h"
#include "bsp_bluetooth.h"
#include "bsp_JY901S.h"
#include "bsp_road_classification.h"
#include "bsp_WonderEcho.h"
#include "bsp_tf_image_load.h"
#include "ff.h"
#include "usart.h"
#include "stm32h7xx_hal_flash_ex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Jetson timeout (ms); no valid frame within this period -> offline ---- */
#define JETSON_TIMEOUT_MS  500U
#define GPS_ONLINE_TIMEOUT_MS  3000U

/* Encoder feedback threshold used to enable road classification. */
#define ROAD_CLASSIFICATION_MOVING_SPEED_THRESHOLD  3.0

/* ---- On-demand magnetometer calibration motion (ms / internal Wz units) ---- */
#define MAG_CALIB_WZ                 50.0f
#define MAG_CALIB_STILL_END_MS       2000U
#define MAG_CALIB_CW_END_MS         14000U
#define MAG_CALIB_PAUSE_END_MS      15000U
#define MAG_CALIB_CCW_END_MS        27000U

/* ---- Central command slew-rate limits (internal speed units per second) ---- */
#define CHASSIS_TRANSLATION_SLEW_PER_S  300.0f
#define CHASSIS_ROTATION_SLEW_PER_S     200.0f
#define CHASSIS_SMOOTH_DEFAULT_DT_S       0.01f
#define CHASSIS_SMOOTH_MAX_DT_S           0.05f

/* Bank 2 sectors 6/7 are reserved by the Keil IROM limit for GPS route storage. */
#define GPS_ROUTE_FLASH_SLOT0_ADDR       0x081C0000U
#define GPS_ROUTE_FLASH_SLOT1_ADDR       0x081E0000U
#define GPS_ROUTE_FLASH_SLOT0_SECTOR     FLASH_SECTOR_6
#define GPS_ROUTE_FLASH_SLOT1_SECTOR     FLASH_SECTOR_7
#define GPS_ROUTE_FLASH_MAGIC            0x47505254U /* "GPRT" */
#define GPS_ROUTE_FLASH_VERSION          1U
#define GPS_ROUTE_FLASH_RECORD_SIZE      192U
#define GPS_ROUTE_FLASH_SEQUENCE_OFFSET  8U
#define GPS_ROUTE_FLASH_DATA_OFFSET      12U
#define GPS_ROUTE_FLASH_CRC_OFFSET       172U
#define GPS_ROUTE_FLASH_WORD_SIZE        32U

/* SD card is the primary, human-readable route store; Flash remains backup. */
#define GPS_ROUTE_SD_DIR                 "0:/GPS"
#define GPS_ROUTE_SD_FILE                "0:/GPS/GPSPTS.CSV"
#define GPS_ROUTE_SD_TEMP                "0:/GPS/GPSPTS.TMP"
#define GPS_ROUTE_SD_BUFFER_SIZE         1024U
#define GPS_ROUTE_SD_LOCK_TICKS          20U
#define GPS_ROUTE_SD_RETRY_MS            1000U
#define GPS_ROUTE_SD_LOAD_OK             0U
#define GPS_ROUTE_SD_LOAD_NOT_FOUND      1U
#define GPS_ROUTE_SD_LOAD_ERROR          2U

/* ---- Runtime GPS route; restored from SD primary storage or Flash backup. ---- */
static GPS_Point_t chassis_gps_route[MAX_WAYPOINTS] = {0};
static uint8_t chassis_gps_route_count = 0U;
static __ALIGNED(32) uint32_t chassis_gps_route_flash_record_words[GPS_ROUTE_FLASH_RECORD_SIZE / sizeof(uint32_t)];
static char chassis_gps_route_sd_buffer[GPS_ROUTE_SD_BUFFER_SIZE];
static GPS_Point_t chassis_gps_route_sd_points[MAX_WAYPOINTS];
static FIL chassis_gps_route_sd_file;
static uint8_t chassis_gps_route_sd_sync_done = 0U;
static uint8_t chassis_gps_route_sd_save_pending = 0U;
static uint8_t chassis_gps_route_runtime_modified = 0U;
static uint32_t chassis_gps_route_sd_last_retry_tick = 0U;
static GPS_Point_t chassis_usb_route_pending[MAX_WAYPOINTS];
static uint16_t chassis_usb_route_received_mask = 0U;
static uint8_t chassis_usb_route_expected_count = 0U;
static uint8_t chassis_usb_route_loop_enable = 1U;
static uint32_t chassis_usb_route_sequence = 0U;

/* Sole chassis instance; not directly accessible externally, only via pointer */
chassis_move_t    chassis_move    = {0};
static volatile int gui_req_mode = -1;
static uint8_t chassis_manual_indoor_mode = 0U;
static volatile uint8_t chassis_init_done = 0U;
static char chassis_init_status[64] = "\xE7\xAD\x89\xE5\xBE\x85\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96";
static uint8_t chassis_mag_hw_ready = 0U;
static uint8_t chassis_mag_initialized = 0U;
static uint8_t chassis_mag_initializing = 0U;
static float chassis_smoothed_vx = 0.0f;
static float chassis_smoothed_vy = 0.0f;
static float chassis_smoothed_wz = 0.0f;
static uint32_t chassis_smooth_last_tick = 0U;
static uint32_t chassis_scene_sequence = 0U;

volatile uint8_t g_chassis_gps_route_count_debug = 0U;
volatile ChassisGPSRouteResult_t g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_NONE;
volatile ChassisGPSRouteStorageStatus_t g_chassis_gps_route_storage_status_debug = CHASSIS_GPS_ROUTE_STORAGE_NONE;
volatile uint32_t g_chassis_gps_route_storage_sequence_debug = 0U;
volatile uint8_t g_chassis_gps_route_storage_slot_debug = 0xFFU;
volatile ChassisGPSRouteSDStatus_t g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_WAITING;
volatile uint32_t g_chassis_gps_route_sd_save_count_debug = 0U;
volatile uint32_t g_chassis_gps_route_sd_load_count_debug = 0U;
volatile uint32_t g_chassis_gps_route_sd_delete_count_debug = 0U;
volatile uint32_t g_chassis_gps_route_sd_last_fresult_debug = FR_OK;
volatile uint8_t g_chassis_mag_initialized_debug = 0U;
volatile uint8_t g_chassis_powerless_debug = 0U;

/*
 * 全局可见的 JY901S 调试镜像。volatile 用于保证即使打开优化，Keil Watch 读取的
 * 也是 RAM 中真实更新的内容。该镜像不被任何控制流程读取。
 */
volatile ChassisJY901SDebug_t g_chassis_jy901s_debug;

static uint8_t chassis_mode_available(chassis_move_t *chassis, CarMode_t mode);
static uint8_t chassis_is_gps_online(chassis_move_t *chassis);
static uint8_t chassis_is_mag_online(chassis_move_t *chassis);
static uint8_t chassis_is_moving_for_road_classification(const chassis_move_t *chassis);
static void chassis_mag_calibration_step(uint32_t elapsed_ms);
static void chassis_load_gps_route_from_flash(void);
static uint8_t chassis_save_gps_route_to_flash(void);
static void chassis_persist_gps_route(void);
static void chassis_clear_gps_route(chassis_move_t *chassis);
static void chassis_process_usb_gps_route(chassis_move_t *chassis);
static void chassis_service_gps_route_sd(chassis_move_t *chassis);
static uint8_t chassis_add_remote_gps_point(chassis_move_t *chassis,
                                            const BT_RemotePoint_t *remote_point);

/*
 * 复制刚收到的 JY901S 完整数据帧，仅服务于在线调试。
 * update_sequence 前后各加一次，使 Watch 在实时刷新时可确认读取到的是完整快照。
 * 此函数不修改 source，不调用任何 PID/电机/模式函数，也不改动 chassis_move。
 */
static void chassis_update_jy901s_debug_snapshot(const JY901S_Data_t *source)
{
    uint8_t i;

    if (source == NULL)
    {
        return;
    }

    g_chassis_jy901s_debug.update_sequence++;

    for (i = 0U; i < 3U; i++)
    {
        g_chassis_jy901s_debug.acc[i] = source->acc[i];
        g_chassis_jy901s_debug.gyro[i] = source->gyro[i];
        g_chassis_jy901s_debug.angle[i] = source->angle[i];
        g_chassis_jy901s_debug.mag[i] = source->mag[i];
    }

    g_chassis_jy901s_debug.temperature = source->temperature;
    g_chassis_jy901s_debug.update_flag = source->update_flag;
    g_chassis_jy901s_debug.last_update_tick = source->last_update_tick;
    g_chassis_jy901s_debug.online = source->online;

    g_chassis_jy901s_debug.update_sequence++;
}

/*
 * Store an initialization status string for external monitoring.
 * Thread-safe single-writer; read via chassis_get_status_text().
 */
static void chassis_set_init_status(const char *status)
{
    if (status == NULL) return;

    snprintf(chassis_init_status, sizeof(chassis_init_status), "%s", status);
}

void chassis_get_status_text(char *buf, uint32_t size)
{
    if (buf == NULL || size == 0U) return;

    if (qmc5883_calibrating) {
        snprintf(buf, size, "Mag calib %us", (unsigned)qmc5883_calibration_remaining_s);
    } else if (!chassis_init_done) {
        snprintf(buf, size, "%s", chassis_init_status);
    } else {
        snprintf(buf, size, "Modules Ready");
    }
}

/*
 * Copy current PID parameters to all four motor PID controllers.
 * Called after any PID parameter change to keep motors in sync.
 */
static void chassis_apply_pid_params(chassis_move_t *chassis)
{
    uint8_t i;

    if (chassis == NULL) return;

    for (i = 0U; i < 4U; i++)
    {
        chassis->motor[i].speed_pid.Kp = chassis->pid_param.kp;
        chassis->motor[i].speed_pid.Ki = chassis->pid_param.ki;
        chassis->motor[i].speed_pid.Kd = chassis->pid_param.kd;
        chassis->motor[i].speed_pid.OutMax = chassis->pid_param.max_out;
        chassis->motor[i].speed_pid.max_iout = chassis->pid_param.max_iout;
    }
}



/*
 * Copy current chassis telemetry into the provided output structure.
 * Thread-safe snapshot; callable from any task or ISR-deferred context.
 */
void chassis_get_telemetry(ChassisTelemetry_t *out)
{
    uint8_t i;

    if (out == NULL) return;

    taskENTER_CRITICAL();
    out->mode = chassis_move.mode;
    out->gps_lat = chassis_move.date_to_usb.current_lat;
    out->gps_lon = chassis_move.date_to_usb.current_lon;
    out->gps_sats = chassis_move.date_to_usb.current_sats;
    out->gps_last_update_tick = chassis_move.date_to_usb.last_update_tick;
    out->gps_year = chassis_move.date_to_usb.current_year;
    out->gps_month = chassis_move.date_to_usb.current_month;
    out->gps_day = chassis_move.date_to_usb.current_day;
    out->gps_week = chassis_move.date_to_usb.current_week;
    out->gps_hour = chassis_move.date_to_usb.current_hour;
    out->gps_minute = chassis_move.date_to_usb.current_minute;
    out->gps_second = chassis_move.date_to_usb.current_second;
    out->gps_time_update_tick = chassis_move.date_to_usb.time_update_tick;
    out->ins_roll = chassis_move.imu.ins.euler.roll;
    out->ins_pitch = chassis_move.imu.ins.euler.pitch;
    out->ins_yaw = chassis_move.imu.ins.euler.yaw;
    out->ins_last_update_tick = chassis_move.imu.ins_last_update_tick;
    for (i = 0U; i < 3U; i++)
    {
        out->jy901s_acc[i] = chassis_move.imu.jy901s.acc[i];
        out->jy901s_gyro[i] = chassis_move.imu.jy901s.gyro[i];
    }
    out->jy901s_last_update_tick = chassis_move.imu.jy901s.last_update_tick;
    out->jy901s_online = chassis_move.imu.jy901s.online;
    out->mag_yaw = chassis_move.imu.mag.yaw;
    out->mag_pitch = chassis_move.imu.mag.pitch;
    out->mag_roll = chassis_move.imu.mag.roll;
    out->mag_last_update_tick = chassis_move.imu.mag_last_update_tick;
    out->vx_set = chassis_move.Vx_set;
    out->vy_set = chassis_move.Vy_set;
    out->wz_set = chassis_move.Wz_set;
    for (i = 0U; i < 4U; i++)
    {
        out->motor_speed[i] = chassis_move.motor[i].speed;
        out->motor_speed_set[i] = chassis_move.motor[i].speed_set;
        out->motor_last_update_tick[i] = chassis_move.motor[i].last_update_tick;
    }
    out->qmc_calibrating = qmc5883_calibrating;
    out->qmc_calibration_remaining_s = qmc5883_calibration_remaining_s;
    out->gps_route_count = chassis_gps_route_count;
    for (i = 0U; i < MAX_WAYPOINTS; i++)
    {
        out->gps_route[i] = chassis_gps_route[i];
    }
    out->gps_current_wp_index = chassis_move.nav.current_wp_index;
    out->gps_is_navigating = chassis_move.nav.is_navigating;
    out->gps_loop_enable = chassis_move.nav.loop_enable;
    out->gps_nav_phase = chassis_move.nav.phase;
    out->gps_distance_error = chassis_move.nav.distance_error;
    out->gps_heading_error = chassis_move.nav.heading_error;
    taskEXIT_CRITICAL();
}

/*
 * Copy current chassis settings (gain, remote, PID) into the output structure.
 * Thread-safe snapshot; callable from any task or ISR-deferred context.
 */
void chassis_get_settings(ChassisSettings_t *out)
{
    if (out == NULL) return;

    taskENTER_CRITICAL();
    out->gain = chassis_move.gain;
    out->remote = chassis_move.remote;
    out->pid_param = chassis_move.pid_param;
    taskEXIT_CRITICAL();
}

/*
 * Set a single gain parameter for a specific control mode.
 * Takes effect immediately on the next chassis_set_control() call.
 */
void chassis_set_gain_param(ChassisGainParam_t param, float value)
{
    taskENTER_CRITICAL();
    switch (param)
    {
        case CHASSIS_GAIN_PARAM_GPS:    chassis_move.gain.gps = value;    break;
        case CHASSIS_GAIN_PARAM_INDOOR: chassis_move.gain.indoor = value; break;
        case CHASSIS_GAIN_PARAM_REMOTE: chassis_move.gain.remote = value; break;
        case CHASSIS_GAIN_PARAM_LINE:   chassis_move.gain.line = value;   break;
        case CHASSIS_GAIN_PARAM_VOICE:  chassis_move.gain.voice = value;  break;
        default: break;
    }
    taskEXIT_CRITICAL();
}

/*
 * Set a single remote control parameter.
 * Takes effect immediately on the next remote control update cycle.
 */
void chassis_set_remote_param(ChassisRemoteParam_t param, float value)
{
    taskENTER_CRITICAL();
    switch (param)
    {
        case CHASSIS_REMOTE_PARAM_BT_SPEED:        chassis_move.remote.bt_speed = value; break;
        case CHASSIS_REMOTE_PARAM_BT_WZ:           chassis_move.remote.bt_wz = value; break;
        case CHASSIS_REMOTE_PARAM_WECHAT_VX_SCALE: chassis_move.remote.wechat_vx_scale = value; break;
        case CHASSIS_REMOTE_PARAM_ROS_MAX_SPEED:   chassis_move.remote.ros_max_speed = value; break;
        default: break;
    }
    taskEXIT_CRITICAL();
}

/*
 * Set a single PID parameter and re-apply all PID params to all four motors.
 * Calling this while motors are running may cause a brief control transient.
 */
void chassis_set_pid_param(ChassisPidParam_t param, double value)
{
    taskENTER_CRITICAL();
    switch (param)
    {
        case CHASSIS_PID_PARAM_KP:      chassis_move.pid_param.kp = value; break;
        case CHASSIS_PID_PARAM_KI:      chassis_move.pid_param.ki = value; break;
        case CHASSIS_PID_PARAM_KD:      chassis_move.pid_param.kd = value; break;
        case CHASSIS_PID_PARAM_MAX_OUT: chassis_move.pid_param.max_out = value; break;
        default: break;
    }
    chassis_apply_pid_params(&chassis_move);
    taskEXIT_CRITICAL();
}

/*
 * Request a mode switch from the GUI / touch interface.
 * The request is processed in the next chassis_mode_change() cycle.
 * Invalid or unavailable mode requests are silently ignored there.
 */
void chassis_request_mode(CarMode_t mode)
{
    if (mode > CAR_MODE_VOICE) return;

    taskENTER_CRITICAL();
    gui_req_mode = (int)mode;
    taskEXIT_CRITICAL();
}


/* ============================================================
 *  Internal: Stop (zero all omnidirectional target speeds)
 * ============================================================ */
static void chassis_stop(chassis_move_t *chassis)
{
    chassis->Vx_set = 0.0f;
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = 0.0f;
}

static void chassis_clear_motor_output(chassis_move_t *chassis)
{
    uint8_t i;

    if (chassis == NULL) return;

    for (i = 0U; i < 4U; i++)
    {
        chassis->motor[i].speed_set = 0;
        chassis->motor[i].speed_pid.Target = 0.0;
        chassis->motor[i].speed_pid.Out = 0.0;
        chassis->motor[i].speed_pid.Error0 = 0.0;
        chassis->motor[i].speed_pid.Error1 = 0.0;
        chassis->motor[i].speed_pid.ErrorInt = 0.0;
        chassis->motor[i].speed_pid.D_error = 0.0;
    }
}

static double chassis_nmea_to_degree(double coordinate, char direction)
{
    double degrees = floor(coordinate / 100.0);
    double decimal = degrees + (coordinate - degrees * 100.0) / 60.0;

    if (direction == 'S' || direction == 'W')
    {
        decimal = -decimal;
    }

    return decimal;
}

static float chassis_slew_toward(float current, float target, float max_delta)
{
    float delta = target - current;

    if (delta > max_delta) return current + max_delta;
    if (delta < -max_delta) return current - max_delta;
    return target;
}

static void chassis_apply_control_smoothing(chassis_move_t *chassis)
{
    uint32_t now;
    float dt_s;
    float target_vx;
    float target_vy;
    float target_wz;

    if (chassis == NULL) return;

    now = HAL_GetTick();

    /*
     * Preserve the original pure-GPS dead-zone behavior: directly use the
     * navigation output (including Min_speed) and stop immediately when the
     * waypoint controller writes zero. GPS+ROS still uses the common smoother.
     */
    if (chassis->mode == CAR_MODE_GPS)
    {
        chassis_smoothed_vx = chassis->Vx_set;
        chassis_smoothed_vy = chassis->Vy_set;
        chassis_smoothed_wz = chassis->Wz_set;
        chassis_smooth_last_tick = now;

        g_gps_debug.navigation.update_sequence++;
        g_gps_debug.navigation.command_vx = chassis->Vx_set;
        g_gps_debug.navigation.command_vy = chassis->Vy_set;
        g_gps_debug.navigation.command_wz = chassis->Wz_set;
        g_gps_debug.navigation.update_sequence++;
        return;
    }

    if (chassis_smooth_last_tick == 0U)
    {
        dt_s = CHASSIS_SMOOTH_DEFAULT_DT_S;
    }
    else
    {
        dt_s = (float)(now - chassis_smooth_last_tick) * 0.001f;
        if (dt_s <= 0.0f) dt_s = CHASSIS_SMOOTH_DEFAULT_DT_S;
        if (dt_s > CHASSIS_SMOOTH_MAX_DT_S) dt_s = CHASSIS_SMOOTH_MAX_DT_S;
    }
    chassis_smooth_last_tick = now;

    target_vx = chassis->Vx_set;
    target_vy = chassis->Vy_set;
    target_wz = chassis->Wz_set;

    chassis_smoothed_vx = chassis_slew_toward(
        chassis_smoothed_vx, target_vx, CHASSIS_TRANSLATION_SLEW_PER_S * dt_s);
    chassis_smoothed_vy = chassis_slew_toward(
        chassis_smoothed_vy, target_vy, CHASSIS_TRANSLATION_SLEW_PER_S * dt_s);
    chassis_smoothed_wz = chassis_slew_toward(
        chassis_smoothed_wz, target_wz, CHASSIS_ROTATION_SLEW_PER_S * dt_s);

    chassis->Vx_set = chassis_smoothed_vx;
    chassis->Vy_set = chassis_smoothed_vy;
    chassis->Wz_set = chassis_smoothed_wz;

    if (chassis->mode == CAR_MODE_GPS_ROS)
    {
        g_gps_debug.navigation.update_sequence++;
        g_gps_debug.navigation.command_vx = chassis->Vx_set;
        g_gps_debug.navigation.command_vy = chassis->Vy_set;
        g_gps_debug.navigation.command_wz = chassis->Wz_set;
        g_gps_debug.navigation.update_sequence++;
    }
}

static void chassis_reset_control_smoothing(chassis_move_t *chassis)
{
    chassis_smoothed_vx = 0.0f;
    chassis_smoothed_vy = 0.0f;
    chassis_smoothed_wz = 0.0f;
    chassis_smooth_last_tick = HAL_GetTick();

    if (chassis != NULL)
    {
        chassis->Vx_set = 0.0f;
        chassis->Vy_set = 0.0f;
        chassis->Wz_set = 0.0f;
    }
}

static void chassis_force_powerless_output(chassis_move_t *chassis)
{
    if (chassis != NULL)
    {
        chassis_stop(chassis);
        chassis_clear_motor_output(chassis);
        chassis_reset_control_smoothing(chassis);
    }

    /* TB6612 STBY is tied high. PWM=0 plus both direction inputs low gives coast. */
    Motor_SetAllPWM(0, 0, 0, 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 |
                             GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
}

static uint32_t chassis_gps_route_crc32(const uint8_t *data, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t i;
    uint8_t bit;

    if (data == NULL) return 0U;

    for (i = 0U; i < size; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
        }
    }

    return ~crc;
}

static uint8_t chassis_gps_route_record_valid(uint32_t address,
                                               uint32_t *sequence,
                                               uint8_t *count)
{
    const uint8_t *raw = (const uint8_t *)address;
    uint32_t magic;
    uint16_t version;
    uint32_t stored_crc;
    uint32_t calculated_crc;
    uint8_t i;
    double lat;
    double lon;

    memcpy(&magic, &raw[0], sizeof(magic));
    memcpy(&version, &raw[4], sizeof(version));
    if (magic != GPS_ROUTE_FLASH_MAGIC || version != GPS_ROUTE_FLASH_VERSION) return 0U;
    if (raw[6] > MAX_WAYPOINTS) return 0U;

    memcpy(&stored_crc, &raw[GPS_ROUTE_FLASH_CRC_OFFSET], sizeof(stored_crc));
    calculated_crc = chassis_gps_route_crc32(raw, GPS_ROUTE_FLASH_CRC_OFFSET);
    if (stored_crc != calculated_crc) return 0U;

    for (i = 0U; i < raw[6]; i++)
    {
        memcpy(&lat, &raw[GPS_ROUTE_FLASH_DATA_OFFSET + (uint32_t)i * 16U], sizeof(lat));
        memcpy(&lon, &raw[GPS_ROUTE_FLASH_DATA_OFFSET + (uint32_t)i * 16U + 8U], sizeof(lon));
        if (lat != lat || lon != lon || lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return 0U;
    }

    if (sequence != NULL) memcpy(sequence, &raw[GPS_ROUTE_FLASH_SEQUENCE_OFFSET], sizeof(*sequence));
    if (count != NULL) *count = raw[6];
    return 1U;
}

static uint8_t chassis_gps_route_sequence_newer(uint32_t left, uint32_t right)
{
    return ((int32_t)(left - right) > 0) ? 1U : 0U;
}

static void chassis_load_gps_route_from_flash(void)
{
    uint32_t sequence[2] = {0U, 0U};
    uint8_t count[2] = {0U, 0U};
    uint8_t valid[2];
    uint8_t slot;
    uint8_t i;
    const uint8_t *raw;

    valid[0] = chassis_gps_route_record_valid(GPS_ROUTE_FLASH_SLOT0_ADDR, &sequence[0], &count[0]);
    valid[1] = chassis_gps_route_record_valid(GPS_ROUTE_FLASH_SLOT1_ADDR, &sequence[1], &count[1]);

    if (!valid[0] && !valid[1])
    {
        chassis_gps_route_count = 0U;
        g_chassis_gps_route_count_debug = 0U;
        g_chassis_gps_route_storage_status_debug = CHASSIS_GPS_ROUTE_STORAGE_EMPTY;
        g_chassis_gps_route_storage_sequence_debug = 0U;
        g_chassis_gps_route_storage_slot_debug = 0xFFU;
        return;
    }

    if (valid[0] && valid[1])
    {
        slot = chassis_gps_route_sequence_newer(sequence[1], sequence[0]) ? 1U : 0U;
    }
    else
    {
        slot = valid[1] ? 1U : 0U;
    }

    raw = (const uint8_t *)(slot == 0U ? GPS_ROUTE_FLASH_SLOT0_ADDR : GPS_ROUTE_FLASH_SLOT1_ADDR);
    chassis_gps_route_count = count[slot];
    for (i = 0U; i < MAX_WAYPOINTS; i++)
    {
        if (i < chassis_gps_route_count)
        {
            memcpy(&chassis_gps_route[i].lat,
                   &raw[GPS_ROUTE_FLASH_DATA_OFFSET + (uint32_t)i * 16U],
                   sizeof(chassis_gps_route[i].lat));
            memcpy(&chassis_gps_route[i].lon,
                   &raw[GPS_ROUTE_FLASH_DATA_OFFSET + (uint32_t)i * 16U + 8U],
                   sizeof(chassis_gps_route[i].lon));
        }
        else
        {
            chassis_gps_route[i].lat = 0.0;
            chassis_gps_route[i].lon = 0.0;
        }
    }

    g_chassis_gps_route_count_debug = chassis_gps_route_count;
    g_chassis_gps_route_storage_status_debug = (chassis_gps_route_count > 0U) ?
                                               CHASSIS_GPS_ROUTE_STORAGE_LOADED :
                                               CHASSIS_GPS_ROUTE_STORAGE_EMPTY;
    g_chassis_gps_route_storage_sequence_debug = sequence[slot];
    g_chassis_gps_route_storage_slot_debug = slot;
}

static uint8_t chassis_save_gps_route_to_flash(void)
{
    uint8_t *raw = (uint8_t *)chassis_gps_route_flash_record_words;
    uint32_t sequence[2] = {0U, 0U};
    uint8_t valid[2];
    uint8_t current_slot;
    uint8_t target_slot;
    uint32_t target_address;
    uint32_t target_sector;
    uint32_t next_sequence;
    uint32_t crc;
    uint32_t offset;
    uint32_t sector_error = 0U;
    uint32_t magic = GPS_ROUTE_FLASH_MAGIC;
    uint16_t version = GPS_ROUTE_FLASH_VERSION;
    uint8_t i;
    FLASH_EraseInitTypeDef erase;
    HAL_StatusTypeDef status;

    valid[0] = chassis_gps_route_record_valid(GPS_ROUTE_FLASH_SLOT0_ADDR, &sequence[0], NULL);
    valid[1] = chassis_gps_route_record_valid(GPS_ROUTE_FLASH_SLOT1_ADDR, &sequence[1], NULL);

    if (valid[0] && valid[1]) current_slot = chassis_gps_route_sequence_newer(sequence[1], sequence[0]) ? 1U : 0U;
    else if (valid[1]) current_slot = 1U;
    else current_slot = 0U;

    if (!valid[0] && !valid[1])
    {
        target_slot = 0U;
        next_sequence = 1U;
    }
    else
    {
        target_slot = (current_slot == 0U) ? 1U : 0U;
        next_sequence = sequence[current_slot] + 1U;
        if (next_sequence == 0U) next_sequence = 1U;
    }

    target_address = (target_slot == 0U) ? GPS_ROUTE_FLASH_SLOT0_ADDR : GPS_ROUTE_FLASH_SLOT1_ADDR;
    target_sector = (target_slot == 0U) ? GPS_ROUTE_FLASH_SLOT0_SECTOR : GPS_ROUTE_FLASH_SLOT1_SECTOR;

    memset(chassis_gps_route_flash_record_words, 0xFF, sizeof(chassis_gps_route_flash_record_words));
    memcpy(&raw[0], &magic, sizeof(magic));
    memcpy(&raw[4], &version, sizeof(version));
    raw[6] = chassis_gps_route_count;
    raw[7] = 0U;
    memcpy(&raw[GPS_ROUTE_FLASH_SEQUENCE_OFFSET], &next_sequence, sizeof(next_sequence));
    for (i = 0U; i < chassis_gps_route_count; i++)
    {
        memcpy(&raw[GPS_ROUTE_FLASH_DATA_OFFSET + (uint32_t)i * 16U],
               &chassis_gps_route[i].lat,
               sizeof(chassis_gps_route[i].lat));
        memcpy(&raw[GPS_ROUTE_FLASH_DATA_OFFSET + (uint32_t)i * 16U + 8U],
               &chassis_gps_route[i].lon,
               sizeof(chassis_gps_route[i].lon));
    }
    crc = chassis_gps_route_crc32(raw, GPS_ROUTE_FLASH_CRC_OFFSET);
    memcpy(&raw[GPS_ROUTE_FLASH_CRC_OFFSET], &crc, sizeof(crc));

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_2;
    erase.Sector = target_sector;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        g_chassis_gps_route_storage_status_debug = CHASSIS_GPS_ROUTE_STORAGE_ERASE_ERROR;
        return 0U;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        g_chassis_gps_route_storage_status_debug = CHASSIS_GPS_ROUTE_STORAGE_ERASE_ERROR;
        return 0U;
    }

    for (offset = 0U; offset < GPS_ROUTE_FLASH_RECORD_SIZE; offset += GPS_ROUTE_FLASH_WORD_SIZE)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                   target_address + offset,
                                   (uint32_t)&raw[offset]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            g_chassis_gps_route_storage_status_debug = CHASSIS_GPS_ROUTE_STORAGE_PROGRAM_ERROR;
            return 0U;
        }
    }
    HAL_FLASH_Lock();

    SCB_InvalidateDCache_by_Addr((uint32_t *)target_address, GPS_ROUTE_FLASH_RECORD_SIZE);
    __DSB();
    __ISB();

    if (!chassis_gps_route_record_valid(target_address, NULL, NULL))
    {
        g_chassis_gps_route_storage_status_debug = CHASSIS_GPS_ROUTE_STORAGE_VERIFY_ERROR;
        return 0U;
    }

    g_chassis_gps_route_storage_status_debug = CHASSIS_GPS_ROUTE_STORAGE_SAVED;
    g_chassis_gps_route_storage_sequence_debug = next_sequence;
    g_chassis_gps_route_storage_slot_debug = target_slot;
    return 1U;
}

static char *chassis_gps_route_sd_next_line(char **cursor)
{
    char *line;
    char *end;
    size_t length;

    if (cursor == NULL || *cursor == NULL || **cursor == '\0') return NULL;

    line = *cursor;
    end = strchr(line, '\n');
    if (end != NULL)
    {
        *end = '\0';
        *cursor = end + 1;
    }
    else
    {
        *cursor = line + strlen(line);
    }

    length = strlen(line);
    if (length > 0U && line[length - 1U] == '\r') line[length - 1U] = '\0';
    return line;
}

static uint8_t chassis_gps_route_sd_parse(uint8_t *count)
{
    char *cursor;
    char *line;
    char *end;
    char *crc_line;
    unsigned long parsed_count;
    unsigned long parsed_index;
    unsigned long stored_crc;
    uint32_t calculated_crc;
    uint32_t crc_length;
    uint8_t i;
    double lat;
    double lon;

    if (count == NULL) return 0U;

    crc_line = strstr(chassis_gps_route_sd_buffer, "CRC32=");
    if (crc_line == NULL ||
        (crc_line != chassis_gps_route_sd_buffer && *(crc_line - 1) != '\n')) return 0U;

    stored_crc = strtoul(crc_line + 6, &end, 16);
    if (end == crc_line + 6 || (*end != '\r' && *end != '\n' && *end != '\0')) return 0U;
    crc_length = (uint32_t)(crc_line - chassis_gps_route_sd_buffer);
    calculated_crc = chassis_gps_route_crc32((const uint8_t *)chassis_gps_route_sd_buffer,
                                              crc_length);
    if ((uint32_t)stored_crc != calculated_crc) return 0U;

    cursor = chassis_gps_route_sd_buffer;
    line = chassis_gps_route_sd_next_line(&cursor);
    if (line == NULL || strcmp(line, "GPS_ROUTE_V1") != 0) return 0U;

    line = chassis_gps_route_sd_next_line(&cursor);
    if (line == NULL || strncmp(line, "COUNT=", 6) != 0) return 0U;
    parsed_count = strtoul(line + 6, &end, 10);
    if (*end != '\0' || parsed_count > MAX_WAYPOINTS) return 0U;

    line = chassis_gps_route_sd_next_line(&cursor);
    if (line == NULL || strcmp(line, "INDEX,LATITUDE,LONGITUDE") != 0) return 0U;

    for (i = 0U; i < (uint8_t)parsed_count; i++)
    {
        line = chassis_gps_route_sd_next_line(&cursor);
        if (line == NULL) return 0U;

        parsed_index = strtoul(line, &end, 10);
        if (parsed_index != (unsigned long)i + 1UL || *end != ',') return 0U;

        lat = strtod(end + 1, &end);
        if (*end != ',') return 0U;
        lon = strtod(end + 1, &end);
        if (*end != '\0' || lat != lat || lon != lon ||
            lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return 0U;

        chassis_gps_route_sd_points[i].lat = lat;
        chassis_gps_route_sd_points[i].lon = lon;
    }

    line = chassis_gps_route_sd_next_line(&cursor);
    if (line == NULL || strncmp(line, "CRC32=", 6) != 0) return 0U;

    *count = (uint8_t)parsed_count;
    return 1U;
}

static uint8_t chassis_load_gps_route_sd_file_locked(const char *path, uint8_t *count)
{
    FRESULT result;
    FSIZE_t size;
    UINT bytes_read = 0U;

    if (path == NULL || count == NULL) return GPS_ROUTE_SD_LOAD_ERROR;

    result = f_open(&chassis_gps_route_sd_file, path, FA_READ | FA_OPEN_EXISTING);
    g_chassis_gps_route_sd_last_fresult_debug = (uint32_t)result;
    if (result == FR_NO_FILE || result == FR_NO_PATH) return GPS_ROUTE_SD_LOAD_NOT_FOUND;
    if (result != FR_OK)
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_OPEN_ERROR;
        return GPS_ROUTE_SD_LOAD_ERROR;
    }

    size = f_size(&chassis_gps_route_sd_file);
    if (size == 0U || size >= GPS_ROUTE_SD_BUFFER_SIZE)
    {
        f_close(&chassis_gps_route_sd_file);
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_FORMAT_ERROR;
        return GPS_ROUTE_SD_LOAD_ERROR;
    }

    result = f_read(&chassis_gps_route_sd_file,
                    chassis_gps_route_sd_buffer,
                    (UINT)size,
                    &bytes_read);
    f_close(&chassis_gps_route_sd_file);
    g_chassis_gps_route_sd_last_fresult_debug = (uint32_t)result;
    if (result != FR_OK || bytes_read != (UINT)size)
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_READ_ERROR;
        return GPS_ROUTE_SD_LOAD_ERROR;
    }

    chassis_gps_route_sd_buffer[bytes_read] = '\0';
    if (!chassis_gps_route_sd_parse(count))
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_FORMAT_ERROR;
        return GPS_ROUTE_SD_LOAD_ERROR;
    }

    return GPS_ROUTE_SD_LOAD_OK;
}

static uint8_t chassis_save_gps_route_to_sd_locked(void)
{
    FRESULT result;
    UINT bytes_written = 0U;
    uint32_t crc;
    int written;
    uint32_t offset = 0U;
    uint8_t verify_count = 0U;
    uint8_t i;

    result = f_mkdir(GPS_ROUTE_SD_DIR);
    if (result != FR_OK && result != FR_EXIST)
    {
        g_chassis_gps_route_sd_last_fresult_debug = (uint32_t)result;
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_OPEN_ERROR;
        return 0U;
    }

    written = snprintf(chassis_gps_route_sd_buffer,
                       GPS_ROUTE_SD_BUFFER_SIZE,
                       "GPS_ROUTE_V1\r\nCOUNT=%u\r\nINDEX,LATITUDE,LONGITUDE\r\n",
                       (unsigned)chassis_gps_route_count);
    if (written < 0 || (uint32_t)written >= GPS_ROUTE_SD_BUFFER_SIZE) return 0U;
    offset = (uint32_t)written;

    for (i = 0U; i < chassis_gps_route_count; i++)
    {
        written = snprintf(&chassis_gps_route_sd_buffer[offset],
                           GPS_ROUTE_SD_BUFFER_SIZE - offset,
                           "%u,%.10f,%.10f\r\n",
                           (unsigned)(i + 1U),
                           chassis_gps_route[i].lat,
                           chassis_gps_route[i].lon);
        if (written < 0 || (uint32_t)written >= GPS_ROUTE_SD_BUFFER_SIZE - offset) return 0U;
        offset += (uint32_t)written;
    }

    crc = chassis_gps_route_crc32((const uint8_t *)chassis_gps_route_sd_buffer, offset);
    written = snprintf(&chassis_gps_route_sd_buffer[offset],
                       GPS_ROUTE_SD_BUFFER_SIZE - offset,
                       "CRC32=%08lX\r\n",
                       (unsigned long)crc);
    if (written < 0 || (uint32_t)written >= GPS_ROUTE_SD_BUFFER_SIZE - offset) return 0U;
    offset += (uint32_t)written;

    result = f_open(&chassis_gps_route_sd_file,
                    GPS_ROUTE_SD_TEMP,
                    FA_WRITE | FA_CREATE_ALWAYS);
    g_chassis_gps_route_sd_last_fresult_debug = (uint32_t)result;
    if (result != FR_OK)
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_OPEN_ERROR;
        return 0U;
    }

    result = f_write(&chassis_gps_route_sd_file,
                     chassis_gps_route_sd_buffer,
                     (UINT)offset,
                     &bytes_written);
    if (result == FR_OK && bytes_written == (UINT)offset) result = f_sync(&chassis_gps_route_sd_file);
    f_close(&chassis_gps_route_sd_file);
    g_chassis_gps_route_sd_last_fresult_debug = (uint32_t)result;
    if (result != FR_OK || bytes_written != (UINT)offset)
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_WRITE_ERROR;
        return 0U;
    }

    if (chassis_load_gps_route_sd_file_locked(GPS_ROUTE_SD_TEMP, &verify_count) != GPS_ROUTE_SD_LOAD_OK ||
        verify_count != chassis_gps_route_count)
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_VERIFY_ERROR;
        return 0U;
    }

    for (i = 0U; i < verify_count; i++)
    {
        if (fabs(chassis_gps_route_sd_points[i].lat - chassis_gps_route[i].lat) > 0.00000001 ||
            fabs(chassis_gps_route_sd_points[i].lon - chassis_gps_route[i].lon) > 0.00000001)
        {
            g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_VERIFY_ERROR;
            return 0U;
        }
    }

    result = f_unlink(GPS_ROUTE_SD_FILE);
    if (result != FR_OK && result != FR_NO_FILE && result != FR_NO_PATH)
    {
        g_chassis_gps_route_sd_last_fresult_debug = (uint32_t)result;
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_WRITE_ERROR;
        return 0U;
    }

    result = f_rename(GPS_ROUTE_SD_TEMP, GPS_ROUTE_SD_FILE);
    g_chassis_gps_route_sd_last_fresult_debug = (uint32_t)result;
    if (result != FR_OK)
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_WRITE_ERROR;
        return 0U;
    }

    g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_SAVED;
    g_chassis_gps_route_sd_save_count_debug++;
    return 1U;
}

static uint8_t chassis_delete_gps_route_from_sd_locked(void)
{
    FRESULT result_file;
    FRESULT result_temp;

    result_file = f_unlink(GPS_ROUTE_SD_FILE);
    result_temp = f_unlink(GPS_ROUTE_SD_TEMP);
    g_chassis_gps_route_sd_last_fresult_debug = (uint32_t)result_file;

    if ((result_file != FR_OK && result_file != FR_NO_FILE && result_file != FR_NO_PATH) ||
        (result_temp != FR_OK && result_temp != FR_NO_FILE && result_temp != FR_NO_PATH))
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_WRITE_ERROR;
        return 0U;
    }

    g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_DELETED;
    g_chassis_gps_route_sd_delete_count_debug++;
    return 1U;
}

static void chassis_apply_gps_route_from_sd(chassis_move_t *chassis, uint8_t count)
{
    uint8_t i;

    chassis_gps_route_count = count;
    for (i = 0U; i < MAX_WAYPOINTS; i++)
    {
        if (i < count)
        {
            chassis_gps_route[i] = chassis_gps_route_sd_points[i];
        }
        else
        {
            chassis_gps_route[i].lat = 0.0;
            chassis_gps_route[i].lon = 0.0;
        }
    }
    g_chassis_gps_route_count_debug = count;

    if (chassis != NULL &&
        (chassis->mode == CAR_MODE_GPS || chassis->mode == CAR_MODE_GPS_ROS))
    {
        if (count > 0U) Navigation_Set_Route_Loop(&chassis->nav, chassis_gps_route, count);
        else Navigation_Stop(chassis);
    }
}

static void chassis_persist_gps_route(void)
{
    uint8_t sd_saved = 0U;

    chassis_gps_route_runtime_modified = 1U;
    if (bsp_tf_fs_is_mounted() && bsp_tf_fs_lock(GPS_ROUTE_SD_LOCK_TICKS))
    {
        sd_saved = (chassis_gps_route_count > 0U) ?
                   chassis_save_gps_route_to_sd_locked() :
                   chassis_delete_gps_route_from_sd_locked();
        bsp_tf_fs_unlock();
    }
    else
    {
        g_chassis_gps_route_sd_status_debug = bsp_tf_fs_is_mounted() ?
                                               CHASSIS_GPS_ROUTE_SD_LOCK_ERROR :
                                               CHASSIS_GPS_ROUTE_SD_FLASH_FALLBACK;
    }

    chassis_gps_route_sd_save_pending = sd_saved ? 0U : 1U;
    (void)chassis_save_gps_route_to_flash();
}

static void chassis_service_gps_route_sd(chassis_move_t *chassis)
{
    uint32_t now;
    uint8_t load_result;
    uint8_t loaded_count = 0U;
    uint8_t sd_saved;
    uint8_t loaded_from_temp = 0U;

    if (!bsp_tf_fs_is_mounted()) return;

    now = HAL_GetTick();
    if (chassis_gps_route_sd_sync_done && !chassis_gps_route_sd_save_pending) return;
    if (chassis_gps_route_sd_sync_done &&
        (now - chassis_gps_route_sd_last_retry_tick) < GPS_ROUTE_SD_RETRY_MS) return;
    if (!bsp_tf_fs_lock(GPS_ROUTE_SD_LOCK_TICKS))
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_LOCK_ERROR;
        return;
    }

    chassis_gps_route_sd_last_retry_tick = now;

    if (chassis_gps_route_sd_sync_done || chassis_gps_route_runtime_modified)
    {
        sd_saved = (chassis_gps_route_count > 0U) ?
                   chassis_save_gps_route_to_sd_locked() :
                   chassis_delete_gps_route_from_sd_locked();
        chassis_gps_route_sd_save_pending = sd_saved ? 0U : 1U;
        chassis_gps_route_sd_sync_done = 1U;
        bsp_tf_fs_unlock();
        return;
    }

    load_result = chassis_load_gps_route_sd_file_locked(GPS_ROUTE_SD_FILE, &loaded_count);
    if (load_result != GPS_ROUTE_SD_LOAD_OK)
    {
        load_result = chassis_load_gps_route_sd_file_locked(GPS_ROUTE_SD_TEMP, &loaded_count);
        loaded_from_temp = (load_result == GPS_ROUTE_SD_LOAD_OK) ? 1U : 0U;
    }

    if (load_result == GPS_ROUTE_SD_LOAD_OK)
    {
        chassis_apply_gps_route_from_sd(chassis, loaded_count);
        if (loaded_from_temp)
        {
            (void)f_unlink(GPS_ROUTE_SD_FILE);
            (void)f_rename(GPS_ROUTE_SD_TEMP, GPS_ROUTE_SD_FILE);
        }
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_LOADED;
        g_chassis_gps_route_sd_load_count_debug++;
        chassis_gps_route_sd_save_pending = 0U;
    }
    else if (chassis_gps_route_count > 0U)
    {
        sd_saved = chassis_save_gps_route_to_sd_locked();
        chassis_gps_route_sd_save_pending = sd_saved ? 0U : 1U;
        if (!sd_saved) g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_FLASH_FALLBACK;
    }
    else
    {
        g_chassis_gps_route_sd_status_debug = CHASSIS_GPS_ROUTE_SD_NOT_FOUND;
        chassis_gps_route_sd_save_pending = 0U;
    }

    chassis_gps_route_sd_sync_done = 1U;
    bsp_tf_fs_unlock();

    if (load_result == GPS_ROUTE_SD_LOAD_OK)
    {
        (void)chassis_save_gps_route_to_flash();
    }
}

static void chassis_clear_gps_route(chassis_move_t *chassis)
{
    uint8_t i;
    uint8_t is_nav_mode;

    is_nav_mode = (chassis != NULL &&
                  (chassis->mode == CAR_MODE_GPS || chassis->mode == CAR_MODE_GPS_ROS)) ? 1U : 0U;

    for (i = 0U; i < MAX_WAYPOINTS; i++)
    {
        chassis_gps_route[i].lat = 0.0;
        chassis_gps_route[i].lon = 0.0;
    }
    chassis_gps_route_count = 0U;
    g_chassis_gps_route_count_debug = 0U;
    g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_CLEARED;

    if (chassis != NULL)
    {
        if (is_nav_mode) Navigation_Stop(chassis);

        for (i = 0U; i < MAX_WAYPOINTS; i++)
        {
            chassis->nav.route[i].lat = 0.0;
            chassis->nav.route[i].lon = 0.0;
        }
        chassis->nav.total_waypoints = 0U;
        chassis->nav.current_wp_index = 0U;
        chassis->nav.target_pos.lat = 0.0;
        chassis->nav.target_pos.lon = 0.0;
        chassis->nav.distance_error = 0.0f;
        chassis->nav.heading_error = 0.0f;
    }

    chassis_persist_gps_route();
}

static void chassis_process_usb_gps_route(chassis_move_t *chassis)
{
    gps_route_cmd_t command = USB_GetGpsRouteCmd();
    uint16_t expected_mask;
    uint8_t i;

    if (command.update_sequence == chassis_usb_route_sequence) return;
    chassis_usb_route_sequence = command.update_sequence;

    if (command.command == JETSON_GPS_ROUTE_SPEED)
    {
        float speed_percent = (float)command.latitude;
        if (speed_percent != speed_percent) return;
        if (speed_percent < 20.0f) speed_percent = 20.0f;
        if (speed_percent > 100.0f) speed_percent = 100.0f;
        Navigation_Set_Speed_Percent((uint8_t)(speed_percent + 0.5f));
        return;
    }

    if (command.command == JETSON_GPS_ROUTE_CLEAR)
    {
        chassis_usb_route_expected_count = 0U;
        chassis_usb_route_received_mask = 0U;
        chassis_clear_gps_route(chassis);
        return;
    }

    if (command.command == JETSON_GPS_ROUTE_BEGIN)
    {
        if (command.total == 0U || command.total > MAX_WAYPOINTS) return;
        chassis_usb_route_expected_count = command.total;
        chassis_usb_route_loop_enable = command.loop_enable ? 1U : 0U;
        chassis_usb_route_received_mask = 0U;
        memset(chassis_usb_route_pending, 0, sizeof(chassis_usb_route_pending));
        return;
    }

    if (command.command == JETSON_GPS_ROUTE_POINT)
    {
        if (chassis_usb_route_expected_count == 0U ||
            command.total != chassis_usb_route_expected_count ||
            command.index >= chassis_usb_route_expected_count ||
            command.latitude != command.latitude ||
            command.longitude != command.longitude ||
            command.latitude < -90.0 || command.latitude > 90.0 ||
            command.longitude < -180.0 || command.longitude > 180.0) return;

        chassis_usb_route_pending[command.index].lat = command.latitude;
        chassis_usb_route_pending[command.index].lon = command.longitude;
        chassis_usb_route_received_mask |= (uint16_t)(1U << command.index);
        return;
    }

    if (command.command != JETSON_GPS_ROUTE_COMMIT ||
        command.total != chassis_usb_route_expected_count ||
        chassis_usb_route_expected_count == 0U) return;

    expected_mask = (uint16_t)((1U << chassis_usb_route_expected_count) - 1U);
    if (chassis_usb_route_received_mask != expected_mask) return;

    chassis_gps_route_count = chassis_usb_route_expected_count;
    for (i = 0U; i < MAX_WAYPOINTS; i++)
    {
        if (i < chassis_gps_route_count) chassis_gps_route[i] = chassis_usb_route_pending[i];
        else
        {
            chassis_gps_route[i].lat = 0.0;
            chassis_gps_route[i].lon = 0.0;
        }
    }
    g_chassis_gps_route_count_debug = chassis_gps_route_count;
    g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_POINT_ADDED;

    if (chassis != NULL && chassis->mode == CAR_MODE_GPS)
    {
        if (chassis_is_gps_online(chassis) && chassis_is_mag_online(chassis))
        {
            if (chassis_usb_route_loop_enable)
                Navigation_Set_Route_Loop(&chassis->nav, chassis_gps_route, chassis_gps_route_count);
            else
                Navigation_Set_Route(&chassis->nav, chassis_gps_route, chassis_gps_route_count);
        }
        else
        {
            /* Do not allow an older active route to resume when sensors recover. */
            Navigation_Stop(chassis);
        }
    }
    chassis_persist_gps_route();
    chassis_usb_route_expected_count = 0U;
    chassis_usb_route_received_mask = 0U;
}

static uint8_t chassis_add_current_gps_point(chassis_move_t *chassis)
{
    PT_GNGGA gga;
    GPS_Point_t point;

    if (chassis_gps_route_count >= MAX_WAYPOINTS)
    {
        g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_FULL;
        return 0U;
    }

    gga = GetGNGGA();
    if (gga == NULL || gga->last_update_tick == 0U ||
        (HAL_GetTick() - gga->last_update_tick) > GPS_ONLINE_TIMEOUT_MS ||
        gga->qf < 1U || gga->lat < 1.0 || gga->lon < 1.0 ||
        (gga->lat_dir != 'N' && gga->lat_dir != 'S') ||
        (gga->lon_dir != 'E' && gga->lon_dir != 'W'))
    {
        g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_INVALID_FIX;
        return 0U;
    }

    point.lat = chassis_nmea_to_degree(gga->lat, gga->lat_dir);
    point.lon = chassis_nmea_to_degree(gga->lon, gga->lon_dir);
    chassis_gps_route[chassis_gps_route_count] = point;
    chassis_gps_route_count++;
    g_chassis_gps_route_count_debug = chassis_gps_route_count;
    g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_POINT_ADDED;

    if (chassis != NULL &&
        (chassis->mode == CAR_MODE_GPS || chassis->mode == CAR_MODE_GPS_ROS))
    {
        Navigation_Set_Route_Loop(&chassis->nav,
                                  chassis_gps_route,
                                  chassis_gps_route_count);
    }

    chassis_persist_gps_route();

    return 1U;
}

static uint8_t chassis_add_remote_gps_point(chassis_move_t *chassis,
                                            const BT_RemotePoint_t *remote_point)
{
    GPS_Point_t point;

    if (remote_point == NULL ||
        remote_point->lat != remote_point->lat ||
        remote_point->lon != remote_point->lon ||
        remote_point->lat < -90.0 || remote_point->lat > 90.0 ||
        remote_point->lon < -180.0 || remote_point->lon > 180.0)
    {
        g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_INVALID_FIX;
        return 0U;
    }

    if (chassis_gps_route_count >= MAX_WAYPOINTS)
    {
        g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_FULL;
        return 0U;
    }

    point.lat = remote_point->lat;
    point.lon = remote_point->lon;
    chassis_gps_route[chassis_gps_route_count] = point;
    chassis_gps_route_count++;
    g_chassis_gps_route_count_debug = chassis_gps_route_count;
    g_chassis_gps_route_last_result_debug = CHASSIS_GPS_ROUTE_RESULT_POINT_ADDED;

    if (chassis != NULL &&
        (chassis->mode == CAR_MODE_GPS || chassis->mode == CAR_MODE_GPS_ROS))
    {
        Navigation_Set_Route_Loop(&chassis->nav,
                                  chassis_gps_route,
                                  chassis_gps_route_count);
    }

    chassis_persist_gps_route();
    return 1U;
}

static void chassis_init_magnetometer_once(chassis_move_t *chassis)
{
    if (chassis_mag_initialized || chassis_mag_initializing) return;

    chassis_mag_initializing = 1U;
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96QMC5883");
    QMC5883_SetCalibrationStepCallback(chassis_mag_calibration_step);
    Magnetometer_Calibration();
    QMC5883_SetCalibrationStepCallback(NULL);
    Motor_SetAllPWM(0, 0, 0, 0);
    chassis_stop(chassis);
    chassis_clear_motor_output(chassis);
    chassis_reset_control_smoothing(chassis);

    if (chassis != NULL)
    {
        QMC5883_GetAngles(&chassis->imu.mag);
        chassis->imu.mag_last_update_tick = chassis->imu.mag.last_update_tick;
    }

    chassis_mag_initialized = 1U;
    chassis_mag_initializing = 0U;
    g_chassis_mag_initialized_debug = 1U;
    chassis_set_init_status("\xE6\xA8\xA1\xE5\x9D\x97\xE5\xB7\xB2\xE5\xB0\xB1\xE7\xBB\xAA");
}

/* ============================================================
 *  Internal: Start GPS cyclic cruise
 * ============================================================ */
static void chassis_start_gps_navigation(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    if (chassis_gps_route_count == 0U)
    {
        Navigation_Stop(chassis);
        return;
    }

    Navigation_Set_Route_Loop(&chassis->nav,
                              chassis_gps_route,
                              chassis_gps_route_count);
}

static uint8_t chassis_is_jetson_online(chassis_move_t *chassis)
{
    if (chassis == NULL) return 0U;

    return (chassis->jetson_last_tick != 0U &&
            (uint32_t)(HAL_GetTick() - chassis->jetson_last_tick) < JETSON_TIMEOUT_MS) ? 1U : 0U;
}

static uint8_t chassis_is_moving_for_road_classification(const chassis_move_t *chassis)
{
    uint8_t i;

    if (chassis == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (fabs(chassis->motor[i].speed) >= ROAD_CLASSIFICATION_MOVING_SPEED_THRESHOLD)
        {
            return 1U;
        }
    }

    return 0U;
}

/*
 * Return 1 if the given year is a leap year, 0 otherwise.
 * Handles Gregorian rules: every 4th year, except centuries, but including every 400th.
 */
static uint8_t chassis_is_leap_year(uint16_t year)
{
    return ((year % 4U == 0U && year % 100U != 0U) || (year % 400U == 0U)) ? 1U : 0U;
}

/*
 * Return the number of days in a given month of a given year.
 * Handles February leap-year adjustment automatically.
 * Returns 31 for invalid month numbers.
 */
static uint8_t chassis_days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t days[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    if (month < 1U || month > 12U) return 31U;
    if (month == 2U && chassis_is_leap_year(year)) return 29U;

    return days[month - 1U];
}

/*
 * Return the day of the week (0 = Sunday .. 6 = Saturday) for a given date.
 * Uses a Zeller-like congruence formula. Returns 0 for invalid inputs.
 */
static uint8_t chassis_weekday(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t offset[12] = {0U, 3U, 2U, 5U, 0U, 3U, 5U, 1U, 4U, 6U, 2U, 4U};
    uint16_t y = year;

    if (month < 1U || month > 12U || day < 1U) return 0U;
    if (month < 3U) y--;

    return (uint8_t)((y + y / 4U - y / 100U + y / 400U + offset[month - 1U] + day) % 7U);
}

/*
 * Convert UTC GPS time from the AGRIC sentence to local time (UTC+8) and store it
 * in chassis->date_to_usb. Skips stale data (age > GPS_ONLINE_TIMEOUT_MS) and
 * validates all time fields before writing.
 */
static void chassis_store_gps_time(chassis_move_t *chassis, PT_AGRIC agric, uint32_t now)
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;

    if (chassis == NULL || agric == NULL) return;
    if (agric->last_update_tick == 0U || agric->Month == 0U || agric->Day == 0U) return;
    if ((now - agric->last_update_tick) > GPS_ONLINE_TIMEOUT_MS) return;
    if (agric->Hour > 23U || agric->Minute > 59U || agric->Second > 59U) return;

    year = (agric->Year < 100U) ? (uint16_t)(2000U + agric->Year) : (uint16_t)agric->Year;
    month = agric->Month;
    day = agric->Day;
    hour = agric->Hour;

    if (month < 1U || month > 12U || day > chassis_days_in_month(year, month)) return;

    hour = (uint8_t)(hour + 8U);
    if (hour >= 24U)
    {
        hour = (uint8_t)(hour - 24U);
        day++;
        if (day > chassis_days_in_month(year, month))
        {
            day = 1U;
            month++;
            if (month > 12U)
            {
                month = 1U;
                year++;
            }
        }
    }

    chassis->date_to_usb.current_year = year;
    chassis->date_to_usb.current_month = month;
    chassis->date_to_usb.current_day = day;
    chassis->date_to_usb.current_week = chassis_weekday(year, month, day);
    chassis->date_to_usb.current_hour = hour;
    chassis->date_to_usb.current_minute = agric->Minute;
    chassis->date_to_usb.current_second = agric->Second;
    chassis->date_to_usb.time_update_tick = now;
}

static uint8_t chassis_is_gps_online(chassis_move_t *chassis)
{
    uint32_t tick;

    if (chassis == NULL) return 0U;

    tick = chassis->date_to_usb.last_update_tick;
    return (tick != 0U && (HAL_GetTick() - tick) <= GPS_ONLINE_TIMEOUT_MS) ? 1U : 0U;
}

static uint8_t chassis_is_mag_online(chassis_move_t *chassis)
{
    uint32_t tick;

    if (chassis == NULL) return 0U;

    tick = chassis->imu.mag_last_update_tick;
    return (tick != 0U && (HAL_GetTick() - tick) <= 1000U) ? 1U : 0U;
}

static uint8_t chassis_is_bt_online(void)
{
    return (BT_IsOnline() || BT_IsActive()) ? 1U : 0U;
}

/*
 * Check whether a given operating mode is available based on current sensor and
 * communication link status. Returns 1 if available, 0 otherwise.
 * Used by mode arbitration to prevent switching into a mode whose prerequisites
 * are not met (e.g., GPS mode without GPS fix).
 */
static uint8_t chassis_mode_available(chassis_move_t *chassis, CarMode_t mode)
{
    uint8_t jetson_online;
    uint8_t jetson_mode;

    if (chassis == NULL) return 0U;

    jetson_online = chassis_is_jetson_online(chassis);
    jetson_mode = chassis->cmd_vel.mode;

    switch (mode)
    {
        case CAR_MODE_IDLE:
            return 1U;

        case CAR_MODE_GPS:
            /* Pure-GPS mode is also the explicit magnetometer-calibration
             * entry point.  Allow it before a GPS fix exists; navigation
             * itself still refuses to move until GPS and magnetometer data
             * are valid. */
            (void)jetson_online;
            (void)jetson_mode;
            return 1U;

        case CAR_MODE_GPS_ROS:
            return (chassis_is_gps_online(chassis) &&
                    jetson_online &&
                    jetson_mode == JETSON_MODE_GPS_ROS) ? 1U : 0U;

        case CAR_MODE_REMOTE:
            return (jetson_online && jetson_mode == JETSON_MODE_REMOTE) ? 1U : 0U;

        case CAR_MODE_ROS_INDOOR:
            return (jetson_online && jetson_mode == JETSON_MODE_INDOOR) ? 1U : 0U;

        case CAR_MODE_INDOOR:
            return chassis_is_bt_online();

        case CAR_MODE_VOICE:
            return App_Voice_IsActive();

        default:
            return 0U;
    }
}



/* ============================================================
 *  Public: Switch vehicle operating mode.
 *  Retains navigation state only when switching between nav modes; resets otherwise.
 * ============================================================ */
static void Chassis_SetMode(chassis_move_t *chassis, CarMode_t mode)
{
    uint8_t was_nav_mode;
    uint8_t is_nav_mode;

    if (chassis == NULL) return;

    was_nav_mode = (chassis->mode == CAR_MODE_GPS || chassis->mode == CAR_MODE_GPS_ROS) ? 1U : 0U;
    is_nav_mode = (mode == CAR_MODE_GPS || mode == CAR_MODE_GPS_ROS) ? 1U : 0U;

    /* Only pure GPS entry starts the one-time rotating calibration.  GPS+ROS
       keeps the power-up stream and does not rotate the vehicle implicitly. */
    if (mode == CAR_MODE_GPS)
    {
        chassis_init_magnetometer_once(chassis);
    }

    if (!was_nav_mode || !is_nav_mode)
    {
        Navigation_Stop(chassis);
    }
    chassis_stop(chassis);
    chassis->mode = mode;

    if (mode != CAR_MODE_VOICE)
    {
        App_Voice_Clear();
    }

    if (!was_nav_mode && is_nav_mode)
    {
        chassis_start_gps_navigation(chassis);
    }
}

/* ============================================================
 *  Sensor data refresh: Magnetometer + JY901S IMU + Encoders + USB + GPS
 *  Called each cycle before the control loop; records per-sensor update timestamps.
 * ============================================================ */
void chassis_feedback_update(chassis_move_t *chassis)
{
    uint32_t now;

    if (chassis == NULL) return;

    now = HAL_GetTick();

    /* Magnetometer: read every cycle after hardware init at power-up.
       Default calibration (offset=0, scale=1) until first GPS entry. */
    if (chassis_mag_hw_ready)
    {
        QMC5883_GetAngles(&chassis->imu.mag);
        chassis->imu.mag_last_update_tick = chassis->imu.mag.last_update_tick;
    }

    {
        JY901S_Data_t jy901s_data;
        (void)JY901S_GetData(&jy901s_data);
        chassis->imu.jy901s = jy901s_data;
        chassis_update_jy901s_debug_snapshot(&jy901s_data);
        if (jy901s_data.online)
        {
            /* Only fresh frames update the INS attitude used by the display. */
            chassis->imu.ins.euler.roll = jy901s_data.angle[0];
            chassis->imu.ins.euler.pitch = jy901s_data.angle[1];
            chassis->imu.ins.euler.yaw = jy901s_data.angle[2];
            chassis->imu.ins_last_update_tick = jy901s_data.last_update_tick;
        }
    }

    /* Encoders -> chassis->motor[i].speed */
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis->motor[i].speed = Encoder_Rpm_Get(i);
        chassis->motor[i].last_update_tick = now;
    }

    /* Drain the interrupt-side USB CDC ring before parsing the byte stream. */
    {
        uint8_t usb_rx_data[64U];
        uint16_t usb_rx_size;

        do
        {
            usb_rx_size = CDC_ReadRxData(usb_rx_data, (uint16_t)sizeof(usb_rx_data));
            if (usb_rx_size > 0U)
            {
                USB_ProcessRxData(usb_rx_data, usb_rx_size);
            }
        } while (usb_rx_size > 0U);
    }
    chassis->cmd_vel = USB_GetCmdVel();
    {
        scene_cmd_t scene = USB_GetSceneCmd();

        if (scene.update_sequence != chassis_scene_sequence)
        {
            chassis_scene_sequence = scene.update_sequence;
            /*
             * 微信/Jetson indoor、outdoor 对路面显示的手动覆盖暂时停用。
             * LVGL 现在直接读取 BSP_RoadClassification_GetResult() 的模型结果。
             * 保留 scene_cmd 的接收和序号消费，避免改变现有串口协议。
             *
            if (scene.scene == JETSON_SCENE_INDOOR)
            {
                BT_SetRoadDisplay(BT_ROAD_DISPLAY_MARBLE);
            }
            else if (scene.scene == JETSON_SCENE_OUTDOOR)
            {
                BT_SetRoadDisplay(BT_ROAD_DISPLAY_ASPHALT);
            }
             */
        }
    }
    chassis_process_usb_gps_route(chassis);
    if (chassis->cmd_vel.last_update_tick != 0U)
    {
        chassis->jetson_last_tick = chassis->cmd_vel.last_update_tick;
    }

    /* GPS position -> chassis->date_to_usb (NMEA to decimal degrees) */
    {
        PT_GNGGA pGGA = GetGNGGA();
        if (pGGA->qf >= 1 && pGGA->lat >= 1.0f)
        {
            double lat_deg = floor(pGGA->lat / 100.0)
                           + (pGGA->lat - floor(pGGA->lat / 100.0) * 100.0) / 60.0;
            double lon_deg = floor(pGGA->lon / 100.0)
                           + (pGGA->lon - floor(pGGA->lon / 100.0) * 100.0) / 60.0;
            chassis->date_to_usb.current_lat = (float)lat_deg;
            chassis->date_to_usb.current_lon = (float)lon_deg;
            chassis->date_to_usb.current_sats = pGGA->sats;
            chassis->date_to_usb.last_update_tick = now;
        }
    }

    /* Target heading relative to vehicle heading [0, 360) */
    {
        PT_AGRIC pAGRIC = GetAGRIC();
        chassis_store_gps_time(chassis, pAGRIC, now);
    }

    {
        float tgt = chassis->nav.target_bearing;
        float yaw = chassis->imu.mag.yaw;
        float diff = tgt - yaw;
        while (diff < 0.0f)    diff += 360.0f;
        while (diff >= 360.0f) diff -= 360.0f;
        chassis->date_to_usb.heading_to_target_deg = diff;
    }
}

/* ============================================================
 *  Init: Encoders / Motors / Bluetooth / UART / IMU / Remote params / Gain / PID
 *  Called once from the FreeRTOS task.
 * ============================================================ */
static void chassis_init(chassis_move_t *chassis)
{
    double speed_pid_param[3];

	BT_ServicePendingAck();
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96\xE7\xBC\x96\xE7\xA0\x81\xE5\x99\xA8");
    Encoder_Init();
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96\xE7\x94\xB5\xE6\x9C\xBA");
    Motor_Init();
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96\xE8\x93\x9D\xE7\x89\x99");
	BT_ServicePendingAck();
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96USB");
    USB_Init();
	BT_ServicePendingAck();
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96UART2");
    uart_init(&huart2, UART_DMA_ToIdle_RX);
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96JY901S");
    JY901S_Init();
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96QMC5883");
    QMC5883_InitHW();
    chassis_mag_hw_ready = 1U;

    /*
     * 路面识别只维护自己的 NanoEdge 缓冲和结果快照。即使模型初始化失败，
     * 底盘初始化与后续控制仍按原有路径继续，绝不因此改变 PID 或电机输出。
     */
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96\xE8\xB7\xAF\xE9\x9D\xA2\xE8\xAF\x86\xE5\x88\xAB");
    (void)BSP_RoadClassification_Init();
	BT_ServicePendingAck();

    /* Remote control parameter defaults */
    chassis->remote.bt_speed         = BT_REMOTE_SPEED;
    chassis->remote.bt_wz            = BT_REMOTE_WZ;
    chassis->remote.wechat_vx_scale  = WECHAT_VX_SCALE;
    chassis->remote.wechat_vz_scale  = WECHAT_VZ_SCALE;
    chassis->remote.wechat_speed_gain = WECHAT_SPEED_GAIN;
    chassis->remote.wechat_max_speed = WECHAT_MAX_SPEED;
    chassis->remote.wechat_max_wz    = WECHAT_MAX_WZ;
    chassis->remote.ros_vx_scale     = ROS_LINE_VX_SCALE;
    chassis->remote.ros_vz_scale     = ROS_LINE_VZ_SCALE;
    chassis->remote.ros_max_speed    = ROS_LINE_MAX_SPEED;
    chassis->remote.ros_max_wz       = ROS_LINE_MAX_WZ;

    /* Per-mode control gain defaults */
    chassis->gain.gps    = CHASSIS_GAIN_GPS;
    chassis->gain.indoor = CHASSIS_GAIN_INDOOR;
    chassis->gain.remote = CHASSIS_GAIN_REMOTE;
    chassis->gain.line   = CHASSIS_GAIN_LINE;
    chassis->gain.voice  = CHASSIS_GAIN_VOICE;

    chassis->pid_param.kp = MOTOR_SPEED_PID_KP;
    chassis->pid_param.ki = MOTOR_SPEED_PID_KI;
    chassis->pid_param.kd = MOTOR_SPEED_PID_KD;
    chassis->pid_param.max_out = MOTOR_SPEED_PID_MAX_OUT;
    chassis->pid_param.max_iout = MOTOR_SPEED_PID_MAX_IOUT;
    speed_pid_param[0] = chassis->pid_param.kp;
    speed_pid_param[1] = chassis->pid_param.ki;
    speed_pid_param[2] = chassis->pid_param.kd;

    /* WonderEcho needs brief stabilization after power-on, then init PB0/PB1 software I2C.
       Init remains in the chassis task; no new FreeRTOS task, avoiding concurrent chassis speed writes. */
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96\xE8\xAF\xAD\xE9\x9F\xB3");
    osDelay(200);
	BT_ServicePendingAck();
    WonderEcho_I2C_Init();

    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96PID");
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_init(&chassis->motor[i].speed_pid, PID_POSITION,
                 speed_pid_param,
                 chassis->pid_param.max_out,
                 chassis->pid_param.max_iout);
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis->motor[i].speed_set = 0;
    }
    chassis_set_init_status("\xE5\xBA\x95\xE7\x9B\x98\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96\xE5\xAE\x8C\xE6\x88\x90");
}

/* ============================================================
 *  Step 4: Chassis core control loop.
 *    Vx/Vy/Wz (written by step 3) -> kinematics decomposition -> PID calculation.
 *    PID output is sent to motors uniformly by step 5 chassis_send_cmd.
 * ============================================================ */
void chassis_control_loop(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

    /* 1. Omnidirectional kinematics: Vx/Vy/Wz -> 4 motor target speeds */
    chassis->motor[0].speed_set =  chassis->Vx_set + chassis->Vy_set + chassis->Wz_set;
    chassis->motor[1].speed_set =  chassis->Vx_set - chassis->Vy_set - chassis->Wz_set;
    chassis->motor[2].speed_set =  chassis->Vx_set - chassis->Vy_set + chassis->Wz_set;
    chassis->motor[3].speed_set =  chassis->Vx_set + chassis->Vy_set - chassis->Wz_set;

    /* 2. PID speed closed loop (result stored in motor[i].speed_pid.Out) */
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Calculate(&chassis->motor[i].speed_pid,
                      chassis->motor[i].speed,
                      chassis->motor[i].speed_set);
    }
}

/* ============================================================
 *  On-demand QMC5883 calibration motion step.
 *  Keeps the original magnetometer sampling/calculation unchanged while
 *  using the existing encoder feedback, kinematics and motor speed PID.
 * ============================================================ */
static void chassis_mag_calibration_step(uint32_t elapsed_ms)
{
    uint8_t i;
    uint32_t now;

    now = HAL_GetTick();

    if (BT_IsPowerless())
    {
        g_chassis_powerless_debug = 1U;
        chassis_force_powerless_output(&chassis_move);
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        chassis_move.motor[i].speed = Encoder_Rpm_Get(i);
        chassis_move.motor[i].last_update_tick = now;
    }

    chassis_move.Vx_set = 0.0f; 
    chassis_move.Vy_set = 0.0f;
    chassis_move.Wz_set = 0.0f;

    if (elapsed_ms >= MAG_CALIB_STILL_END_MS && elapsed_ms < MAG_CALIB_CW_END_MS)
    {
        chassis_move.Wz_set = -MAG_CALIB_WZ;
    }
    else if (elapsed_ms >= MAG_CALIB_PAUSE_END_MS && elapsed_ms < MAG_CALIB_CCW_END_MS)
    {
        chassis_move.Wz_set = MAG_CALIB_WZ;
    }

    chassis_apply_control_smoothing(&chassis_move);
    chassis_control_loop(&chassis_move);

    for (i = 0U; i < 4U; i++)
    {
        Motor_SetPWM((int16_t)chassis_move.motor[i].speed_pid.Out, i);
    }
}

/* ============================================================
 *  Step 1: Chassis control mode arbitration.
 *  Priority: Bluetooth powerless latch > GUI request > Bluetooth > Manual hold
 *            > Voice > Jetson -> auto fallback to IDLE.
 * ============================================================ */
void chassis_mode_change(chassis_move_t *chassis)
{
    BT_ModeReq_t req;
    BT_RemotePoint_t remote_point;
    CarMode_t requested_mode;
    uint8_t jetson_online;
    uint8_t jetson_mode;

    if (chassis == NULL) return;

    g_chassis_powerless_debug = BT_IsPowerless();
    if (g_chassis_powerless_debug)
    {
        gui_req_mode = -1;
        chassis_manual_indoor_mode = 0U;
        if (chassis->mode != CAR_MODE_IDLE) Chassis_SetMode(chassis, CAR_MODE_IDLE);
        chassis_force_powerless_output(chassis);
        return;
    }

    if (BT_GetAndClearRemotePoint(&remote_point))
    {
        BT_ReportRemotePointResult(chassis_add_remote_gps_point(chassis, &remote_point));
    }

    if (gui_req_mode >= 0 && gui_req_mode <= 6) {
        requested_mode = (CarMode_t)gui_req_mode;
        if (chassis_mode_available(chassis, requested_mode))
        {
            chassis_manual_indoor_mode = (requested_mode == CAR_MODE_INDOOR) ? 1U : 0U;
            Chassis_SetMode(chassis, requested_mode);
        }
        else if (!chassis_mode_available(chassis, chassis->mode))
        {
            chassis_manual_indoor_mode = 0U;
            Chassis_SetMode(chassis, CAR_MODE_IDLE);
        }
        gui_req_mode = -1;
        return;
    }

    req = BT_GetAndClearModeReq();
    if (req != BT_MODE_REQ_NONE)
    {
    }

    switch (req)
    {
        case BT_MODE_REQ_GPS:
            chassis_manual_indoor_mode = 0U;
            if (chassis_mode_available(chassis, CAR_MODE_GPS))
            {
                Chassis_SetMode(chassis, CAR_MODE_GPS);
            }
            else if (!chassis_mode_available(chassis, chassis->mode))
            {
                Chassis_SetMode(chassis, CAR_MODE_IDLE);
            }
            return;

        case BT_MODE_REQ_INDOOR:
            if (chassis_mode_available(chassis, CAR_MODE_INDOOR))
            {
                chassis_manual_indoor_mode = 1U;
                Chassis_SetMode(chassis, CAR_MODE_INDOOR);
            }
            else if (!chassis_mode_available(chassis, chassis->mode))
            {
                chassis_manual_indoor_mode = 0U;
                Chassis_SetMode(chassis, CAR_MODE_IDLE);
            }
            return;

        case BT_MODE_REQ_GPS_ADD_POINT:
            (void)chassis_add_current_gps_point(chassis);
            return;

        case BT_MODE_REQ_GPS_CLEAR_POINTS:
            chassis_clear_gps_route(chassis);
            return;

        default:
            break;
    }

    if (!chassis_mode_available(chassis, chassis->mode))
    {
        /* GPS_ROS and GPS_ONLY are independent modes. Never fall back between them. */
        if (chassis->mode != CAR_MODE_IDLE)
        {
            Chassis_SetMode(chassis, CAR_MODE_IDLE);
        }
        return;
    }

    if (BT_IsActive() && chassis_mode_available(chassis, CAR_MODE_INDOOR))
    {
        chassis_manual_indoor_mode = 1U;
        if (chassis->mode != CAR_MODE_INDOOR)
        {
            Chassis_SetMode(chassis, CAR_MODE_INDOOR);
        }
        return;
    }

    if (chassis_manual_indoor_mode)
    {
        if (chassis_mode_available(chassis, CAR_MODE_INDOOR))
        {
            if (chassis->mode != CAR_MODE_INDOOR)
            {
                Chassis_SetMode(chassis, CAR_MODE_INDOOR);
            }
        }
        else
        {
            chassis_manual_indoor_mode = 0U;
            if (chassis->mode != CAR_MODE_IDLE)
            {
                Chassis_SetMode(chassis, CAR_MODE_IDLE);
            }
        }
        return;
    }

    if (App_Voice_IsActive() && chassis_mode_available(chassis, CAR_MODE_VOICE))
    {
        if (chassis->mode != CAR_MODE_VOICE)
        {
            Chassis_SetMode(chassis, CAR_MODE_VOICE);
        }
        return;
    }

    jetson_online = chassis_is_jetson_online(chassis);
    jetson_mode = chassis->cmd_vel.mode;

    if (jetson_online)
    {
        if (jetson_mode == JETSON_MODE_REMOTE)
        {
            if (chassis_mode_available(chassis, CAR_MODE_REMOTE) && chassis->mode != CAR_MODE_REMOTE)
            {
                Chassis_SetMode(chassis, CAR_MODE_REMOTE);
            }
            return;
        }

        if (jetson_mode == JETSON_MODE_INDOOR)
        {
            if (chassis_mode_available(chassis, CAR_MODE_ROS_INDOOR) && chassis->mode != CAR_MODE_ROS_INDOOR)
            {
                Chassis_SetMode(chassis, CAR_MODE_ROS_INDOOR);
            }
            return;
        }

        if (jetson_mode == JETSON_MODE_GPS_ROS)
        {
            if (chassis_mode_available(chassis, CAR_MODE_GPS_ROS) && chassis->mode != CAR_MODE_GPS_ROS)
            {
                Chassis_SetMode(chassis, CAR_MODE_GPS_ROS);
            }
            return;
        }

        if (jetson_mode == JETSON_MODE_GPS_ONLY)
        {
            if (chassis_mode_available(chassis, CAR_MODE_GPS) && chassis->mode != CAR_MODE_GPS)
            {
                Chassis_SetMode(chassis, CAR_MODE_GPS);
            }
            return;
        }
    }

    if (chassis->mode == CAR_MODE_REMOTE || chassis->mode == CAR_MODE_ROS_INDOOR)
    {
        Chassis_SetMode(chassis, CAR_MODE_IDLE);
    }
}

/* ============================================================
 *  Internal: Apply per-mode control gain.
 *  Called at the end of chassis_set_control; scales Vx/Vy/Wz uniformly.
 * ============================================================ */
static void chassis_apply_gain(chassis_move_t *chassis)
{
    float gain = 1.0f;

    if (chassis == NULL) return;

    switch (chassis->mode)
    {
        case CAR_MODE_IDLE:
            gain = 1.0f;
            break;

        case CAR_MODE_GPS:    gain = chassis->gain.gps;    break;
        /* GPS+ROS uses the same ROS command gain as the proven indoor Nav2 path. */
        case CAR_MODE_GPS_ROS: gain = chassis->gain.line;  break;
        case CAR_MODE_INDOOR: gain = chassis->gain.indoor; break;
        case CAR_MODE_REMOTE: gain = chassis->gain.remote; break;
        case CAR_MODE_ROS_INDOOR: gain = chassis->gain.line; break;
        case CAR_MODE_VOICE:  gain = chassis->gain.voice;  break;
        default: break;
    }

    chassis->Vx_set *= gain;
    chassis->Vy_set *= gain;
    chassis->Wz_set *= gain;
}

/* ============================================================
 *  Step 3: Set chassis control targets.
 *  Writes Vx/Vy/Wz according to current mode; applies mode gain at the end.
 * ============================================================ */
void chassis_set_control(chassis_move_t *chassis)
{
    uint8_t jetson_online;
    uint8_t jetson_mode;

    if (chassis == NULL) return;

    if (BT_IsPowerless())
    {
        g_chassis_powerless_debug = 1U;
        chassis_force_powerless_output(chassis);
        return;
    }

    jetson_online = chassis_is_jetson_online(chassis);
    jetson_mode = chassis->cmd_vel.mode;

    switch (chassis->mode)
    {
        case CAR_MODE_IDLE:
            chassis_stop(chassis);
            break;

        case CAR_MODE_INDOOR:
            if (BT_IsActive())
            {
                Remote_Control_Update(chassis);
            }
            else
            {
                chassis_stop(chassis);
            }
            break;

        case CAR_MODE_REMOTE:
            if (jetson_online && jetson_mode == JETSON_MODE_REMOTE)
            {
                Remote_WeChat_Update(chassis);
            }
            else
            {
                chassis_stop(chassis);
            }
            break;

        case CAR_MODE_ROS_INDOOR:
            if (jetson_online && jetson_mode == JETSON_MODE_INDOOR)
            {
                Remote_ROS_Update(chassis);
            }
            else
            {
                chassis_stop(chassis);
            }
            break;

        case CAR_MODE_GPS:
            if (chassis_mode_available(chassis, CAR_MODE_GPS))
            {
                Navigation_Update_Loop(chassis);
            }
            else
            {
                chassis_stop(chassis);
            }
            break;

        case CAR_MODE_GPS_ROS:
            if (jetson_online && jetson_mode == JETSON_MODE_GPS_ROS && chassis_mode_available(chassis, CAR_MODE_GPS_ROS))
            {
                Remote_ROS_Update(chassis);
            }
            else
            {
                chassis_stop(chassis);
            }
            break;

        case CAR_MODE_VOICE:
            if (App_Voice_IsActive())
            {
                App_Voice_ApplyControl(chassis);
            }
            else
            {
                chassis_stop(chassis);
            }
            break;

        default:
            chassis_stop(chassis);
            break;
    }

    chassis_apply_gain(chassis);
    chassis_apply_control_smoothing(chassis);
}

static uint8_t chassis_telemetry_mode(const chassis_move_t *chassis)
{
    switch (chassis->mode)
    {
        case CAR_MODE_GPS_ROS:    return JETSON_MODE_GPS_ROS;
        case CAR_MODE_REMOTE:     return JETSON_MODE_REMOTE;
        case CAR_MODE_ROS_INDOOR: return JETSON_MODE_INDOOR;
        case CAR_MODE_GPS:        return JETSON_MODE_GPS_ONLY;
        default:                  return 0U;
    }
}

static void chassis_send_sensor_telemetry(chassis_move_t *chassis)
{
    static uint32_t last_send_tick = 0U;
    static uint16_t sequence = 0U;
    static uint8_t route_slot = 0U;
    usb_sensor_telemetry_t telemetry = {0};
    PT_GNGGA gga;
    PT_AGRIC agric;
    uint32_t now = HAL_GetTick();
    uint8_t agric_fresh;
    uint8_t gga_fresh;

    if ((uint32_t)(now - last_send_tick) < 50U)
    {
        return;
    }
    last_send_tick = now;
    gga = GetGNGGA();
    agric = GetAGRIC();
    agric_fresh = (agric != NULL && agric->last_update_tick != 0U &&
                   (uint32_t)(now - agric->last_update_tick) <= 2000U) ? 1U : 0U;
    gga_fresh = (gga != NULL && gga->qf != 0U && gga->last_update_tick != 0U &&
                 (uint32_t)(now - gga->last_update_tick) <= 2500U) ? 1U : 0U;

    telemetry.sequence = ++sequence;
    telemetry.car_mode = chassis_telemetry_mode(chassis);
    telemetry.route_total = chassis->nav.total_waypoints;
    telemetry.navigation_active = chassis->nav.is_navigating;
    telemetry.loop_enable = chassis->nav.loop_enable;

    if (agric_fresh && agric->Postype != 0U)
    {
        telemetry.flags |= USB_SENSOR_FLAG_GPS_VALID;
        telemetry.latitude = agric->lat;
        telemetry.longitude = agric->lon;
        telemetry.altitude = agric->alt;
        telemetry.fix_quality = agric->Postype;
        telemetry.satellites = (uint8_t)(agric->NumGPSSta + agric->NumBDSSta +
                                         agric->NumGLOSta + agric->NumGalSta);
    }
    else if (gga_fresh)
    {
        telemetry.flags |= USB_SENSOR_FLAG_GPS_VALID;
        telemetry.latitude = chassis_nmea_to_degree(gga->lat, gga->lat_dir);
        telemetry.longitude = chassis_nmea_to_degree(gga->lon, gga->lon_dir);
        telemetry.altitude = (double)gga->Alt;
        telemetry.fix_quality = gga->qf;
        telemetry.satellites = gga->sats;
    }

    if (agric_fresh)
    {
        telemetry.gnss_heading = agric->Heading;
        telemetry.gnss_speed = agric->Speed;
        telemetry.velocity_north = agric->VelocityOfNorth;
        telemetry.velocity_east = agric->VelocityOfEast;
        telemetry.heading_status = agric->HeadingStat;
        if (agric->HeadingStat != 0U)
        {
            telemetry.flags |= USB_SENSOR_FLAG_GNSS_HEAD_VALID;
        }
    }
    if (chassis->imu.mag_last_update_tick != 0U &&
        (uint32_t)(now - chassis->imu.mag_last_update_tick) <= 1000U)
    {
        telemetry.flags |= USB_SENSOR_FLAG_MAG_VALID;
        telemetry.mag_yaw = chassis->imu.mag.yaw;
        telemetry.mag_pitch = chassis->imu.mag.pitch;
        telemetry.mag_roll = chassis->imu.mag.roll;
    }

    if (chassis->imu.jy901s.online)
    {
        telemetry.flags |= USB_SENSOR_FLAG_IMU_VALID;
        telemetry.imu_roll = chassis->imu.jy901s.angle[0];
        telemetry.imu_pitch = chassis->imu.jy901s.angle[1];
        telemetry.imu_yaw = chassis->imu.jy901s.angle[2];
        telemetry.gyro_x = chassis->imu.jy901s.gyro[0];
        telemetry.gyro_y = chassis->imu.jy901s.gyro[1];
        telemetry.gyro_z = chassis->imu.jy901s.gyro[2];
        telemetry.acc_x = chassis->imu.jy901s.acc[0];
        telemetry.acc_y = chassis->imu.jy901s.acc[1];
        telemetry.acc_z = chassis->imu.jy901s.acc[2];
    }

    for (uint8_t i = 0U; i < 4U; i++)
    {
        telemetry.motor_speed[i] = (float)chassis->motor[i].speed;
    }

    if (chassis->nav.total_waypoints > 0U)
    {
        telemetry.flags |= USB_SENSOR_FLAG_TARGET_VALID | USB_SENSOR_FLAG_ROUTE_VALID;
        telemetry.target_latitude = chassis->nav.target_pos.lat;
        telemetry.target_longitude = chassis->nav.target_pos.lon;
        if (route_slot >= chassis->nav.total_waypoints)
        {
            route_slot = 0U;
        }
        telemetry.route_slot = route_slot;
        telemetry.route_latitude = chassis->nav.route[route_slot].lat;
        telemetry.route_longitude = chassis->nav.route[route_slot].lon;
        route_slot++;
    }
    else
    {
        route_slot = 0U;
    }

    USB_SendSensorTelemetry(&telemetry);
}

/* ============================================================
 *  Step 5: Send chassis control commands.
 *    Motor output remains at 100 Hz; sensor telemetry is limited to 20 Hz.
 * ============================================================ */
void chassis_send_cmd(chassis_move_t *chassis)
{
    if (chassis == NULL) return;

	BT_ServicePendingAck();

    if (BT_IsPowerless())
    {
        g_chassis_powerless_debug = 1U;
        chassis_force_powerless_output(chassis);
        chassis_send_sensor_telemetry(chassis);
        return;
    }

    if (chassis->mode == CAR_MODE_IDLE)
    {
        chassis_reset_control_smoothing(chassis);
        chassis_clear_motor_output(chassis);
        for (uint8_t i = 0; i < 4; i++)
        {
            Motor_SetPWM(0, i);
        }
        chassis_send_sensor_telemetry(chassis);
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        Motor_SetPWM((int16_t)chassis->motor[i].speed_pid.Out, i);
    }

    chassis_send_sensor_telemetry(chassis);
}

/* ============================================================
 *  FreeRTOS task entry
 * ============================================================ */
void chassis_task(void *pvParameters)
{
    uint32_t voice_tick = 0U;

    /* -- One-time initialization -- */ 
    chassis_init(&chassis_move);
    chassis_load_gps_route_from_flash();
    chassis_set_init_status("QMC5883\xE7\xAD\x89\xE5\xBE\x85\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96");
    Motor_SetAllPWM(0, 0, 0, 0);
    chassis_stop(&chassis_move);
    chassis_clear_motor_output(&chassis_move);
    chassis_reset_control_smoothing(&chassis_move);
    chassis_set_init_status("\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96GPS");
    GPS_Init();
    chassis_set_init_status("\xE6\xA8\xA1\xE5\x9D\x97\xE5\xB7\xB2\xE5\xB0\xB1\xE7\xBB\xAA");
    chassis_init_done = 1U;

//		/* 路面识别直行测试：仅本次测试使用 */
//chassis_move.mode = CAR_MODE_INDOOR;  /* 只用于避开 IDLE 的强制停车；不执行蓝牙控制 */
//chassis_move.Vx_set = 100.0f;          /* 先从 30 开始测试 */
//chassis_move.Vy_set = 0.0f;           /* 不横移 */
//chassis_move.Wz_set = 0.0f;           /* 不旋转 */
    /* -- Default mode; chassis_mode_change auto-arbitrates based on availability -- */
    /* -- Main loop (100Hz) -- */
    while (1)
    {
        chassis_feedback_update(&chassis_move);   /* Sensors + USB data refresh */
        chassis_service_gps_route_sd(&chassis_move); /* SD primary route store; Flash fallback */

        /*
         * 复用现有 10 ms 底盘任务，不创建新任务。BSP 只在 ACC、GYRO 均更新后
         * 才接收一条六轴样本，收满 32 条才推理；它只写自己的结果快照，绝不写入
         * chassis_move 的速度、模式、PID 或电机控制字段。
         */
        BSP_RoadClassification_Process(
            &chassis_move.imu.jy901s,
            chassis_is_moving_for_road_classification(&chassis_move));

        if (HAL_GetTick() - voice_tick >= 50U)
        {
            voice_tick = HAL_GetTick();
            App_Voice_Recognition_Update(&chassis_move);
        }
        
        chassis_mode_change(&chassis_move);       /* Mode switch */
		    chassis_set_control(&chassis_move);       /* Control targets (priority-adjusted) */
        chassis_control_loop(&chassis_move);      /* Kinematics + PID */
        chassis_send_cmd(&chassis_move);          /* Motor output + USB telemetry */
        osDelay(10);
    }
}

/* GPS_Init() is synchronous; service Bluetooth ACKs during its existing 500 ms waits. */
void GPS_InitBackgroundHook(void)
{
	BT_ServicePendingAck();
}
