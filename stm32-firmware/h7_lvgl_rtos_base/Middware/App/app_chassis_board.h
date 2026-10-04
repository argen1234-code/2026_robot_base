#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "main.h"
#include "pid.h"
#include "bsp_QMC5883.h"
#include "bsp_usb.h"
#include "app_Navigation.h"
#include "bsp_JY901S.h"
#include "app_remote_control.h"

#define chassis_board_task 1

/* ---- Per-mode control gain (runtime adjustable via GUI / debugger) ---- */
#define CHASSIS_GAIN_GPS      1.0f
#define CHASSIS_GAIN_INDOOR   1.0f
#define CHASSIS_GAIN_REMOTE   1.0f
#define CHASSIS_GAIN_LINE     1.0f
#define CHASSIS_GAIN_VOICE    1.0f

/* ---- Chassis motor speed PID parameters ---- */
#define MOTOR_SPEED_PID_KP       1.05f
#define MOTOR_SPEED_PID_KI       0.1f
#define MOTOR_SPEED_PID_KD       0.1f
//#define MOTOR_SPEED_PID_KP       0.0f
//#define MOTOR_SPEED_PID_KI       0.0f
//#define MOTOR_SPEED_PID_KD       0.0f



//#define MOTOR_SPEED_PID_KP       1.60f
//#define MOTOR_SPEED_PID_KI       0.15f
//#define MOTOR_SPEED_PID_KD       0.10f
//#define MOTOR_SPEED_PID_KP       1.22f
//#define MOTOR_SPEED_PID_KI       0.09f
//#define MOTOR_SPEED_PID_KD       0.08f
#define MOTOR_SPEED_PID_MAX_OUT  150.0f
#define MOTOR_SPEED_PID_MAX_IOUT 40.0f

/* ============================================================
 *  IMU attitude data structures
 * ============================================================ */

/* Quaternion (raw IMU output) */
typedef struct {
    float q0, q1, q2, q3;
} Quaternion_t;

/* INS-computed Euler angles */
typedef struct {
    float yaw;      /* Yaw   0~360 deg */
    float pitch;    /* Pitch */
    float roll;     /* Roll */
} INS_Euler_t;

/* INS full data: raw quaternion + computed Euler angles */
typedef struct {
    Quaternion_t quaternion;   /* Raw quaternion */
    INS_Euler_t  euler;        /* Computed Euler angles */
} INS_Data_t;

/* IMU subsystem: magnetometer + INS attitude */
typedef struct {
    EulerAngles mag;                /* QMC5883 magnetometer Euler angles */
    uint32_t mag_last_update_tick;   /* Magnetometer last update tick */
    INS_Data_t  ins;                /* IMU/INS attitude data */
    uint32_t ins_last_update_tick;   /* IMU last update tick */
    JY901S_Data_t jy901s;           /* JY901S 9-axis sensor data */
} IMU_Data_t;

/*
 * Keil 调试专用的 JY901S 全量快照。
 *
 * chassis_move 是 app_chassis_board.c 内部的 static 私有对象，Keil 在非该文件的
 * 断点处无法稳定解析 chassis_move.imu.jy901s。故额外提供这个具名、全局、volatile
 * 结构体，便于在任意断点的 Watch 中直接添加 g_chassis_jy901s_debug 并展开查看。
 *
 * 它仅在收到一帧新的 JY901S 数据后复制该帧内容；不作为控制输入，也绝不写回
 * chassis_move、PID、底盘模式或电机控制量。
 */
typedef struct ChassisJY901SDebug_s
{
    uint32_t update_sequence; /* 偶数：完整快照；奇数：调试器刚好停在复制过程中。 */
    float acc[3];             /* X/Y/Z 加速度，单位 g。 */
    float gyro[3];            /* X/Y/Z 角速度，单位 deg/s。 */
    float angle[3];           /* roll/pitch/yaw，单位 deg。 */
    int16_t mag[3];           /* X/Y/Z 原始磁场数据。 */
    float temperature;        /* 温度，单位摄氏度。 */
    uint32_t update_flag;     /* 本帧有效数据位：ACC/GYRO/ANGLE/MAG/TEMP。 */
    uint32_t last_update_tick;/* JY901S 收到该帧时的 HAL tick，单位 ms。 */
    uint8_t online;           /* 1：JY901S 在线；0：离线或尚未收到有效数据。 */
} ChassisJY901SDebug_t;

/* 可直接加入 Keil Watch 的 JY901S 调试变量。业务代码仅允许读取，禁止写入。 */
extern volatile ChassisJY901SDebug_t g_chassis_jy901s_debug;

/* ============================================================
 *  Chassis motor + speed PID
 * ============================================================ */

typedef struct {
    double speed;              /* Current speed (rpm, encoder feedback) */
    double speed_set;          /* Target speed (rpm) */
    double angle;              /* Current angle */
    double angle_set;          /* Target angle */
    uint32_t last_update_tick;  /* Encoder last update tick */
    PID_t  speed_pid;          /* Speed loop PID controller */
} chassis_motor_t;

/* ============================================================
 *  Vehicle operating modes
 * ============================================================ */

typedef enum {
    CAR_MODE_IDLE = 0,          /* Idle (all control sources offline) */
    CAR_MODE_GPS,               /* GPS-only navigation (no ROS fusion) */
    CAR_MODE_GPS_ROS,           /* Jetson-controlled GPS + ROS fusion navigation */
    CAR_MODE_REMOTE,            /* WeChat mini-program remote (via Jetson) */
    CAR_MODE_ROS_INDOOR,        /* ROS/Nav2 indoor autonomous navigation */
    CAR_MODE_INDOOR,            /* Bluetooth remote control */
    CAR_MODE_VOICE              /* WonderEcho voice recognition control */
} CarMode_t;

/* ============================================================
 *  USB telemetry data (STM32 -> Jetson)
 * ============================================================ */

typedef struct {
    float heading_to_target_deg;   /* Target bearing relative to heading: 0=front, 90=right, 180=rear, 270=left, CW [0,360) */
    float current_lat;             /* Current latitude (decimal) */
    float current_lon;             /* Current longitude (decimal) */
    uint8_t current_sats;          /* Satellites in use */
    uint32_t last_update_tick;     /* GPS last valid fix timestamp */
    uint16_t current_year;         /* Beijing time year */
    uint8_t current_month;         /* Beijing time month */
    uint8_t current_day;           /* Beijing time day */
    uint8_t current_week;          /* Beijing time weekday (0=Sun) */
    uint8_t current_hour;          /* Beijing time hour */
    uint8_t current_minute;        /* Beijing time minute */
    uint8_t current_second;        /* Beijing time second */
    uint32_t time_update_tick;     /* GPS time last update tick */
} date_to_usb_t;

/* ============================================================
 *  Per-mode control gain struct
 * ============================================================ */
typedef struct {
    float gps;      /* GPS navigation gain */
    float indoor;   /* Bluetooth remote gain */
    float remote;   /* WeChat remote gain */
    float line;     /* ROS indoor navigation gain */
    float voice;    /* Voice control gain */
} chassis_gain_t;

/* ============================================================
 *  Speed PID parameter struct (runtime adjustable)
 * ============================================================ */
typedef struct {
    double kp;          /* Proportional gain */
    double ki;          /* Integral gain */
    double kd;          /* Derivative gain */
    double max_out;     /* Output limit */
    double max_iout;    /* Integral output limit */
} chassis_pid_param_t;

/* ============================================================
 *  Chassis omnidirectional motion control struct
 *  (singleton instance in app_chassis_board.c, access via pointer)
 * ============================================================ */

typedef struct chassis_move_s {
    /* ---- IMU sensors ---- */
    IMU_Data_t          imu;       /* Magnetometer + INS attitude */

    /* ---- GPS navigation ---- */
    Navigation_State_t  nav;       /* Navigation controller state */

    /* ---- Operating mode ---- */
    CarMode_t           mode;      /* Current mode */

    /* ---- USB Jetson downstream data ---- */
    cmd_vel_t           cmd_vel;        /* Latest frame parse result (mode + vx + vz) */
    uint32_t            jetson_last_tick; /* Last valid Jetson frame tick */

    /* ---- USB telemetry data ---- */
    date_to_usb_t       date_to_usb; /* STM32 -> Jetson */

    /* ---- Remote control parameters ---- */
    RemoteControl_t     remote;     /* Per-mode remote control params (runtime adjustable) */

    /* ---- Control gains ---- */
    chassis_gain_t      gain;       /* Per-mode Vx/Vy/Wz gain */

    /* ---- PID parameters ---- */
    chassis_pid_param_t pid_param;  /* Runtime-adjustable speed PID params */

    /* ---- Omnidirectional target velocity (before kinematic decomposition) ---- */
    float Vx_set;                  /* X-axis target velocity (longitudinal) */
    float Vy_set;                  /* Y-axis target velocity (lateral) */
    float Wz_set;                  /* Z-axis target angular velocity (rotation) */

    /* ---- 4 motors [FL:front-left, FR:front-right, RL:rear-left, RR:rear-right] ---- */
    chassis_motor_t     motor[4];

} chassis_move_t;

/* ============================================================
 *  Gain parameter selector enums for chassis_set_gain_param()
 * ============================================================ */
typedef enum {
    CHASSIS_GAIN_PARAM_GPS = 0,
    CHASSIS_GAIN_PARAM_INDOOR,
    CHASSIS_GAIN_PARAM_REMOTE,
    CHASSIS_GAIN_PARAM_LINE,
    CHASSIS_GAIN_PARAM_VOICE
} ChassisGainParam_t;

typedef enum {
    CHASSIS_REMOTE_PARAM_BT_SPEED = 0,
    CHASSIS_REMOTE_PARAM_BT_WZ,
    CHASSIS_REMOTE_PARAM_WECHAT_VX_SCALE,
    CHASSIS_REMOTE_PARAM_ROS_MAX_SPEED
} ChassisRemoteParam_t;

typedef enum {
    CHASSIS_PID_PARAM_KP = 0,
    CHASSIS_PID_PARAM_KI,
    CHASSIS_PID_PARAM_KD,
    CHASSIS_PID_PARAM_MAX_OUT
} ChassisPidParam_t;

/* ============================================================
 *  Telemetry snapshot (read-only copy for LVGL display)
 * ============================================================ */
typedef struct {
    CarMode_t mode;
    float gps_lat;
    float gps_lon;
    uint8_t gps_sats;
    uint32_t gps_last_update_tick;
    uint16_t gps_year;
    uint8_t gps_month;
    uint8_t gps_day;
    uint8_t gps_week;
    uint8_t gps_hour;
    uint8_t gps_minute;
    uint8_t gps_second;
    uint32_t gps_time_update_tick;
    float ins_roll;
    float ins_pitch;
    float ins_yaw;
    uint32_t ins_last_update_tick;
    float jy901s_acc[3];
    float jy901s_gyro[3];
    uint32_t jy901s_last_update_tick;
    uint8_t jy901s_online;
    float mag_yaw;
    float mag_pitch;
    float mag_roll;
    uint32_t mag_last_update_tick;
    float vx_set;
    float vy_set;
    float wz_set;
    double motor_speed[4];
    double motor_speed_set[4];
    uint32_t motor_last_update_tick[4];
    uint8_t qmc_calibrating;
    uint16_t qmc_calibration_remaining_s;
    GPS_Point_t gps_route[MAX_WAYPOINTS];
    uint8_t gps_route_count;
    uint8_t gps_current_wp_index;
    uint8_t gps_is_navigating;
    uint8_t gps_loop_enable;
    Navigation_Phase_t gps_nav_phase;
    float gps_distance_error;
    float gps_heading_error;
} ChassisTelemetry_t;

/* ============================================================
 *  Settings snapshot (read-only copy of current settings)
 * ============================================================ */
typedef struct {
    chassis_gain_t gain;
    RemoteControl_t remote;
    chassis_pid_param_t pid_param;
} ChassisSettings_t;

typedef enum {
    CHASSIS_GPS_ROUTE_RESULT_NONE = 0,
    CHASSIS_GPS_ROUTE_RESULT_POINT_ADDED,
    CHASSIS_GPS_ROUTE_RESULT_CLEARED,
    CHASSIS_GPS_ROUTE_RESULT_INVALID_FIX,
    CHASSIS_GPS_ROUTE_RESULT_FULL
} ChassisGPSRouteResult_t;

typedef enum {
    CHASSIS_GPS_ROUTE_STORAGE_NONE = 0,
    CHASSIS_GPS_ROUTE_STORAGE_LOADED,
    CHASSIS_GPS_ROUTE_STORAGE_SAVED,
    CHASSIS_GPS_ROUTE_STORAGE_EMPTY,
    CHASSIS_GPS_ROUTE_STORAGE_ERASE_ERROR,
    CHASSIS_GPS_ROUTE_STORAGE_PROGRAM_ERROR,
    CHASSIS_GPS_ROUTE_STORAGE_VERIFY_ERROR
} ChassisGPSRouteStorageStatus_t;

typedef enum {
    CHASSIS_GPS_ROUTE_SD_WAITING = 0,
    CHASSIS_GPS_ROUTE_SD_LOADED,
    CHASSIS_GPS_ROUTE_SD_SAVED,
    CHASSIS_GPS_ROUTE_SD_DELETED,
    CHASSIS_GPS_ROUTE_SD_NOT_FOUND,
    CHASSIS_GPS_ROUTE_SD_LOCK_ERROR,
    CHASSIS_GPS_ROUTE_SD_OPEN_ERROR,
    CHASSIS_GPS_ROUTE_SD_READ_ERROR,
    CHASSIS_GPS_ROUTE_SD_WRITE_ERROR,
    CHASSIS_GPS_ROUTE_SD_FORMAT_ERROR,
    CHASSIS_GPS_ROUTE_SD_VERIFY_ERROR,
    CHASSIS_GPS_ROUTE_SD_FLASH_FALLBACK
} ChassisGPSRouteSDStatus_t;

/* ============================================================
 *  External interface
 * ============================================================ */

extern void chassis_task(void *pvParameters);
extern void chassis_get_status_text(char *buf, uint32_t size);
extern void chassis_get_telemetry(ChassisTelemetry_t *out);
extern void chassis_get_settings(ChassisSettings_t *out);
extern void chassis_set_gain_param(ChassisGainParam_t param, float value);
extern void chassis_set_remote_param(ChassisRemoteParam_t param, float value);
extern void chassis_set_pid_param(ChassisPidParam_t param, double value);
extern void chassis_request_mode(CarMode_t mode);

extern volatile int gui_req_mode;   /* GUI mode request (-1=none, 0..6=CarMode_t) */
extern volatile CarMode_t chassis_current_mode_debug;
extern volatile int chassis_last_bt_req_debug;
extern chassis_move_t *const chassis_debug;
extern volatile uint8_t g_chassis_gps_route_count_debug;
extern volatile ChassisGPSRouteResult_t g_chassis_gps_route_last_result_debug;
extern volatile ChassisGPSRouteStorageStatus_t g_chassis_gps_route_storage_status_debug;
extern volatile uint32_t g_chassis_gps_route_storage_sequence_debug;
extern volatile uint8_t g_chassis_gps_route_storage_slot_debug;
extern volatile ChassisGPSRouteSDStatus_t g_chassis_gps_route_sd_status_debug;
extern volatile uint32_t g_chassis_gps_route_sd_save_count_debug;
extern volatile uint32_t g_chassis_gps_route_sd_load_count_debug;
extern volatile uint32_t g_chassis_gps_route_sd_delete_count_debug;
extern volatile uint32_t g_chassis_gps_route_sd_last_fresult_debug;
extern volatile uint8_t g_chassis_mag_initialized_debug;
extern volatile uint8_t g_chassis_powerless_debug;

/* Main loop 5 steps (defined here for external module replacement) */
extern void chassis_mode_change(chassis_move_t *chassis);
extern void chassis_feedback_update(chassis_move_t *chassis);
extern void chassis_set_control(chassis_move_t *chassis);
extern void chassis_control_loop(chassis_move_t *chassis);
extern void chassis_send_cmd(chassis_move_t *chassis);

#endif
