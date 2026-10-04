#ifndef BSP_USB_H
#define BSP_USB_H

#include "main.h"

/* Jetson → STM32 帧模式 */
typedef enum {
    JETSON_MODE_GPS_ROS  = 1, /* Jetson GPS + ROS fusion navigation */
    JETSON_MODE_REMOTE   = 2, /* WeChat remote control */
    JETSON_MODE_INDOOR   = 3, /* ROS/Nav2 indoor navigation */
    JETSON_MODE_GPS_ONLY = 4  /* STM32 standalone GPS navigation */
} JetsonMode_t;

/* Jetson 下发的 12 字节帧解析结果 */
typedef struct {
    uint8_t mode;    /* 1=GPS_ROS, 2=REMOTE, 3=INDOOR, 4=GPS_ONLY */
    float   vx;      /* 线速度 (m/s), 正值前进 */
    float   vz;      /* 角速度 (rad/s), 正值左转 (Z轴向上) */
    uint32_t last_update_tick; /* Last checksum-valid cmd_vel frame tick. */
    uint32_t update_sequence;  /* Increments once per valid cmd_vel frame. */
} cmd_vel_t;

/* Jetson scene_cmd: BB 55 cmd XOR, cmd=1 indoor / 2 outdoor. */
typedef enum {
    JETSON_SCENE_NONE    = 0,
    JETSON_SCENE_INDOOR  = 1,
    JETSON_SCENE_OUTDOOR = 2
} JetsonScene_t;

typedef struct {
    JetsonScene_t scene;
    uint32_t last_update_tick;
    uint32_t update_sequence;
} scene_cmd_t;

typedef enum {
    JETSON_GPS_ROUTE_NONE   = 0,
    JETSON_GPS_ROUTE_BEGIN  = 1,
    JETSON_GPS_ROUTE_POINT  = 2,
    JETSON_GPS_ROUTE_COMMIT = 3,
    JETSON_GPS_ROUTE_CLEAR  = 4,
    JETSON_GPS_ROUTE_SPEED  = 5
} JetsonGpsRouteCommand_t;

typedef struct {
    JetsonGpsRouteCommand_t command;
    uint8_t index;
    uint8_t total;
    uint8_t loop_enable;
    double latitude;
    double longitude;
    uint32_t last_update_tick;
    uint32_t update_sequence;
} gps_route_cmd_t;

#define USB_SENSOR_FLAG_GPS_VALID       0x01U
#define USB_SENSOR_FLAG_MAG_VALID       0x02U
#define USB_SENSOR_FLAG_IMU_VALID       0x04U
#define USB_SENSOR_FLAG_GNSS_HEAD_VALID 0x08U
#define USB_SENSOR_FLAG_ROUTE_VALID     0x10U
#define USB_SENSOR_FLAG_TARGET_VALID    0x20U

typedef struct {
    uint8_t flags;
    uint8_t car_mode;
    uint8_t satellites;
    uint8_t fix_quality;
    uint8_t route_total;
    uint8_t route_slot;
    uint8_t navigation_active;
    uint8_t heading_status;
    uint8_t loop_enable;
    uint16_t sequence;

    double latitude;
    double longitude;
    double altitude;
    float gnss_heading;
    float gnss_speed;
    float velocity_north;
    float velocity_east;
    float mag_yaw;
    float mag_pitch;
    float mag_roll;
    float imu_roll;
    float imu_pitch;
    float imu_yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float motor_speed[4];
    double target_latitude;
    double target_longitude;
    double route_latitude;
    double route_longitude;
} usb_sensor_telemetry_t;

void       USB_Init(void);
void       USB_ProcessRxData(uint8_t *pBuf, uint16_t Size);
cmd_vel_t  USB_GetCmdVel(void);
scene_cmd_t USB_GetSceneCmd(void);
gps_route_cmd_t USB_GetGpsRouteCmd(void);
void       USB_SendSensorTelemetry(const usb_sensor_telemetry_t *telemetry);

#endif
