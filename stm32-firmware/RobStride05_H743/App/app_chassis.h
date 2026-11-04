#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H
#include <stdint.h>
#include "jy901s.h"
/* Commands: 1 apply gains/config, 2 wheel speed, 3 absolute heading,
 * 4 stop/release, 5 reset heading zero while stopped.
 * Angles are continuous degrees from boot; positive is left/CCW viewed above.
 * Wheel speeds/measured currents use forward-positive vehicle coordinates.
 * RS05 closes the wheel speed loop internally in speed mode (2). */
typedef struct {
 uint32_t seq,key,command,left_slot;
 float left_sign,right_sign,yaw_sign;
 float target_deg,left_rad_s,right_rad_s,turn_limit_rad_s;
 float angle_kp,angle_ki,angle_kd;
} ChassisCommand;
typedef struct {
 uint32_t snapshot_seq,executed_seq,result,active,mode,published_tick_ms,imu_online;
 Jy901sData imu;
 float yaw_deg,target_deg,angle_error_deg,turn_rad_s;
 float left_target_rad_s,right_target_rad_s,left_speed_rad_s,right_speed_rad_s;
 float left_error_rad_s,right_error_rad_s,left_iq_a,right_iq_a;
 ChassisCommand config;
} ChassisStatus;
void AppChassis_Init(void);
void AppChassis_Service(uint32_t now);
void AppChassis_Cancel(void);
uint32_t AppChassis_Active(void);
#endif
