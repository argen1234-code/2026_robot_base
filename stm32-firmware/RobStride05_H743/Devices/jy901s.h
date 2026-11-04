#ifndef JY901S_H
#define JY901S_H
#include <stdint.h>
typedef struct {
 uint32_t ready,angle_tick_ms,angle_frames,gyro_frames,checksum_errors;
 float roll_deg,pitch_deg,raw_yaw_deg,yaw_deg,gyro_z_deg_s;
} Jy901sData;
typedef struct {
 Jy901sData data;
 uint8_t frame[11],used;
 float previous_yaw;
} Jy901s;
void Jy901s_Init(Jy901s *imu);
void Jy901s_Input(Jy901s *imu,uint8_t byte,uint32_t now);
void Jy901s_Zero(Jy901s *imu);
#endif
