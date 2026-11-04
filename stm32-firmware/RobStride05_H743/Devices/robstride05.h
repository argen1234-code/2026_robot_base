#ifndef ROBSTRIDE05_H
#define ROBSTRIDE05_H
#include "bsp_can.h"
enum {
 RS_RUN_MODE=0x7005, RS_SPEED_REF=0x700A, RS_CURRENT_LIMIT=0x7018,
 RS_POSITION=0x7019, RS_CURRENT=0x701A, RS_VELOCITY=0x701B,
 RS_VBUS=0x701C, RS_ACCEL=0x7022, RS_REPORT_INTERVAL=0x7026, RS_TIMEOUT=0x7028
};
enum {RS_ACCEL_LEGACY=0x201E,RS_TIMEOUT_LEGACY=0x200C};
typedef struct {
 uint32_t feedback_ms, fault_ms, feedback_count, parameter_count, version;
 uint32_t fault, warning; uint8_t fault_bits, motor_state, version_valid;
 float position_rad, velocity_rad_s, torque_nm, temperature_c;
 uint16_t param_index; uint32_t param_raw, param_ms; bool param_ok;
 uint32_t identity_count,identity_ms; uint8_t identity_id,uid[8];
} Rs05Feedback;
CanFrame Rs05_Command(uint8_t motor_id,uint8_t type,uint8_t value);
CanFrame Rs05_Read(uint8_t motor_id,uint16_t index);
CanFrame Rs05_WriteRaw(uint8_t motor_id,uint16_t index,uint32_t raw);
CanFrame Rs05_WriteFloat(uint8_t motor_id,uint16_t index,float value);
CanFrame Rs05_Reporting(uint8_t motor_id,bool on);
CanFrame Rs05_VersionRequest(uint8_t motor_id);
CanFrame Rs05_IdentityRequest(uint8_t motor_id); /* Type 0 read-only query; reply is addressed to FE. */
bool Rs05_Parse(uint8_t motor_id,const CanFrame *f,Rs05Feedback *out);
float Rs05_RawFloat(uint32_t raw);
uint32_t Rs05_FloatRaw(float value);
#endif
