#ifndef APP_BLUETOOTH_H
#define APP_BLUETOOTH_H
#include <stdint.h>
typedef struct {
 uint32_t armed,arm_phase,moving,last_motion_ms,result;
 uint32_t rx_bytes,frames,accepted,rejected,timeouts,uart_errors,tx_drops;
 float speed_rad_s,speed_cap_rad_s,current_a,accel_rad_s2;
 uint32_t left_slot;
 float left_sign,right_sign;
 char last_payload[80];
} BluetoothDebug;
/* Observation only. Existing g_debug layout is unchanged. */
extern volatile BluetoothDebug g_bt_debug;
void AppBluetooth_Init(void);
void AppBluetooth_Service(uint32_t now);
void AppBluetooth_Cancel(void);
uint32_t AppBluetooth_OwnsMotors(void);
#endif
