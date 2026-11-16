#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdint.h>

typedef struct chassis_move_s chassis_move_t;

void Telemetry_Init(void);
void Telemetry_Update(chassis_move_t *chassis);
uint32_t Telemetry_GetDropCount(void);

#endif
