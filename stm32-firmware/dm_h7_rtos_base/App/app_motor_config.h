#ifndef APP_MOTOR_CONFIG_H
#define APP_MOTOR_CONFIG_H

#include <stdint.h>

#define MOTOR_CFG_FAULT_NONE            0U
#define MOTOR_CFG_FAULT_NOT_READY       1U
#define MOTOR_CFG_FAULT_ENABLE_FAILED   2U
#define MOTOR_CFG_FAULT_CONFIG          3U
#define MOTOR_CFG_FAULT_BUS_OFF         4U

void App_MotorConfig_Init(void);
void App_MotorConfig_Poll(uint32_t now);
void App_MotorConfig_HandleBtRequest(const char *payload);

void App_MotorConfig_RequestRestart(void);
void App_MotorConfig_ForceFault(uint8_t fault);
uint8_t App_MotorConfig_AllReady(void);
uint8_t App_MotorConfig_IsConfigActive(void);
uint8_t App_MotorConfig_GetFault(void);
void App_MotorConfig_GetMasks(uint8_t *verified_mask, uint8_t *enabled_mask);

#endif
