#ifndef APP_MOTOR_H
#define APP_MOTOR_H
#include "app_debug.h"
enum {CMD_PREPARE=1,CMD_ENABLE=2,CMD_RUN=3,CMD_STOP=4,CMD_DISABLE=5,
 CMD_CLEAR_FAULT=6};
enum {MOTOR_DISABLED=0,MOTOR_PREPARED=1,MOTOR_ENABLED=2,MOTOR_RUNNING=3,
 MOTOR_STOPPING=4,MOTOR_FAULT=5,MOTOR_CONFIGURING=6,MOTOR_ENABLING=7};
enum {RESULT_OK=0,RESULT_BAD_COMMAND=1,RESULT_RANGE=2,RESULT_STATE=3,
 RESULT_CONFIRM=4,RESULT_OFFLINE=5,RESULT_TX=6,RESULT_READBACK=7,
 RESULT_MOTOR_FAULT=8,RESULT_TIMEOUT=9,RESULT_BUSOFF=10,RESULT_PROTECTION=11,
 RESULT_RESERVED_12=12,RESULT_HEALTH=13,RESULT_OVERSPEED=14,RESULT_SERVICE_GAP=15,
 RESULT_PENDING=16,RESULT_BUSY=17};
void AppMotor_Init(void);
void AppMotor_Service(void); /* call every 1 ms from main, no waits */
void AppMotor_GetSnapshot(MotorStatus *out); /* main context only */
void AppMotor_SetHealth(uint32_t fault);
uint32_t AppMotor_PairReady(uint32_t mode);
void AppMotor_PairTargets(float first,float second);
float AppMotor_FeedbackSpeed(uint32_t slot);
float AppMotor_FeedbackCurrent(uint32_t slot);
float AppMotor_TargetSpeed(uint32_t slot);
/* Local transport adapters. MOTOR-task context only; reuse the same jobs,
 * identity checks, limits, arbitration and ramps without writing SWD mailboxes. */
uint32_t AppMotor_AuthorizePair(void);
uint32_t AppMotor_LocalCommand(uint32_t slot,const MotorCommand *command);
uint32_t AppMotor_LocalResult(uint32_t slot);
uint32_t AppMotor_RemoteTargets(float first,float second);
void AppMotor_RemoteStop(void);
void AppMotor_StopAll(void);
#endif
