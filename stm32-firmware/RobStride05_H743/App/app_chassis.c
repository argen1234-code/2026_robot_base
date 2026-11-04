#include "app_chassis.h"
#include "app_motor.h"
#include "app_bluetooth.h"
#include "app_config.h"
#include "app_pid.h"
#include "bsp_jy901s.h"
#include "main.h"
#include <math.h>
#include <string.h>
static Jy901s imu;
static AppPid angle_pid;
static ChassisCommand config,candidate;
static ChassisStatus status;
static uint32_t seen,candidate_valid,control_tick,publish_tick;
static float clamp(float value,float limit){return fmaxf(-limit,fminf(limit,value));}
uint32_t AppChassis_Active(void){return status.active;}
static void reset_pid(void)
{AppPid_Reset(&angle_pid);}
void AppChassis_Init(void)
{
 Jy901s_Init(&imu);BspJy901s_Init();
 config.left_slot=1;config.left_sign=-1;config.right_sign=1;config.yaw_sign=1;
 config.turn_limit_rad_s=2;
 config.angle_kp=0.04f;config.angle_ki=0;config.angle_kd=0.005f;
 g_debug.chassis_cmd=config;status.config=config;
}
void AppChassis_Cancel(void)
{
 if(status.active)AppMotor_PairTargets(0,0);
 status.active=0;status.mode=0;status.turn_rad_s=0;reset_pid();
 seen=g_debug.chassis_cmd.seq;candidate_valid=0;
}
static uint32_t execute(const ChassisCommand *c)
{
 uint32_t result,i;float value[10];
 if(c->command==4){AppBluetooth_Cancel();AppChassis_Cancel();return RESULT_OK;}
 if(AppBluetooth_OwnsMotors())return RESULT_BUSY;
 if(c->key!=APP_ENABLE_KEY)return RESULT_CONFIRM;
 if(c->command==5) {
  if(status.active || g_debug.drive.active)return RESULT_STATE;
  Jy901s_Zero(&imu);return RESULT_OK;
 }
 if(c->command<1 || c->command>3)return RESULT_BAD_COMMAND;
 memcpy(value,&c->left_sign,sizeof(value));
 for(i=0;i<10;i++)if(!isfinite(value[i]))return RESULT_RANGE;
 if(c->left_slot>1 || fabsf(c->left_sign)!=1 || fabsf(c->right_sign)!=1 ||
    fabsf(c->yaw_sign)!=1 || c->turn_limit_rad_s<=0)return RESULT_RANGE;
 if((status.active || g_debug.drive.active) && (c->left_slot!=config.left_slot || c->left_sign!=config.left_sign ||
    c->right_sign!=config.right_sign || c->yaw_sign!=config.yaw_sign))return RESULT_STATE;
 if(c->command!=1) {
  if(g_debug.drive.active)return RESULT_STATE;
  result=AppMotor_PairReady(2);if(result)return result;
  if(c->command==3 && (!imu.data.ready || !status.imu_online))return RESULT_OFFLINE;
  reset_pid();status.active=1;status.mode=c->command==3?2U:1U;
 }
 config=*c;return RESULT_OK;
}
void AppChassis_Service(uint32_t now)
{
 uint8_t byte;uint32_t seq,result;ChassisCommand c;float left,right,raw[2];
 while(BspJy901s_Read(&byte))Jy901s_Input(&imu,byte,now);
 status.imu_online=imu.data.ready && now-imu.data.angle_tick_ms<=250U;
 seq=g_debug.chassis_cmd.seq;__DMB();
 if(seq && !(seq&1U) && seq!=seen) {
  c=g_debug.chassis_cmd;__DMB();
  if(seq==g_debug.chassis_cmd.seq) {
   if(candidate_valid && !memcmp(&c,&candidate,sizeof(c))) {
    seen=seq;candidate_valid=0;status.result=execute(&c);status.executed_seq=seq;
   }else {candidate=c;candidate_valid=1;}
  }
 }else candidate_valid=0;
 if(status.active) {
  result=AppMotor_PairReady(2);
  if(result || (status.mode==2 && !status.imu_online)) {
   AppChassis_Cancel();status.result=result?result:RESULT_OFFLINE;
  }
 }
 status.yaw_deg=config.yaw_sign*imu.data.yaw_deg;
 status.target_deg=config.target_deg;status.angle_error_deg=config.target_deg-status.yaw_deg;
 if(now-control_tick>=20U) {
  float dt=(now-control_tick)*0.001f;control_tick=now;
  if(status.active) {
   left=config.left_rad_s;right=config.right_rad_s;
   if(status.mode==2) {
    status.turn_rad_s=AppPid_Step(&angle_pid,status.angle_error_deg,status.yaw_deg,
      config.angle_kp,config.angle_ki,0,dt,config.turn_limit_rad_s);
    status.turn_rad_s=clamp(status.turn_rad_s-config.angle_kd*config.yaw_sign*imu.data.gyro_z_deg_s,config.turn_limit_rad_s);
    left=-status.turn_rad_s;right=status.turn_rad_s;
   }
   status.left_target_rad_s=left;status.right_target_rad_s=right;
   raw[config.left_slot]=left*config.left_sign;raw[1U-config.left_slot]=right*config.right_sign;
   AppMotor_PairTargets(raw[0],raw[1]);
  }else {status.left_target_rad_s=status.right_target_rad_s=0;}
 }
 status.left_speed_rad_s=AppMotor_FeedbackSpeed(config.left_slot)*config.left_sign;
 status.right_speed_rad_s=AppMotor_FeedbackSpeed(1U-config.left_slot)*config.right_sign;
 status.left_target_rad_s=AppMotor_TargetSpeed(config.left_slot)*config.left_sign;
 status.right_target_rad_s=AppMotor_TargetSpeed(1U-config.left_slot)*config.right_sign;
 status.left_error_rad_s=status.left_target_rad_s-status.left_speed_rad_s;
 status.right_error_rad_s=status.right_target_rad_s-status.right_speed_rad_s;
 status.left_iq_a=AppMotor_FeedbackCurrent(config.left_slot)*config.left_sign;
 status.right_iq_a=AppMotor_FeedbackCurrent(1U-config.left_slot)*config.right_sign;
 status.imu=imu.data;status.config=config;
 if(now-publish_tick>=20U) {
  publish_tick=now;status.published_tick_ms=now;status.snapshot_seq+=2;
  g_debug.chassis.snapshot_seq=status.snapshot_seq-1;__DMB();
  /* Publish everything except the commit word, then commit even last. */
  memcpy((void *)((volatile uint8_t *)&g_debug.chassis+4),(uint8_t *)&status+4,sizeof(status)-4);
  __DMB();g_debug.chassis.snapshot_seq=status.snapshot_seq;
 }
}
