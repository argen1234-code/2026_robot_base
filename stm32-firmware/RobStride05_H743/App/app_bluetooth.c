#include "app_bluetooth.h"
#include "app_motor.h"
#include "app_config.h"
#include "bsp_bluetooth.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile BluetoothDebug g_bt_debug;
static char frame[80],motion[8];
static uint32_t length,frame_active,saw_cr,frame_tick,arm_tick,waiting,authorizing;
static MotorCommand limits;

static void reply(const char *text)
{if(!BspBluetooth_Send(text))g_bt_debug.tx_drops++;}
static void error(uint32_t result)
{
 char text[40];g_bt_debug.result=result;g_bt_debug.rejected++;
 snprintf(text,sizeof(text),"ERR %lu\r\n",(unsigned long)result);reply(text);
}
uint32_t AppBluetooth_OwnsMotors(void)
{return g_bt_debug.armed || g_bt_debug.arm_phase;}
void AppBluetooth_Cancel(void)
{
 if(AppBluetooth_OwnsMotors())AppMotor_PairTargets(0,0);
 g_bt_debug.armed=g_bt_debug.arm_phase=g_bt_debug.moving=0;
 waiting=authorizing=length=frame_active=saw_cr=0;motion[0]='\0';
 BspBluetooth_DiscardRx();
}
void AppBluetooth_Init(void)
{
 memset((void *)&g_bt_debug,0,sizeof(g_bt_debug));
 memset(&limits,0,sizeof(limits));
 limits.confirm_key=APP_ENABLE_KEY;limits.speed_limit_rad_s=APP_DEFAULT_SPEED_CAP;
 limits.current_limit_a=APP_DEFAULT_CURRENT_A;limits.accel_rad_s2=APP_DEFAULT_ACCEL;
 g_bt_debug.speed_rad_s=1;g_bt_debug.speed_cap_rad_s=limits.speed_limit_rad_s;
 g_bt_debug.current_a=limits.current_limit_a;g_bt_debug.accel_rad_s2=limits.accel_rad_s2;
 g_bt_debug.left_slot=1;g_bt_debug.left_sign=-1;g_bt_debug.right_sign=1;
 AppBluetooth_Cancel();BspBluetooth_Init();
}
static void state(void)
{
 char text[192];
 snprintf(text,sizeof(text),"STATE armed=%lu phase=%lu moving=%lu speed=%.3f result=%lu v1=%.3f v2=%.3f\r\n",
  (unsigned long)g_bt_debug.armed,(unsigned long)g_bt_debug.arm_phase,
  (unsigned long)g_bt_debug.moving,(double)g_bt_debug.speed_rad_s,
  (unsigned long)g_bt_debug.result,(double)g_debug.motor.velocity_rad_s,(double)g_debug.motor2.velocity_rad_s);
 reply(text);
}
static uint32_t number(const char **cursor,float *out,char end)
{
 char *next;double value;
 if(!**cursor || isspace((unsigned char)**cursor))return 0;
 value=strtod(*cursor,&next);
 if(next==*cursor || *next!=end || !isfinite(value) || fabs(value)>1e6)return 0;
 *out=(float)value;*cursor=end?next+1:next;return 1;
}
static uint32_t move(const char *token,uint32_t now)
{
 float left=0,right=0,raw[2],speed=g_bt_debug.speed_rad_s;uint32_t result;
 if(!strcmp(token,"F")){left=right=speed;}
 else if(!strcmp(token,"B")){left=right=-speed;}
 else if(!strcmp(token,"L") || !strcmp(token,"Q")){left=-speed;right=speed;}
 else if(!strcmp(token,"R") || !strcmp(token,"E")){left=speed;right=-speed;}
 else if(!strcmp(token,"FL")){right=speed;}
 else if(!strcmp(token,"FR")){left=speed;}
 else if(!strcmp(token,"BL")){left=-speed;}
 else if(!strcmp(token,"BR")){right=-speed;}
 else return RESULT_BAD_COMMAND;
 if(!g_bt_debug.armed)return RESULT_STATE;
 raw[g_bt_debug.left_slot]=left*g_bt_debug.left_sign;
 raw[1U-g_bt_debug.left_slot]=right*g_bt_debug.right_sign;
 result=AppMotor_RemoteTargets(raw[0],raw[1]);
 if(result)return result;
 strcpy(motion,token);g_bt_debug.moving=(left!=0 || right!=0);
 g_bt_debug.last_motion_ms=now;return RESULT_OK;
}
static void process(uint32_t now)
{
 uint32_t i,result;const char *cursor;float a,b,c;
 frame[length]='\0';
 for(i=0;i<length;i++)frame[i]=(char)toupper((unsigned char)frame[i]);
 memcpy((void *)g_bt_debug.last_payload,frame,length+1);
 g_bt_debug.frames++;
 if(!strcmp(frame,"PING")){reply("PONG RS05-BT/1\r\n");return;}
 if(!strcmp(frame,"STATUS")){state();return;}
 if(!strcmp(frame,"!") || !strcmp(frame,"DISARM") || !strcmp(frame,"RELEASE")) {
  AppMotor_StopAll();g_bt_debug.result=RESULT_OK;reply("OK DISARM\r\n");return;
 }
 if(!strcmp(frame,"S") || !strcmp(frame,"STOP")) {
  /* Stop is accepted even before ARM and also cancels SWD chassis control. */
  if(g_bt_debug.arm_phase)AppMotor_StopAll();
  else AppMotor_RemoteStop();
  g_bt_debug.moving=0;motion[0]='\0';g_bt_debug.result=RESULT_OK;reply("OK S\r\n");return;
 }
 if(!strcmp(frame,"ARM") || !strcmp(frame,"~") || !strcmp(frame,"RESUME")) {
  if(g_bt_debug.arm_phase){reply("PENDING ARM\r\n");return;}
  if(g_bt_debug.armed){g_bt_debug.result=RESULT_OK;reply("OK ARM\r\n");return;}
  if(g_debug.drive.active || AppChassis_Active()){error(RESULT_BUSY);return;}
  result=AppMotor_PairReady(2);
  if(result==RESULT_OK) {
   g_bt_debug.armed=1;g_bt_debug.result=RESULT_OK;AppMotor_PairTargets(0,0);reply("OK ARM\r\n");return;
  }
  result=AppMotor_AuthorizePair();
  if(result!=RESULT_OK && result!=RESULT_OFFLINE){error(result);return;}
  authorizing=(result==RESULT_OFFLINE);
  g_bt_debug.arm_phase=1;waiting=0;arm_tick=now;
  g_bt_debug.result=RESULT_PENDING;reply("PENDING ARM\r\n");return;
 }
 if(!strncmp(frame,"SPEED=",6)) {
  cursor=frame+6;
  if(!number(&cursor,&a,'\0') || a<0 || a>APP_SPEED_MAX_RAD_S){error(RESULT_RANGE);return;}
  if(g_bt_debug.moving){error(RESULT_BUSY);return;}
  g_bt_debug.speed_rad_s=a;g_bt_debug.result=RESULT_OK;reply("OK SPEED\r\n");return;
 }
 if(!strncmp(frame,"LIMIT=",6)) {
  cursor=frame+6;
  if(!number(&cursor,&a,',') || !number(&cursor,&b,',') || !number(&cursor,&c,'\0') ||
     a<=0 || a>APP_SPEED_MAX_RAD_S || b<=0 || b>APP_CURRENT_MAX_A || c<=0 || c>APP_ACCEL_MAX_RAD_S2) {
   error(RESULT_RANGE);return;
  }
  if(AppBluetooth_OwnsMotors()){error(RESULT_BUSY);return;}
  limits.speed_limit_rad_s=a;limits.current_limit_a=b;limits.accel_rad_s2=c;
  g_bt_debug.speed_cap_rad_s=a;g_bt_debug.current_a=b;g_bt_debug.accel_rad_s2=c;
  g_bt_debug.result=RESULT_OK;reply("OK LIMIT\r\n");return;
 }
 if(!strncmp(frame,"MAP=",4)) {
  cursor=frame+4;
  if(!number(&cursor,&a,',') || !number(&cursor,&b,',') || !number(&cursor,&c,'\0') ||
     (a!=0 && a!=1) || fabsf(b)!=1 || fabsf(c)!=1){error(RESULT_RANGE);return;}
  if(AppBluetooth_OwnsMotors()){error(RESULT_BUSY);return;}
  g_bt_debug.left_slot=(uint32_t)a;g_bt_debug.left_sign=b;g_bt_debug.right_sign=c;
  g_bt_debug.result=RESULT_OK;reply("OK MAP\r\n");return;
 }
 if(!strcmp(frame,"H")) {
  if(!g_bt_debug.armed){error(RESULT_STATE);return;}
  g_bt_debug.last_motion_ms=now;g_bt_debug.result=RESULT_OK;reply("OK H\r\n");return;
 }
 result=move(frame,now);
 if(result) {
  if(g_bt_debug.moving)AppBluetooth_Cancel();
  error(result);return;
 }
 g_bt_debug.result=RESULT_OK;g_bt_debug.accepted++;reply("OK MOVE\r\n");
}
void AppBluetooth_Service(uint32_t now)
{
 uint8_t byte;uint32_t budget=128,result,slot,errors=BspBluetooth_Errors();
 if(errors!=g_bt_debug.uart_errors) {
  g_bt_debug.uart_errors=errors;AppBluetooth_Cancel();error(RESULT_OFFLINE);
 }
 if(g_bt_debug.moving && now-g_bt_debug.last_motion_ms>=APP_BT_TIMEOUT_MS) {
  AppBluetooth_Cancel();g_bt_debug.timeouts++;error(RESULT_TIMEOUT);
 }
 if(g_bt_debug.armed && (result=AppMotor_PairReady(2))!=RESULT_OK) {
  AppMotor_StopAll();error(result);
 }
 if(g_bt_debug.arm_phase) {
  if(authorizing) {
   result=AppMotor_AuthorizePair();
   if(result==RESULT_OK)authorizing=0;
   else if(result!=RESULT_OFFLINE || now-arm_tick>=4000U){AppMotor_StopAll();error(result);}
  }
 }
 if(g_bt_debug.arm_phase && !authorizing) {
  slot=(g_bt_debug.arm_phase-1U)&1U;
  result=waiting?AppMotor_LocalResult(slot):RESULT_OK;
  if(now-arm_tick>=4000U || (result!=RESULT_OK && result!=RESULT_PENDING)) {
   AppMotor_StopAll();error(result==RESULT_OK || result==RESULT_PENDING?RESULT_TIMEOUT:result);
  }else if(result==RESULT_OK) {
   if(waiting){g_bt_debug.arm_phase++;waiting=0;}
   if(g_bt_debug.arm_phase==5U) {
    g_bt_debug.arm_phase=0;g_bt_debug.armed=1;g_bt_debug.result=RESULT_OK;
    reply("OK ARM\r\n");
   }else {
    slot=(g_bt_debug.arm_phase-1U)&1U;
    limits.command=g_bt_debug.arm_phase<=2U?CMD_PREPARE:CMD_ENABLE;
    result=AppMotor_LocalCommand(slot,&limits);waiting=1;
    if(result!=RESULT_OK && result!=RESULT_PENDING){AppMotor_StopAll();error(result);}
   }
  }
 }
 if(frame_active && now-frame_tick>250U){frame_active=length=saw_cr=0;}
 while(budget-- && BspBluetooth_Read(&byte)) {
  g_bt_debug.rx_bytes++;frame_tick=now;
  if(byte=='@'){frame_active=1;length=saw_cr=0;continue;}
  if(!frame_active)continue;
  if(saw_cr) {
   if(byte=='\n')process(now);else error(RESULT_BAD_COMMAND);
   frame_active=length=saw_cr=0;continue;
  }
  if(byte=='\r'){saw_cr=1;continue;}
  if(byte<' ' || byte>'~' || length>=sizeof(frame)-1U) {
   frame_active=length=saw_cr=0;error(RESULT_BAD_COMMAND);continue;
  }
  frame[length++]=(char)byte;
 }
 BspBluetooth_Service();
}
