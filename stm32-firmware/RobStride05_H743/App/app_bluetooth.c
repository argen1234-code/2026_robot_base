#include "app_bluetooth.h"
#include "app_motor.h"
#include "app_config.h"
#include "app_usb.h"
#include "bsp_bluetooth.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile BluetoothDebug g_bt_debug;
static char frame[80],motion[8];
static uint32_t length,frame_active,frame_tick,arm_tick,waiting,authorizing;
static uint32_t last_was_cr,escape_pending;
/* Only side-effect-free queries may be completed by an idle, unterminated
 * frame. Motion/ARM/configuration always require an explicit terminator. */
#define BT_IDLE_FRAME_MS 60U
/* Minimal manual hold: latched by STOP/DISARM/timeout/fault, cleared only by
 * an explicit RELEASE. It is deliberately separate from motor ownership so
 * Bluetooth can still ARM while the ROS/USB bridge stays locked out. */
static uint32_t manual_hold;
static MotorCommand limits;

static void reply(const char *text)
{if(!BspBluetooth_Send(text))g_bt_debug.tx_drops++;}
/* Tail-only raw-byte trace: index is monotonic, slot is index & 15. */
static void rx_note(uint8_t byte)
{
 uint32_t index=g_bt_debug.rx_recent_index;
 g_bt_debug.rx_recent[index&15U]=byte;
 g_bt_debug.rx_recent_index=index+1U;
}
static void trim_frame(void)
{
 uint32_t start=0,end=length;
 while(start<end && frame[start]==' ')start++;
 while(end>start && frame[end-1]==' ')end--;
 if(start)memmove(frame,frame+start,end-start);
 length=end-start;
 frame[length]='\0';
}
static void parser_reset(void)
{
 length=frame_active=escape_pending=last_was_cr=0;
}
static void error(uint32_t result)
{
 char text[40];g_bt_debug.result=result;g_bt_debug.rejected++;
 snprintf(text,sizeof(text),"ERR %lu\r\n",(unsigned long)result);reply(text);
}
/* Latch the hold and stop at zero. AppMotor_StopAll() cancels both local
 * transports; AppBluetooth_Cancel() intentionally preserves manual_hold. */
static void hold_stop(void)
{
 manual_hold=1;
 AppMotor_StopAll();
}
/* Lease/UART loss: zero the running target and disarm Bluetooth. A partial
 * ARM transaction still needs the full stop so it cannot finish unauthorized. */
static void hold_zero(void)
{
 manual_hold=1;
 AppUsb_Cancel();
 if(g_bt_debug.arm_phase)AppMotor_StopAll();
 else AppBluetooth_Cancel();
}
uint32_t AppBluetooth_OwnsMotors(void)
{return g_bt_debug.armed || g_bt_debug.arm_phase;}
uint32_t AppBluetooth_BlocksUsb(void)
{return (AppBluetooth_OwnsMotors() || manual_hold) ? 1U : 0U;}
void AppBluetooth_Cancel(void)
{
 if(AppBluetooth_OwnsMotors())AppMotor_PairTargets(0,0);
 g_bt_debug.armed=g_bt_debug.arm_phase=g_bt_debug.moving=0;
 waiting=authorizing=0;parser_reset();motion[0]='\0';
 BspBluetooth_DiscardRx();
}
void AppBluetooth_Init(void)
{
 memset((void *)&g_bt_debug,0,sizeof(g_bt_debug));
 manual_hold=0;
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
 uint32_t result;const char *cursor;float a,b,c;
 trim_frame();
 memcpy((void *)g_bt_debug.last_payload,frame,length+1);
 g_bt_debug.frames++;
 if(!length){error(RESULT_BAD_COMMAND);return;}
 if(!strcmp(frame,"PING")){reply("PONG RS05-BT/1\r\n");return;}
 if(!strcmp(frame,"STATUS")){state();return;}
 if(!strcmp(frame,"RELEASE")) {
  /* Explicitly end the manual hold. Stopping also clears USB ownership and
   * zeroes its pending command tick, so no old speed can be replayed; a new
   * USB ARM plus a fresh velocity frame is required from here. */
  AppMotor_StopAll();AppUsb_Cancel();manual_hold=0;
  g_bt_debug.result=RESULT_OK;reply("OK RELEASE\r\n");return;
 }
 if(!strcmp(frame,"!") || !strcmp(frame,"DISARM")) {
  hold_stop();g_bt_debug.result=RESULT_OK;reply("OK DISARM\r\n");return;
 }
 if(!strcmp(frame,"S") || !strcmp(frame,"STOP")) {
  /* Stop is accepted even before ARM. Latch the hold first so a speed frame
   * from the ROS bridge cannot overwrite it on the next service cycle. */
  manual_hold=1;AppUsb_Cancel();
  if(g_bt_debug.arm_phase)AppMotor_StopAll();
  else if(g_bt_debug.armed)AppMotor_PairTargets(0,0);
  else AppMotor_RemoteStop();
  g_bt_debug.moving=0;motion[0]='\0';g_bt_debug.result=RESULT_OK;reply("OK S\r\n");return;
 }
 if(!strcmp(frame,"ARM") || !strcmp(frame,"~") || !strcmp(frame,"RESUME")) {
  uint32_t usb_owned=AppUsb_OwnsMotors();
  if(g_bt_debug.arm_phase){reply("PENDING ARM\r\n");return;}
  if(g_bt_debug.armed){g_bt_debug.result=RESULT_OK;reply("OK ARM\r\n");return;}
  if(g_debug.drive.active || AppChassis_Active()){error(RESULT_BUSY);return;}
  /* Every manual session retains priority after any cancellation or fault. */
  manual_hold=1;
  if(usb_owned)AppUsb_Cancel(); /* hand over at zero, never dual ownership */
  result=AppMotor_PairReady(2);
  if(result!=RESULT_OK && usb_owned) {
   /* USB was mid-transaction: abandon its partial jobs at zero, then run the
    * normal Bluetooth PREPARE/ENABLE transaction below. */
   AppMotor_StopAll();
   result=AppMotor_PairReady(2);
  }
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
  /* UART loss is a motion fault: stop and latch the hold so ROS cannot resume. */
  g_bt_debug.uart_errors=errors;hold_zero();error(RESULT_OFFLINE);
 }
 if(g_bt_debug.moving && now-g_bt_debug.last_motion_ms>=APP_BT_TIMEOUT_MS) {
  /* Motion lease expired without a new direction: stop at zero but KEEP the
   * armed session and manual hold. This is a direction pause, not a DISARM;
   * F/L/R can resume immediately and H never restores the old direction. */
  AppMotor_PairTargets(0,0);
  g_bt_debug.moving=0;motion[0]='\0';
  g_bt_debug.timeouts++;g_bt_debug.result=RESULT_OK;
  reply("OK TIMEOUT STOP\r\n");
 }
 if(g_bt_debug.armed && (result=AppMotor_PairReady(2))!=RESULT_OK) {
  hold_stop();error(result);
 }
 if(g_bt_debug.arm_phase) {
  if(authorizing) {
   result=AppMotor_AuthorizePair();
   if(result==RESULT_OK)authorizing=0;
   else if(result!=RESULT_OFFLINE || now-arm_tick>=4000U){hold_stop();error(result);}
  }
 }
 if(g_bt_debug.arm_phase && !authorizing) {
  slot=(g_bt_debug.arm_phase-1U)&1U;
  result=waiting?AppMotor_LocalResult(slot):RESULT_OK;
  if(now-arm_tick>=4000U || (result!=RESULT_OK && result!=RESULT_PENDING)) {
   hold_stop();error(result==RESULT_OK || result==RESULT_PENDING?RESULT_TIMEOUT:result);
  }else if(result==RESULT_OK) {
   if(waiting){g_bt_debug.arm_phase++;waiting=0;}
   if(g_bt_debug.arm_phase==5U) {
    g_bt_debug.arm_phase=0;g_bt_debug.armed=1;g_bt_debug.result=RESULT_OK;
    reply("OK ARM\r\n");
   }else {
    slot=(g_bt_debug.arm_phase-1U)&1U;
    limits.command=g_bt_debug.arm_phase<=2U?CMD_PREPARE:CMD_ENABLE;
    result=AppMotor_LocalCommand(slot,&limits);waiting=1;
    if(result!=RESULT_OK && result!=RESULT_PENDING){hold_stop();error(result);}
   }
  }
 }
 while(budget-- && BspBluetooth_Read(&byte)) {
  g_bt_debug.rx_bytes++;frame_tick=now;rx_note(byte);
  if(byte=='@'){frame_active=1;length=0;escape_pending=0;last_was_cr=0;continue;}
  if(!frame_active) {
   /* Outside a frame terminators and noise are ignored; CRLF is one event. */
   last_was_cr=(byte=='\r')?1U:0U;
   continue;
  }
  if(escape_pending) {
   escape_pending=0;
   if(byte=='r'){process(now);parser_reset();last_was_cr=1;continue;}
   if(byte=='n'){process(now);parser_reset();last_was_cr=0;continue;}
   frame_active=length=0;last_was_cr=0;error(RESULT_BAD_COMMAND);continue;
  }
  if(byte=='\\'){escape_pending=1;continue;}
  if(byte=='\r'){process(now);parser_reset();last_was_cr=1;continue;}
  if(byte=='\n'){process(now);parser_reset();last_was_cr=0;continue;}
  if(byte<' ' || byte>'~' || length>=sizeof(frame)-1U) {
   frame_active=length=escape_pending=0;last_was_cr=0;error(RESULT_BAD_COMMAND);continue;
  }
  last_was_cr=0;
  frame[length++]=(char)toupper((unsigned char)byte);
 }
 if(frame_active && now-frame_tick>=BT_IDLE_FRAME_MS) {
  /* A missing terminator is never treated as a motion/ARM/config command.
   * PING and STATUS are the only both-side-effect-free queries allowed once. */
  trim_frame();
  if((length==4U && !memcmp(frame,"PING",4U)) ||
     (length==6U && !memcmp(frame,"STATUS",6U))) {
   process(now);parser_reset();
  }else if(now-frame_tick>250U)parser_reset();
 }
 BspBluetooth_Service();
}
