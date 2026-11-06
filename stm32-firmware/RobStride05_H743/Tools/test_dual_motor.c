/* The REAL production service runs against two deterministic CAN peers.
 * No firmware test commands, duplicate state machine, hardware or motion. */
#include <assert.h>
#include <stdio.h>
#include "app_motor.c"
#include "app_usb.h"
#include "usbd_cdc_if.h"

volatile AppDebug g_debug;
MockSCB mock_scb;

/* ---- USB CDC hardware mock: the real App/app_usb.c is linked and exercised. */
USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t cdc_ring[8192];static uint32_t cdc_head,cdc_tail,cdc_overflow;
static uint32_t cdc_tx_frames;
uint8_t CDC_Transmit_FS(uint8_t *buf,uint16_t len)
{(void)buf;(void)len;cdc_tx_frames++;return USBD_OK;}
uint16_t CDC_ReadRxData(uint8_t *buf,uint16_t max)
{
 uint16_t n=0;
 while(n<max && cdc_tail!=cdc_head)buf[n++]=cdc_ring[cdc_tail++%sizeof(cdc_ring)];
 return n;
}
uint32_t CDC_GetRxOverflowCount(void){return cdc_overflow;}

static uint8_t bt_ring[4096];static uint32_t bt_head,bt_tail,bt_errors;
static char bt_output[65536];static size_t bt_output_len;
void BspBluetooth_Init(void){bt_head=bt_tail=bt_errors=0;bt_output_len=0;bt_output[0]='\0';}
bool BspBluetooth_Read(uint8_t *byte){if(bt_head==bt_tail)return false;*byte=bt_ring[bt_tail++%4096];return true;}
void BspBluetooth_DiscardRx(void){bt_tail=bt_head;}
bool BspBluetooth_Send(const char *text){size_t n=strlen(text);assert(bt_output_len+n<sizeof(bt_output));memcpy(bt_output+bt_output_len,text,n+1);bt_output_len+=n;return true;}
void BspBluetooth_Service(void){}
uint32_t BspBluetooth_Errors(void){return bt_errors;}
void BspJy901s_Init(void){}
static uint8_t imu_ring[1024];static unsigned imu_head,imu_tail;
bool BspJy901s_Read(uint8_t *byte){if(imu_head==imu_tail)return false;*byte=imu_ring[imu_tail++%1024];return true;}
typedef struct {
 uint8_t id,uid,state;
 uint32_t mode,current,accel,timeout,speed;
 float velocity;
 uint32_t speed_writes,stops,enables;
} Peer;
static Peer peer[2];
static uint32_t tick,rx_head,rx_tail,id_writes,collisions,cancels;
static CanFrame rx_frames[128];
static bool tx_active,bus_off;
static bool plant_enabled;
static float plant_yaw,plant_rate;
static CanStats cs;
uint32_t HAL_GetTick(void){return tick;}
uint32_t BspTime_Us(void){return tick*1000U;}
bool BspTime_DebugFrozen(void){return false;}
bool BspCan_Start(void){return true;}
bool BspCan_IsBusOff(void){return bus_off;}
bool BspCan_Recover(void){bus_off=false;return true;}
void BspCan_GetStats(CanStats *out){*out=cs;}
void BspCan_Cancel(void){if(tx_active)cancels++;tx_active=false;}
void BspCan_DiscardRx(void){rx_tail=rx_head;}
bool BspCan_Receive(CanFrame *f)
{if(rx_tail==rx_head)return false;*f=rx_frames[rx_tail++%128U];return true;}
static void enqueue(CanFrame f)
{assert(rx_head-rx_tail<128U);f.tick_ms=tick;rx_frames[rx_head++%128U]=f;cs.rx++;}
static void feedback(Peer *p)
{
 CanFrame f={0};float v=plant_enabled?p->velocity:Rs05_RawFloat(p->speed);
 uint16_t encoded=(uint16_t)((v+50.0f)*65535.0f/100.0f);
 f.id=(2UL<<24)|((uint32_t)p->state<<22)|((uint32_t)p->id<<8)|APP_HOST_ID;
 f.data[0]=0x80;f.data[2]=(uint8_t)(encoded>>8);f.data[3]=(uint8_t)encoded;
 f.data[4]=0x80;f.data[6]=1;f.data[7]=44;enqueue(f);
}
static uint32_t *parameter(Peer *p,uint16_t index)
{
 switch(index) {
 case RS_RUN_MODE:return &p->mode;
 case RS_CURRENT_LIMIT:return &p->current;
 case RS_ACCEL:return &p->accel;
 case RS_TIMEOUT:return &p->timeout;
 case RS_SPEED_REF:return &p->speed;
 default:return NULL;
 }
}
bool BspCan_SendBegin(const CanFrame *f)
{
 uint32_t i,type=(f->id>>24)&31U;
 if(tx_active){collisions++;return false;}
 if(bus_off)return false;
 tx_active=true;
 for(i=0;i<2U;i++)if(peer[i].id && peer[i].id==(f->id&255U)) {
  Peer *p=&peer[i];CanFrame r={0};
  uint16_t index=(uint16_t)(f->data[0]|((uint16_t)f->data[1]<<8));
  assert(type!=18 || index!=0x7006); /* Never send current targets. */
  uint32_t *value=parameter(p,index);
  if(type==0) {
   r.id=((uint32_t)p->id<<8)|0xFEU;memset(r.data,p->uid,8);enqueue(r);
  }else if(type==7) {
   p->id=(uint8_t)(f->id>>16);id_writes++;
  }else if(type==3) {
   p->state=2;p->enables++;feedback(p);
  }else if(type==4) {
   p->state=0;p->speed=0;p->stops++;feedback(p);
  }else if(type==18 && value) {
   memcpy(value,f->data+4,4);
   if(index==RS_SPEED_REF)p->speed_writes++;
  }else if(type==17) {
   uint32_t raw=value?*value:Rs05_FloatRaw(index==RS_VBUS?24.0f:index==RS_CURRENT?p->velocity*0.375f:0.0f);
   r.id=(17UL<<24)|((uint32_t)p->id<<8)|APP_HOST_ID;
   r.data[0]=(uint8_t)index;r.data[1]=(uint8_t)(index>>8);
   memcpy(r.data+4,&raw,4);enqueue(r);
  }
 }
 return true;
}
uint32_t BspCan_SendPoll(void)
{if(!tx_active)return CAN_TX_IDLE;tx_active=false;cs.tx_ok++;return CAN_TX_OK;}
static void imu_frame(uint8_t type,float value,bool corrupt)
{
 uint8_t frame[11]={0x55,0,0,0,0,0,0,0,0,0,0},i;
 int16_t raw=(int16_t)(value*32768.0f/(type==0x53?180.0f:2000.0f));
 frame[1]=type;frame[6]=(uint8_t)raw;frame[7]=(uint8_t)((uint16_t)raw>>8);
 for(i=0;i<10;i++)frame[10]=(uint8_t)(frame[10]+frame[i]);
 if(corrupt)frame[10]++;
 for(i=0;i<11;i++)imu_ring[imu_head++%1024]=frame[i];
}
static void run_ms(uint32_t duration)
{
 uint32_t end=tick+duration,i;
 for(;tick<end;tick+=2U) {
  if(plant_enabled) {
   for(i=0;i<2;i++) {
    float target=peer[i].state==2?Rs05_RawFloat(peer[i].speed):0;
    /* Motor-internal speed response; the MCU only supplies speed targets. */
    peer[i].velocity+=(target-peer[i].velocity)*0.025f;
   }
   plant_rate=(peer[0].velocity+peer[1].velocity)*20;
   plant_yaw+=plant_rate*0.002f;
   if(tick%100==0) {
    float yaw=fmodf(plant_yaw+180,360);if(yaw<0)yaw+=360;yaw-=180;
    imu_frame(0x52,plant_rate,false);imu_frame(0x53,yaw,false);
   }
  }
  if(tick%20U==0U)for(i=0;i<2U;i++)if(peer[i].id)feedback(&peer[i]);
  AppMotor_Service();
 }
}
static void setup_command(uint32_t command,bool isolated)
{
 uint32_t seq=g_debug.setup.seq+2U;
 g_debug.setup.seq=seq-1U;g_debug.setup.key=APP_ENABLE_KEY;
 g_debug.setup.command=command;g_debug.setup.old_id=127;g_debug.setup.new_id=126;
 g_debug.setup.isolated_key=isolated?APP_ISOLATED_KEY:0;g_debug.setup.seq=seq;
}
static void command(uint32_t slot,uint32_t cmd,float target)
{
 volatile MotorCommand *m=slot?&g_debug.cmd2:&g_debug.cmd;
 uint32_t seq=m->seq+2U;
 m->seq=seq-1U;m->command=cmd;m->confirm_key=APP_ENABLE_KEY;
 m->target_rad_s=target;m->speed_limit_rad_s=2;
 m->current_limit_a=0.5f;m->accel_rad_s2=5;m->seq=seq;
}
static void drive_command(uint32_t cmd,float first,float second)
{
 uint32_t seq=(g_debug.drive_cmd.seq+2U)&~1U;
 g_debug.drive_cmd.seq=seq-1U;g_debug.drive_cmd.key=APP_ENABLE_KEY;
 g_debug.drive_cmd.command=cmd;
 g_debug.drive_cmd.motor1_rad_s=first;g_debug.drive_cmd.motor2_rad_s=second;
 g_debug.drive_cmd.seq=seq;
}
static void drive_checks(void)
{
 uint32_t i;
 /* Prepare via the SAME public mailbox at the existing full 33 rad/s cap. */
 for(i=0;i<2U;i++) {
  command(i,CMD_PREPARE,0);
  (i?&g_debug.cmd2:&g_debug.cmd)->speed_limit_rad_s=33;
 }
 run_ms(500);
 for(i=0;i<2U;i++) {
  command(i,CMD_ENABLE,0);
  (i?&g_debug.cmd2:&g_debug.cmd)->speed_limit_rad_s=33;
 }
 run_ms(400);
 /* Zero-speed sensor noise must not prevent acquiring WASD. */
 peer[0].speed=Rs05_FloatRaw(0.065f);peer[1].speed=Rs05_FloatRaw(-0.08f);
 drive_command(1,0,0);run_ms(40);
 assert(g_debug.drive.active && g_debug.drive.result==RESULT_OK);
 drive_command(2,33,-33);run_ms(40);
 assert(g_debug.motor.target_rad_s==33 && g_debug.motor2.target_rad_s==-33);
 command(0,CMD_RUN,1);run_ms(40);
 assert(g_debug.motor.result==RESULT_STATE && g_debug.drive.active);
 assert(g_debug.motor.target_rad_s==33 && g_debug.motor2.target_rad_s==-33);
 /* Odd/incomplete pair commits never change just one wheel. */
 g_debug.drive_cmd.seq++;g_debug.drive_cmd.motor1_rad_s=-1;run_ms(40);
 assert(g_debug.motor.target_rad_s==33 && g_debug.motor2.target_rad_s==-33);
 g_debug.drive_cmd.seq++;
 /* A bad second target rejects the entire vector, clears both previous ones. */
 drive_command(2,1,33.1f);run_ms(40);
 assert(g_debug.drive.result==RESULT_RANGE && !g_debug.drive.active);
 assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 run_ms(1000);
 drive_command(1,0,0);run_ms(40);assert(g_debug.drive.active);
 drive_command(2,1,-1);run_ms(40);
 for(i=0;i<5U;i++) {
  run_ms(180);drive_command(2,1,-1);run_ms(20);
  assert(g_debug.drive.active && g_debug.drive.result==RESULT_OK);
 }
 /* The board stops even if the browser AND host stop servicing entirely. */
 run_ms(700);
 assert(!g_debug.drive.active && g_debug.drive.result==RESULT_TIMEOUT);
 assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 drive_command(2,1,-1);run_ms(40);
 assert(g_debug.drive.result==RESULT_STATE && !g_debug.drive.active);
 run_ms(1000);
 drive_command(1,0,0);run_ms(40);assert(g_debug.drive.active);
 drive_command(2,1,-1);run_ms(40);
 drive_command(3,0,0);run_ms(40);
 assert(!g_debug.drive.active && g_debug.drive.result==RESULT_OK);
 assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 run_ms(400);
 command(0,CMD_RUN,1);g_debug.cmd.speed_limit_rad_s=33;run_ms(40);
 assert(g_debug.motor.state==MOTOR_RUNNING && g_debug.motor.target_rad_s==1);
 assert(g_debug.motor2.target_rad_s==0);
 /* Acquire while an independent RUN still has a nonzero target/ramp and
  * feedback. Both targets are taken over at zero, using the prepared ramps. */
 drive_command(1,2,-2);run_ms(40);
 assert(g_debug.drive.active && g_debug.drive.result==RESULT_OK);
 assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 drive_command(3,0,0);run_ms(40);
 assert(!g_debug.drive.active);
 command(0,CMD_STOP,0);run_ms(400);
 drive_command(1,0,0);run_ms(40);assert(g_debug.drive.active);
 drive_command(2,1,-1);run_ms(40);
 /* One wheel's fault must also zero the healthy wheel in this mode. */
 motors[1].fb.fault=1;run_ms(40);
 assert(!g_debug.drive.active && g_debug.motor.target_rad_s==0);
 assert(g_debug.motor2.state==MOTOR_FAULT);
 motors[1].fb.fault=0;
 g_debug.stop_all_seq+=2;run_ms(100);
 printf("PASS drive: paired full-range targets, isolated ownership, torn commit, reject both, lease expiry, stale update, release, independent-control restoration, peer fault\n");
}
static void chassis_command(uint32_t command_number)
{
 uint32_t seq=(g_debug.chassis_cmd.seq+2)&~1U;
 g_debug.chassis_cmd.seq=seq-1;g_debug.chassis_cmd.key=APP_ENABLE_KEY;
 g_debug.chassis_cmd.command=command_number;g_debug.chassis_cmd.seq=seq;
 run_ms(40);
}
static void chassis_checks(void)
{
 Jy901s parser;float y;unsigned i;
 Jy901s_Init(&parser);
 /* Stream fragments/noise, checksum error, both wrap directions, boot zero. */
 imu_frame(0x53,179,false);imu_frame(0x53,-179,false);imu_frame(0x53,179,false);
 imu_frame(0x53,70,true);
 while(imu_head!=imu_tail){uint8_t b=imu_ring[imu_tail++%1024];Jy901s_Input(&parser,b,tick);}
 assert(parser.data.ready && fabsf(parser.data.yaw_deg)<0.02f && parser.data.checksum_errors==1);
 for(i=0;i<2;i++){command(i,CMD_PREPARE,0);}
 run_ms(500);assert(g_debug.motor.state==MOTOR_PREPARED && g_debug.motor2.state==MOTOR_PREPARED);
 assert(g_debug.motor.mode==2 && g_debug.motor2.mode==2);
 assert(peer[0].mode==2 && peer[1].mode==2);
 for(i=0;i<2;i++)command(i,CMD_ENABLE,0);
 run_ms(400);assert(g_debug.motor.enabled && g_debug.motor2.enabled);
 plant_enabled=true;run_ms(120);
 assert(g_debug.chassis.imu_online && fabsf(g_debug.chassis.yaw_deg)<0.02f);
 g_debug.chassis_cmd.left_rad_s=1;g_debug.chassis_cmd.right_rad_s=1;
 chassis_command(2);run_ms(4000);
 assert(g_debug.chassis.active && g_debug.chassis.result==0);
 assert(fabsf(g_debug.chassis.left_speed_rad_s-1)<0.05f && fabsf(g_debug.chassis.right_speed_rad_s-1)<0.05f);
 assert(g_debug.motor.target_rad_s==1 && g_debug.motor2.target_rad_s==-1);
 assert(peer[0].id==126 && peer[1].id==127);
 assert(Rs05_RawFloat(peer[0].speed)<0 && Rs05_RawFloat(peer[1].speed)>0);
 assert(fabsf(g_debug.chassis.left_iq_a-AppMotor_FeedbackCurrent(1)*-1)<0.001f);
 assert(g_debug.chassis.left_iq_a>0 && g_debug.chassis.right_iq_a>0);
 /* Live tuning must preserve control and target. */
 g_debug.chassis_cmd.angle_kp=0.05f;chassis_command(1);run_ms(300);
 assert(g_debug.chassis.active && g_debug.chassis.config.angle_kp==0.05f);
 g_debug.chassis_cmd.target_deg=90;chassis_command(3);run_ms(12000);
 y=g_debug.chassis.yaw_deg;
 assert(g_debug.chassis.active && fabsf(y-90)<2.0f);
 g_debug.chassis_cmd.target_deg=0;chassis_command(3);run_ms(12000);
 assert(fabsf(g_debug.chassis.yaw_deg)<2.0f);
 g_debug.chassis_cmd.target_deg=-90;chassis_command(3);run_ms(12000);
 assert(fabsf(g_debug.chassis.yaw_deg+90)<2.0f);
 g_debug.chassis_cmd.target_deg=360;chassis_command(3);run_ms(22000);
 assert(fabsf(g_debug.chassis.yaw_deg-360)<2.0f);
 /* IMU loss cancels heading; pure wheel-speed control needs no IMU. */
 plant_enabled=false;run_ms(300);
 assert(!g_debug.chassis.active && !g_debug.chassis.imu_online);
 assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 chassis_command(3);assert(g_debug.chassis.result==RESULT_OFFLINE);
 chassis_command(2);assert(g_debug.chassis.active && g_debug.chassis.mode==1);
 plant_enabled=true;run_ms(200);
 /* Single-motor stop cancels ownership and the peer target; zero is explicit. */
 command(0,CMD_STOP,0);run_ms(200);
 assert(!g_debug.chassis.active && g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 chassis_command(5);assert(fabsf(g_debug.chassis.yaw_deg)<0.1f);
 plant_enabled=false;g_debug.stop_all_seq+=2;run_ms(100);
 printf("PASS chassis: JY901 checksum/wrap/boot zero, mode2 prepare/enable, speed targets only, forward signs, measured current, live heading PID, +90/0/-90/360 degree convergence, peer stop, explicit zero\n");
}
static void bt_feed(const char *text)
{while(*text){assert(bt_head-bt_tail<sizeof(bt_ring));bt_ring[bt_head++%4096]=(uint8_t)*text++;}}
static void bt_send(const char *text){bt_feed(text);run_ms(40);}
static void bluetooth_checks(void)
{
 uint32_t i,frames;
 /* Full production parser -> motor jobs -> deterministic CAN peers. */
 bt_send("@PING\r\n");assert(strstr(bt_output,"PONG RS05-BT/1"));
 bt_send("@F\r\n");assert(!g_bt_debug.armed && g_bt_debug.result==RESULT_STATE);
 bt_feed("noise@AR");run_ms(20);assert(!g_bt_debug.armed);
 bt_send("M\r\n");run_ms(1300);
 if(!g_bt_debug.armed)fprintf(stderr,"BT ARM phase=%lu result=%lu states=%lu/%lu jobs=%lu/%lu; %s\n",(unsigned long)g_bt_debug.arm_phase,(unsigned long)g_bt_debug.result,(unsigned long)g_debug.motor.state,(unsigned long)g_debug.motor2.state,(unsigned long)g_debug.motor.job_command,(unsigned long)g_debug.motor2.job_command,bt_output);
 assert(g_bt_debug.armed && !g_bt_debug.arm_phase);
 assert(g_debug.motor.enable_requested && g_debug.motor2.enable_requested);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 assert(strstr(bt_output,"OK ARM\r\n"));
 bt_send("@SPEED=33\r\n");bt_send("@F\r\n");
 assert(g_debug.motor.target_rad_s==33 && g_debug.motor2.target_rad_s==-33);
 for(i=0;i<4;i++){run_ms(160);bt_send("@f\r\n");assert(g_bt_debug.armed && g_bt_debug.moving);}
 /* Repeat commands renew, rather than toggling direction as the reference did. */
 bt_send("@S\r\n");bt_send("@SPEED=1\r\n");
 bt_send("@L\r\n");assert(g_debug.motor.target_rad_s==1 && g_debug.motor2.target_rad_s==1);
 bt_send("@R\r\n");assert(g_debug.motor.target_rad_s==-1 && g_debug.motor2.target_rad_s==-1);
 bt_send("@FL\r\n");assert(g_debug.motor.target_rad_s==1 && g_debug.motor2.target_rad_s==0);
 bt_send("@FR\r\n");assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==-1);
 bt_send("@BL\r\n");assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==1);
 bt_send("@BR\r\n");assert(g_debug.motor.target_rad_s==-1 && g_debug.motor2.target_rad_s==0);
 bt_send("@B\r\n");assert(g_debug.motor.target_rad_s==-1 && g_debug.motor2.target_rad_s==1);
 command(0,CMD_RUN,2);run_ms(40);assert(g_debug.motor.result==RESULT_STATE);
 drive_command(1,0,0);run_ms(40);assert(g_debug.drive.result==RESULT_BUSY && g_bt_debug.moving);
 /* Queries/garbage cannot renew the 600 ms lease. Expiry stops at zero but
  * keeps the armed session, so the next direction needs no ARM. */
 for(i=0;i<5;i++){bt_send("@STATUS\r\n");run_ms(100);}
 assert(g_bt_debug.armed && g_bt_debug.timeouts==1);
 assert(!g_bt_debug.moving &&
        g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 bt_send("@H\r\n");
 assert(g_bt_debug.armed && !g_bt_debug.moving &&
        g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 bt_send("@F\r\n"); /* direct resume, no ARM */
 assert(g_bt_debug.moving && g_debug.motor.target_rad_s==1 &&
        g_debug.motor2.target_rad_s==-1);
 bt_errors++;run_ms(40); /* UART fault still revokes */
 assert(!g_bt_debug.armed && !g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 bt_send("@ARM\r\n");assert(g_bt_debug.armed);
 bt_send("@F\r\n");command(0,CMD_STOP,0);run_ms(40);
 assert(!AppBluetooth_OwnsMotors() && !g_debug.motor2.target_rad_s);
 /* Framing: LF and CR are both valid; CRLF is exactly one command. */
 frames=g_bt_debug.frames;
 bt_send("@PING\n");assert(g_bt_debug.frames==frames+1);
 bt_send("@PING\r");assert(g_bt_debug.frames==frames+2);
 bt_send("\n");assert(g_bt_debug.frames==frames+2); /* stray LF: ignored */
 bt_feed("@PING\r");run_ms(10);bt_feed("\n");run_ms(10);
 assert(g_bt_debug.frames==frames+3); /* split CRLF: no duplicate */
 bt_feed("@");for(i=0;i<100;i++)bt_feed("A");bt_send("\r\n@PING\r\n");
 assert(g_bt_debug.frames==frames+4); /* overlong rejected, then resync */
 bt_send("@AR");run_ms(300);bt_send("M\r\n");assert(!g_bt_debug.armed);
 bt_send("@!\r\n");run_ms(100);
 assert(!peer[0].state && !peer[1].state);
 bt_send("@LIMIT=2,2.4,20\r\n");bt_send("@ARM\r\n");run_ms(1300);
 assert(g_bt_debug.armed);
 bt_send("@SPEED=3\r\n");bt_send("@F\r\n");
 assert(g_bt_debug.result==RESULT_RANGE && !g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 bt_send("@SPEED=NAN\r\n");assert(g_bt_debug.result==RESULT_RANGE);
 bt_send("@!\r\n");run_ms(100);
 bt_send("@ARM\r\n");bt_send("@S\r\n");run_ms(1000);
 assert(!g_bt_debug.armed && !g_bt_debug.arm_phase && !peer[0].state && !peer[1].state);
 bt_send("@ARM\r\n");run_ms(1300);bt_send("@SPEED=1\r\n");bt_send("@F\r\n");
 peer[1].state=0;feedback(&peer[1]);run_ms(40);
 assert(!g_bt_debug.armed && !g_debug.motor.enable_requested && !g_debug.motor2.enable_requested);
 bt_send("@!\r\n");run_ms(100);
 assert(!collisions);
 printf("PASS bluetooth: CRLF streaming/resync, async zero-speed ARM, full +/-33, directions, repeat renewal, SWD exclusion/stop, 600ms expiry, UART loss, per-motor cap, cancel ARM, peer fault, DISARM\n");
}
static uint32_t bt_text_count(const char *needle)
{
 uint32_t n=0;const char *p=bt_output;
 while((p=strstr(p,needle))!=NULL){n++;p++;}
 return n;
}
static void bluetooth_framing_checks(void)
{
 uint32_t frames,rejected,pongs,index,accepted;
 /* Every physical/LF/CR/literal terminator and space trimming. */
 frames=g_bt_debug.frames;pongs=bt_text_count("PONG RS05-BT/1");
 bt_send("@PING\r\n");
 assert(g_bt_debug.frames==frames+1 && bt_text_count("PONG RS05-BT/1")==pongs+1);
 index=g_bt_debug.rx_recent_index;assert(index>=7U);
 assert(g_bt_debug.rx_recent[(index-7U)&15U]=='@');
 assert(g_bt_debug.rx_recent[(index-6U)&15U]=='P');
 assert(g_bt_debug.rx_recent[(index-1U)&15U]=='\n');
 bt_send("@PING\n");assert(g_bt_debug.frames==frames+2);
 bt_send("@PING\r");assert(g_bt_debug.frames==frames+3);
 bt_send("@PING\\r\\n");assert(g_bt_debug.frames==frames+4);
 bt_send("@ PING \r\n");assert(g_bt_debug.frames==frames+5);
 bt_feed("@PI");run_ms(10);bt_feed("NG\r");run_ms(10);bt_feed("\n");run_ms(40);
 assert(g_bt_debug.frames==frames+6); /* fragmented CRLF, exactly one */
 assert(bt_text_count("PONG RS05-BT/1")==pongs+6);

 /* Unterminated PING/STATUS: once after ~60 ms idle, never repeated. */
 frames=g_bt_debug.frames;bt_feed("@PING");run_ms(20);
 assert(g_bt_debug.frames==frames);
 run_ms(100);assert(g_bt_debug.frames==frames+1);
 run_ms(100);assert(g_bt_debug.frames==frames+1);
 frames=g_bt_debug.frames;bt_feed("@STATUS");run_ms(100);
 assert(g_bt_debug.frames==frames+1);
 run_ms(100);assert(g_bt_debug.frames==frames+1);

 /* Unknown/malformed/literal input is still rejected, never weakened. */
 frames=g_bt_debug.frames;rejected=g_bt_debug.rejected;
 bt_send("@NOPE\r\n");
 assert(g_bt_debug.frames==frames+1 && g_bt_debug.rejected==rejected+1);
 assert(g_bt_debug.result==RESULT_BAD_COMMAND);
 frames=g_bt_debug.frames;rejected=g_bt_debug.rejected;
 bt_feed("@PING\x01\r\n");run_ms(40);
 assert(g_bt_debug.frames==frames && g_bt_debug.rejected==rejected+1);
 frames=g_bt_debug.frames;rejected=g_bt_debug.rejected;
 bt_send("@NOPE\\r\\n");
 assert(g_bt_debug.frames==frames+1 && g_bt_debug.rejected==rejected+1);

 /* Motion/ARM/configuration require a terminator: a lone @F is inert. */
 bt_send("@ARM\r\n");run_ms(2000);assert(g_bt_debug.armed);
 bt_send("@SPEED=2\r\n");assert(g_bt_debug.result==RESULT_OK);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 frames=g_bt_debug.frames;bt_feed("@F");run_ms(300); /* expired incomplete motion frame */
 assert(g_bt_debug.frames==frames && !g_bt_debug.moving);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 bt_send("L\r\n"); /* no '@' after the discarded fragment: ignored */
 assert(!g_bt_debug.moving && !g_debug.motor.target_rad_s &&
        !g_debug.motor2.target_rad_s);
 bt_feed("@F");run_ms(100);
 assert(!g_bt_debug.moving);
 bt_send("L\r\n"); /* preserve fragmented FL inside the original 250ms window */
 assert(g_debug.motor.target_rad_s==2.0f && g_debug.motor2.target_rad_s==0.0f &&
        g_bt_debug.moving);
 accepted=g_bt_debug.accepted;
 bt_send("@F\r\n"); /* CRLF must execute exactly once */
 assert(g_bt_debug.accepted==accepted+1 &&
        g_debug.motor.target_rad_s==2.0f && g_debug.motor2.target_rad_s==-2.0f);
 bt_send("@S\r\n");run_ms(40);bt_send("@!\r\n");run_ms(100);
 printf("PASS bluetooth framing: CRLF/LF/CR/literal, split CRLF once, trim, idle PING/STATUS once, no-tail motion inert, malformed/unknown rejected, raw-byte diagnostics\n");
}
/* ============================================================================
 * Bluetooth priority over the ROS/USB bridge.
 * Real app_bluetooth.c + app_usb.c + app_motor.c; only CDC and CAN are mocked.
 * ==========================================================================*/
static void cdc_feed(const void *data,uint32_t length)
{
 uint32_t i;const uint8_t *p=(const uint8_t *)data;
 for(i=0;i<length;i++) {
  assert(cdc_head-cdc_tail<sizeof(cdc_ring));
  cdc_ring[cdc_head++%sizeof(cdc_ring)]=p[i];
 }
}
static void usb_velocity(float vx,float wz)
{
 uint8_t f[12];uint32_t i;uint8_t checksum=0;
 f[0]=APP_USB_DOWN_CMD_VEL;f[1]=APP_USB_MAGIC;f[2]=2U;
 memcpy(f+3,&vx,sizeof(vx));memcpy(f+7,&wz,sizeof(wz));
 for(i=2U;i<11U;i++)checksum^=f[i];
 f[11]=checksum;cdc_feed(f,sizeof(f));run_ms(40);
}
static void usb_service_cmd(uint8_t command)
{
 uint8_t f[4]={APP_USB_DOWN_SERVICE,APP_USB_MAGIC,command,command};
 cdc_feed(f,sizeof(f));run_ms(20);
}
static void usb_arm(void){usb_service_cmd(APP_USB_CMD_ARM);}
static void usb_disarm(void){usb_service_cmd(APP_USB_CMD_DISARM);}
static void usb_priority_checks(void)
{
 uint32_t before;
 hUsbDeviceFS.dev_state=USBD_STATE_CONFIGURED;
 hUsbDeviceFS.pClassData=(void *)1;
 cdc_head=cdc_tail=cdc_overflow=0;

 /* Recover from the prior Bluetooth fault test and establish Bluetooth. */
 bt_send("@ARM\r\n");run_ms(1800);
 assert(g_bt_debug.armed && AppBluetooth_OwnsMotors());
 bt_send("@RELEASE\r\n");run_ms(100);
 assert(!AppBluetooth_BlocksUsb() && !AppBluetooth_OwnsMotors());
 assert(!AppUsb_OwnsMotors());

 /* ROS/USB connects, arms and moves. */
 usb_arm();run_ms(1800);
 if(!(AppUsb_OwnsMotors() && g_usb_debug.armed &&
      g_usb_debug.last_result==RESULT_OK))
  fprintf(stderr,"USB ARM failed: own=%lu armed=%lu phase=%lu result=%lu btblocks=%lu btarmed=%lu drive=%lu chassis=%lu motors=%lu/%lu jobs=%lu/%lu\n",
   (unsigned long)AppUsb_OwnsMotors(),(unsigned long)g_usb_debug.armed,
   (unsigned long)g_usb_debug.arm_phase,(unsigned long)g_usb_debug.last_result,
   (unsigned long)AppBluetooth_BlocksUsb(),(unsigned long)g_bt_debug.armed,
   (unsigned long)g_debug.drive.active,(unsigned long)AppChassis_Active(),
   (unsigned long)g_debug.motor.state,(unsigned long)g_debug.motor2.state,
   (unsigned long)g_debug.motor.job_command,(unsigned long)g_debug.motor2.job_command);
 assert(AppUsb_OwnsMotors() && g_usb_debug.armed &&
        g_usb_debug.last_result==RESULT_OK);
 usb_velocity(-0.1f,0.0f);
 assert(g_debug.motor.target_rad_s>2.0f && g_debug.motor2.target_rad_s<-2.0f);

 /* Bluetooth ARM takes over an already-armed/moving USB session at zero. */
 bt_send("@ARM\r\n");run_ms(1400);
 assert(g_bt_debug.armed && !AppUsb_OwnsMotors());
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 assert(g_usb_debug.armed==0U && g_usb_debug.arm_phase==0U);
 before=cdc_tx_frames;
 usb_velocity(-0.1f,0.0f);usb_velocity(0.0f,0.0f);
 usb_arm();usb_disarm();run_ms(160);
 assert(g_bt_debug.armed && !AppUsb_OwnsMotors());
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 assert(g_usb_debug.last_result==RESULT_BUSY);
 assert(cdc_tx_frames>before); /* telemetry keeps flowing while blocked */

 /* Cancellation on the very first ARM (before STOP) retains manual hold. */
 AppBluetooth_Cancel();
 usb_arm();usb_velocity(-0.1f,0.0f);
 assert(AppBluetooth_BlocksUsb() && !AppUsb_OwnsMotors());
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 bt_send("@ARM\r\n");assert(g_bt_debug.armed);

 /* Bluetooth STOP blocks the ROS bridge, even when Bluetooth later times out. */
 bt_send("@SPEED=2\r\n");bt_send("@F\r\n");
 assert(g_debug.motor.target_rad_s==2.0f && g_debug.motor2.target_rad_s==-2.0f);
 usb_velocity(-0.1f,0.0f);usb_velocity(0.0f,0.0f);usb_arm();usb_disarm();
 assert(g_bt_debug.armed && g_debug.motor.target_rad_s==2.0f &&
        g_debug.motor2.target_rad_s==-2.0f);
 bt_send("@S\r\n");run_ms(40);
 assert(g_bt_debug.armed && g_debug.motor.target_rad_s==0.0f &&
        g_debug.motor2.target_rad_s==0.0f);
 usb_velocity(-0.1f,0.0f);usb_arm();usb_disarm();run_ms(80);
 assert(g_bt_debug.armed && !g_debug.motor.target_rad_s &&
        !g_debug.motor2.target_rad_s);

 /* 600 ms moving lease expires -> zero target, still armed + held. */
 bt_send("@F\r\n");assert(g_debug.motor.target_rad_s==2.0f);
 run_ms(700);
 assert(g_bt_debug.armed && !g_bt_debug.moving && g_bt_debug.timeouts>=1);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 usb_arm();usb_velocity(-0.1f,0.0f);run_ms(80);
 assert(!AppUsb_OwnsMotors() && g_usb_debug.last_result==RESULT_BUSY);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);

 /* Explicit RELEASE clears the hold; USB needs a new ARM and a NEW velocity. */
 bt_send("@RELEASE\r\n");run_ms(100);
 assert(!AppBluetooth_BlocksUsb());
 usb_velocity(-0.1f,0.0f); /* stale frame while disarmed: must not move */
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 usb_arm();run_ms(1800);
 assert(AppUsb_OwnsMotors() && g_usb_debug.last_result==RESULT_OK);
 run_ms(100);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 usb_velocity(-0.1f,0.0f);
 assert(g_debug.motor.target_rad_s>2.0f && g_debug.motor2.target_rad_s<-2.0f);

 /* Global stop clears USB ownership but must not erase a Bluetooth hold. */
 bt_send("@S\r\n");run_ms(40); /* Bluetooth not armed: latch hold + cancel USB */
 assert(!AppUsb_OwnsMotors() && !AppBluetooth_OwnsMotors());
 usb_velocity(-0.1f,0.0f);run_ms(100); /* ROS must not re-apply its old speed */
 assert(!AppUsb_OwnsMotors() && !g_debug.motor.target_rad_s &&
        !g_debug.motor2.target_rad_s);
 g_debug.stop_all_seq+=2U;run_ms(100);
 assert(!AppUsb_OwnsMotors() && !AppBluetooth_OwnsMotors());
 usb_arm();run_ms(40);assert(g_usb_debug.last_result==RESULT_BUSY);
 bt_send("@RELEASE\r\n");run_ms(100);

 /* A motor/CAN fault during Bluetooth ownership stops and holds. */
 bt_send("@ARM\r\n");run_ms(1800);assert(g_bt_debug.armed);
 bt_send("@SPEED=2\r\n");bt_send("@F\r\n");
 motors[1].fb.fault=1U;run_ms(100);
 assert(!g_bt_debug.armed && !g_debug.motor.target_rad_s &&
        !g_debug.motor2.target_rad_s);
 usb_arm();run_ms(40);assert(g_usb_debug.last_result==RESULT_BUSY);
 motors[1].fb.fault=0U;bt_send("@RELEASE\r\n");run_ms(100);

 /* Bluetooth takeover while USB is still in its multi-step ARM transaction.
  * Force stale identity so USB remains in arm_phase deterministically. */
 motors[0].fb.identity_ms=tick-2000U;motors[1].fb.identity_ms=tick-2000U;
 usb_arm();
 assert(AppUsb_OwnsMotors() && g_usb_debug.arm_phase && !g_usb_debug.armed);
 bt_send("@ARM\r\n");run_ms(2600);
 assert(g_bt_debug.armed && !AppUsb_OwnsMotors());
 assert(g_usb_debug.arm_phase==0U && g_usb_debug.armed==0U);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);

 /* DISARM keeps the manual hold (only RELEASE clears it). */
 bt_send("@!\r\n");run_ms(100);
 assert(!AppBluetooth_OwnsMotors() && !g_debug.motor.target_rad_s &&
        !g_debug.motor2.target_rad_s);
 usb_arm();usb_velocity(-0.1f,0.0f);run_ms(80);
 assert(!AppUsb_OwnsMotors() && g_usb_debug.last_result==RESULT_BUSY);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 bt_send("@RELEASE\r\n");run_ms(100);
 assert(!AppBluetooth_BlocksUsb());
 bt_send("@ARM\r\n");run_ms(1800);assert(g_bt_debug.armed);

 /* Global emergency stop is still the highest priority. */
 bt_send("@F\r\n");assert(g_debug.motor.target_rad_s==2.0f);
 g_debug.stop_all_seq+=2U;run_ms(100);
 assert(!g_bt_debug.armed && !AppUsb_OwnsMotors());
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 printf("PASS bluetooth priority: USB moving takeover at zero, blocked USB velocity/ARM/DISARM, STOP hold, 600ms timeout hold, explicit RELEASE, global stop preserving hold, motor fault\n");
}
static void bluetooth_switch_checks(void)
{
 uint32_t rejected,timeouts,accepted;size_t outlen;
 bt_send("@ARM\r\n");run_ms(2000);assert(g_bt_debug.armed);
 bt_send("@SPEED=2\r\n");
 rejected=g_bt_debug.rejected;
 /* Direct WASD-style switching while armed, including S then F. */
 bt_send("@F\r\n");assert(g_debug.motor.target_rad_s==2.0f &&
                          g_debug.motor2.target_rad_s==-2.0f);
 bt_send("@L\r\n");assert(g_debug.motor.target_rad_s==2.0f &&
                          g_debug.motor2.target_rad_s==2.0f);
 bt_send("@R\r\n");assert(g_debug.motor.target_rad_s==-2.0f &&
                          g_debug.motor2.target_rad_s==-2.0f);
 bt_send("@S\r\n");assert(g_bt_debug.armed &&
                          !g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 bt_send("@F\r\n");assert(g_debug.motor.target_rad_s==2.0f &&
                          g_debug.motor2.target_rad_s==-2.0f);
 assert(g_bt_debug.rejected==rejected);

 /* Expiry stops at zero, keeps armed/hold, notifies once, and never ERR9. */
 timeouts=g_bt_debug.timeouts;rejected=g_bt_debug.rejected;outlen=bt_output_len;
 run_ms(700);
 assert(g_bt_debug.armed && !g_bt_debug.moving);
 assert(!g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 assert(g_bt_debug.timeouts==timeouts+1);
 assert(g_bt_debug.result==RESULT_OK && g_bt_debug.rejected==rejected);
 assert(strstr(bt_output+outlen,"OK TIMEOUT STOP\r\n")!=NULL);
 assert(strstr(bt_output+outlen,"ERR 9\r\n")==NULL);
 run_ms(200);assert(g_bt_debug.timeouts==timeouts+1);

 /* H does not restore the old direction, and next L needs no ARM. */
 bt_send("@H\r\n");
 assert(g_bt_debug.armed && !g_bt_debug.moving &&
        !g_debug.motor.target_rad_s && !g_debug.motor2.target_rad_s);
 run_ms(700);assert(g_bt_debug.timeouts==timeouts+1 && g_bt_debug.armed);
 accepted=g_bt_debug.accepted;rejected=g_bt_debug.rejected;
 bt_send("@L\r\n");
 assert(g_bt_debug.moving && g_debug.motor.target_rad_s==2.0f &&
        g_debug.motor2.target_rad_s==2.0f);
 assert(g_bt_debug.accepted==accepted+1 && g_bt_debug.rejected==rejected);

 /* H extends the live lease; expiry still works after that. */
 run_ms(400);bt_send("@H\r\n");run_ms(300);
 assert(g_bt_debug.moving && g_bt_debug.armed &&
        g_bt_debug.timeouts==timeouts+1);
 run_ms(400);
 assert(!g_bt_debug.moving && g_bt_debug.armed &&
        g_bt_debug.timeouts==timeouts+2);

 /* USB velocity/ARM/DISARM cannot steal a live Bluetooth direction. */
 bt_send("@F\r\n");assert(g_debug.motor.target_rad_s==2.0f);
 usb_velocity(-0.1f,0.0f);usb_arm();usb_disarm();run_ms(100);
 assert(g_bt_debug.armed && g_debug.motor.target_rad_s==2.0f &&
        g_debug.motor2.target_rad_s==-2.0f && !AppUsb_OwnsMotors());
 assert(g_usb_debug.last_result==RESULT_BUSY);
 bt_send("@!\r\n");run_ms(100);
 assert(!g_bt_debug.armed && !g_debug.motor.target_rad_s &&
        !g_debug.motor2.target_rad_s);
 printf("PASS bluetooth switching: F/L/R/S/F direct, expiry keeps ARM, OK TIMEOUT STOP once, H no resume, H renews lease, L resumes without ARM, ROS blocked\n");
}
int main(void)
{
 peer[0].id=127;peer[0].uid=11;peer[1].uid=22;
 AppMotor_Init();run_ms(1100);
 assert(g_debug.motor.online && !g_debug.motor2.online);
 command(0,CMD_PREPARE,0);run_ms(40);
 assert(g_debug.motor.result==RESULT_CONFIRM && !peer[0].enables);
 setup_command(1,false);run_ms(40);
 assert(!g_debug.setup.dual_ready && g_debug.setup.result==RESULT_OFFLINE);
 setup_command(2,false);run_ms(40);assert(!id_writes);
 setup_command(2,true);run_ms(500);
 assert(id_writes==1U && peer[0].id==126 && g_debug.setup.result==RESULT_OK);
 assert(!g_debug.setup.dual_ready);
 peer[1].id=127;run_ms(1100);
 setup_command(1,false);run_ms(60);
 assert(g_debug.setup.dual_ready && g_debug.setup.result==RESULT_OK);
 command(0,CMD_PREPARE,0);command(1,CMD_PREPARE,0);run_ms(500);
 assert(g_debug.motor.state==MOTOR_PREPARED && g_debug.motor2.state==MOTOR_PREPARED);
 assert(g_debug.motor.result==0 && g_debug.motor2.result==0);
 command(0,CMD_ENABLE,0);command(1,CMD_ENABLE,0);run_ms(400);
 assert(g_debug.motor.state==MOTOR_ENABLED && g_debug.motor2.state==MOTOR_ENABLED);
 command(0,CMD_RUN,0.5f);command(1,CMD_RUN,-0.3f);run_ms(400);
 assert(g_debug.motor.state==MOTOR_RUNNING && g_debug.motor2.state==MOTOR_RUNNING);
 assert(g_debug.motor.target_rad_s==0.5f && g_debug.motor2.target_rad_s==-0.3f);
 assert(g_debug.motor.velocity_rad_s>0.45f && g_debug.motor2.velocity_rad_s< -0.25f);
 assert(g_debug.motor.current_limit_a==0.5f && g_debug.motor2.current_limit_a==0.5f);
 command(0,CMD_DISABLE,0);run_ms(100);
 assert(g_debug.motor.state==MOTOR_DISABLED && g_debug.motor2.state==MOTOR_RUNNING);
 assert(g_debug.motor2.target_rad_s==-0.3f);
 /* A disabled motor's polling cannot cancel its running peer's transmission. */
 assert(!collisions);
 g_debug.stop_all_seq=2;run_ms(100);
 assert(!g_debug.motor.enable_requested && !g_debug.motor2.enable_requested);
 assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 assert(peer[0].state==0 && peer[1].state==0);
 assert(!g_debug.motor.job_command && !g_debug.motor2.job_command);
 drive_checks();
 chassis_checks();
 bluetooth_checks();
 bluetooth_framing_checks();
 usb_priority_checks();
 bluetooth_switch_checks();
 /* Shared bus failure revokes authorization and never restores motion. */
 bus_off=true;run_ms(1100);
 assert(!g_debug.setup.dual_ready && !peer[0].state && !peer[1].state);
 printf("PASS dual: isolation, ID verification, two concurrent prepare/enable/run, independent disable, all-stop, bus-off; collisions=%lu cancels=%lu\n",(unsigned long)collisions,(unsigned long)cancels);
 return 0;
}
