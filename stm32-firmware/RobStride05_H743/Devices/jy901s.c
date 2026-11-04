#include "jy901s.h"
#include <string.h>
static int16_t word(const uint8_t *p){return (int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8));}
void Jy901s_Init(Jy901s *imu){memset(imu,0,sizeof(*imu));}
void Jy901s_Zero(Jy901s *imu){imu->data.yaw_deg=0;}
void Jy901s_Input(Jy901s *imu,uint8_t byte,uint32_t now)
{
 uint8_t sum=0,i;float yaw,delta;
 if(!imu->used && byte!=0x55)return;
 imu->frame[imu->used++]=byte;
 if(imu->used<11)return;
 for(i=0;i<10;i++)sum=(uint8_t)(sum+imu->frame[i]);
 if(sum!=imu->frame[10]) {
  imu->data.checksum_errors++;
  for(i=1;i<11 && imu->frame[i]!=0x55;i++){}
  imu->used=(uint8_t)(11-i);
  memmove(imu->frame,imu->frame+i,imu->used);return;
 }
 imu->used=0;
 if(imu->frame[1]==0x52) {
  imu->data.gyro_z_deg_s=word(imu->frame+6)*(2000.0f/32768.0f);
  imu->data.gyro_frames++;
 }else if(imu->frame[1]==0x53) {
  imu->data.roll_deg=word(imu->frame+2)*(180.0f/32768.0f);
  imu->data.pitch_deg=word(imu->frame+4)*(180.0f/32768.0f);
  yaw=word(imu->frame+6)*(180.0f/32768.0f);
  if(imu->data.ready) {
   delta=yaw-imu->previous_yaw;
   if(delta>180)delta-=360;
   if(delta< -180)delta+=360;
   imu->data.yaw_deg+=delta;
  }
  imu->previous_yaw=yaw;imu->data.raw_yaw_deg=yaw;
  imu->data.ready=1;imu->data.angle_tick_ms=now;imu->data.angle_frames++;
 }
}
