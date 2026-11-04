#include "robstride05.h"
#include "app_config.h"
#include <string.h>

static uint32_t le32(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void put32(uint8_t *p,uint32_t v)
{p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);}
static float scaled(const uint8_t *p,float limit)
{uint16_t u=(uint16_t)(((uint16_t)p[0]<<8)|p[1]);return (float)u*(2.0f*limit/65535.0f)-limit;}
float Rs05_RawFloat(uint32_t raw) {float f;memcpy(&f,&raw,4);return f;}
uint32_t Rs05_FloatRaw(float f) {uint32_t raw;memcpy(&raw,&f,4);return raw;}
CanFrame Rs05_Command(uint8_t motor_id,uint8_t type,uint8_t value)
{CanFrame f={0};f.id=((uint32_t)type<<24)|(APP_HOST_ID<<8)|motor_id;f.data[0]=value;return f;}
CanFrame Rs05_Read(uint8_t motor_id,uint16_t i)
{CanFrame f=Rs05_Command(motor_id,17,0);f.data[0]=(uint8_t)i;f.data[1]=(uint8_t)(i>>8);return f;}
CanFrame Rs05_WriteRaw(uint8_t motor_id,uint16_t i,uint32_t v)
{CanFrame f=Rs05_Read(motor_id,i);f.id=(f.id & 0xFFFFFFU)|(18UL<<24);put32(f.data+4,v);return f;}
CanFrame Rs05_WriteFloat(uint8_t motor_id,uint16_t i,float v) {return Rs05_WriteRaw(motor_id,i,Rs05_FloatRaw(v));}
CanFrame Rs05_Reporting(uint8_t motor_id,bool on)
{CanFrame f=Rs05_Command(motor_id,24,1);uint8_t i;for(i=0;i<6;i++)f.data[i]=(uint8_t)(i+1);f.data[6]=on?1:0;return f;}
CanFrame Rs05_VersionRequest(uint8_t motor_id)
{CanFrame f=Rs05_Command(motor_id,4,0);f.data[1]=0xC4;return f;}
CanFrame Rs05_IdentityRequest(uint8_t motor_id) {return Rs05_Command(motor_id,0,0);}
bool Rs05_Parse(uint8_t motor_id,const CanFrame *f,Rs05Feedback *o)
{
 uint8_t type=(uint8_t)((f->id>>24)&0x1F);
 /* Type-0 replies use destination FE, unlike feedback/parameter replies. */
 if(type==0 && (f->id&0xFFU)==0xFEU && ((f->id>>8)&0xFFU)==motor_id) {
  o->identity_id=(uint8_t)(f->id>>8);memcpy(o->uid,f->data,8);o->identity_ms=f->tick_ms;o->identity_count++;return true;
 }
 if((f->id&0xFFU)!=APP_HOST_ID || ((f->id>>8)&0xFFU)!=motor_id)return false;
 if(type==2 || type==24) {
  /* Version replies reuse type 2: never interpret the signature as position. */
  if(type==2 && f->data[0]==0 && f->data[1]==0xC4 && f->data[2]==0x56) {
   o->version=((uint32_t)f->data[3]<<24)|((uint32_t)f->data[4]<<16)|((uint32_t)f->data[5]<<8)|f->data[6];
   o->version_valid=1;return true;
  }
  o->position_rad=scaled(f->data,12.57f);o->velocity_rad_s=scaled(f->data+2,50.0f);
  o->torque_nm=scaled(f->data+4,5.5f);
  o->temperature_c=(float)(((uint16_t)f->data[6]<<8)|f->data[7])*0.1f;
  o->fault_bits=(uint8_t)((f->id>>16)&0x3FU);o->motor_state=(uint8_t)((f->id>>22)&3U);
  o->feedback_ms=f->tick_ms;o->feedback_count++;return true;
 }
 if(type==17) {
  o->param_index=(uint16_t)((uint16_t)f->data[0]|((uint16_t)f->data[1]<<8));
  o->param_raw=le32(f->data+4);o->param_ok=((f->id>>16)&0xFFU)==0;
  o->param_ms=f->tick_ms;o->parameter_count++;return true;
 }
 if(type==21) {o->fault=le32(f->data);o->warning=le32(f->data+4);o->fault_ms=f->tick_ms;return true;}
 return false;
}
