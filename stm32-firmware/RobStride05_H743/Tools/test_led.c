/* Exercise the real LED mailbox/render scheduler with a bounded DMA peer. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_led.c"
static LedPixel sent[APP_LED_COUNT];
static uint32_t sends,poll_state;
bool BspWs2812_Init(void){return true;}
uint32_t BspWs2812_Poll(uint32_t now){(void)now;return poll_state;}
uint32_t BspWs2812_LastError(void){return poll_state==LED_TX_ERROR?LED_ERR_DMA:0;}
uint32_t BspTime_Us(void){return 1;}
bool BspWs2812_Send(const LedPixel *p,uint8_t b,uint32_t order,uint32_t now){
 (void)b;(void)order;(void)now;memcpy(sent,p,sizeof(sent));sends++;return true;
}
static void commit(uint32_t seq,uint32_t mode,uint32_t speed){
 AppLedCommand c={0};c.seq=seq;c.enabled=1;c.mode=mode;c.brightness=64;
 c.speed_percent=speed;c.red=23;c.green=45;c.blue=67;
 c.cycle_count=3;c.cycle_interval_ms=1000;
 c.cycle_modes[0]=LED_SOLID;c.cycle_modes[1]=LED_SCANNER;c.cycle_modes[2]=LED_SOLID;
 g_led_cmd=c;
}
int main(void){
 uint32_t i,before;
 AppLed_Init();AppLed_Service(0);
 commit(1,LED_SOLID,100);AppLed_Service(2);
 assert(g_led.executed_seq==0); /* odd/uncommitted command ignored */
 g_led_cmd.seq=2;AppLed_Service(4);
 assert(g_led.executed_seq==2 && g_led.result==0 && g_led.led_count==16);
 for(i=0;i<APP_LED_COUNT;i++)assert(sent[i].r==23 && sent[i].g==45 && sent[i].b==67);
 commit(4,LED_AUTO,400);AppLed_Service(10);
 assert(g_led.active_mode==LED_SOLID && g_led.cycle_index==0);
 AppLed_Service(990);assert(g_led.active_mode==LED_SOLID);
 AppLed_Service(1010);assert(g_led.active_mode==LED_SCANNER && g_led.cycle_index==1);
 AppLed_Service(2010);assert(g_led.active_mode==LED_SOLID && g_led.cycle_index==2);
 AppLed_Service(3010);assert(g_led.cycle_index==0); /* wrap; duplicates preserved */
 commit(6,LED_AUTO,1);g_led_cmd.cycle_count=9;AppLed_Service(3040);
 assert(g_led.executed_seq==6 && g_led.result==1 && g_led.cycle_count==3 && g_led.speed_percent==400);
 commit(8,LED_SOLID,100);g_led_cmd.enabled=0;AppLed_Service(3060);
 for(i=0;i<APP_LED_COUNT;i++)assert(sent[i].r==0 && sent[i].g==0 && sent[i].b==0);
 before=sends;AppLed_Service(4000);assert(sends==before); /* black once */
 commit(10,LED_SOLID,100);AppLed_Service(4020);assert(sent[15].b==67);
 commit(12,LED_RAINBOW,100);poll_state=LED_TX_BUSY;before=sends;AppLed_Service(4040);
 assert(sends==before);poll_state=LED_TX_IDLE;AppLed_Service(4042);assert(sends==before+1);
 commit(14,LED_AUTO,100);AppLed_Service(0xFFFFFFF0U);AppLed_Service(984);
 assert(g_led.cycle_index==1); /* interval survives 32-bit tick rollover */
 poll_state=LED_TX_ERROR;AppLed_Service(1004);assert(!g_led.ready && g_led.errors==1);
 puts("PASS LED: atomic mailbox, all 16 pixels, playlist order/duplicates/wrap, real-time interval, off/resume, DMA busy/error isolation");
 return 0;
}
