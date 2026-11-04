#include "app_led.h"
#include "app_config.h"
#include "bsp_ws2812.h"
#include "bsp_time.h"
#include "main.h"

#if APP_LED_FRAME_MS < 10 || APP_LED_FRAME_MS > 1000
#error APP_LED_FRAME_MS_must_be_10_to_1000
#endif
#if APP_LED_AUTO_PERIOD_MS < 1000
#error APP_LED_AUTO_PERIOD_MS_must_be_at_least_1000
#endif

volatile AppLedDebug g_led={
 .enabled=APP_LED_DEFAULT_ENABLED,.mode=APP_LED_DEFAULT_MODE,
 .brightness=APP_LED_DEFAULT_BRIGHTNESS,.speed_percent=100U,
 .red=32U,.green=180U,.blue=255U,.led_count=APP_LED_COUNT,
 .cycle_count=6U,.cycle_interval_ms=APP_LED_AUTO_PERIOD_MS,
 .cycle_modes={LED_RAINBOW,LED_COMETS,LED_AURORA,LED_FIRE,LED_PLASMA,LED_SCANNER}
};
volatile AppLedCommand g_led_cmd;
static LedPixel pixels[APP_LED_COUNT];
static uint8_t heat[APP_LED_COUNT];
static uint32_t phase_ms,phase_fraction,last_frame,next_frame,fire_step;
static uint32_t random_state=0xA7432812UL;
static bool primed,off_sent,was_busy;
static uint32_t cycle_started_ms;

static void accept_command(uint32_t now_ms)
{
 AppLedCommand command;uint32_t seq,i;bool valid;
 seq=g_led_cmd.seq;
 if(!seq || (seq&1U) || seq==g_led.executed_seq)return;
 __DMB();command=g_led_cmd;__DMB();
 if(g_led_cmd.seq!=seq)return;
 valid=command.enabled<=1U && command.mode<=LED_SOLID &&
       command.brightness<=255U && command.speed_percent>=1U &&
       command.speed_percent<=400U && command.color_order<=1U &&
       command.red<=255U && command.green<=255U && command.blue<=255U &&
       command.cycle_count>=1U && command.cycle_count<=LED_PLAYLIST_MAX &&
       command.cycle_interval_ms>=1000U && command.cycle_interval_ms<=600000U;
 if(valid)for(i=0U;i<command.cycle_count;i++)
  if(command.cycle_modes[i]<LED_RAINBOW || command.cycle_modes[i]>LED_SOLID)valid=false;
 if(valid) {
  g_led.enabled=command.enabled;g_led.mode=command.mode;
  g_led.brightness=command.brightness;g_led.speed_percent=command.speed_percent;
  g_led.color_order=command.color_order;
  g_led.red=command.red;g_led.green=command.green;g_led.blue=command.blue;
  g_led.cycle_count=command.cycle_count;g_led.cycle_interval_ms=command.cycle_interval_ms;
  for(i=0U;i<LED_PLAYLIST_MAX;i++)g_led.cycle_modes[i]=command.cycle_modes[i];
  g_led.cycle_index=0U;cycle_started_ms=now_ms;
  /* Apply a new selection on the next frame; never modify an in-flight DMA. */
  phase_ms=0U;phase_fraction=0U;off_sent=false;next_frame=now_ms;
 }
 g_led.result=valid?0U:1U;__DMB();g_led.executed_seq=seq;
}

static uint8_t clamp8(uint32_t v) {return (uint8_t)(v>255U?255U:v);}
static uint32_t random32(void)
{
 random_state^=random_state<<13;random_state^=random_state>>17;
 random_state^=random_state<<5;return random_state;
}
/* Integer smooth wave: no floating point, heap, LUT allocation or trig call. */
static uint8_t wave(uint32_t phase)
{
 uint32_t p=phase&255U,t=p<128U?p*2U:(255U-p)*2U;
 return (uint8_t)((t*t*(765U-2U*t)+32512U)/65025U);
}
static LedPixel hsv(uint32_t hue,uint32_t saturation,uint32_t value)
{
 LedPixel c;uint32_t h=(hue&255U)*6U,sector=h>>8,f=h&255U;
 uint8_t v=clamp8(value),p=(uint8_t)(v*(255U-saturation)/255U);
 uint8_t q=(uint8_t)(v*(255U-saturation*f/255U)/255U);
 uint8_t t=(uint8_t)(v*(255U-saturation*(255U-f)/255U)/255U);
 switch(sector) {
 case 0:c.r=v;c.g=t;c.b=p;break;
 case 1:c.r=q;c.g=v;c.b=p;break;
 case 2:c.r=p;c.g=v;c.b=t;break;
 case 3:c.r=p;c.g=q;c.b=v;break;
 case 4:c.r=t;c.g=p;c.b=v;break;
 default:c.r=v;c.g=p;c.b=q;break;
 }
 return c;
}
static void add(LedPixel *p,LedPixel c)
{
 p->r=clamp8((uint32_t)p->r+c.r);
 p->g=clamp8((uint32_t)p->g+c.g);
 p->b=clamp8((uint32_t)p->b+c.b);
}
static void render(uint32_t mode,LedPixel color)
{
 uint32_t i,t=phase_ms,head,span,d,v,k,p;LedPixel c;
 for(i=0U;i<APP_LED_COUNT;i++) {
  pixels[i].r=0U;pixels[i].g=0U;pixels[i].b=0U;
 }
 switch(mode) {
 case LED_RAINBOW: /* Flowing hue helix plus sparse white glitter. */
  for(i=0U;i<APP_LED_COUNT;i++) {
   pixels[i]=hsv(t/12U+i*256U/APP_LED_COUNT,230U,
                  80U+wave(t/8U+i*27U)*175U/255U);
   if((random32()&127U)==0U) {
    c.r=150U;c.g=190U;c.b=220U;add(&pixels[i],c);
   }
  }
  break;
 case LED_COMETS: /* Two colored comets moving in opposite directions. */
  span=APP_LED_COUNT*256U;head=(t*3U)%span;
  for(i=0U;i<APP_LED_COUNT;i++) {
   d=(head+span-i*256U)%span;
   v=d<1024U?255U-d*255U/1024U:0U;
   pixels[i]=hsv(t/24U,255U,v);
   d=(i*256U+head)%span;
   v=d<768U?255U-d*255U/768U:0U;
   add(&pixels[i],hsv(t/24U+128U,240U,v));
  }
  break;
 case LED_AURORA: /* Cyan, violet and green waves with moving highlights. */
  for(i=0U;i<APP_LED_COUNT;i++) {
   k=wave(t/18U+i*23U);
   pixels[i]=hsv(95U+k*95U/255U,200U,
                  45U+wave(t/11U+i*37U)*210U/255U);
  }
  break;
 case LED_FIRE: /* Cooling/diffusion/sparks, from the DIN end outward. */
  if(fire_step!=t/35U) {
   fire_step=t/35U;
   for(i=0U;i<APP_LED_COUNT;i++) {
    k=random32()%35U;heat[i]=(uint8_t)(heat[i]>k?heat[i]-k:0U);
   }
   for(i=APP_LED_COUNT;i-->2U;)
    heat[i]=(uint8_t)(((uint32_t)heat[i-1U]+heat[i-2U]*2U)/3U);
   if((random32()&255U)<155U) {
    i=random32()%(APP_LED_COUNT<3U?APP_LED_COUNT:3U);
    heat[i]=clamp8((uint32_t)heat[i]+140U+random32()%116U);
   }
  }
  for(i=0U;i<APP_LED_COUNT;i++) {
   k=heat[i]*191U/255U;p=(k&63U)*4U;
   if(k>128U) {pixels[i].r=255U;pixels[i].g=255U;pixels[i].b=(uint8_t)p;}
   else if(k>64U) {pixels[i].r=255U;pixels[i].g=(uint8_t)p;}
   else pixels[i].r=(uint8_t)p;
  }
  break;
 case LED_PLASMA: /* Three interfering smooth waves. */
  for(i=0U;i<APP_LED_COUNT;i++) {
   k=((uint32_t)wave(t/9U+i*31U)+wave(t/15U-i*19U)+
       wave(t/21U+i*47U))/3U;
   pixels[i]=hsv(t/32U+k,240U,65U+k*190U/255U);
  }
  break;
 case LED_SCANNER: /* Soft bouncing beam in the editable RGB color. */
  span=(APP_LED_COUNT>1U?APP_LED_COUNT-1U:1U)*256U;
  p=(t*2U)%(span*2U);head=p<span?p:span*2U-p;
  if(APP_LED_COUNT==1U)head=0U;
  for(i=0U;i<APP_LED_COUNT;i++) {
   p=i*256U;d=p>head?p-head:head-p;
   v=d<640U?(640U-d)*255U/640U:0U;
   pixels[i].r=(uint8_t)(color.r*v/255U);
   pixels[i].g=(uint8_t)(color.g*v/255U);
   pixels[i].b=(uint8_t)(color.b*v/255U);
  }
  break;
 case LED_SOLID:
  for(i=0U;i<APP_LED_COUNT;i++)pixels[i]=color;
  break;
 default:break;
 }
}
void AppLed_Init(void)
{
 primed=false;off_sent=false;was_busy=false;phase_ms=0U;phase_fraction=0U;
 cycle_started_ms=0U;
 fire_step=0U;g_led.ready=BspWs2812_Init()?1U:0U;
 g_led.last_error=BspWs2812_LastError();
 if(!g_led.ready)g_led.errors++;
}
void AppLed_Service(uint32_t now_ms)
{
 uint32_t state,mode,speed,elapsed,begin,cost,i;uint8_t brightness;
 bool enabled;LedPixel color;
 g_led.service_count++;
 accept_command(now_ms);
 if(!g_led.ready)return;
 state=BspWs2812_Poll(now_ms);
 if(state==LED_TX_ERROR) {
  g_led.ready=0U;g_led.busy=0U;g_led.errors++;
  g_led.last_error=BspWs2812_LastError();return; /* Motor/OLED keep running. */
 }
 g_led.busy=state==LED_TX_BUSY?1U:0U;
 if(was_busy && state==LED_TX_IDLE) {g_led.frames_done++;was_busy=false;}
 if(!primed) {
  primed=true;last_frame=now_ms;next_frame=now_ms+2U;return;
 }
 if(state==LED_TX_BUSY || (int32_t)(now_ms-next_frame)<0)return;
 enabled=g_led.enabled!=0U;
 if(!enabled && off_sent)return;
 begin=BspTime_Us();
 mode=g_led.mode;brightness=clamp8(g_led.brightness);
 speed=g_led.speed_percent;if(speed<1U)speed=1U;if(speed>400U)speed=400U;
 color.r=clamp8(g_led.red);color.g=clamp8(g_led.green);color.b=clamp8(g_led.blue);
 elapsed=now_ms-last_frame;if(elapsed>200U)elapsed=200U;
 last_frame=now_ms;next_frame=now_ms+APP_LED_FRAME_MS;
 phase_fraction+=elapsed*speed;phase_ms+=phase_fraction/100U;phase_fraction%=100U;
 if(mode>LED_SOLID)mode=LED_AUTO;
 if(mode==LED_AUTO) {
  uint32_t count=g_led.cycle_count,interval=g_led.cycle_interval_ms;
  if(count<1U || count>LED_PLAYLIST_MAX)count=6U;
  if(interval<1000U || interval>600000U)interval=APP_LED_AUTO_PERIOD_MS;
  if(enabled) {
   uint32_t steps=(uint32_t)(now_ms-cycle_started_ms)/interval;
   g_led.cycle_index=(g_led.cycle_index+steps)%count;
   cycle_started_ms+=steps*interval;
  } else cycle_started_ms=now_ms;
  mode=g_led.cycle_modes[g_led.cycle_index%count];
  if(mode<LED_RAINBOW || mode>LED_SOLID)mode=LED_RAINBOW;
 }
 g_led.active_mode=mode;
 if(enabled) {render(mode,color);off_sent=false;}
 else {
  for(i=0U;i<APP_LED_COUNT;i++) {pixels[i].r=0U;pixels[i].g=0U;pixels[i].b=0U;}
 }
 if(BspWs2812_Send(pixels,brightness,g_led.color_order,now_ms)) {
  was_busy=true;g_led.busy=1U;g_led.frames_started++;
  g_led.last_frame_ms=now_ms;if(!enabled)off_sent=true;
 }
 cost=BspTime_Us()-begin;g_led.last_render_us=cost;
 if(cost>g_led.max_render_us)g_led.max_render_us=cost;
}
