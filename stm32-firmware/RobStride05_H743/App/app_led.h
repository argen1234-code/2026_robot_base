#ifndef APP_LED_H
#define APP_LED_H
#include <stdint.h>
enum { LED_AUTO=0, LED_RAINBOW=1, LED_COMETS=2, LED_AURORA=3,
       LED_FIRE=4, LED_PLASMA=5, LED_SCANNER=6, LED_SOLID=7 };
#define LED_PLAYLIST_MAX 8U
/* Host commits odd sequence -> payload -> fresh even sequence LAST.
 * Only the LED task consumes this mailbox; no motor/CAN state is changed. */
typedef struct {
 uint32_t seq,enabled,mode,brightness,speed_percent,color_order;
 uint32_t red,green,blue,cycle_count,cycle_interval_ms;
 uint32_t cycle_modes[LED_PLAYLIST_MAX];
} AppLedCommand;
typedef struct {
 /* Editable live controls; no motor command/enable is involved. */
 uint32_t enabled;       /* 0 sends black once; 1 animates. */
 uint32_t mode;          /* LED_* above; 0 plays the custom playlist. */
 uint32_t brightness;    /* 0..255; default 64. */
 uint32_t speed_percent; /* 1..400; default 100. */
 uint32_t color_order;   /* 0 GRB (normal WS2812); 1 RGB. */
 uint32_t red,green,blue; /* 0..255 each, used by solid/scanner. */
 /* Observe only. DMA success does not prove a strip is connected/lighting. */
 uint32_t ready,busy,active_mode,led_count;
 uint32_t service_count,frames_started,frames_done,errors,last_error;
 uint32_t last_frame_ms,last_render_us,max_render_us;
 uint32_t executed_seq,result; /* 0 accepted; 1 invalid payload. */
 uint32_t cycle_index,cycle_count,cycle_interval_ms;
 uint32_t cycle_modes[LED_PLAYLIST_MAX];
} AppLedDebug;
extern volatile AppLedDebug g_led;
extern volatile AppLedCommand g_led_cmd;
void AppLed_Init(void);
void AppLed_Service(uint32_t now_ms);
#endif
