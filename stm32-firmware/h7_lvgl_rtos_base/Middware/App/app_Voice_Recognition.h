#ifndef APP_VOICE_RECOGNITION_H
#define APP_VOICE_RECOGNITION_H

#include "app_chassis_board.h"

typedef struct
{
    uint32_t update_count;
    uint32_t handled_count;
    uint32_t zero_count;
    uint32_t unknown_count;
    uint32_t feedback_count;
    uint32_t last_update_tick;
    uint32_t last_handled_tick;
    uint8_t  last_voice_id;
    uint8_t  last_handled_id;
    uint8_t  last_feedback_id;
    uint8_t  last_handled_valid;
    float    voice_speed;
    float    last_vx_set;
    float    last_vy_set;
    float    last_wz_set;
    CarMode_t last_mode;
} Voice_Debug_t;

extern volatile Voice_Debug_t voice_debug;

void App_Voice_Recognition_Update(chassis_move_t *chassis);
uint8_t App_Voice_IsActive(void);
void App_Voice_Clear(void);
void App_Voice_ApplyControl(chassis_move_t *chassis);

#endif
