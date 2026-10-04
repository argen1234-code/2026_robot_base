#include "app_Voice_Recognition.h"
#include "bsp_WonderEcho.h"

#define VOICE_SPEED_DEFAULT     20.0f
#define VOICE_SPEED_MIN         10.0f
#define VOICE_SPEED_MAX         50.0f
#define VOICE_SPEED_STEP        10.0f
#define VOICE_TURN_SPEED        20.0f
#define VOICE_FEEDBACK_CMD_TYPE 0x00U

static float voice_speed = VOICE_SPEED_DEFAULT;
static uint8_t last_feedback_id = 0U;
static uint8_t voice_active = 0U;
static float voice_vx_set = 0.0f;
static float voice_vy_set = 0.0f;
static float voice_wz_set = 0.0f;

volatile Voice_Debug_t voice_debug = {0};

static uint8_t App_Voice_IsMovingForwardOrBackward(chassis_move_t *chassis)
{
    (void)chassis;

    if (!voice_active)
    {
        return 0U;
    }

    if ((voice_vy_set != 0.0f) || (voice_wz_set != 0.0f))
    {
        return 0U;
    }

    if ((voice_vx_set > 0.0f) || (voice_vx_set < 0.0f))
    {
        return 1U;
    }

    return 0U;
}

static void App_Voice_LimitSpeed(void)
{
    if (voice_speed > VOICE_SPEED_MAX)
    {
        voice_speed = VOICE_SPEED_MAX;
    }
    else if (voice_speed < VOICE_SPEED_MIN)
    {
        voice_speed = VOICE_SPEED_MIN;
    }
}

static void App_Voice_SpeakFeedback(uint8_t voice_id)
{
    if (voice_id == 0U)
    {
        last_feedback_id = 0U;
        voice_debug.last_feedback_id = 0U;
        return;
    }

    if (voice_id != last_feedback_id)
    {
        WonderEcho_Speak(VOICE_FEEDBACK_CMD_TYPE, voice_id);
        last_feedback_id = voice_id;
        voice_debug.last_feedback_id = voice_id;
        voice_debug.feedback_count++;
    }
}

static void App_Voice_SaveDebug(chassis_move_t *chassis, uint8_t voice_id, uint8_t handled)
{
    voice_debug.last_voice_id = voice_id;
    voice_debug.voice_speed = voice_speed;
    voice_debug.last_handled_valid = handled;

    if (chassis != NULL)
    {
        voice_debug.last_mode = chassis->mode;
        voice_debug.last_vx_set = voice_vx_set;
        voice_debug.last_vy_set = voice_vy_set;
        voice_debug.last_wz_set = voice_wz_set;
    }

    if (handled)
    {
        voice_debug.handled_count++;
        voice_debug.last_handled_id = voice_id;
        voice_debug.last_handled_tick = HAL_GetTick();
    }
}

uint8_t App_Voice_IsActive(void)
{
    return voice_active;
}

void App_Voice_Clear(void)
{
    voice_active = 0U;
    voice_vx_set = 0.0f;
    voice_vy_set = 0.0f;
    voice_wz_set = 0.0f;
}

void App_Voice_ApplyControl(chassis_move_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    chassis->Vx_set = voice_vx_set;
    chassis->Vy_set = voice_vy_set;
    chassis->Wz_set = voice_wz_set;
}

void App_Voice_Recognition_Update(chassis_move_t *chassis)
{
    uint8_t voice_id;
    uint8_t update_line_speed = 0U;
    uint8_t handled = 0U;

    if (chassis == NULL)
    {
        return;
    }

    voice_debug.update_count++;
    voice_debug.last_update_tick = HAL_GetTick();

    voice_id = WonderEcho_GetResult();
    if (voice_id == 0U)
    {
        voice_debug.zero_count++;
        App_Voice_SpeakFeedback(0U);
    }

    switch (voice_id)
    {
        case WONDERECHO_CMD_FORWARD:
            voice_active = 1U;
            voice_vx_set = voice_speed;
            voice_vy_set = 0.0f;
            voice_wz_set = 0.0f;
            App_Voice_SpeakFeedback(voice_id);
            handled = 1U;
            break;

        case WONDERECHO_CMD_BACKWARD:
            voice_active = 1U;
            voice_vx_set = -voice_speed;
            voice_vy_set = 0.0f;
            voice_wz_set = 0.0f;
            App_Voice_SpeakFeedback(voice_id);
            handled = 1U;
            break;

        case WONDERECHO_CMD_TURN_LEFT:
            voice_active = 1U;
            voice_vx_set = 0.0f;
            voice_vy_set = 0.0f;
            voice_wz_set = -VOICE_TURN_SPEED;
            App_Voice_SpeakFeedback(voice_id);
            handled = 1U;
            break;

        case WONDERECHO_CMD_TURN_RIGHT:
            voice_active = 1U;
            voice_vx_set = 0.0f;
            voice_vy_set = 0.0f;
            voice_wz_set = VOICE_TURN_SPEED;
            App_Voice_SpeakFeedback(voice_id);
            handled = 1U;
            break;

        case WONDERECHO_CMD_STOP:
            voice_active = 1U;
            voice_vx_set = 0.0f;
            voice_vy_set = 0.0f;
            voice_wz_set = 0.0f;
            App_Voice_SpeakFeedback(voice_id);
            handled = 1U;
            break;

        case WONDERECHO_CMD_SPEED_UP:
            update_line_speed = App_Voice_IsMovingForwardOrBackward(chassis);
            voice_speed += VOICE_SPEED_STEP;
            App_Voice_LimitSpeed();
            if (update_line_speed)
            {
                voice_vx_set = (voice_vx_set > 0.0f) ? voice_speed : -voice_speed;
            }
            App_Voice_SpeakFeedback(voice_id);
            handled = 1U;
            break;

        case WONDERECHO_CMD_SPEED_DOWN:
            update_line_speed = App_Voice_IsMovingForwardOrBackward(chassis);
            voice_speed -= VOICE_SPEED_STEP;
            App_Voice_LimitSpeed();
            if (update_line_speed)
            {
                voice_vx_set = (voice_vx_set > 0.0f) ? voice_speed : -voice_speed;
            }
            App_Voice_SpeakFeedback(voice_id);
            handled = 1U;
            break;

        default:
            if (voice_id != 0U)
            {
                voice_debug.unknown_count++;
            }
            break;
    }

    App_Voice_SaveDebug(chassis, voice_id, handled);
}
