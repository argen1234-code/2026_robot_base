#include "app_telemetry.h"
#include "app_chassis_board.h"
#include "bsp_uart.h"
#include "main.h"
#include <string.h>

#define TELEMETRY_RATE_MS   50U     /* 20 Hz */
#define TELEMETRY_CHANNELS  32U

static uint8_t s_frame[TELEMETRY_CHANNELS * 4U + 4U];
static uint32_t s_last_tick = 0U;
static uint32_t s_drop_count = 0U;

static void telemetry_put(uint8_t index, float value)
{
    memcpy(&s_frame[index * 4U], &value, 4U);
}

void Telemetry_Init(void)
{
    memset(s_frame, 0, sizeof(s_frame));
    s_last_tick = 0U;
    s_drop_count = 0U;
}

void Telemetry_Update(chassis_move_t *chassis)
{
    uint32_t now;
    uint32_t state_nibbles = 0U;
    uint8_t i;

    if (chassis == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - s_last_tick) < TELEMETRY_RATE_MS)
    {
        return;
    }
    s_last_tick = now;

    telemetry_put(0U, (float)chassis->loop_count);
    telemetry_put(1U, (float)chassis->mode);
    telemetry_put(2U, (float)chassis->fault_flags);
    telemetry_put(3U, (float)chassis->bt_keys);

    telemetry_put(4U, chassis->Vx_set);
    telemetry_put(5U, chassis->Vy_set);
    telemetry_put(6U, chassis->Wz_set);
    telemetry_put(7U, chassis->Vx_cmd);
    telemetry_put(8U, chassis->Vy_cmd);
    telemetry_put(9U, chassis->Wz_cmd);

    for (i = 0U; i < 4U; i++)
    {
        telemetry_put((uint8_t)(10U + i), (float)chassis->motor[i].speed_set);
        telemetry_put((uint8_t)(14U + i), (float)chassis->motor[i].speed);
        telemetry_put((uint8_t)(18U + i), (float)chassis->motor[i].torque_set);
        telemetry_put((uint8_t)(22U + i), chassis->motor[i].t_mos_c);
        telemetry_put((uint8_t)(26U + i), chassis->motor[i].t_rotor_c);

        state_nibbles |= ((uint32_t)(chassis->motor[i].state & 0x0FU) << (i * 4U));
    }

    telemetry_put(30U, (float)state_nibbles);
    telemetry_put(31U, (float)chassis->loop_dt_ms);

    s_frame[TELEMETRY_CHANNELS * 4U + 0U] = 0x00U;
    s_frame[TELEMETRY_CHANNELS * 4U + 1U] = 0x00U;
    s_frame[TELEMETRY_CHANNELS * 4U + 2U] = 0x80U;
    s_frame[TELEMETRY_CHANNELS * 4U + 3U] = 0x7FU;

    if (BSP_UART_TelemetrySend(s_frame, (uint16_t)sizeof(s_frame)) == 0U)
    {
        s_drop_count++;
    }
}

uint32_t Telemetry_GetDropCount(void)
{
    return s_drop_count;
}
