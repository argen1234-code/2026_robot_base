#include "bsp_dm6215.h"
#include "bsp_fdcan.h"
#include "main.h"
#include <string.h>

#define DM_PARAM_RESPONSE_TIMEOUT_MS   200U

static uint8_t s_motor_count = 0U;
static uint8_t s_esc_ids[DM_MAX_MOTORS] = {0};
static uint8_t s_mst_ids[DM_MAX_MOTORS] = {0};

static DM_Feedback_t s_feedback[DM_MAX_MOTORS];
static volatile uint32_t s_feedback_seq[DM_MAX_MOTORS];

static volatile uint8_t s_param_expect_valid = 0U;
static volatile uint32_t s_param_expect_tick = 0U;
static volatile uint8_t s_param_response_valid = 0U;
static DM_ParamResponse_t s_param_response;

static DM_Stats_t s_stats;

/* ------------------------------------------------------------------------- */
/* 数值换算                                                                   */
/* ------------------------------------------------------------------------- */

static uint16_t float_to_uint(float value, float min, float max, uint8_t bits)
{
    float span;
    float max_int;

    if (value < min)
    {
        value = min;
    }
    if (value > max)
    {
        value = max;
    }

    span = max - min;
    max_int = (float)((1UL << bits) - 1UL);

    /* +0.5 做四舍五入，避免 0 被截断到边界值 */
    return (uint16_t)((value - min) * max_int / span + 0.5f);
}

static float uint_to_float(uint16_t value, float min, float max, uint8_t bits)
{
    float span = max - min;
    float max_int = (float)((1UL << bits) - 1UL);

    return ((float)value) * span / max_int + min;
}

/* ------------------------------------------------------------------------- */
/* 发送                                                                       */
/* ------------------------------------------------------------------------- */

uint8_t DM_SendMit(uint16_t esc_id, float pos_rad, float vel_rad_s,
                   float kp, float kd, float torque_nm)
{
    uint8_t data[8];
    uint16_t p;
    uint16_t v;
    uint16_t kp_i;
    uint16_t kd_i;
    uint16_t t;

    p    = float_to_uint(pos_rad,    DM_P_MIN_RAD, DM_P_MAX_RAD, 16U);
    v    = float_to_uint(vel_rad_s,  DM_V_MIN_RAD_S, DM_V_MAX_RAD_S, 12U);
    kp_i = float_to_uint(kp,         DM_KP_MIN, DM_KP_MAX, 12U);
    kd_i = float_to_uint(kd,         DM_KD_MIN, DM_KD_MAX, 12U);
    t    = float_to_uint(torque_nm,  DM_T_MIN_NM, DM_T_MAX_NM, 12U);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)(p);
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0FU) << 4) | (kp_i >> 8));
    data[4] = (uint8_t)(kp_i);
    data[5] = (uint8_t)(kd_i >> 4);
    data[6] = (uint8_t)(((kd_i & 0x0FU) << 4) | (t >> 8));
    data[7] = (uint8_t)(t);

    return BSP_FDCAN_Send(esc_id, data);
}

uint8_t DM_SendTorque(uint8_t slot, float torque_nm)
{
    if (slot >= s_motor_count)
    {
        return 0U;
    }

    return DM_SendMit(s_esc_ids[slot], 0.0f, 0.0f, 0.0f, 0.0f, torque_nm);
}

static uint8_t dm_send_special(uint16_t esc_id, uint8_t command)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    data[7] = command;
    return BSP_FDCAN_Send(esc_id, data);
}

uint8_t DM_SendEnable(uint16_t esc_id)
{
    return dm_send_special(esc_id, 0xFCU);
}

uint8_t DM_SendDisable(uint16_t esc_id)
{
    return dm_send_special(esc_id, 0xFDU);
}

uint8_t DM_SendClearError(uint16_t esc_id)
{
    return dm_send_special(esc_id, 0xFBU);
}

uint8_t DM_SendSaveZero(uint16_t esc_id)
{
    return dm_send_special(esc_id, 0xFEU);
}

static uint8_t dm_send_param(uint16_t esc_id, uint8_t opcode, uint8_t rid,
                             uint32_t raw)
{
    uint8_t data[8];

    data[0] = (uint8_t)(esc_id & 0xFFU);
    data[1] = (uint8_t)((esc_id >> 8) & 0xFFU);
    data[2] = opcode;
    data[3] = rid;
    data[4] = (uint8_t)(raw & 0xFFU);
    data[5] = (uint8_t)((raw >> 8) & 0xFFU);
    data[6] = (uint8_t)((raw >> 16) & 0xFFU);
    data[7] = (uint8_t)((raw >> 24) & 0xFFU);

    return BSP_FDCAN_Send(DM_PARAM_CAN_ID, data);
}

static void dm_arm_param_expect(void)
{
    s_param_expect_tick = HAL_GetTick();
    s_param_expect_valid = 1U;
}

uint8_t DM_ReadParam(uint16_t esc_id, uint8_t rid)
{
    dm_arm_param_expect();
    return dm_send_param(esc_id, DM_OP_READ, rid, 0U);
}

uint8_t DM_WriteParam(uint16_t esc_id, uint8_t rid, uint32_t raw)
{
    dm_arm_param_expect();
    return dm_send_param(esc_id, DM_OP_WRITE, rid, raw);
}

uint8_t DM_SaveParams(uint16_t esc_id)
{
    dm_arm_param_expect();
    return dm_send_param(esc_id, DM_OP_SAVE, 1U, 0U);
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* ------------------------------------------------------------------------- */

void DM_Init(uint8_t motor_count, const uint8_t *esc_ids, const uint8_t *mst_ids)
{
    uint8_t i;

    if (motor_count > DM_MAX_MOTORS)
    {
        motor_count = DM_MAX_MOTORS;
    }

    s_motor_count = motor_count;

    memset(s_feedback, 0, sizeof(s_feedback));
    memset((void *)s_feedback_seq, 0, sizeof(s_feedback_seq));
    memset(&s_stats, 0, sizeof(s_stats));
    memset(&s_param_response, 0, sizeof(s_param_response));
    s_param_expect_valid = 0U;
    s_param_response_valid = 0U;

    for (i = 0U; i < DM_MAX_MOTORS; i++)
    {
        s_esc_ids[i] = (i < motor_count) ? esc_ids[i] : 0U;
        s_mst_ids[i] = (i < motor_count) ? mst_ids[i] : 0U;
    }
}

/* ------------------------------------------------------------------------- */
/* 接收                                                                       */
/* ------------------------------------------------------------------------- */

static uint8_t dm_find_slot_by_mst(uint16_t id)
{
    uint8_t i;

    for (i = 0U; i < s_motor_count; i++)
    {
        if (s_mst_ids[i] == (uint8_t)id)
        {
            return i;
        }
    }

    return 0xFFU;
}

static void dm_decode_feedback(uint8_t slot, const uint8_t data[8])
{
    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;
    uint8_t esc_id;
    uint8_t state;
    uint32_t tick;

    esc_id = (uint8_t)(data[0] & 0x0FU);
    state  = (uint8_t)(data[0] >> 4);

    p_int = (uint16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[2]);
    v_int = (uint16_t)(((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4));
    t_int = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | (uint16_t)data[5]);

    tick = HAL_GetTick();

    /* seqlock：写期间为奇数，任务侧发现序号变化会重读 */
    s_feedback_seq[slot]++;
    __DMB();

    s_feedback[slot].esc_id    = esc_id;
    s_feedback[slot].state     = state;
    s_feedback[slot].pos_rad   = uint_to_float(p_int, DM_P_MIN_RAD, DM_P_MAX_RAD, 16U);
    s_feedback[slot].vel_rad_s = uint_to_float(v_int, DM_V_MIN_RAD_S, DM_V_MAX_RAD_S, 12U);
    s_feedback[slot].torque_nm = uint_to_float(t_int, DM_T_MIN_NM, DM_T_MAX_NM, 12U);
    s_feedback[slot].t_mos_c   = data[6];
    s_feedback[slot].t_rotor_c = data[7];
    s_feedback[slot].last_rx_tick = tick;
    s_feedback[slot].rx_count++;

    if (esc_id != s_esc_ids[slot])
    {
        s_feedback[slot].id_mismatch_count++;
        s_stats.id_mismatch_count++;
    }

    __DMB();
    s_feedback_seq[slot]++;
}

void DM_HandleFrame(uint16_t id, const uint8_t data[8])
{
    uint8_t slot;

    /* 参数应答：D0-1 = ESC_ID（小端，D1 通常为 0），D2 = 操作码 */
    if ((s_param_expect_valid != 0U) &&
        ((HAL_GetTick() - s_param_expect_tick) <= DM_PARAM_RESPONSE_TIMEOUT_MS) &&
        (data[1] == 0U) &&
        ((data[2] == DM_OP_READ) ||
         (data[2] == DM_OP_WRITE) ||
         (data[2] == DM_OP_SAVE) ||
         (data[2] == 0x66U) ||
         (data[2] == 0x99U)))
    {
        s_param_response.esc_id = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
        s_param_response.opcode = data[2];
        s_param_response.rid    = data[3];
        s_param_response.raw    = ((uint32_t)data[4]) |
                                  ((uint32_t)data[5] << 8) |
                                  ((uint32_t)data[6] << 16) |
                                  ((uint32_t)data[7] << 24);
        s_param_response.tick   = HAL_GetTick();

        s_param_expect_valid = 0U;
        s_param_response_valid = 1U;
        return;
    }

    if (id == 0U)
    {
        /* 出厂默认 MST_ID=0：四台会撞在一起，只计数不映射 */
        s_stats.default_feedback_count++;
        return;
    }

    slot = dm_find_slot_by_mst(id);
    if (slot == 0xFFU)
    {
        s_stats.unknown_feedback_count++;
        return;
    }

    dm_decode_feedback(slot, data);
}

uint8_t DM_GetParamResponse(DM_ParamResponse_t *out)
{
    if ((out == NULL) || (s_param_response_valid == 0U))
    {
        return 0U;
    }

    *out = s_param_response;
    s_param_response_valid = 0U;
    return 1U;
}

void DM_ClearParamResponse(void)
{
    s_param_response_valid = 0U;
    s_param_expect_valid = 0U;
}

/* ------------------------------------------------------------------------- */
/* 任务侧读取                                                                 */
/* ------------------------------------------------------------------------- */

uint8_t DM_GetFeedback(uint8_t slot, DM_Feedback_t *out)
{
    uint8_t retry;

    if ((out == NULL) || (slot >= s_motor_count))
    {
        return 0U;
    }

    for (retry = 0U; retry < 4U; retry++)
    {
        uint32_t seq1 = s_feedback_seq[slot];

        if ((seq1 & 1U) != 0U)
        {
            continue;
        }

        __DMB();
        *out = s_feedback[slot];
        __DMB();

        if (seq1 == s_feedback_seq[slot])
        {
            return 1U;
        }
    }

    /* 连续冲突时做一次尽力拷贝，避免任务一直拿不到数据 */
    *out = s_feedback[slot];
    return 1U;
}

uint8_t DM_GetSlotByEscId(uint8_t esc_id)
{
    uint8_t i;

    for (i = 0U; i < s_motor_count; i++)
    {
        if (s_esc_ids[i] == esc_id)
        {
            return i;
        }
    }

    return 0xFFU;
}

uint8_t DM_GetMotorCount(void)
{
    return s_motor_count;
}

uint8_t DM_GetEscId(uint8_t slot)
{
    return (slot < s_motor_count) ? s_esc_ids[slot] : 0U;
}

uint8_t DM_GetMstId(uint8_t slot)
{
    return (slot < s_motor_count) ? s_mst_ids[slot] : 0U;
}

void DM_GetStats(DM_Stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = s_stats;
}
