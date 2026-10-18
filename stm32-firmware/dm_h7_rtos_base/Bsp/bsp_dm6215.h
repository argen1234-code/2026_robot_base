#ifndef BSP_DM6215_H
#define BSP_DM6215_H

#include <stdint.h>

#define DM_MAX_MOTORS       4U

/* DM-H6215 默认映射范围；若电机 RID21/22/23 被改过，必须同步修改 */
#define DM_P_MAX_RAD        12.5f
#define DM_P_MIN_RAD        (-12.5f)
#define DM_V_MAX_RAD_S      45.0f
#define DM_V_MIN_RAD_S      (-45.0f)
#define DM_T_MAX_NM         10.0f
#define DM_T_MIN_NM         (-10.0f)
#define DM_KP_MAX           500.0f
#define DM_KP_MIN           0.0f
#define DM_KD_MAX           5.0f
#define DM_KD_MIN           0.0f

/* 参数帧 */
#define DM_PARAM_CAN_ID     0x7FFU
#define DM_OP_READ          0x33U
#define DM_OP_WRITE         0x55U
#define DM_OP_SAVE          0xAAU

/* 寄存器编号 */
#define DM_RID_MST_ID       7U
#define DM_RID_ESC_ID       8U
#define DM_RID_CTRL_MODE    10U
#define DM_RID_P_MAX        21U
#define DM_RID_V_MAX        22U
#define DM_RID_T_MAX        23U

#define DM_CTRL_MODE_MIT    1U

typedef struct
{
    uint8_t  esc_id;        /* 反馈帧里报告的电机 ID */
    uint8_t  state;         /* 0=失能 1=使能 >=8=故障 */
    float    pos_rad;
    float    vel_rad_s;
    float    torque_nm;
    uint8_t  t_mos_c;
    uint8_t  t_rotor_c;
    uint32_t last_rx_tick;
    uint32_t rx_count;
    uint32_t id_mismatch_count;
} DM_Feedback_t;

typedef struct
{
    uint16_t esc_id;
    uint8_t  opcode;
    uint8_t  rid;
    uint32_t raw;
    uint32_t tick;
} DM_ParamResponse_t;

typedef struct
{
    uint32_t default_feedback_count;
    uint32_t unknown_feedback_count;
    uint32_t id_mismatch_count;
    uint32_t param_timeout_count;
} DM_Stats_t;

void DM_Init(uint8_t motor_count, const uint8_t *esc_ids, const uint8_t *mst_ids);

uint8_t DM_SendMit(uint16_t esc_id, float pos_rad, float vel_rad_s,
                   float kp, float kd, float torque_nm);
uint8_t DM_SendTorque(uint8_t slot, float torque_nm);

uint8_t DM_SendEnable(uint16_t esc_id);
uint8_t DM_SendDisable(uint16_t esc_id);
uint8_t DM_SendClearError(uint16_t esc_id);
uint8_t DM_SendSaveZero(uint16_t esc_id);

uint8_t DM_ReadParam(uint16_t esc_id, uint8_t rid);
uint8_t DM_WriteParam(uint16_t esc_id, uint8_t rid, uint32_t raw);
uint8_t DM_SaveParams(uint16_t esc_id);

void DM_HandleFrame(uint16_t id, const uint8_t data[8]);
uint8_t DM_GetParamResponse(DM_ParamResponse_t *out);
void DM_ClearParamResponse(void);

uint8_t DM_GetFeedback(uint8_t slot, DM_Feedback_t *out);
uint8_t DM_GetSlotByEscId(uint8_t esc_id);
uint8_t DM_GetMotorCount(void);
uint8_t DM_GetEscId(uint8_t slot);
uint8_t DM_GetMstId(uint8_t slot);
void DM_GetStats(DM_Stats_t *out);

#endif
