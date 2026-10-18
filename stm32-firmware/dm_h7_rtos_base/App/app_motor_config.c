#include "app_motor_config.h"
#include "bsp_fdcan.h"
#include "bsp_dm6215.h"
#include "bsp_bluetooth.h"
#include "main.h"
#include "cmsis_os2.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MOTOR_COUNT                 4U
#define MOTOR_PROBE_TIMEOUT_MS      50U
#define MOTOR_ENABLE_TIMEOUT_MS     100U
#define MOTOR_ENABLE_RETRIES        3U
#define MOTOR_CFG_READ_TIMEOUT_MS   50U
#define MOTOR_POWER_ON_DELAY_MS     500U   /* 等电机/收发器上电启动 */

typedef enum
{
    MOTOR_STATE_PROBE = 0,
    MOTOR_STATE_ENABLE,
    MOTOR_STATE_READY,
    MOTOR_STATE_CONFIG,
    MOTOR_STATE_FAULT
} MotorState_t;

static const uint8_t s_esc_ids[MOTOR_COUNT] = {1U, 2U, 3U, 4U};
static const uint8_t s_mst_ids[MOTOR_COUNT] = {0x51U, 0x52U, 0x53U, 0x54U};

static volatile MotorState_t s_state = MOTOR_STATE_PROBE;
static uint8_t s_verified_mask = 0U;
static uint8_t s_enabled_mask = 0U;
static uint8_t s_fault = MOTOR_CFG_FAULT_NONE;
static volatile uint8_t s_restart_req = 0U;

static uint8_t s_config_slot = 0U;
static uint16_t s_config_addr = 1U;

static void motor_can_rx(const BSP_FDCAN_Frame_t *frame)
{
    DM_HandleFrame(frame->id, frame->data);
}

static uint8_t dm_read_param_wait(uint16_t esc_id, uint8_t rid,
                                  uint32_t timeout_ms,
                                  DM_ParamResponse_t *out)
{
    DM_ParamResponse_t response;
    uint32_t start;

    DM_ClearParamResponse();

    if (DM_ReadParam(esc_id, rid) == 0U)
    {
        return 0U;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (DM_GetParamResponse(&response) != 0U)
        {
            if (out != NULL)
            {
                *out = response;
            }
            return 1U;
        }
        osDelay(5U);
    }

    return 0U;
}

static uint8_t dm_write_param_wait(uint16_t esc_id, uint8_t rid, uint32_t raw,
                                   uint32_t timeout_ms,
                                   DM_ParamResponse_t *out)
{
    DM_ParamResponse_t response;
    uint32_t start;

    DM_ClearParamResponse();

    if (DM_WriteParam(esc_id, rid, raw) == 0U)
    {
        return 0U;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (DM_GetParamResponse(&response) != 0U)
        {
            if (out != NULL)
            {
                *out = response;
            }

            /* 写 ESC_ID 时应答里的地址也会变成新 ID，兼容两种回显 */
            if ((response.opcode == DM_OP_WRITE) &&
                ((response.raw == raw) || (response.esc_id == raw)))
            {
                return 1U;
            }
        }
        osDelay(5U);
    }

    return 0U;
}

static uint8_t dm_save_param_wait(uint16_t esc_id, uint32_t timeout_ms)
{
    DM_ParamResponse_t response;
    uint32_t start;

    DM_ClearParamResponse();

    if (DM_SaveParams(esc_id) == 0U)
    {
        return 0U;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (DM_GetParamResponse(&response) != 0U)
        {
            if (response.opcode == DM_OP_SAVE)
            {
                return 1U;
            }
        }
        osDelay(5U);
    }

    return 0U;
}

static uint8_t motor_probe(void)
{
    uint8_t i;
    uint8_t ok;

    s_verified_mask = 0U;
    s_enabled_mask = 0U;
    s_fault = MOTOR_CFG_FAULT_NONE;
    s_state = MOTOR_STATE_PROBE;

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        DM_ParamResponse_t response;

        ok = 1U;

        if ((dm_read_param_wait(s_esc_ids[i], DM_RID_ESC_ID,
                                MOTOR_PROBE_TIMEOUT_MS, &response) == 0U) ||
            (response.raw != s_esc_ids[i]))
        {
            ok = 0U;
        }

        if ((ok != 0U) &&
            ((dm_read_param_wait(s_esc_ids[i], DM_RID_MST_ID,
                                 MOTOR_PROBE_TIMEOUT_MS, &response) == 0U) ||
             (response.raw != s_mst_ids[i])))
        {
            ok = 0U;
        }

        if ((ok != 0U) &&
            ((dm_read_param_wait(s_esc_ids[i], DM_RID_CTRL_MODE,
                                 MOTOR_PROBE_TIMEOUT_MS, &response) == 0U) ||
             (response.raw != DM_CTRL_MODE_MIT)))
        {
            ok = 0U;
        }

        if (ok != 0U)
        {
            s_verified_mask |= (uint8_t)(1U << i);
        }
    }

    if (s_verified_mask != 0x0FU)
    {
        /* 缺失电机/无 ACK 可能把 CAN 推到 bus-off，恢复后继续允许配置 */
        if (BSP_FDCAN_IsBusOff() != 0U)
        {
            (void)BSP_FDCAN_Recover();
        }

        s_state = MOTOR_STATE_FAULT;
        s_fault = MOTOR_CFG_FAULT_NOT_READY;
        return 0U;
    }

    return 1U;
}

static uint8_t motor_enable_all(void)
{
    uint8_t i;
    uint8_t retry;

    s_state = MOTOR_STATE_ENABLE;
    s_enabled_mask = 0U;

    for (i = 0U; i < MOTOR_COUNT; i++)
    {
        uint8_t enabled = 0U;

        for (retry = 0U; retry < MOTOR_ENABLE_RETRIES; retry++)
        {
            DM_Feedback_t feedback;
            uint32_t start;

            (void)DM_SendClearError(s_esc_ids[i]);
            osDelay(5U);

            (void)DM_SendEnable(s_esc_ids[i]);
            osDelay(5U);

            /* 反馈是命令驱动的，发一帧 0 力矩请求反馈 */
            (void)DM_SendTorque(i, 0.0f);

            start = HAL_GetTick();
            while ((HAL_GetTick() - start) < MOTOR_ENABLE_TIMEOUT_MS)
            {
                if ((DM_GetFeedback(i, &feedback) != 0U) &&
                    (feedback.rx_count > 0U) &&
                    (feedback.state == 1U) &&
                    ((HAL_GetTick() - feedback.last_rx_tick) <= 100U))
                {
                    enabled = 1U;
                    break;
                }
                osDelay(5U);
            }

            if (enabled != 0U)
            {
                break;
            }
        }

        if (enabled != 0U)
        {
            s_enabled_mask |= (uint8_t)(1U << i);
        }
        else
        {
            break;
        }
    }

    if (s_enabled_mask != 0x0FU)
    {
        for (i = 0U; i < MOTOR_COUNT; i++)
        {
            (void)DM_SendDisable(s_esc_ids[i]);
        }

        s_state = MOTOR_STATE_FAULT;
        s_fault = MOTOR_CFG_FAULT_ENABLE_FAILED;
        return 0U;
    }

    s_state = MOTOR_STATE_READY;
    s_fault = MOTOR_CFG_FAULT_NONE;
    return 1U;
}

static uint8_t motor_try_start(void)
{
    if (motor_probe() == 0U)
    {
        return 0U;
    }

    return motor_enable_all();
}

void App_MotorConfig_Init(void)
{
    BSP_FDCAN_SetRxCallback(motor_can_rx);
    BSP_FDCAN_Init();
    DM_Init(MOTOR_COUNT, s_esc_ids, s_mst_ids);

    s_config_slot = 0U;
    s_config_addr = 1U;
    s_restart_req = 0U;

    /* 板级 Power_OUT1/2 在 main 中已打开，等电机和 CAN 收发器启动后再探测 */
    osDelay(MOTOR_POWER_ON_DELAY_MS);

    (void)motor_try_start();
}

void App_MotorConfig_Poll(uint32_t now)
{
    (void)now;

    if (s_restart_req != 0U)
    {
        s_restart_req = 0U;
        (void)motor_try_start();
    }

    if ((s_state == MOTOR_STATE_READY) && (BSP_FDCAN_IsBusOff() != 0U))
    {
        s_state = MOTOR_STATE_FAULT;
        s_fault = MOTOR_CFG_FAULT_BUS_OFF;
    }
}

void App_MotorConfig_RequestRestart(void)
{
    s_restart_req = 1U;
}

void App_MotorConfig_ForceFault(uint8_t fault)
{
    s_state = MOTOR_STATE_FAULT;
    s_fault = fault;
}

uint8_t App_MotorConfig_AllReady(void)
{
    return (s_state == MOTOR_STATE_READY) ? 1U : 0U;
}

uint8_t App_MotorConfig_IsConfigActive(void)
{
    return (s_state == MOTOR_STATE_CONFIG) ? 1U : 0U;
}

uint8_t App_MotorConfig_GetFault(void)
{
    return s_fault;
}

void App_MotorConfig_GetMasks(uint8_t *verified_mask, uint8_t *enabled_mask)
{
    if (verified_mask != NULL)
    {
        *verified_mask = s_verified_mask;
    }
    if (enabled_mask != NULL)
    {
        *enabled_mask = s_enabled_mask;
    }
}

/* ------------------------------------------------------------------------- */
/* 蓝牙配置命令                                                               */
/* ------------------------------------------------------------------------- */

static void cfg_reply(const char *fmt, ...)
{
    char line[96];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    BT_QueueText(line);
}

static uint8_t cfg_tokenize(char *buf, char *tokens[], uint8_t max_tokens)
{
    uint8_t count = 0U;
    char *p = buf;

    while (*p != '\0')
    {
        while (*p == ' ')
        {
            p++;
        }
        if (*p == '\0')
        {
            break;
        }

        if (count < max_tokens)
        {
            tokens[count++] = p;
        }

        while ((*p != '\0') && (*p != ' '))
        {
            p++;
        }

        if (*p == ' ')
        {
            *p = '\0';
            p++;
        }
    }

    return count;
}

static uint8_t cfg_parse_u32(const char *text, uint32_t *out)
{
    char *end = NULL;
    unsigned long value;

    if ((text == NULL) || (out == NULL))
    {
        return 0U;
    }

    value = strtoul(text, &end, 0);
    if ((end == text) || (*end != '\0'))
    {
        return 0U;
    }

    *out = (uint32_t)value;
    return 1U;
}

static void cfg_read_and_report(void)
{
    DM_ParamResponse_t r;
    uint32_t esc_id = 0U;
    uint32_t mst_id = 0U;
    uint32_t mode = 0U;
    float p_max = 0.0f;
    float v_max = 0.0f;
    float t_max = 0.0f;

    if ((dm_read_param_wait(s_config_addr, DM_RID_ESC_ID,
                            MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U))
    {
        esc_id = r.raw;
    }
    if ((dm_read_param_wait(s_config_addr, DM_RID_MST_ID,
                            MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U))
    {
        mst_id = r.raw;
    }
    if ((dm_read_param_wait(s_config_addr, DM_RID_CTRL_MODE,
                            MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U))
    {
        mode = r.raw;
    }
    if ((dm_read_param_wait(s_config_addr, DM_RID_P_MAX,
                            MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U))
    {
        memcpy(&p_max, &r.raw, sizeof(p_max));
    }
    if ((dm_read_param_wait(s_config_addr, DM_RID_V_MAX,
                            MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U))
    {
        memcpy(&v_max, &r.raw, sizeof(v_max));
    }
    if ((dm_read_param_wait(s_config_addr, DM_RID_T_MAX,
                            MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U))
    {
        memcpy(&t_max, &r.raw, sizeof(t_max));
    }

    /* 不用 %f：newlib-nano 默认不链接浮点 printf，用 ×10 定点输出 */
    cfg_reply("cfg read esc=%lu mst=0x%lX mode=%lu P10=%ld V10=%ld T10=%ld\r\n",
              (unsigned long)esc_id, (unsigned long)mst_id,
              (unsigned long)mode,
              (long)(p_max * 10.0f),
              (long)(v_max * 10.0f),
              (long)(t_max * 10.0f));
}

void App_MotorConfig_HandleBtRequest(const char *payload)
{
    char buf[64];
    char *tokens[4] = {0};
    uint8_t count;
    uint32_t value;

    if (payload == NULL)
    {
        return;
    }

    (void)snprintf(buf, sizeof(buf), "%s", payload);
    count = cfg_tokenize(buf, tokens, 4U);
    if (count == 0U)
    {
        return;
    }

    /* 进入配置模式，禁止运动 */
    s_state = MOTOR_STATE_CONFIG;

    if (strcmp(tokens[0], "cfgselect") == 0)
    {
        uint32_t slot = 1U;
        uint32_t addr = 1U;

        if (count >= 2U)
        {
            (void)cfg_parse_u32(tokens[1], &slot);
        }
        if (count >= 3U)
        {
            (void)cfg_parse_u32(tokens[2], &addr);
        }

        if ((slot < 1U) || (slot > 4U) || (addr < 1U) || (addr > 0x7FU))
        {
            cfg_reply("cfg err bad select\r\n");
            return;
        }

        s_config_slot = (uint8_t)(slot - 1U);
        s_config_addr = (uint16_t)addr;

        cfg_reply("cfg sel slot=%lu addr=%lu target=%u mst=0x%02X\r\n",
                  (unsigned long)slot, (unsigned long)addr,
                  (unsigned)s_esc_ids[s_config_slot],
                  (unsigned)s_mst_ids[s_config_slot]);
    }
    else if (strcmp(tokens[0], "cfgscan") == 0)
    {
        char found[64];
        uint32_t max_id = 16U;
        uint32_t id;
        uint16_t len = 0U;

        if (count >= 2U)
        {
            (void)cfg_parse_u32(tokens[1], &max_id);
        }
        if ((max_id == 0U) || (max_id > 0x7FU))
        {
            max_id = 16U;
        }

        found[0] = '\0';
        for (id = 1U; id <= max_id; id++)
        {
            DM_ParamResponse_t r;

            if ((dm_read_param_wait((uint16_t)id, DM_RID_ESC_ID,
                                    20U, &r) != 0U) &&
                (r.raw == id))
            {
                len += (uint16_t)snprintf(&found[len],
                                          sizeof(found) - len,
                                          " %lu", (unsigned long)id);
                if (len >= (sizeof(found) - 8U))
                {
                    break;
                }
            }
        }

        /* 扫描会对不存在的 ID 发大量无 ACK 帧，结束后复位 CAN 错误状态 */
        (void)BSP_FDCAN_Recover();

        cfg_reply("cfg scan:%s\r\n", (found[0] == '\0') ? " none" : found);
    }
    else if (strcmp(tokens[0], "cfgid") == 0)
    {
        DM_ParamResponse_t r;

        if ((count < 2U) || (cfg_parse_u32(tokens[1], &value) == 0U) ||
            (value < 1U) || (value > 0x7FU))
        {
            cfg_reply("cfg err bad id\r\n");
            return;
        }

        if (dm_write_param_wait(s_config_addr, DM_RID_ESC_ID, value,
                                MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U)
        {
            uint16_t old_addr = s_config_addr;
            s_config_addr = (uint16_t)value;
            cfg_reply("cfg id ok old=%u new=%lu\r\n",
                      (unsigned)old_addr, (unsigned long)value);
        }
        else
        {
            cfg_reply("cfg err id timeout\r\n");
        }
    }
    else if (strcmp(tokens[0], "cfgmst") == 0)
    {
        DM_ParamResponse_t r;

        if ((count < 2U) || (cfg_parse_u32(tokens[1], &value) == 0U))
        {
            cfg_reply("cfg err bad mst\r\n");
            return;
        }

        if (dm_write_param_wait(s_config_addr, DM_RID_MST_ID, value,
                                MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U)
        {
            cfg_reply("cfg mst ok 0x%lX\r\n", (unsigned long)value);
        }
        else
        {
            cfg_reply("cfg err mst timeout\r\n");
        }
    }
    else if (strcmp(tokens[0], "cfgmode") == 0)
    {
        DM_ParamResponse_t r;

        if ((count < 2U) || (cfg_parse_u32(tokens[1], &value) == 0U) ||
            (value < 1U) || (value > 3U))
        {
            cfg_reply("cfg err bad mode\r\n");
            return;
        }

        if (dm_write_param_wait(s_config_addr, DM_RID_CTRL_MODE, value,
                                MOTOR_CFG_READ_TIMEOUT_MS, &r) != 0U)
        {
            cfg_reply("cfg mode ok %lu\r\n", (unsigned long)value);
        }
        else
        {
            cfg_reply("cfg err mode timeout\r\n");
        }
    }
    else if (strcmp(tokens[0], "cfgread") == 0)
    {
        cfg_read_and_report();
    }
    else if (strcmp(tokens[0], "cfgsave") == 0)
    {
        /* 保存参数必须在失能状态 */
        (void)DM_SendDisable(s_config_addr);
        osDelay(20U);

        if (dm_save_param_wait(s_config_addr, 200U) != 0U)
        {
            cfg_reply("cfg save ok addr=%u\r\n", (unsigned)s_config_addr);
        }
        else
        {
            cfg_reply("cfg err save timeout\r\n");
        }
    }
    else if (strcmp(tokens[0], "cfgverify") == 0)
    {
        DM_ParamResponse_t r;
        uint8_t ok = 1U;

        if ((dm_read_param_wait(s_config_addr, DM_RID_ESC_ID,
                                MOTOR_CFG_READ_TIMEOUT_MS, &r) == 0U) ||
            (r.raw != s_esc_ids[s_config_slot]))
        {
            ok = 0U;
        }
        if ((ok != 0U) &&
            ((dm_read_param_wait(s_config_addr, DM_RID_MST_ID,
                                 MOTOR_CFG_READ_TIMEOUT_MS, &r) == 0U) ||
             (r.raw != s_mst_ids[s_config_slot])))
        {
            ok = 0U;
        }

        if (ok != 0U)
        {
            cfg_reply("cfg verify ok slot=%u\r\n",
                      (unsigned)(s_config_slot + 1U));
        }
        else
        {
            cfg_reply("cfg err verify slot=%u\r\n",
                      (unsigned)(s_config_slot + 1U));
        }
    }
    else if (strcmp(tokens[0], "cfgdone") == 0)
    {
        s_state = MOTOR_STATE_FAULT;
        s_fault = MOTOR_CFG_FAULT_NOT_READY;

        if (motor_try_start() != 0U)
        {
            cfg_reply("cfg done ready\r\n");
        }
        else
        {
            cfg_reply("cfg done not ready fault=%u\r\n",
                      (unsigned)s_fault);
        }
    }
    else if (strcmp(tokens[0], "cfgabort") == 0)
    {
        s_state = MOTOR_STATE_FAULT;
        s_fault = MOTOR_CFG_FAULT_NOT_READY;
        cfg_reply("cfg abort\r\n");
    }
    else if (strcmp(tokens[0], "cfgstatus") == 0)
    {
        cfg_reply("cfg status state=%u verified=0x%02X enabled=0x%02X slot=%u addr=%u fault=%u\r\n",
                  (unsigned)s_state, (unsigned)s_verified_mask,
                  (unsigned)s_enabled_mask,
                  (unsigned)(s_config_slot + 1U),
                  (unsigned)s_config_addr, (unsigned)s_fault);
    }
    else if (strcmp(tokens[0], "cfghelp") == 0)
    {
        cfg_reply("cfg: select|scan|id|mst|mode|read|save|verify|done|abort|status\r\n");
    }
    else
    {
        cfg_reply("cfg err unknown\r\n");
    }
}
