#include "bsp_WonderEcho.h"
#include "main.h"

#define WONDERECHO_I2C_DELAY_VALUE      20U
#define WONDERECHO_ACK_TIMEOUT          80U

volatile WonderEcho_Debug_t wonderecho_debug = {0};

#define WONDERECHO_I2C_SCL(a)   if (a) \
                                    HAL_GPIO_WritePin(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin, GPIO_PIN_SET); \
                                else \
                                    HAL_GPIO_WritePin(SOFT_I2C_SCL_GPIO_Port, SOFT_I2C_SCL_Pin, GPIO_PIN_RESET)

#define WONDERECHO_I2C_SDA(a)   if (a) \
                                    HAL_GPIO_WritePin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin, GPIO_PIN_SET); \
                                else \
                                    HAL_GPIO_WritePin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin, GPIO_PIN_RESET)

/* ============================================================
 *  软件 I2C 短延时
 *  说明: 与 touch_iic.c 保持同类实现，避免额外占用硬件定时器资源。
 * ============================================================ */
static void WonderEcho_I2C_Delay(uint32_t delay)
{
    volatile uint16_t i;

    while (delay--)
    {
        for (i = 0; i < 8; i++);
    }
}

/* ============================================================
 *  WonderEcho 软件 I2C GPIO 初始化
 *  PB0 -> SCL, PB1 -> SDA
 *  两个引脚均配置为开漏输出 + 内部上拉:
 *    1. 输出 0 时主动拉低总线;
 *    2. 输出 1 时释放总线，由上拉电阻保持高电平;
 *    3. SDA 读取 ACK/数据时不切换输入输出方向，直接释放 SDA 后读引脚电平。
 * ============================================================ */
void WonderEcho_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = SOFT_I2C_SCL_Pin | SOFT_I2C_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 空闲状态下 I2C 总线必须保持高电平，避免模块上电后误判起始信号。 */
    WONDERECHO_I2C_SCL(1);
    WONDERECHO_I2C_SDA(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
}

/* ============================================================
 *  I2C 起始信号
 *  SCL 为高电平时 SDA 从高到低跳变，通知从机开始一次传输。
 * ============================================================ */
static void WonderEcho_I2C_Start(void)
{
    WONDERECHO_I2C_SDA(1);
    WONDERECHO_I2C_SCL(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);

    WONDERECHO_I2C_SDA(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    WONDERECHO_I2C_SCL(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
}

/* ============================================================
 *  I2C 停止信号
 *  SCL 为高电平时 SDA 从低到高跳变，释放本次传输。
 * ============================================================ */
static void WonderEcho_I2C_Stop(void)
{
    WONDERECHO_I2C_SCL(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    WONDERECHO_I2C_SDA(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);

    WONDERECHO_I2C_SCL(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    WONDERECHO_I2C_SDA(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
}

/* ============================================================
 *  主机发送 ACK
 *  读多字节时使用。本工程当前只读单字节结果，保留该函数便于后续扩展。
 * ============================================================ */
static void WonderEcho_I2C_ACK(void)
{
    WONDERECHO_I2C_SCL(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    WONDERECHO_I2C_SDA(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    WONDERECHO_I2C_SCL(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);

    WONDERECHO_I2C_SCL(0);
    WONDERECHO_I2C_SDA(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
}

/* ============================================================
 *  主机发送 NACK
 *  单字节读取结束后发送 NACK，告诉从机本次读取完成。
 * ============================================================ */
static void WonderEcho_I2C_NoACK(void)
{
    WONDERECHO_I2C_SCL(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    WONDERECHO_I2C_SDA(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    WONDERECHO_I2C_SCL(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);

    WONDERECHO_I2C_SCL(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
}

/* ============================================================
 *  等待从机 ACK
 *  释放 SDA 后读取电平。若 WonderEcho 未接线、掉电或总线异常，
 *  timeout 计数会强制退出，避免底盘任务卡死在 I2C 等待中。
 *  返回值: 1=收到 ACK, 0=超时或无应答
 * ============================================================ */
static uint8_t WonderEcho_I2C_WaitACK(void)
{
    uint16_t timeout = WONDERECHO_ACK_TIMEOUT;

    WONDERECHO_I2C_SDA(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    WONDERECHO_I2C_SCL(1);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);

    while ((HAL_GPIO_ReadPin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin) != GPIO_PIN_RESET) && (timeout > 0U))
    {
        timeout--;
        WonderEcho_I2C_Delay(1U);
    }

    WONDERECHO_I2C_SCL(0);
    WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);

    if (timeout == 0U)
    {
        return 0U;
    }

    return 1U;
}

/* ============================================================
 *  写 1 字节数据
 *  I2C 高位先发，每一位在 SCL 高电平期间保持稳定。
 *  返回值: 1=从机 ACK, 0=无应答
 * ============================================================ */
static uint8_t WonderEcho_I2C_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8U; i++)
    {
        WONDERECHO_I2C_SDA(data & 0x80U);
        WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
        WONDERECHO_I2C_SCL(1);
        WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
        WONDERECHO_I2C_SCL(0);
        WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
        data <<= 1;
    }

    /* 发送完 8 位后释放 SDA，让 WonderEcho 在第 9 个时钟周期拉低应答。 */
    WONDERECHO_I2C_SDA(1);
    return WonderEcho_I2C_WaitACK();
}

/* ============================================================
 *  读 1 字节数据
 *  主机释放 SDA，由 WonderEcho 在 SCL 高电平期间输出数据位。
 *  ack_mode=1 发送 ACK，ack_mode=0 发送 NACK。
 * ============================================================ */
static uint8_t WonderEcho_I2C_ReadByte(uint8_t ack_mode)
{
    uint8_t i;
    uint8_t data = 0U;

    WONDERECHO_I2C_SDA(1);

    for (i = 0; i < 8U; i++)
    {
        data <<= 1;
        WONDERECHO_I2C_SCL(1);
        WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
        if (HAL_GPIO_ReadPin(SOFT_I2C_SDA_GPIO_Port, SOFT_I2C_SDA_Pin) == GPIO_PIN_SET)
        {
            data |= 0x01U;
        }
        WONDERECHO_I2C_SCL(0);
        WonderEcho_I2C_Delay(WONDERECHO_I2C_DELAY_VALUE);
    }

    if (ack_mode)
    {
        WonderEcho_I2C_ACK();
    }
    else
    {
        WonderEcho_I2C_NoACK();
    }

    return data;
}

/* ============================================================
 *  读取 WonderEcho 识别结果寄存器
 *  访问流程:
 *    1. 写地址 + 结果寄存器 0x64;
 *    2. 重复起始信号;
 *    3. 读地址 + 读取 1 字节命令 ID;
 *    4. 单字节读取后发送 NACK 并停止。
 *  任一阶段无 ACK 都立即 STOP 并返回 0，保证通信异常不影响底盘循环。
 * ============================================================ */
uint8_t WonderEcho_GetResult(void)
{
    uint8_t result;

    wonderecho_debug.read_count++;
    wonderecho_debug.last_read_tick = HAL_GetTick();
    wonderecho_debug.last_fail_step = 0U;

    WonderEcho_I2C_Start();
    if (!WonderEcho_I2C_WriteByte(WONDERECHO_WRITE_ADDR))
    {
        WonderEcho_I2C_Stop();
        wonderecho_debug.read_fail_count++;
        wonderecho_debug.last_result = 0U;
        wonderecho_debug.last_fail_step = 1U;
        return 0U;
    }
    if (!WonderEcho_I2C_WriteByte(WONDERECHO_RESULT_REG))
    {
        WonderEcho_I2C_Stop();
        wonderecho_debug.read_fail_count++;
        wonderecho_debug.last_result = 0U;
        wonderecho_debug.last_fail_step = 2U;
        return 0U;
    }

    WonderEcho_I2C_Start();
    if (!WonderEcho_I2C_WriteByte(WONDERECHO_READ_ADDR))
    {
        WonderEcho_I2C_Stop();
        wonderecho_debug.read_fail_count++;
        wonderecho_debug.last_result = 0U;
        wonderecho_debug.last_fail_step = 3U;
        return 0U;
    }

    result = WonderEcho_I2C_ReadByte(0U);
    WonderEcho_I2C_Stop();
    wonderecho_debug.last_result = result;

    return result;
}

/* ============================================================
 *  WonderEcho 播报指令写入
 *  向 0x6E 寄存器连续写入 cmd 和 idNum。当前语音控制逻辑不依赖播报，
 *  但保留接口供后续做语音反馈或调试提示。
 * ============================================================ */
void WonderEcho_Speak(uint8_t cmd, uint8_t idNum)
{
    wonderecho_debug.speak_count++;
    wonderecho_debug.last_speak_tick = HAL_GetTick();
    wonderecho_debug.last_speak_cmd = cmd;
    wonderecho_debug.last_speak_id = idNum;
    wonderecho_debug.last_fail_step = 0U;

    WonderEcho_I2C_Start();
    if (!WonderEcho_I2C_WriteByte(WONDERECHO_WRITE_ADDR))
    {
        WonderEcho_I2C_Stop();
        wonderecho_debug.last_fail_step = 4U;
        return;
    }
    if (!WonderEcho_I2C_WriteByte(WONDERECHO_SPEAK_REG))
    {
        WonderEcho_I2C_Stop();
        wonderecho_debug.last_fail_step = 5U;
        return;
    }
    if (!WonderEcho_I2C_WriteByte(cmd))
    {
        WonderEcho_I2C_Stop();
        wonderecho_debug.last_fail_step = 6U;
        return;
    }
    if (!WonderEcho_I2C_WriteByte(idNum))
    {
        wonderecho_debug.last_fail_step = 7U;
    }
    WonderEcho_I2C_Stop();
}
