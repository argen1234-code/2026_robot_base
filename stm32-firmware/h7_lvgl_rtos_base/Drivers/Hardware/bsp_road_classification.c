#include "bsp_road_classification.h"

/*
 * 仅本实现文件接触 NanoEdge 原始 API。
 * 上层模块只使用 bsp_road_classification.h 中的自有类型，避免把库的 enum ABI
 * 和内部符号扩散到已验证的底盘、通信及界面代码。
 *
 * 本次库由 NanoEdge 以 soft-float、short-enum 导出，而 STM32H743 工程本身使用
 * 硬浮点。这里之所以可以安全共存，是因为 NanoEdge 的公开函数只传递指针和整数，
 * 没有任何按值传递的 float 参数或返回值；库内部软浮点代码调用 expf/logf/sqrtf
 * 时，ARMCC 链接器会选择其 base-AAPCS 入口，再由运行库跳转到硬浮点实现。
 * 因此只需让本文件按短 enum 编译，绝不能为了模型去改变全工程浮点 ABI。
 * 如果未来 NanoEdge 公共 API 出现按值传递的 float/double，则必须重新审查 ABI，
 * 不可照搬本次结论。
 */
#include "NanoEdgeAI/NanoEdgeAI.h"

#include <string.h>

/*
 * 编译期先核对“头文件契约”。运行时 Init() 还会通过 getter 再核对静态库本体，
 * 因而能同时发现：只换了头文件、只换了 .a、或误放入其它模型这三类常见问题。
 * ARMCC 5 不使用 C11 _Static_assert，这里用预处理错误保持兼容。
 */
#if (NEAI_INPUT_SIGNAL_LENGTH != BSP_ROAD_CLASSIFICATION_WINDOW_SAMPLES)
#error "NanoEdge input signal length does not match road classification BSP"
#endif

#if (NEAI_INPUT_AXIS_NUMBER != BSP_ROAD_CLASSIFICATION_AXIS_COUNT)
#error "NanoEdge axis count does not match road classification BSP"
#endif

#if (NEAI_NUMBER_OF_CLASSES != BSP_ROAD_CLASSIFICATION_CLASS_COUNT)
#error "NanoEdge class count does not match road classification BSP"
#endif

/*
 * 此常量对应本次 ZIP 内导出的 NanoEdgeAI.h。运行时核对 ID 可以阻止“只替换了
 * .a 或只替换了 .h”的半更新状态；遇到这种情况时 BSP 进入 ERROR，但底盘照常跑。
 */
#define BSP_ROAD_CLASSIFICATION_EXPECTED_MODEL_ID "6a5fc92cfd819c3c7aa16876"

/*
 * NanoEdge 导出包保留了训练数据集名称。它们只用于校验 .h 与 .a 确实来自同一
 * 个 ZIP；BSP 对上层公开的类别名仍是简洁的 outdoor / indoor。
 */
static const char * const s_expected_model_class_names[BSP_ROAD_CLASSIFICATION_CLASS_COUNT] =
{
    "jy901s_nanoedge_scu_outdoor",
    "jy901s_nanoedge_live_indoor2"
};

/*
 * NanoEdge 输入缓冲的布局必须是“时间优先”：
 * [第 0 帧 Ax, Ay, Az, Gx, Gy, Gz, 第 1 帧 Ax, ...]。
 * 原始训练 CSV 的列顺序正是 ACC_X_g、ACC_Y_g、ACC_Z_g、
 * GYRO_X_dps、GYRO_Y_dps、GYRO_Z_dps；不能改成六个轴各自连续存放。这个布局
 * 与 jy901s_nanoedge_logger.py 的 flatten_window() 一致。
 */
static float s_input_signal[BSP_ROAD_CLASSIFICATION_WINDOW_SAMPLES *
                            BSP_ROAD_CLASSIFICATION_AXIS_COUNT];

static BSP_RoadClassificationResult_t s_result;
static uint32_t s_last_collected_imu_tick;
static uint32_t s_last_collected_acc_sequence;
static uint32_t s_last_collected_gyro_sequence;
static uint8_t s_collected_samples;
static uint8_t s_model_initialized;
static float s_latest_imu_sample[BSP_ROAD_CLASSIFICATION_AXIS_COUNT];

/*
 * 为 Keil Watch 保留的外部可见调试镜像。该变量从不参与任何控制计算，只在本
 * 文件的极短临界区内由 s_result 和最新 IMU 输入复制而来。
 */
volatile BSP_RoadClassificationDebug_t g_bsp_road_classification_debug;

/*
 * 结果读取者可能是其他 FreeRTOS 任务。推理过程不关中断，只有复制/发布结果
 * 的极短临界区关闭中断，避免读取到概率数组写入一半的快照。
 */
static uint32_t BSP_RoadClassification_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void BSP_RoadClassification_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/*
 * 公开结果中类别名以固定数组保存，调用者无需包含 NanoEdgeAI.h 或持有库内部
 * 指针。复制时总会写入终止符，预热/错误状态会稳定显示为 "unknown"。
 */
static void BSP_RoadClassification_CopyClassName(char *destination,
                                                  BSP_RoadClassificationClass_t class_id)
{
    const char *source;
    uint32_t i;

    if (destination == NULL)
    {
        return;
    }

    source = BSP_RoadClassification_GetClassName(class_id);
    for (i = 0U; i < (BSP_ROAD_CLASSIFICATION_CLASS_NAME_LENGTH - 1U); i++)
    {
        destination[i] = source[i];
        if (source[i] == '\0')
        {
            break;
        }
    }

    /*
     * 即使未来标签变长，也保证结果快照是合法 C 字符串；剩余字节清零可避免
     * Keil Watch 中残留上一次较长标签的尾部。
     */
    for (; i < BSP_ROAD_CLASSIFICATION_CLASS_NAME_LENGTH; i++)
    {
        destination[i] = '\0';
    }
}

/*
 * 在写入 s_result 的同一个临界区内刷新 Keil 调试镜像，保证观察者不会看到
 * "类别已更新但概率仍是上一帧" 这种长期不一致的状态。update_sequence 先变奇、
 * 结束后变偶：调试器恰好在写入过程中暂停时，可用该字段判断是否需要继续一次。
 */
static void BSP_RoadClassification_UpdateDebugSnapshot(void)
{
    uint32_t i;

    g_bsp_road_classification_debug.update_sequence++;
    g_bsp_road_classification_debug.model_initialized = s_model_initialized;
    g_bsp_road_classification_debug.state = (uint8_t)s_result.state;
    g_bsp_road_classification_debug.class_id = (int8_t)s_result.class_id;
    g_bsp_road_classification_debug.collected_samples = s_result.collected_samples;
    g_bsp_road_classification_debug.has_result = s_result.has_result;
    g_bsp_road_classification_debug.last_neai_status = s_result.last_neai_status;
    g_bsp_road_classification_debug.last_imu_tick = s_result.last_imu_tick;
    g_bsp_road_classification_debug.last_classification_tick = s_result.last_classification_tick;

    for (i = 0U; i < BSP_ROAD_CLASSIFICATION_AXIS_COUNT; i++)
    {
        g_bsp_road_classification_debug.latest_imu_sample[i] = s_latest_imu_sample[i];
    }

    for (i = 0U; i < BSP_ROAD_CLASSIFICATION_PROBABILITY_SLOTS; i++)
    {
        g_bsp_road_classification_debug.probabilities[i] = s_result.probabilities[i];
    }

    for (i = 0U; i < BSP_ROAD_CLASSIFICATION_CLASS_NAME_LENGTH; i++)
    {
        g_bsp_road_classification_debug.class_name[i] = s_result.class_name[i];
    }

    g_bsp_road_classification_debug.update_sequence++;
}

static void BSP_RoadClassification_PublishWarmup(void)
{
    uint32_t primask = BSP_RoadClassification_EnterCritical();
    uint32_t i;

    if (s_model_initialized != 0U)
    {
        s_result.state = BSP_ROAD_CLASSIFICATION_WARMUP;
    }
    s_result.class_id = BSP_ROAD_CLASS_UNKNOWN;
    for (i = 0U; i < BSP_ROAD_CLASSIFICATION_PROBABILITY_SLOTS; i++)
    {
        s_result.probabilities[i] = 0.0f;
    }
    BSP_RoadClassification_CopyClassName(s_result.class_name, s_result.class_id);
    s_result.collected_samples = s_collected_samples;
    s_result.has_result = 0U;

    BSP_RoadClassification_UpdateDebugSnapshot();

    BSP_RoadClassification_ExitCritical(primask);
}

/*
 * 采样间断后不保留半窗数据。重置只影响 AI 自己的输入缓存，不会反向影响
 * JY901S、底盘控制、PID 积分项或电机 PWM。
 */
static void BSP_RoadClassification_ResetWindow(void)
{
    /*
     * 只丢弃 AI 自己尚未送入推理的半窗；保留最近已消费的 ACC/GYRO 序号，
     * 因而串口恢复时必须先收到两组真正的新帧，绝不会把掉线前的快照重放。
     */
    s_collected_samples = 0U;
    s_last_collected_imu_tick = 0U;
    memset(s_input_signal, 0, sizeof(s_input_signal));
    BSP_RoadClassification_PublishWarmup();
}

static void BSP_RoadClassification_PublishError(int32_t neai_status)
{
    uint32_t primask = BSP_RoadClassification_EnterCritical();
    uint32_t i;

    s_result.state = BSP_ROAD_CLASSIFICATION_ERROR;
    s_result.class_id = BSP_ROAD_CLASS_UNKNOWN;
    for (i = 0U; i < BSP_ROAD_CLASSIFICATION_PROBABILITY_SLOTS; i++)
    {
        s_result.probabilities[i] = 0.0f;
    }
    BSP_RoadClassification_CopyClassName(s_result.class_name, s_result.class_id);
    s_result.last_neai_status = neai_status;
    s_result.collected_samples = s_collected_samples;
    s_result.has_result = 0U;

    BSP_RoadClassification_UpdateDebugSnapshot();

    BSP_RoadClassification_ExitCritical(primask);
}

static void BSP_RoadClassification_PublishResult(int class_id,
                                                  const float *probabilities,
                                                  uint32_t classification_tick)
{
    uint32_t primask = BSP_RoadClassification_EnterCritical();
    uint32_t i;

    s_result.state = BSP_ROAD_CLASSIFICATION_READY;
    s_result.class_id = (BSP_RoadClassificationClass_t)class_id;
    for (i = 0U; i < BSP_ROAD_CLASSIFICATION_CLASS_COUNT; i++)
    {
        s_result.probabilities[i] = probabilities[i];
    }
    /* 兼容原有四槽 Watch；下标 2~3 不是新模型输出，必须始终明确清零。 */
    for (; i < BSP_ROAD_CLASSIFICATION_PROBABILITY_SLOTS; i++)
    {
        s_result.probabilities[i] = 0.0f;
    }
    BSP_RoadClassification_CopyClassName(s_result.class_name, s_result.class_id);
    s_result.last_classification_tick = classification_tick;
    s_result.last_neai_status = (int32_t)NEAI_OK;
    s_result.collected_samples = 0U;
    s_result.has_result = 1U;

    BSP_RoadClassification_UpdateDebugSnapshot();

    BSP_RoadClassification_ExitCritical(primask);
}

uint8_t BSP_RoadClassification_Init(void)
{
    enum neai_state neai_state;
    const char *model_id;
    const char *model_class_name;
    uint32_t i;

    memset(s_input_signal, 0, sizeof(s_input_signal));
    memset(&s_result, 0, sizeof(s_result));
    memset(s_latest_imu_sample, 0, sizeof(s_latest_imu_sample));
    s_result.state = BSP_ROAD_CLASSIFICATION_UNINITIALIZED;
    s_result.class_id = BSP_ROAD_CLASS_UNKNOWN;
    BSP_RoadClassification_CopyClassName(s_result.class_name, s_result.class_id);
    s_result.last_neai_status = (int32_t)NEAI_NOT_INITIALIZED;
    s_last_collected_imu_tick = 0U;
    s_last_collected_acc_sequence = 0U;
    s_last_collected_gyro_sequence = 0U;
    s_collected_samples = 0U;
    s_model_initialized = 0U;

    /* 即使 NanoEdge 尚未初始化，Keil 也能立即看到清晰的“未初始化”状态。 */
    BSP_RoadClassification_UpdateDebugSnapshot();

    /* 加载 NanoEdge 预训练模型。失败时仅记录错误，调用者仍可继续运行底盘任务。 */
    neai_state = neai_classification_init();
    if (neai_state != NEAI_OK)
    {
        BSP_RoadClassification_PublishError((int32_t)neai_state);
        return 0U;
    }

    /*
     * 二次校验可防止日后误替换为其它 NanoEdge 导出包后发生缓冲区越界或类别错标。
     * 本次模型固定为 32 样本、6 轴、2 类，且模型 ID 必须与导入的 ZIP 一致。
     */
    model_id = neai_get_id();
    if ((neai_get_input_signal_size() != (int)BSP_ROAD_CLASSIFICATION_WINDOW_SAMPLES) ||
        (neai_get_axis_number() != (int)BSP_ROAD_CLASSIFICATION_AXIS_COUNT) ||
        (neai_get_number_of_classes() != (int)BSP_ROAD_CLASSIFICATION_CLASS_COUNT) ||
        (model_id == NULL) ||
        (strcmp(model_id, BSP_ROAD_CLASSIFICATION_EXPECTED_MODEL_ID) != 0))
    {
        BSP_RoadClassification_PublishError((int32_t)NEAI_INVALID_PARAM);
        return 0U;
    }

    /*
     * 原始类名也是模型契约的一部分。这样即使某个未来模型同样恰好是
     * “32 x 6、2 类”，也不会被错误解释成当前室外/室内模型。
     */
    for (i = 0U; i < BSP_ROAD_CLASSIFICATION_CLASS_COUNT; i++)
    {
        model_class_name = neai_get_class_name((int)i);
        if ((model_class_name == NULL) ||
            (strcmp(model_class_name, s_expected_model_class_names[i]) != 0))
        {
            BSP_RoadClassification_PublishError((int32_t)NEAI_INVALID_PARAM);
            return 0U;
        }
    }

    s_model_initialized = 1U;
    BSP_RoadClassification_PublishWarmup();
    return 1U;
}

void BSP_RoadClassification_Process(const JY901S_Data_t *imu_data, uint8_t vehicle_moving)
{
    uint32_t now;
    uint32_t input_offset;
    float probabilities[NEAI_NUMBER_OF_CLASSES];
    int class_id = -1;
    enum neai_state neai_state;

    if ((s_model_initialized == 0U) || (imu_data == NULL))
    {
        return;
    }

    now = HAL_GetTick();

    /*
     * 停车期间不收集振动样本，也不保留停车前的类别。持续吞掉当前序号，确保重新
     * 起步后必须等到 ACC/GYRO 都出现真正的新帧才开始一个全新的 32 帧窗口。
     */
    if (vehicle_moving == 0U)
    {
        s_last_collected_acc_sequence = imu_data->acc_update_sequence;
        s_last_collected_gyro_sequence = imu_data->gyro_update_sequence;
        if (s_collected_samples != 0U || s_result.state == BSP_ROAD_CLASSIFICATION_READY)
        {
            BSP_RoadClassification_ResetWindow();
        }
        return;
    }

    /*
     * 调度频率为 10 ms，IMU 实际帧率约 9~10 Hz。若串口掉线或长期未更新，
     * 立即丢弃半窗并回到预热状态，避免跨越长空档拼出一个伪时序窗口。
     */
    if ((imu_data->online == 0U) || (imu_data->last_update_tick == 0U) ||
        ((uint32_t)(now - imu_data->last_update_tick) > BSP_ROAD_CLASSIFICATION_MAX_SAMPLE_GAP_MS))
    {
        if (s_collected_samples != 0U || s_result.state == BSP_ROAD_CLASSIFICATION_READY)
        {
            BSP_RoadClassification_ResetWindow();
        }
        return;
    }

    /*
     * JY901S 的 ACC、GYRO、角度等属于不同 UART 帧，last_update_tick 每到一类
     * 帧都会变化。训练 logger 的规则则是“ACC 与 GYRO 自上次写样本后都更新过”
     * 才生成一行。因此仅比较 tick 会把同一物理采样拆成两行，导致窗口时序和
     * 训练集不一致；这里用两个只读序号完成严格配对。
     *
     * DMA 一次送入多帧时，两个序号可能在同一个 HAL tick 内同时变化；序号而非
     * tick 作为去重依据，正好避免毫秒分辨率造成的漏样本。
     */
    if ((imu_data->acc_update_sequence == s_last_collected_acc_sequence) ||
        (imu_data->gyro_update_sequence == s_last_collected_gyro_sequence))
    {
        /*
         * 即使其他 IMU 帧仍在到达，只要没有新的六轴配对样本，就不能继续把旧
         * 识别结果当作新数据。超过上限后回到预热，不写入任何底盘控制量。
         */
        if ((s_last_collected_imu_tick != 0U) &&
            ((uint32_t)(now - s_last_collected_imu_tick) >
             BSP_ROAD_CLASSIFICATION_MAX_SAMPLE_GAP_MS))
        {
            BSP_RoadClassification_ResetWindow();
        }
        return;
    }

    /* 两条有效六轴样本之间出现异常间隔时，先丢弃旧窗口，再从当前样本重新预热。 */
    if ((s_last_collected_imu_tick != 0U) &&
        ((uint32_t)(imu_data->last_update_tick - s_last_collected_imu_tick) >
         BSP_ROAD_CLASSIFICATION_MAX_SAMPLE_GAP_MS))
    {
        BSP_RoadClassification_ResetWindow();
    }

    /*
     * 只有在确认两组数据均为新帧后才提交其序号。后续 100 Hz 循环即使重复看到
     * 相同快照，也会在上面的配对判断处返回，不会重复污染 32 帧输入窗口。
     */
    s_last_collected_acc_sequence = imu_data->acc_update_sequence;
    s_last_collected_gyro_sequence = imu_data->gyro_update_sequence;
    s_last_collected_imu_tick = imu_data->last_update_tick;

    /* 防御性保护：正常流程在第 32 帧推理后会清零，异常状态也不能越界写缓冲。 */
    if (s_collected_samples >= BSP_ROAD_CLASSIFICATION_WINDOW_SAMPLES)
    {
        BSP_RoadClassification_ResetWindow();
    }
    input_offset = (uint32_t)s_collected_samples * BSP_ROAD_CLASSIFICATION_AXIS_COUNT;

    /*
     * 与原始训练 CSV 完全一致的单位和轴序：acc 已是 g，gyro 已是 deg/s。
     * 此处没有坐标变换、归一化或滤波，NanoEdge 导出库会执行训练时固化的特征提取。
     */
    s_input_signal[input_offset + 0U] = imu_data->acc[0];
    s_input_signal[input_offset + 1U] = imu_data->acc[1];
    s_input_signal[input_offset + 2U] = imu_data->acc[2];
    s_input_signal[input_offset + 3U] = imu_data->gyro[0];
    s_input_signal[input_offset + 4U] = imu_data->gyro[1];
    s_input_signal[input_offset + 5U] = imu_data->gyro[2];

    /* 单独保存最新一帧，供 Keil 验证实际进入 NanoEdge 的轴序和物理单位。 */
    s_latest_imu_sample[0] = imu_data->acc[0];
    s_latest_imu_sample[1] = imu_data->acc[1];
    s_latest_imu_sample[2] = imu_data->acc[2];
    s_latest_imu_sample[3] = imu_data->gyro[0];
    s_latest_imu_sample[4] = imu_data->gyro[1];
    s_latest_imu_sample[5] = imu_data->gyro[2];

    s_collected_samples++;
    {
        uint32_t primask = BSP_RoadClassification_EnterCritical();
        s_result.last_imu_tick = imu_data->last_update_tick;
        s_result.collected_samples = s_collected_samples;
        BSP_RoadClassification_UpdateDebugSnapshot();
        BSP_RoadClassification_ExitCritical(primask);
    }

    if (s_collected_samples < BSP_ROAD_CLASSIFICATION_WINDOW_SAMPLES)
    {
        return;
    }

    /*
     * 推理仅在已有 chassis_task 内同步执行。其输出只写入本 BSP 的结果快照；
     * 不调用任何电机/PID/模式切换函数，因此不会改变原有验证通过的底盘行为。
     */
    neai_state = neai_classification(s_input_signal, probabilities, &class_id);
    s_collected_samples = 0U;

    if ((neai_state == NEAI_OK) && (class_id >= 0) &&
        (class_id < (int)BSP_ROAD_CLASSIFICATION_CLASS_COUNT))
    {
        BSP_RoadClassification_PublishResult(class_id, probabilities, now);
    }
    else
    {
        /* 返回 NEAI_OK 却给出越界类别同样属于库输出异常，不能当作有效识别结果。 */
        BSP_RoadClassification_PublishError((int32_t)((neai_state == NEAI_OK) ?
                                                      NEAI_INVALID_PARAM : neai_state));
    }
}

void BSP_RoadClassification_GetResult(BSP_RoadClassificationResult_t *result)
{
    uint32_t primask;

    if (result == NULL)
    {
        return;
    }

    primask = BSP_RoadClassification_EnterCritical();
    *result = s_result;
    BSP_RoadClassification_ExitCritical(primask);
}

const char *BSP_RoadClassification_GetClassName(BSP_RoadClassificationClass_t class_id)
{
    switch (class_id)
    {
        case BSP_ROAD_CLASS_OUTDOOR:
            return "outdoor";
        case BSP_ROAD_CLASS_INDOOR:
            return "indoor";
        default:
            return "unknown";
    }
}
