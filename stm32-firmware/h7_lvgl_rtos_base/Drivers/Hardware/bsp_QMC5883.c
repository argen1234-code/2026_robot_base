#include "bsp_QMC5883.h"


extern I2C_HandleTypeDef hi2c1; 

CalibParams params;
volatile uint8_t qmc5883_calibrating = 0U;
volatile uint16_t qmc5883_calibration_remaining_s = 0U;
volatile QMC5883_Debug_t g_qmc5883_debug;
static QMC5883_CalibrationStepCallback_t qmc5883_calibration_step_callback = 0;

#define RAD_TO_DEG  (180.0 / 3.14159265358979323846)

static void QMC5883_DebugSyncCalibration(void)
{
    g_qmc5883_debug.update_sequence++;
    g_qmc5883_debug.calibrating = qmc5883_calibrating;
    g_qmc5883_debug.calibration_remaining_s = qmc5883_calibration_remaining_s;
    g_qmc5883_debug.calibration.offset_x = params.offset_x;
    g_qmc5883_debug.calibration.offset_y = params.offset_y;
    g_qmc5883_debug.calibration.offset_z = params.offset_z;
    g_qmc5883_debug.calibration.scale_x = params.scale_x;
    g_qmc5883_debug.calibration.scale_y = params.scale_y;
    g_qmc5883_debug.calibration.scale_z = params.scale_z;
    g_qmc5883_debug.update_sequence++;
}

void QMC5883_SetCalibrationStepCallback(QMC5883_CalibrationStepCallback_t callback)
{
    qmc5883_calibration_step_callback = callback;
}

/**
 * Initialize QMC5883 hardware only (registers + safe defaults, no calibration).
 * Call at power-up so magnetic data is available on the GUI immediately.
 * Defer Magnetometer_Calibration() until the vehicle is ready to rotate.
 */
void QMC5883_InitHW(void)
{
    uint8_t data;

    data = 0x0D;
    HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x09, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x0B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x21, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    /* Safe defaults before calibration completes */
    params.offset_x = 0; params.offset_y = 0; params.offset_z = 0;
    params.scale_x = 1.0f; params.scale_y = 1.0f; params.scale_z = 1.0f;
    QMC5883_DebugSyncCalibration();
}

/**
 * Initialize QMC5883 (register config + 30-second calibration).
 * Legacy wrapper calling InitHW + Magnetometer_Calibration.
 */
void QMC5883_Init(void)
{
    QMC5883_InitHW();
    Magnetometer_Calibration();  /* Start 30s calibration */
}

/**
 * Read raw 3-axis magnetometer data from the QMC5883.
 * Reads 6 consecutive bytes from data register 0x00 via I2C.
 * Output values are 16-bit signed integers in ADC counts.
 *
 * @param x  Pointer to store raw X-axis value.
 * @param y  Pointer to store raw Y-axis value.
 * @param z  Pointer to store raw Z-axis value.
 */
void QMC5883_ReadRawData(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
    HAL_StatusTypeDef hal_status;
    uint32_t now;

    hal_status = HAL_I2C_Mem_Read(&hi2c1, QMC5883_ADDR, 0x00,
                                 I2C_MEMADD_SIZE_8BIT, buf, 6, 100);
    now = HAL_GetTick();

    *x = (int16_t)(buf[1] << 8 | buf[0]);
    *y = (int16_t)(buf[3] << 8 | buf[2]);
    *z = (int16_t)(buf[5] << 8 | buf[4]);

    g_qmc5883_debug.update_sequence++;
    g_qmc5883_debug.read_count++;
    g_qmc5883_debug.last_hal_status = (uint8_t)hal_status;
    g_qmc5883_debug.last_i2c_error = hi2c1.ErrorCode;
    g_qmc5883_debug.last_read_tick = now;
    g_qmc5883_debug.calibrating = qmc5883_calibrating;
    g_qmc5883_debug.calibration_remaining_s = qmc5883_calibration_remaining_s;
    if (hal_status == HAL_OK)
    {
        g_qmc5883_debug.read_ok_count++;
        g_qmc5883_debug.last_success_tick = now;
        g_qmc5883_debug.raw_x = *x;
        g_qmc5883_debug.raw_y = *y;
        g_qmc5883_debug.raw_z = *z;
        g_qmc5883_debug.online = 1U;
    }
    else
    {
        g_qmc5883_debug.read_error_count++;
        g_qmc5883_debug.online = 0U;
    }
    g_qmc5883_debug.update_sequence++;
}

/**
 * Perform magnetometer hard-iron calibration.
 * The sensor should be rotated through all axes during the ~30-second
 * routine.  Records per-axis min/max values across 1000 samples, then
 * computes offset (center) and scale correction factors.  A calibration
 * LED blinks every 300 ms while active.
 */
void Magnetometer_Calibration(void)
{
    static int16_t min_x = 32767;  /* Init to max value */
    static int16_t max_x = -32768; /* Init to min value */
    static int16_t min_y = 32767;
    static int16_t max_y = -32768;
    static int16_t min_z = 32767;
    static int16_t max_z = -32768;

    int16_t temp_hx, temp_hy, temp_hz;
    uint8_t control_step;
    uint32_t elapsed_ms;
    
    /* Rotate in place or figure-8 to record full-range magnetic min/max */
    qmc5883_calibrating = 1U;
    qmc5883_calibration_remaining_s = 30U;

    for (int i = 0; i < 1000; i++)
    {
        qmc5883_calibration_remaining_s = (uint16_t)(((1000 - i) * 30 + 999) / 1000);
        QMC5883_ReadRawData(&temp_hx, &temp_hy, &temp_hz);

        if (temp_hx < min_x) min_x = temp_hx;
        if (temp_hx > max_x) max_x = temp_hx;
        if (temp_hy < min_y) min_y = temp_hy;
        if (temp_hy > max_y) max_y = temp_hy;
        if (temp_hz < min_z) min_z = temp_hz;
        if (temp_hz > max_z) max_z = temp_hz;

    /* Calibration done, LED off */
        if (i % 10 == 0) 
        {
#if defined(LED_GPIO_Port) && defined(LED_Pin)
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
#else
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
#endif
        }

        for (control_step = 0U; control_step < 3U; control_step++)
        {
            elapsed_ms = (uint32_t)i * 30U + (uint32_t)control_step * 10U;
            if (qmc5883_calibration_step_callback != 0)
            {
                qmc5883_calibration_step_callback(elapsed_ms);
            }
            HAL_Delay(10);
        }
    }

    /* Calibration done, LED off */
#if defined(LED_GPIO_Port) && defined(LED_Pin)
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); 
#else
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); 
#endif

    qmc5883_calibrating = 0U;
    qmc5883_calibration_remaining_s = 0U;

    params.offset_x = (max_x + min_x) / 2.0f;
    params.offset_y = (max_y + min_y) / 2.0f;
    params.offset_z = (max_z + min_z) / 2.0f;

    float scale_x = (max_x - min_x) / 2.0f;
    float scale_y = (max_y - min_y) / 2.0f;
    float scale_z = (max_z - min_z) / 2.0f;

    float max_scale = scale_x;
    if (scale_y > max_scale) max_scale = scale_y;
    if (scale_z > max_scale) max_scale = scale_z;

    /* Avoid divide-by-zero */
    if(max_scale > 0) {
        params.scale_x = scale_x / max_scale;
        params.scale_y = scale_y / max_scale;
        params.scale_z = scale_z / max_scale;
    }

    QMC5883_DebugSyncCalibration();
}

/**
 * Read magnetometer data with calibration applied.
 * Reads raw ADC values, subtracts hard-iron offset, and normalizes by
 * the per-axis scale factor computed during calibration.
 *
 * @param hx  Pointer to store calibrated X-axis value (normalized).
 * @param hy  Pointer to store calibrated Y-axis value (normalized).
 * @param hz  Pointer to store calibrated Z-axis value (normalized).
 */
void QMC5883_Get_CalibrationData(float *hx, float *hy, float *hz)
{
    int16_t temp_hx = 0;
    int16_t temp_hy = 0;
    int16_t temp_hz = 0;
    QMC5883_ReadRawData(&temp_hx, &temp_hy, &temp_hz);

    *hx = ((float)temp_hx - params.offset_x) * params.scale_x;
    *hy = ((float)temp_hy - params.offset_y) * params.scale_y;
    *hz = ((float)temp_hz - params.offset_z) * params.scale_z;

    g_qmc5883_debug.update_sequence++;
    g_qmc5883_debug.calibrated_x = *hx;
    g_qmc5883_debug.calibrated_y = *hy;
    g_qmc5883_debug.calibrated_z = *hz;
    g_qmc5883_debug.update_sequence++;
}

/**
 * Compute Euler angles from calibrated magnetometer data.
 * Reads calibrated field values, then calculates yaw (0--360 deg),
 * pitch (-180--+180 deg), and roll (-180--+180 deg) using arctan
 * formulas suitable for a 2-axis compass on a level platform.
 *
 * @param angles  Pointer to EulerAngles struct to fill.
 */
void QMC5883_GetAngles(EulerAngles *angles)
{
    static float x = 0;
    static float y = 0;
    static float z = 0;

    QMC5883_Get_CalibrationData(&x, &y, &z);

    /* Compute Yaw -- primary angle for planar motion */
    angles->yaw = atan2((double)y, (double)x) * RAD_TO_DEG;
    if(angles->yaw < 0) angles->yaw += 360.0f;

    /* Compute Pitch */
    angles->pitch = atan2((double)y, sqrt(x*x + z*z)) * RAD_TO_DEG;

    /* Compute Roll */
    angles->roll = atan2((double)x, sqrt(y*y + z*z)) * RAD_TO_DEG;
    angles->last_update_tick = HAL_GetTick();

    g_qmc5883_debug.update_sequence++;
    g_qmc5883_debug.yaw = angles->yaw;
    g_qmc5883_debug.pitch = angles->pitch;
    g_qmc5883_debug.roll = angles->roll;
    g_qmc5883_debug.last_angle_tick = angles->last_update_tick;
    g_qmc5883_debug.calibrating = qmc5883_calibrating;
    g_qmc5883_debug.calibration_remaining_s = qmc5883_calibration_remaining_s;
    g_qmc5883_debug.update_sequence++;

}




