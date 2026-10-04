#ifndef __QMC5883_H
#define __QMC5883_H

#include "stm32h7xx_hal.h"
#include <math.h>

/* I2C 7-bit addr = 0x0D. HAL requires left-shift (8-bit addr), hence 0x1A */
#define QMC5883_ADDR 0x1A

typedef void (*QMC5883_CalibrationStepCallback_t)(uint32_t elapsed_ms);

typedef struct {
    float yaw;      /* Yaw angle (0~360 deg) */
    float pitch;    /* Pitch angle (-180~180 deg) */
    float roll;     /* Roll angle (-180~180 deg) */
    uint32_t last_update_tick;  /* Last update timestamp */
} EulerAngles;

typedef struct {
    float offset_x, offset_y, offset_z;  /* Hard-iron offset */
    float scale_x, scale_y, scale_z;      /* Soft-iron scale */
} CalibParams;

/*
 * Keil Watch-only mirror for the QMC5883 receive and calculation path.
 * The application never reads this structure, so changing it cannot affect
 * navigation, calibration, chassis control, PID, or motor output.
 */
typedef struct {
    uint32_t update_sequence;
    uint32_t read_count;
    uint32_t read_ok_count;
    uint32_t read_error_count;
    uint32_t last_read_tick;
    uint32_t last_success_tick;
    uint32_t last_angle_tick;
    uint32_t last_i2c_error;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    uint8_t last_hal_status;          /* 0=HAL_OK, 1=ERROR, 2=BUSY, 3=TIMEOUT */
    uint8_t online;                  /* Latest raw read returned HAL_OK */
    uint8_t calibrating;
    uint8_t reserved0;
    uint16_t calibration_remaining_s;
    uint16_t reserved1;
    float calibrated_x;
    float calibrated_y;
    float calibrated_z;
    float yaw;
    float pitch;
    float roll;
    CalibParams calibration;
} QMC5883_Debug_t;

/* Initialize QMC5883 hardware only (registers + safe defaults, no calibration). */
void QMC5883_InitHW(void);

/* Initialize QMC5883: configure registers and run 30-second calibration. */
void QMC5883_Init(void);

/* Register an optional 10 ms step callback used during the 30-second calibration. */
void QMC5883_SetCalibrationStepCallback(QMC5883_CalibrationStepCallback_t callback);

/* Read raw 16-bit ADC counts for X/Y/Z axes via I2C. */
void QMC5883_ReadRawData(int16_t *x, int16_t *y, int16_t *z);

/* Compute yaw/pitch/roll from calibrated magnetometer readings. */
void QMC5883_GetAngles(EulerAngles *angles);

/* Run hard-iron calibration: rotate sensor through all axes for ~30 s. */
void Magnetometer_Calibration(void);

/* Read calibrated magnetometer data (offset removed, scale normalized). */
void QMC5883_Get_CalibrationData(float *hx, float *hy, float *hz);

extern volatile uint8_t qmc5883_calibrating;
extern volatile uint16_t qmc5883_calibration_remaining_s;
extern volatile QMC5883_Debug_t g_qmc5883_debug;

#endif
