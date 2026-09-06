/*
 * imu_fusion.h
 */

#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "mpu9250.h"
#include <stdint.h>
#include "stm32f7xx_hal.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;

    float gyro_bias[3];
    float alpha;

    uint32_t last_us;
    uint8_t  first_update;

    float stationary_gyro_thresh_dps;
    float stationary_accel_tol_g;
} IMU_Fusion_t;

void Fusion_Init(IMU_Fusion_t *f);

HAL_StatusTypeDef Fusion_CalibrateGyroBias(IMU_Fusion_t *f, MPU9250_HandleTypeDef *mpu,
                                            uint16_t samples);

HAL_StatusTypeDef Fusion_Update(IMU_Fusion_t *f, MPU9250_HandleTypeDef *mpu, uint32_t current_us);

void Fusion_NotifyBusReset(IMU_Fusion_t *f);

float Fusion_GetReportedRoll(const IMU_Fusion_t *f);
float Fusion_GetReportedPitch(const IMU_Fusion_t *f);
float Fusion_GetReportedYaw(const IMU_Fusion_t *f);

#endif /* IMU_FUSION_H */
