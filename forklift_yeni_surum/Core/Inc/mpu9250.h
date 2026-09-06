/*
 * mpu9250.h
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include "stm32f7xx_hal.h"

#define MPU9250_ADDR        (0x68 << 1)

#define MPU9250_REG_SMPLRT_DIV     0x19
#define MPU9250_REG_CONFIG         0x1A
#define MPU9250_REG_GYRO_CONFIG    0x1B
#define MPU9250_REG_ACCEL_CONFIG   0x1C
#define MPU9250_REG_ACCEL_CONFIG2  0x1D
#define MPU9250_REG_PWR_MGMT_1     0x6B
#define MPU9250_REG_PWR_MGMT_2     0x6C
#define MPU9250_REG_WHO_AM_I       0x75
#define MPU9250_REG_ACCEL_XOUT_H   0x3B

#define MPU9250_WHO_AM_I_VAL       0x71

typedef enum {
    GYRO_FS_250DPS  = 0x00,
    GYRO_FS_500DPS  = 0x08,
    GYRO_FS_1000DPS = 0x10,
    GYRO_FS_2000DPS = 0x18
} MPU9250_GyroFS;

typedef enum {
    ACCEL_FS_2G  = 0x00,
    ACCEL_FS_4G  = 0x08,
    ACCEL_FS_8G  = 0x10,
    ACCEL_FS_16G = 0x18
} MPU9250_AccelFS;

typedef struct {
    I2C_HandleTypeDef *hi2c;

    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;

    float accel_g[3];
    float gyro_dps[3];
    float temp_c;

    float accel_sens;
    float gyro_sens;
} MPU9250_HandleTypeDef;

HAL_StatusTypeDef MPU9250_Init(MPU9250_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c,
                                MPU9250_AccelFS afs, MPU9250_GyroFS gfs);

HAL_StatusTypeDef MPU9250_CheckConnection(MPU9250_HandleTypeDef *dev);

HAL_StatusTypeDef MPU9250_ReadAll(MPU9250_HandleTypeDef *dev);

HAL_StatusTypeDef MPU9250_ReadAccel(MPU9250_HandleTypeDef *dev);

HAL_StatusTypeDef MPU9250_ReadGyro(MPU9250_HandleTypeDef *dev);

void MPU9250_I2C_Hard_Reset(I2C_HandleTypeDef *hi2c);

#endif /* MPU9250_H */
