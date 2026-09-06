/*
 * mpu9250.c
 */

#include "mpu9250.h"

#define I2C_TIMEOUT 3  /* ms */

static HAL_StatusTypeDef MPU9250_WriteReg(MPU9250_HandleTypeDef *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, I2C_TIMEOUT);
}

static HAL_StatusTypeDef MPU9250_ReadRegs(MPU9250_HandleTypeDef *dev, uint8_t reg,
                                           uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             buf, len, I2C_TIMEOUT);
}

HAL_StatusTypeDef MPU9250_CheckConnection(MPU9250_HandleTypeDef *dev)
{
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status = MPU9250_ReadRegs(dev, MPU9250_REG_WHO_AM_I, &who_am_i, 1);

    if (status != HAL_OK) {
        return status;
    }

    (void)who_am_i;
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_Init(MPU9250_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c,
                                MPU9250_AccelFS afs, MPU9250_GyroFS gfs)
{
    HAL_StatusTypeDef status;
    dev->hi2c = hi2c;

    status = MPU9250_CheckConnection(dev);
    if (status != HAL_OK) return status;

    status = MPU9250_WriteReg(dev, MPU9250_REG_PWR_MGMT_1, 0x80);
    if (status != HAL_OK) return status;
    HAL_Delay(20);

    status = MPU9250_WriteReg(dev, MPU9250_REG_PWR_MGMT_1, 0x01);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    status = MPU9250_WriteReg(dev, MPU9250_REG_PWR_MGMT_2, 0x00);
    if (status != HAL_OK) return status;

    status = MPU9250_WriteReg(dev, MPU9250_REG_SMPLRT_DIV, 0x09);
    if (status != HAL_OK) return status;

    status = MPU9250_WriteReg(dev, MPU9250_REG_CONFIG, 0x03);
    if (status != HAL_OK) return status;

    status = MPU9250_WriteReg(dev, MPU9250_REG_GYRO_CONFIG, gfs);
    if (status != HAL_OK) return status;

    status = MPU9250_WriteReg(dev, MPU9250_REG_ACCEL_CONFIG, afs);
    if (status != HAL_OK) return status;

    status = MPU9250_WriteReg(dev, MPU9250_REG_ACCEL_CONFIG2, 0x03);
    if (status != HAL_OK) return status;

    switch (afs) {
        case ACCEL_FS_2G:  dev->accel_sens = 16384.0f; break;
        case ACCEL_FS_4G:  dev->accel_sens = 8192.0f;  break;
        case ACCEL_FS_8G:  dev->accel_sens = 4096.0f;  break;
        case ACCEL_FS_16G: dev->accel_sens = 2048.0f;  break;
        default:           dev->accel_sens = 16384.0f; break;
    }

    switch (gfs) {
        case GYRO_FS_250DPS:  dev->gyro_sens = 131.0f;  break;
        case GYRO_FS_500DPS:  dev->gyro_sens = 65.5f;   break;
        case GYRO_FS_1000DPS: dev->gyro_sens = 32.8f;   break;
        case GYRO_FS_2000DPS: dev->gyro_sens = 16.4f;   break;
        default:              dev->gyro_sens = 131.0f;  break;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadAll(MPU9250_HandleTypeDef *dev)
{
    uint8_t buf[14];
    HAL_StatusTypeDef status = MPU9250_ReadRegs(dev, MPU9250_REG_ACCEL_XOUT_H, buf, 14);
    if (status != HAL_OK) return status;

    uint8_t all_zero = 1, all_ff = 1;
    for (int i = 0; i < 14; i++) {
        if (buf[i] != 0x00) all_zero = 0;
        if (buf[i] != 0xFF) all_ff = 0;
    }
    if (all_zero || all_ff) {
        return HAL_ERROR;
    }

    // --- FROZEN DATA DETECTOR (Kilitlenen Bayat Veri Tespiti) ---
    static uint8_t last_buf[14] = {0};
    static uint8_t frozen_count = 0;
    uint8_t exact_match = 1;

    for (int i = 0; i < 14; i++) {
        if (buf[i] != last_buf[i]) {
            exact_match = 0;
            break;
        }
    }

    if (exact_match) {
        frozen_count++;
        if (frozen_count > 10) { // 200 ms boyunca aynı bayat veri gelirse hata döndür
            frozen_count = 0;
            return HAL_ERROR;
        }
    } else {
        frozen_count = 0;
        for (int i = 0; i < 14; i++) last_buf[i] = buf[i];
    }
    // -----------------------------------------------------------

    dev->accel_raw[0] = (int16_t)(buf[0]  << 8 | buf[1]);
    dev->accel_raw[1] = (int16_t)(buf[2]  << 8 | buf[3]);
    dev->accel_raw[2] = (int16_t)(buf[4]  << 8 | buf[5]);

    dev->temp_raw      = (int16_t)(buf[6]  << 8 | buf[7]);

    dev->gyro_raw[0]  = (int16_t)(buf[8]  << 8 | buf[9]);
    dev->gyro_raw[1]  = (int16_t)(buf[10] << 8 | buf[11]);
    dev->gyro_raw[2]  = (int16_t)(buf[12] << 8 | buf[13]);

    for (int i = 0; i < 3; i++) {
        dev->accel_g[i]   = dev->accel_raw[i] / dev->accel_sens;
        dev->gyro_dps[i]  = dev->gyro_raw[i]  / dev->gyro_sens;
    }

    dev->temp_c = (dev->temp_raw / 333.87f) + 21.0f;

    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadAccel(MPU9250_HandleTypeDef *dev)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status = MPU9250_ReadRegs(dev, MPU9250_REG_ACCEL_XOUT_H, buf, 6);
    if (status != HAL_OK) return status;

    dev->accel_raw[0] = (int16_t)(buf[0] << 8 | buf[1]);
    dev->accel_raw[1] = (int16_t)(buf[2] << 8 | buf[3]);
    dev->accel_raw[2] = (int16_t)(buf[4] << 8 | buf[5]);

    for (int i = 0; i < 3; i++) {
        dev->accel_g[i] = dev->accel_raw[i] / dev->accel_sens;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadGyro(MPU9250_HandleTypeDef *dev)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status = MPU9250_ReadRegs(dev, 0x43, buf, 6);
    if (status != HAL_OK) return status;

    dev->gyro_raw[0] = (int16_t)(buf[0] << 8 | buf[1]);
    dev->gyro_raw[1] = (int16_t)(buf[2] << 8 | buf[3]);
    dev->gyro_raw[2] = (int16_t)(buf[4] << 8 | buf[5]);

    for (int i = 0; i < 3; i++) {
        dev->gyro_dps[i] = dev->gyro_raw[i] / dev->gyro_sens;
    }

    return HAL_OK;
}

static void delay_us(uint32_t us)
{
    us *= 54;
    while (us--) {
        __NOP();
    }
}

void MPU9250_I2C_Hard_Reset(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. STM32 I2C donanım modülünü tamamen resetle ve kapat
    HAL_I2C_DeInit(hi2c);
    delay_us(5);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_us(2);

    for(int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        delay_us(2);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        delay_us(2);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_us(2);

    // 2. Pinleri tekrar I2C alternatif fonksiyon (AF) moduna döndür
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. I2C donanımını HAL kütüphanesi ile yeniden başlat
    HAL_I2C_Init(hi2c);
}
