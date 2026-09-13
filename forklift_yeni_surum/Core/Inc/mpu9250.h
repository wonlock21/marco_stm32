/*
 * mpu9250.h
 *
 * MPU9250 (ivme + jiroskop) I2C sürücüsü
 * STM32 Nucleo F446RE + HAL kütüphanesi için yazılmıştır.
 *
 * Not: Bu sürücü MPU9250'nin ivmeölçer (accel) ve jiroskop (gyro)
 * kısmını okur. Manyetometre (AK8963) ayrı bir I2C adresinde olduğu
 * için burada dahil edilmedi; istersen sonradan ekleyebiliriz.
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include "stm32f7xx_hal.h"

/* ---------- I2C Adresi ---------- */
/* AD0 pini GND'ye bağlıysa 0x68, VCC'ye bağlıysa 0x69 kullanılır.
 * HAL fonksiyonları 8-bit adres beklediği için 1 bit sola kaydırıyoruz. */
#define MPU9250_ADDR        (0x68 << 1)

/* ---------- Register Adresleri ---------- */
#define MPU9250_REG_SMPLRT_DIV     0x19
#define MPU9250_REG_CONFIG         0x1A
#define MPU9250_REG_GYRO_CONFIG    0x1B
#define MPU9250_REG_ACCEL_CONFIG   0x1C
#define MPU9250_REG_ACCEL_CONFIG2  0x1D
#define MPU9250_REG_PWR_MGMT_1     0x6B
#define MPU9250_REG_PWR_MGMT_2     0x6C
#define MPU9250_REG_WHO_AM_I       0x75
#define MPU9250_REG_ACCEL_XOUT_H   0x3B

/* WHO_AM_I beklenen değer (MPU9250 için 0x71, bazı klon/MPU6500 kartlarda 0x70/0x73 da görülebilir) */
#define MPU9250_WHO_AM_I_VAL       0x71

/* ---------- Ölçek Seçenekleri ---------- */
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

/* ---------- Veri Yapısı ---------- */
typedef struct {
    I2C_HandleTypeDef *hi2c;

    int16_t accel_raw[3];   /* X, Y, Z ham veri */
    int16_t gyro_raw[3];
    int16_t temp_raw;

    float accel_g[3];       /* g cinsinden (1g = 9.81 m/s^2) */
    float gyro_dps[3];      /* derece/saniye cinsinden */
    float temp_c;           /* santigrat derece */

    float accel_sens;       /* seçilen FS'ye göre LSB/g */
    float gyro_sens;        /* seçilen FS'ye göre LSB/(deg/s) */
} MPU9250_HandleTypeDef;

/* ---------- Fonksiyonlar ---------- */

/* Sensörü başlatır. Dönüş: HAL_OK basarili, diğerleri hata */
HAL_StatusTypeDef MPU9250_Init(MPU9250_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c,
                                MPU9250_AccelFS afs, MPU9250_GyroFS gfs);

/* WHO_AM_I register'ini okuyup sensörün cevap verip vermediğini kontrol eder */
HAL_StatusTypeDef MPU9250_CheckConnection(MPU9250_HandleTypeDef *dev);

/* Accel + gyro + sıcaklığı tek seferde okur (14 byte burst read) */
HAL_StatusTypeDef MPU9250_ReadAll(MPU9250_HandleTypeDef *dev);

/* Sadece ivme okur */
HAL_StatusTypeDef MPU9250_ReadAccel(MPU9250_HandleTypeDef *dev);

/* Sadece jiroskop okur */
HAL_StatusTypeDef MPU9250_ReadGyro(MPU9250_HandleTypeDef *dev);

void MPU9250_I2C_Hard_Reset(I2C_HandleTypeDef *hi2c);

#endif /* MPU9250_H */
