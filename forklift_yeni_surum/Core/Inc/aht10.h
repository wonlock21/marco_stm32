/**
  * @file    aht10.h
  * @brief   AHT10 Hızlı Hot-Plug / Hard-Reset Destekli Sürücü Kütüphanesi
  */

#ifndef AHT10_H
#define AHT10_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define AHT10_ADDR (0x38 << 1)

/**
  * @brief  AHT10 Sürücü Durum ve Yapılandırma Yapısı (Handle)
  */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef      *i2c_port;
    uint16_t           sda_pin;
    uint16_t           scl_pin;
    uint32_t           i2c_af;

    uint8_t            error_count;  // Ardışık hata sayacı
    bool               is_ready;     // Sensör hazır bayrağı
} AHT10_HandleTypedef;

/* Fonksiyon Prototipleri */
void AHT10_Init_Struct(AHT10_HandleTypedef *dev, I2C_HandleTypeDef *hi2c, GPIO_TypeDef *port, uint16_t sda_pin, uint16_t scl_pin, uint32_t af);
HAL_StatusTypeDef AHT10_Init(AHT10_HandleTypedef *dev);
void AHT10_Read(AHT10_HandleTypedef *dev, volatile float *Temperature, volatile float *Humidity);
void AHT10_I2C_Hard_Reset(AHT10_HandleTypedef *dev);
void AHT10_Process(AHT10_HandleTypedef *dev, volatile float *Temperature, volatile float *Humidity);

#ifdef __cplusplus
}
#endif

#endif /* AHT10_H */
