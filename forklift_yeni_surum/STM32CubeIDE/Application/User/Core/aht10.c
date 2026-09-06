/**
  * @file    aht10.c
  * @brief   AHT10 Sürücü Kütüphanesi Kaynak Kodları (216 MHz Optimize Edilmiş)
  */

#include "aht10.h"

/**
  * @brief  216 MHz CPU Frekansına Göre Optimize Edilmiş Mikro saniye Gecikme Fonksiyonu
  */
__weak void delay_us(uint32_t us) {
    // 216 MHz işlemci saatinde döngü başına düşen clock maliyeti hesabı
    volatile uint32_t count = us * (216000000 / 1000000 / 4);
    while(count--);
}

void AHT10_Init_Struct(AHT10_HandleTypedef *dev, I2C_HandleTypeDef *hi2c, GPIO_TypeDef *port, uint16_t sda_pin, uint16_t scl_pin, uint32_t af) {
    dev->hi2c = hi2c;
    dev->i2c_port = port;
    dev->sda_pin = sda_pin;
    dev->scl_pin = scl_pin;
    dev->i2c_af = af;
    dev->error_count = 0;
    dev->is_ready = false;
}

HAL_StatusTypeDef AHT10_Init(AHT10_HandleTypedef *dev) {
    uint8_t init_cmd[3] = {0xE1, 0x08, 0x00};
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, AHT10_ADDR, init_cmd, 3, 100);
    HAL_Delay(20);
    if (status == HAL_OK) {
        dev->is_ready = true;
    }
    return status;
}

void AHT10_Read(AHT10_HandleTypedef *dev, volatile float *Temperature, volatile float *Humidity) {
    uint8_t measure_cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t rx_data[6];
    uint32_t raw_temp = 0, raw_hum = 0;

    if (HAL_I2C_Master_Transmit(dev->hi2c, AHT10_ADDR, measure_cmd, 3, 50) != HAL_OK) {
        dev->error_count++;
        return;
    }

    HAL_Delay(80); // AHT10 ölçüm dönüş süresi

    if (HAL_I2C_Master_Receive(dev->hi2c, AHT10_ADDR, rx_data, 6, 50) == HAL_OK) {
        if ((rx_data[0] & 0x80) == 0) {
            raw_hum = ((uint32_t)rx_data[1] << 12) | ((uint32_t)rx_data[2] << 4) | (rx_data[3] >> 4);
            *Humidity = ((float)raw_hum * 100.0f) / 1048576.0f;

            raw_temp = (((uint32_t)(rx_data[3] & 0x0F)) << 16) | ((uint32_t)rx_data[4] << 8) | rx_data[5];
            *Temperature = (((float)raw_temp * 200.0f) / 1048576.0f) - 50.0f;

            // Başarılı okuma, hata sayacını sıfırla
            dev->error_count = 0;
        } else {
            dev->error_count++;
        }
    } else {
        dev->error_count++;
    }
}

/**
  * @brief  Mikro saniye (us) seviyesinde çalışan hızlı I2C Hard Reset fonksiyonu
  */
void AHT10_I2C_Hard_Reset(AHT10_HandleTypedef *dev) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. I2C modülünü devre dışı bırak
    __HAL_I2C_DISABLE(dev->hi2c);
    delay_us(5);

    // 2. Pinleri I2C modundan çıkarıp standart Output (Open Drain) yap
    if (dev->i2c_port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (dev->i2c_port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } // İhtiyaca göre diğer portlar eklenebilir

    GPIO_InitStruct.Pin = dev->scl_pin | dev->sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(dev->i2c_port, &GPIO_InitStruct);

    // Başlangıçta SDA ve SCL pinlerini High durumuna çek
    HAL_GPIO_WritePin(dev->i2c_port, dev->scl_pin, GPIO_PIN_SET); // SCL
    HAL_GPIO_WritePin(dev->i2c_port, dev->sda_pin, GPIO_PIN_SET); // SDA
    delay_us(2);

    // 3. 9 Adet Clock darbesi (us seviyesinde çok hızlı tur)
    for(int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(dev->i2c_port, dev->scl_pin, GPIO_PIN_RESET);
        delay_us(2);
        HAL_GPIO_WritePin(dev->i2c_port, dev->scl_pin, GPIO_PIN_SET);
        delay_us(2);
    }

    // 4. Manuel STOP Condition Sinyali
    HAL_GPIO_WritePin(dev->i2c_port, dev->sda_pin, GPIO_PIN_RESET); // SDA = 0
    delay_us(2);
    HAL_GPIO_WritePin(dev->i2c_port, dev->scl_pin, GPIO_PIN_SET);   // SCL = 1
    delay_us(2);
    HAL_GPIO_WritePin(dev->i2c_port, dev->sda_pin, GPIO_PIN_SET);   // SDA = 1 (STOP)
    delay_us(2);

    // 5. Pinleri tekrar I2C Alternate Function moduna bağla
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = dev->i2c_af;
    HAL_GPIO_Init(dev->i2c_port, &GPIO_InitStruct);

    // 6. I2C modülünü yeniden etkinleştir
    __HAL_I2C_ENABLE(dev->hi2c);
    delay_us(5);

    // 7. HAL kütüphanesinin yazılımsal durum kilitlerini anında aç (Ağır init kaldırıldı)
    dev->hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    dev->hi2c->State = HAL_I2C_STATE_READY;
    dev->hi2c->PreviousState = HAL_I2C_MODE_NONE;
    dev->hi2c->Mode = HAL_I2C_MODE_NONE;

    dev->is_ready = true;
}

/**
  * @brief  Ana döngüde çağrılacak süreç ve hata yönetimi
  */
void AHT10_Process(AHT10_HandleTypedef *dev, volatile float *Temperature, volatile float *Humidity) {
    if (dev->error_count >= 2) {
        dev->error_count = 0;

        // Donanımsal I2C hattını temizle (9 clock pulse + stop)
        AHT10_I2C_Hard_Reset(dev);

        // Ağır init atlandı, doğrudan hazır bayrağı güncellendi
        dev->is_ready = true;
    }

    if (dev->is_ready) {
        AHT10_Read(dev, Temperature, Humidity);
    } else {
        if (HAL_I2C_IsDeviceReady(dev->hi2c, AHT10_ADDR, 3, 50) == HAL_OK) {
            AHT10_Init(dev);
        } else {
            dev->error_count++;
        }
    }
}
