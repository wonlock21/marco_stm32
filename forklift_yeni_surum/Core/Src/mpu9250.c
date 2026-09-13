/*
 * mpu9250.c
 *
 * MPU9250 I2C sürücüsü - implementasyon
 */

#include "mpu9250.h"

#define I2C_TIMEOUT 100  /* ms */

/* Yardımcı: tek register yaz */
static HAL_StatusTypeDef MPU9250_WriteReg(MPU9250_HandleTypeDef *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, MPU9250_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1, I2C_TIMEOUT);
}

/* Yardımcı: birden fazla byte oku */
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

    /* Bazı klon kartlarda WHO_AM_I farklı çıkabiliyor, o yüzden sadece
     * HAL_OK dönmesini (yani cihazın I2C hattında cevap vermesini) yeterli sayıyoruz.
     * İstersen aşağıdaki satırı açıp tam eşleşme de zorunlu kılabilirsin: */
    // if (who_am_i != MPU9250_WHO_AM_I_VAL) return HAL_ERROR;

    (void)who_am_i;
    return HAL_OK;
}

HAL_StatusTypeDef MPU9250_Init(MPU9250_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c,
                                MPU9250_AccelFS afs, MPU9250_GyroFS gfs)
{
    HAL_StatusTypeDef status;
    dev->hi2c = hi2c;

    /* 1) Sensörle iletişim var mı kontrol et */
    status = MPU9250_CheckConnection(dev);
    if (status != HAL_OK) return status;

    /* 2) Reset + uyandır (PWR_MGMT_1: reset biti set edilip sensör resetlenir) */
    status = MPU9250_WriteReg(dev, MPU9250_REG_PWR_MGMT_1, 0x80);
    if (status != HAL_OK) return status;
    HAL_Delay(100); /* reset sonrası bekleme */

    /* 3) Uyku modundan çık, clock kaynağı olarak PLL/gyro seç (daha kararlı) */
    status = MPU9250_WriteReg(dev, MPU9250_REG_PWR_MGMT_1, 0x01);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    /* 4) Tüm eksenleri (accel + gyro) aktif tut */
    status = MPU9250_WriteReg(dev, MPU9250_REG_PWR_MGMT_2, 0x00);
    if (status != HAL_OK) return status;

    /* 5) Örnekleme hızı: 1kHz / (1 + SMPLRT_DIV) -> burada 100Hz için 9 seçildi */
    status = MPU9250_WriteReg(dev, MPU9250_REG_SMPLRT_DIV, 0x09);
    if (status != HAL_OK) return status;

    /* 6) DLPF (alçak geçiren filtre) ayarı - gürültüyü azaltmak için */
    status = MPU9250_WriteReg(dev, MPU9250_REG_CONFIG, 0x03);
    if (status != HAL_OK) return status;

    /* 7) Gyro full-scale range ayarı */
    status = MPU9250_WriteReg(dev, MPU9250_REG_GYRO_CONFIG, gfs);
    if (status != HAL_OK) return status;

    /* 8) Accel full-scale range ayarı */
    status = MPU9250_WriteReg(dev, MPU9250_REG_ACCEL_CONFIG, afs);
    if (status != HAL_OK) return status;

    /* 9) Accel DLPF ayarı */
    status = MPU9250_WriteReg(dev, MPU9250_REG_ACCEL_CONFIG2, 0x03);
    if (status != HAL_OK) return status;

    /* 10) Seçilen range'e göre hassasiyet (sensitivity) katsayılarını belirle
     *     Bu değerler MPU9250 datasheet'inden alınmıştır. */
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

    /* I2C hattı sarsıntı/gürültü altında bazen "başarılı" dönüp anlamsız
     * sabit bir blok (hep 0x00 ya da hep 0xFF) döndürebiliyor. HAL_OK
     * dönmesi verinin doğru olduğu anlamına gelmez, o yüzden burada ek
     * bir mantık kontrolü yapıyoruz. Bu okuma fusion katmanına asla
     * girmemeli, aksi halde ani sıçramalara sebep olur. */
    uint8_t all_zero = 1, all_ff = 1;
    for (int i = 0; i < 14; i++) {
        if (buf[i] != 0x00) all_zero = 0;
        if (buf[i] != 0xFF) all_ff = 0;
    }
    if (all_zero || all_ff) {
        return HAL_ERROR;
    }

    /* Register sırası: ACCEL_X, ACCEL_Y, ACCEL_Z, TEMP, GYRO_X, GYRO_Y, GYRO_Z
     * Her biri 2 byte (High, Low), Big-Endian */
    dev->accel_raw[0] = (int16_t)(buf[0]  << 8 | buf[1]);
    dev->accel_raw[1] = (int16_t)(buf[2]  << 8 | buf[3]);
    dev->accel_raw[2] = (int16_t)(buf[4]  << 8 | buf[5]);

    dev->temp_raw      = (int16_t)(buf[6]  << 8 | buf[7]);

    dev->gyro_raw[0]  = (int16_t)(buf[8]  << 8 | buf[9]);
    dev->gyro_raw[1]  = (int16_t)(buf[10] << 8 | buf[11]);
    dev->gyro_raw[2]  = (int16_t)(buf[12] << 8 | buf[13]);

    /* Fiziksel birimlere çevir */
    for (int i = 0; i < 3; i++) {
        dev->accel_g[i]   = dev->accel_raw[i] / dev->accel_sens;
        dev->gyro_dps[i]  = dev->gyro_raw[i]  / dev->gyro_sens;
    }

    /* Datasheet formülü: Temp(C) = (RAW / 333.87) + 21 */
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
    /* Gyro register'ları ACCEL(6) + TEMP(2) = 8 sonra başlar -> 0x43 */
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
    // 216 MHz Cortex-M7 için optimize edilmiş kabaca us gecikmesi
    us *= 54;
    while (us--) {
        __NOP();
    }
}

void MPU9250_I2C_Hard_Reset(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. I2C modülünü devre dışı bırak (Gecikme süresi kısaltıldı)
    __HAL_I2C_DISABLE(hi2c);
    delay_us(5);

    // 2. Pinleri I2C modundan çıkarıp standart Output (Open Drain) yap
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Başlangıçta SDA ve SCL pinlerini High durumuna çek
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // SCL
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // SDA
    delay_us(2);

    // 3. 9 Adet Clock darbesi (us seviyesinde çok hızlı tur)
    for(int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        delay_us(2);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        delay_us(2);
    }

    // 4. Manuel STOP Condition Sinyali
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // SDA = 0
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // SCL = 1
    delay_us(2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   // SDA = 1 (STOP)
    delay_us(2);

    // 5. Pinleri tekrar I2C1 (Alternate Function 4) moduna bağla
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 6. I2C modülünü yeniden etkinleştir
    __HAL_I2C_ENABLE(hi2c);
    delay_us(5);

    // 7. HAL kütüphanesinin yazılımsal durum kilitlerini aç
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->PreviousState = HAL_I2C_MODE_NONE;
    hi2c->Mode = HAL_I2C_MODE_NONE;
}
