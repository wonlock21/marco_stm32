/*
 * imu_fusion.h
 *
 * MPU9250 (accel + gyro) icin complementary filter tabanli
 * sensor fuzyonu. Roll / Pitch / Yaw (derece) uretir.
 *
 * Tasarim notlari:
 *  - Roll & Pitch: accel + gyro complementary filter -> uzun vadede driftsiz
 *    (accel yercekimi vektorunu referans alir).
 *  - Yaw: sadece gyro entegrasyonu ile hesaplanir (manyetometre kullanilmiyor),
 *    bu yuzden uzun surede drift eder. Drifti azaltmak icin:
 *      1) Baslangicta gyro bias kalibrasyonu yapilir (arac dururken).
 *      2) Calisirken hareketsizlik (stationary) tespit edilirse bias
 *         yavasca (yavas bir low-pass ile) guncellenir.
 *  - Bu yapi, AGV'nin encoder tabanli PID'sine "aci duzeltme" girdisi
 *    saglamak icin tasarlandi; harici bir modul bu struct'i okuyup
 *    PID setpoint/feedback olarak kullanabilir.
 */

#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "mpu9250.h"
#include <stdint.h>
#include "stm32f7xx_hal.h"

typedef struct {
    /* Fuzyon ciktisi (derece) */
    float roll;     /* X ekseni etrafinda egim */
    float pitch;    /* Y ekseni etrafinda egim */
    float yaw;      /* Z ekseni etrafinda donus (AGV yonelimi icin asil onemli olan) */

    /* Gyro bias (derece/sn) - kalibrasyonla bulunur, calisirken ince ayarlanir */
    float gyro_bias[3];

    /* Complementary filter katsayisi (0-1 arasi). 1'e yakin -> gyroya daha cok guven
     * (kisa vadede kararli, uzun vadede accel ile duzeltilir). Tipik: 0.96-0.99 */
    float alpha;

    /* Zamanlama */
    uint32_t last_us;
    uint8_t  first_update;   /* ilk update cagrisinda dt hesaplanamaz, atlanir */

    /* Hareketsizlik tespiti icin esik degerleri */
    float stationary_gyro_thresh_dps;   /* bu degerin altinda gyro norm -> "durgun" adayi */
    float stationary_accel_tol_g;       /* |accel_norm - 1g| bu toleransin altinda olmali */
} IMU_Fusion_t;

/* Fuzyon yapisini varsayilan degerlerle baslatir (kalibrasyon henuz yapilmadi). */
void Fusion_Init(IMU_Fusion_t *f);

/* Arac SABIT DURURKEN cagrilmali. 'samples' kadar ornek toplayip
 * gyro bias'ini (X,Y,Z derece/sn) hesaplar. Ayrica roll/pitch/yaw
 * baslangic degerlerini de bu olcumlere gore sifirlar.
 *
 * Ornek kullanim: kalkistan once 1-2 saniye arac hareketsizken cagir. */
HAL_StatusTypeDef Fusion_CalibrateGyroBias(IMU_Fusion_t *f, MPU9250_HandleTypeDef *mpu,
                                            uint16_t samples);

/* Her dongude bir kez cagrilir: MPU9250'den taze veri okur (MPU9250_ReadAll),
 * complementary filter ile roll/pitch/yaw gunceller ve gerekiyorsa
 * hareketsizlik tespitiyle gyro bias'ini ince ayarlar.
 *
 * dt, HAL_GetTick() farkindan otomatik hesaplanir; disaridan dt vermene
 * gerek yok, sadece her dongude cagir. */
HAL_StatusTypeDef Fusion_Update(IMU_Fusion_t *f, MPU9250_HandleTypeDef *mpu, uint32_t current_us);

float Fusion_GetReportedRoll(const IMU_Fusion_t *f);
float Fusion_GetReportedPitch(const IMU_Fusion_t *f);
float Fusion_GetReportedYaw(const IMU_Fusion_t *f);

#endif /* IMU_FUSION_H */
