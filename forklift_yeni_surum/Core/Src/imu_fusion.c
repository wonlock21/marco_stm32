/*
 * imu_fusion.c
 */

#include "imu_fusion.h"
#include <math.h>

#define RAD2DEG(x) ((x) * 57.295779513f)

static float WrapDegrees(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* --- Duraklama tespiti ve bias adaptasyonu için ek ayarlar ---
 * NOT: is_stationary artik tek ornekte degil, ardisik ornekler uzerinden
 * histerezisli sekilde karar veriliyor. Bu sayede tek bir gurultulu
 * ornek (titresim, I2C glitch vb.) bias'i yanlislikla bozamiyor. */
#define STATIONARY_ENTER_COUNT   15u   /* ~100Hz'de yaklasik 150ms kesintisiz durgunluk iste */
#define BIAS_LP_ALPHA            0.02f /* eskiden 0.1f -> tek ornegin bias'a etkisini 5 kat azaltir */
#define ACCEL_GLITCH_MIN_G       0.5f  /* bu araligin disinda accel norm -> muhtemelen bozuk okuma */
#define ACCEL_GLITCH_MAX_G       1.8f
#define ACCEL_TRUST_DEADBAND_G   0.05f /* bu sapmadan sonra accel'e guveni kademeli azalt */

/* Tek global IMU_Fusion_t ornegi varsayimiyla static sayac kullaniliyor.
 * Eger birden fazla IMU_Fusion_t ornegi paralel calisiyorsa bu sayaci
 * struct'a tasiyip (orn. f->stationary_count) header'a eklemek gerekir. */
static uint16_t s_stationary_count = 0;

void Fusion_Init(IMU_Fusion_t *f)
{
    f->roll  = 0.0f;
    f->pitch = 0.0f;
    f->yaw   = 0.0f;

    f->gyro_bias[0] = 0.0f;
    f->gyro_bias[1] = 0.0f;
    f->gyro_bias[2] = 0.0f;

    f->alpha = 0.98f;

    f->last_us      = 0;
    f->first_update = 1;

    /* Drift'i önlemek için eşik değeri daha hassas seviyeye çekildi */
    f->stationary_gyro_thresh_dps = 1.0f;
    f->stationary_accel_tol_g     = 0.15f;
}

HAL_StatusTypeDef Fusion_CalibrateGyroBias(IMU_Fusion_t *f, MPU9250_HandleTypeDef *mpu,
                                            uint16_t samples)
{
    if (samples == 0) return HAL_ERROR;

    float sum[3] = {0.0f, 0.0f, 0.0f};
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t ok_count = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        if (MPU9250_ReadAll(mpu) == HAL_OK)
        {
            sum[0] += mpu->gyro_dps[0];
            sum[1] += mpu->gyro_dps[1];
            sum[2] += mpu->gyro_dps[2];

            accel_sum[0] += mpu->accel_g[0];
            accel_sum[1] += mpu->accel_g[1];
            accel_sum[2] += mpu->accel_g[2];

            ok_count++;
        }
        HAL_Delay(5);
    }

    if (ok_count == 0) return HAL_ERROR;

    f->gyro_bias[0] = sum[0] / ok_count;
    f->gyro_bias[1] = sum[1] / ok_count;
    f->gyro_bias[2] = sum[2] / ok_count;

    float ax = accel_sum[0] / ok_count;
    float ay = accel_sum[1] / ok_count;
    float az = accel_sum[2] / ok_count;

    f->roll  = RAD2DEG(atan2f(ay, az));
    f->pitch = RAD2DEG(atan2f(-ax, sqrtf(ay * ay + az * az)));
    f->yaw   = 0.0f;

    f->first_update = 1; /* İlk update için zaman referansını tazeleyecek */

    return HAL_OK;
}

/* Dikkat: Fonksiyona mikro saniye cinsinden zaman parametresi eklendi */
HAL_StatusTypeDef Fusion_Update(IMU_Fusion_t *f, MPU9250_HandleTypeDef *mpu, uint32_t current_us)
{
    HAL_StatusTypeDef status = MPU9250_ReadAll(mpu);
    if (status != HAL_OK) return status;

    if (f->first_update)
    {
        f->last_us      = current_us;
        f->first_update = 0;
        return HAL_OK;
    }

    /* Mikro saniyeyi saniyeye çevir (Daha yüksek dt hassasiyeti) */
    float dt = (float)(current_us - f->last_us) / 1000000.0f;
    f->last_us = current_us;

    if (dt <= 0.0f)
    {
        return HAL_OK;
    }

    if (dt > 0.5f)
    {
        dt = 0.02f; /* Kesinti durumunda güvenli varsayılan periyot */
    }

    /* Bias'ı çıkar */
    float gx = mpu->gyro_dps[0] - f->gyro_bias[0];
    float gy = mpu->gyro_dps[1] - f->gyro_bias[1];
    float gz = mpu->gyro_dps[2] - f->gyro_bias[2];


    float ax = mpu->accel_g[0];
    float ay = mpu->accel_g[1];
    float az = mpu->accel_g[2];

    float accel_norm = sqrtf(ax*ax + ay*ay + az*az);

    /* --- I2C/veri glitch koruması ---
     * accel_norm 1g'den çok uzaksa bu ya sert bir çarpışma/vuruş ya da
     * bozuk bir okumadır. İkisinde de accel'e güvenmeyip sadece gyro ile
     * entegrasyona devam ediyoruz; bias adaptasyonuna da hiç girmiyoruz. */
    if (accel_norm < ACCEL_GLITCH_MIN_G || accel_norm > ACCEL_GLITCH_MAX_G)
    {
        f->roll  = f->roll  + gx * dt;
        f->pitch = f->pitch + gy * dt;
        f->yaw   = WrapDegrees(f->yaw + gz * dt);
        return HAL_OK;
    }

    /* --- Accel'den roll/pitch --- */
    float roll_acc  = RAD2DEG(atan2f(ay, az));
    float pitch_acc = RAD2DEG(atan2f(-ax, sqrtf(ay * ay + az * az)));

    /* --- Complementary filter (adaptif alpha) ---
     * accel_norm 1g'den saptıkça (araç hızlanıyor/sarsılıyor demektir)
     * accel'e olan güveni kademeli azaltıp gyro'ya kaydırıyoruz. Bu,
     * sert hareketlerde roll/pitch'in "drag" yapmasını (yanlış accel
     * verisinin karışmasını) engeller. */
    float accel_err = fabsf(accel_norm - 1.0f);
    float dyn_alpha = f->alpha;
    if (accel_err > ACCEL_TRUST_DEADBAND_G)
    {
        float extra = (accel_err - ACCEL_TRUST_DEADBAND_G) * 2.0f;
        if (extra > 1.0f) extra = 1.0f;
        dyn_alpha = f->alpha + (1.0f - f->alpha) * extra;
        if (dyn_alpha > 0.999f) dyn_alpha = 0.999f;
    }

    float roll_gyro  = f->roll  + gx * dt;
    float pitch_gyro = f->pitch + gy * dt;

    f->roll  = dyn_alpha * roll_gyro  + (1.0f - dyn_alpha) * roll_acc;
    f->pitch = dyn_alpha * pitch_gyro + (1.0f - dyn_alpha) * pitch_acc;

    /* --- Yaw --- */
    f->yaw = WrapDegrees(f->yaw + gz * dt);

    /* --- HAREKETSIZLIK TESPİTİ VE BIAS ADAPTASYONU ---
     * Histerezisli: yalnızca ardışık STATIONARY_ENTER_COUNT örnek boyunca
     * kesintisiz "duruyor" koşulu sağlanırsa bias güncellemesi başlar.
     * Tek bir gürültülü örnek artık ne bias'ı bozabiliyor ne de yanlış
     * pozitif üretebiliyor; en ufak harekette sayaç anında sıfırlanır. */
    float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz);

    uint8_t raw_stationary =
        (gyro_norm < f->stationary_gyro_thresh_dps) &&
        (accel_err < f->stationary_accel_tol_g);

    if (raw_stationary)
    {
        if (s_stationary_count < STATIONARY_ENTER_COUNT) s_stationary_count++;
    }
    else
    {
        s_stationary_count = 0;
    }

    if (s_stationary_count >= STATIONARY_ENTER_COUNT)
    {
        /* Yavaş, düşük geçişli adaptasyon: tek örneğin bias üzerindeki
         * etkisi eskiye göre 5 kat küçük, bu yüzden gürültü bias'ı
         * artık rastgele yürütmüyor (0.01 derecelik yavaş kayma buradan
         * geliyordu). */
        f->gyro_bias[0] += BIAS_LP_ALPHA * (mpu->gyro_dps[0] - f->gyro_bias[0]);
        f->gyro_bias[1] += BIAS_LP_ALPHA * (mpu->gyro_dps[1] - f->gyro_bias[1]);
        f->gyro_bias[2] += BIAS_LP_ALPHA * (mpu->gyro_dps[2] - f->gyro_bias[2]);
    }

    return HAL_OK;
}

/* --- MİNİMUM HAREKET TESPİTİ (DEADBAND) ENTEGRASYONU --- */

#define ANGLE_REACT_DEADBAND_DEG   1.0f  /* İstenilen genişlik */

/* Son "raporlanan/tepki verilen" açı — sadece deadband aşıldığında güncellenir.
 * NOT: Tıpkı s_stationary_count gibi, bu değişkenler de tekil IMU
 * varsayımıyla static olarak tanımlanmıştır. */
static float s_last_reported_roll  = 0.0f;
static float s_last_reported_pitch = 0.0f;
static float s_last_reported_yaw   = 0.0f;

float Fusion_GetReportedRoll(const IMU_Fusion_t *f)
{
    /* Roll değeri atan2f kaynaklı -180/+180 arasında sarmalama yapabilir */
    float diff = WrapDegrees(f->roll - s_last_reported_roll);

    if (fabsf(diff) >= ANGLE_REACT_DEADBAND_DEG)
    {
        s_last_reported_roll = f->roll;
    }
    return s_last_reported_roll;
}

float Fusion_GetReportedPitch(const IMU_Fusion_t *f)
{
    /* Pitch değeri atan2f(-ax, sqrt(ay^2+az^2)) ile -90/+90 aralığıyla
     * sınırlı olduğundan sarmalama (wrap) yapmaz, direkt fark alınabilir. */
    if (fabsf(f->pitch - s_last_reported_pitch) >= ANGLE_REACT_DEADBAND_DEG)
    {
        s_last_reported_pitch = f->pitch;
    }
    return s_last_reported_pitch;
}

float Fusion_GetReportedYaw(const IMU_Fusion_t *f)
{
    /* Yaw değeri WrapDegrees ile sürekli -180/+180 aralığında tutulduğundan
     * fark hesabı sınır geçişlerinde düzeltilmelidir. */
    float diff = WrapDegrees(f->yaw - s_last_reported_yaw);

    if (fabsf(diff) >= ANGLE_REACT_DEADBAND_DEG)
    {
        s_last_reported_yaw = f->yaw;
    }
    return s_last_reported_yaw;
}
