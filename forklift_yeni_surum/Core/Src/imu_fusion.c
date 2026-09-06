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

#define STATIONARY_ENTER_COUNT   15u
#define STATIONARY_LEAK_STEP     4u
#define BIAS_LP_ALPHA            0.02f
#define ACCEL_GLITCH_MIN_G       0.5f
#define ACCEL_GLITCH_MAX_G       1.8f
#define ACCEL_TRUST_DEADBAND_G   0.05f

#define GYRO_LPF_ALPHA           0.35f

#define MPU_READ_SOFT_RETRY_COUNT 2u
#define BUS_RECOVERY_MAX_EXTRAPOLATE_S 0.15f

static float   s_last_good_gx = 0.0f;
static float   s_last_good_gy = 0.0f;
static float   s_last_good_gz = 0.0f;
static uint8_t s_pending_bus_recovery = 0;

static uint16_t s_stationary_count  = 0;
static float    s_gyro_filt[3]      = {0.0f, 0.0f, 0.0f};
static uint8_t  s_gyro_filt_init    = 0;

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

    f->stationary_gyro_thresh_dps = 1.0f;
    f->stationary_accel_tol_g     = 0.15f;
}

void Fusion_NotifyBusReset(IMU_Fusion_t *f)
{
    f->first_update        = 1;
    s_gyro_filt_init       = 0;
    s_pending_bus_recovery = 1;
}

HAL_StatusTypeDef Fusion_CalibrateGyroBias(IMU_Fusion_t *f, MPU9250_HandleTypeDef *mpu,
                                            uint16_t samples)
{
    if (samples == 0) return HAL_ERROR;

#define CALIB_OUTLIER_MIN_SAMPLES 5u
#define CALIB_OUTLIER_THRESH_DPS  3.0f

    float sum[3] = {0.0f, 0.0f, 0.0f};
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t ok_count = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        if (MPU9250_ReadAll(mpu) == HAL_OK)
        {
            float g[3] = { mpu->gyro_dps[0], mpu->gyro_dps[1], mpu->gyro_dps[2] };
            uint8_t is_outlier = 0;

            if (ok_count >= CALIB_OUTLIER_MIN_SAMPLES)
            {
                for (int a = 0; a < 3; a++)
                {
                    float running_mean = sum[a] / ok_count;
                    if (fabsf(g[a] - running_mean) > CALIB_OUTLIER_THRESH_DPS)
                    {
                        is_outlier = 1;
                        break;
                    }
                }
            }

            if (!is_outlier)
            {
                sum[0] += g[0];
                sum[1] += g[1];
                sum[2] += g[2];

                accel_sum[0] += mpu->accel_g[0];
                accel_sum[1] += mpu->accel_g[1];
                accel_sum[2] += mpu->accel_g[2];

                ok_count++;
            }
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

    f->first_update = 1;

    return HAL_OK;
}

HAL_StatusTypeDef Fusion_Update(IMU_Fusion_t *f, MPU9250_HandleTypeDef *mpu, uint32_t current_us)
{
    HAL_StatusTypeDef status = MPU9250_ReadAll(mpu);

    if (status != HAL_OK)
    {
        for (uint8_t retry = 0; retry < MPU_READ_SOFT_RETRY_COUNT && status != HAL_OK; retry++)
        {
            status = MPU9250_ReadAll(mpu);
        }
        if (status != HAL_OK) return status;
    }

    if (f->first_update)
    {
        if (s_pending_bus_recovery)
        {
            float gap_dt = (float)(current_us - f->last_us) / 1000000.0f;
            if (gap_dt > 0.0f)
            {
                if (gap_dt > BUS_RECOVERY_MAX_EXTRAPOLATE_S)
                {
                    gap_dt = BUS_RECOVERY_MAX_EXTRAPOLATE_S;
                }

                f->roll  = f->roll  + s_last_good_gx * gap_dt;
                f->pitch = f->pitch + s_last_good_gy * gap_dt;
                f->yaw   = WrapDegrees(f->yaw + s_last_good_gz * gap_dt);
            }
            s_pending_bus_recovery = 0;
        }

        f->last_us      = current_us;
        f->first_update = 0;
        return HAL_OK;
    }

    float dt = (float)(current_us - f->last_us) / 1000000.0f;
    f->last_us = current_us;

    if (dt <= 0.0f) return HAL_OK;
    if (dt > 0.5f)  dt = 0.02f;

    float raw_g[3] = { mpu->gyro_dps[0], mpu->gyro_dps[1], mpu->gyro_dps[2] };

    if (!s_gyro_filt_init)
    {
        s_gyro_filt[0] = raw_g[0];
        s_gyro_filt[1] = raw_g[1];
        s_gyro_filt[2] = raw_g[2];
        s_gyro_filt_init = 1;
    }
    else
    {
        for (int a = 0; a < 3; a++)
        {
            // AGV motor jerk/titreşimlerinde verinin çöpe atılmasını önlemek için
            // katı sınır reddi kaldırıldı, doğrudan LPF uygulanıyor:
            s_gyro_filt[a] += GYRO_LPF_ALPHA * (raw_g[a] - s_gyro_filt[a]);
        }
    }

    float gx = s_gyro_filt[0] - f->gyro_bias[0];
    float gy = s_gyro_filt[1] - f->gyro_bias[1];
    float gz = s_gyro_filt[2] - f->gyro_bias[2];

    s_last_good_gx = gx;
    s_last_good_gy = gy;
    s_last_good_gz = gz;

    float ax = mpu->accel_g[0];
    float ay = mpu->accel_g[1];
    float az = mpu->accel_g[2];

    float accel_norm = sqrtf(ax*ax + ay*ay + az*az);

    if (accel_norm < ACCEL_GLITCH_MIN_G || accel_norm > ACCEL_GLITCH_MAX_G)
    {
        f->roll  = f->roll  + gx * dt;
        f->pitch = f->pitch + gy * dt;
        f->yaw   = WrapDegrees(f->yaw + gz * dt);
        return HAL_OK;
    }

    float roll_acc  = RAD2DEG(atan2f(ay, az));
    float pitch_acc = RAD2DEG(atan2f(-ax, sqrtf(ay * ay + az * az)));

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

    float roll_err  = WrapDegrees(roll_acc  - roll_gyro);
    float pitch_err = WrapDegrees(pitch_acc - pitch_gyro);

    f->roll  = WrapDegrees(roll_gyro  + (1.0f - dyn_alpha) * roll_err);
    f->pitch = pitch_gyro + (1.0f - dyn_alpha) * pitch_err;
    f->yaw   = WrapDegrees(f->yaw + gz * dt);

    float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz);
    float raw_gyro_norm = sqrtf(s_gyro_filt[0]*s_gyro_filt[0] +
                                s_gyro_filt[1]*s_gyro_filt[1] +
                                s_gyro_filt[2]*s_gyro_filt[2]);

    uint8_t raw_stationary =
        (raw_gyro_norm < (f->stationary_gyro_thresh_dps + 2.0f)) &&
        (gyro_norm < f->stationary_gyro_thresh_dps) &&
        (accel_err < f->stationary_accel_tol_g);

    if (raw_stationary)
    {
        if (s_stationary_count < STATIONARY_ENTER_COUNT) s_stationary_count++;
    }
    else
    {
        if (s_stationary_count > STATIONARY_LEAK_STEP) s_stationary_count -= STATIONARY_LEAK_STEP;
        else                                            s_stationary_count = 0;
    }

    if (s_stationary_count >= STATIONARY_ENTER_COUNT)
    {
        f->gyro_bias[0] += BIAS_LP_ALPHA * (s_gyro_filt[0] - f->gyro_bias[0]);
        f->gyro_bias[1] += BIAS_LP_ALPHA * (s_gyro_filt[1] - f->gyro_bias[1]);
        f->gyro_bias[2] += BIAS_LP_ALPHA * (s_gyro_filt[2] - f->gyro_bias[2]);

        float max_bias = 4.0f;
        for (int i = 0; i < 3; i++) {
            if (f->gyro_bias[i] > max_bias) f->gyro_bias[i] = max_bias;
            if (f->gyro_bias[i] < -max_bias) f->gyro_bias[i] = -max_bias;
        }
    }

    return HAL_OK;
}

#define ANGLE_REACT_DEADBAND_DEG   1.0f

static float s_last_reported_roll  = 0.0f;
static float s_last_reported_pitch = 0.0f;
static float s_last_reported_yaw   = 0.0f;

float Fusion_GetReportedRoll(const IMU_Fusion_t *f)
{
    float diff = WrapDegrees(f->roll - s_last_reported_roll);

    if (fabsf(diff) >= ANGLE_REACT_DEADBAND_DEG)
    {
        s_last_reported_roll = f->roll;
    }
    return s_last_reported_roll;
}

float Fusion_GetReportedPitch(const IMU_Fusion_t *f)
{
    if (fabsf(f->pitch - s_last_reported_pitch) >= ANGLE_REACT_DEADBAND_DEG)
    {
        s_last_reported_pitch = f->pitch;
    }
    return s_last_reported_pitch;
}

float Fusion_GetReportedYaw(const IMU_Fusion_t *f)
{
    float diff = WrapDegrees(f->yaw - s_last_reported_yaw);

    if (fabsf(diff) >= ANGLE_REACT_DEADBAND_DEG)
    {
        s_last_reported_yaw = f->yaw;
    }
    return s_last_reported_yaw;
}
