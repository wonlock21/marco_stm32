#include "motor_control.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim1;   /* Sağ Motor Geri (D10 PA8) */
extern TIM_HandleTypeDef htim2;   /* Sadece Zamanlama İçin Kullanılan Timer */
extern TIM_HandleTypeDef htim3;   /* Sol Motor Geri (D3 PB4) */
extern TIM_HandleTypeDef htim5;   /* Sol Motor İleri (D5 PI0) */
extern TIM_HandleTypeDef htim12;  /* Sağ Motor İleri (D11 PB15) */

/* ----------------------------- Donanım Ayarları ----------------------------- */
#define MOTOR_EN_GPIO_Port                 GPIOF
#define MOTOR_EN_Pin                       GPIO_PIN_6

#define LEFT_ENCODER_Pin                   GPIO_PIN_2
#define RIGHT_ENCODER_Pin                  GPIO_PIN_3

#define MOTOR_PWM_MAX                      255U
#define MOTOR_PWM_MIN_RUNNING              35U//15U //35U//54U
#define MOTOR_RAMP_STOP_THRESHOLD           30U//50U
#define MIN_COMMAND_RPM                     0.2f//1.0f//50.0f

#define LEFT_ENCODER_PPR                   360U
#define RIGHT_ENCODER_PPR                  360U

#define WHEEL_DIAMETER_MM                  200.0f
#define WHEEL_CIRCUMFERENCE_MM             (WHEEL_DIAMETER_MM * 3.14159265f)
#define RPM_TO_MMPS(rpm)                   ((rpm) * WHEEL_CIRCUMFERENCE_MM / 60.0f)

#define ENCODER_TIMEOUT_COUNTS              3U

#define CONTROL_PERIOD_MS                   10U
#define TELEMETRY_PERIOD_MS                500U

#define PWM_SLEW_STEP                       40

/* Hız Rampası Parametreleri (RPM cinsinden) */
#define RAMP_UP_DURATION_MS                150U
#define EXP_RAMP_K                         1.50f//2.00f

#define RAMP_DOWN_DURATION_MS               90U
#define EXP_DECEL_RAMP_K                    2.0f

#define MAX_TARGET_RPM                     255.0f

/* ------------------------------- PID Ayarları ------------------------------- */
#define LEFT_KP                              12.00f//15.80f
#define LEFT_KI                              6.00f//4.85f
#define LEFT_KD                              0.001f//0.005f

#define RIGHT_KP                             12.00f//15.80f
#define RIGHT_KI                             6.00f//4.85f
#define RIGHT_KD                             0.001f//0.005f

/* Sertleştirilmiş Senkronizasyon Parametreleri */
#define SYNC_COUNT_KP                         0.25f  /* Agresif senkronizasyon kazancı[cite: 1] */
#define SYNC_CORRECTION_LIMIT                35.0f   /* Genişletilmiş düzeltme marjı[cite: 1] */
#define SYNC_COUNT_DEADBAND                    1U    /* Sıfıra yakın tolerans[cite: 1] */

#define INTEGRAL_PWM_MIN                   (-150.0f)
#define INTEGRAL_PWM_Max                    150.0f

#define RPM_ERROR_DEADBAND                    0.01f
#define RPM_FILTER_ALPHA                      0.25f
#define DERIVATIVE_FILTER_ALPHA               0.20f

/* ---------------------------- Encoder Filtre Ayarı -------------------------- */
#define ENCODER_FILTER_INTERVAL_RATIO         0.05f
#define ENCODER_FILTER_MIN_US                  5U
#define ENCODER_FILTER_MAX_US               1000U

typedef enum
{
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE
} MotorDirection_t;

typedef struct
{
    uint32_t start_tick;
    float start_rpm;
    float current_rpm_ramp;
    float target_rpm_cmd;
    uint8_t active;
    uint8_t stop_active;
} WheelRamp_t;

typedef struct
{
    volatile uint32_t pulse_count;
    volatile uint32_t raw_count;
    volatile uint32_t rejected_count;
    volatile uint32_t last_encoder_us;
    volatile uint32_t current_delta_us; /* İki pulse arası ölçülen hassas süre[cite: 1] */

    uint32_t previous_control_count;
    uint32_t dynamic_filter_us;

    float measured_rpm;
    float filtered_rpm;
    float previous_filtered_rpm;
    float derivative_rpm_per_s;

    float target_rpm;

    float kp;
    float ki;
    float kd;

    float p_term;
    float integral_pwm;
    float d_term;

    uint8_t zero_pulse_count;

    int32_t base_pwm;
    int32_t applied_pwm;

    WheelRamp_t ramp;
} WheelControl_t;

static WheelControl_t left_wheel = { .kp = LEFT_KP, .ki = LEFT_KI, .kd = LEFT_KD };
static WheelControl_t right_wheel = { .kp = RIGHT_KP, .ki = RIGHT_KI, .kd = RIGHT_KD };

static volatile MotorDirection_t left_motor_direction = MOTOR_DIR_STOP;
static volatile MotorDirection_t right_motor_direction = MOTOR_DIR_STOP;
static volatile uint8_t control_enabled = 0U;
static volatile uint8_t command_clamped = 0U;

volatile float left_target_rpm_command = 0.0f;
volatile float right_target_rpm_command = 0.0f;

/* Duz surus senkronizasyonu her yeni komutta bu tabandan baslar.[cite: 1] */
static uint32_t left_sync_origin = 0U;
static uint32_t right_sync_origin = 0U;

volatile int32_t left_odo_ticks = 0;
volatile int32_t right_odo_ticks = 0;

static uint32_t last_control_tick = 0U;

/* ------------------------------- Yardımcılar -------------------------------- */
/* Hedef hıza bağlı olarak dinamik kontrol periyodu belirleme */
static uint32_t GetDynamicControlPeriod(void)
{
    float max_target = (left_wheel.target_rpm > right_wheel.target_rpm) ? left_wheel.target_rpm : right_wheel.target_rpm;
    max_target = fabsf(max_target);

    if (max_target < 3.0f)
    {
        /* 1-3 RPM gibi çok düşük hızlar için 200 ms periyot.
           (360 PPR'da 1 RPM = 166 ms pulse aralığı) */
        return 200U;
    }
    else if (max_target < 15.0f)
    {
        return 50U; // 3 - 15 RPM arası için 50 ms
    }
    else if (max_target < 30.0f)
    {
        return 30U; // 15 - 30 RPM arası için 30 ms
    }
    else
    {
        return 10U; // 30 RPM üstü için yüksek hızlı standart 10 ms
    }
}

static float ClampFloat(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static int32_t ClampInt32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static int32_t SlewLimit(int32_t current, int32_t requested, int32_t step)
{
    if (requested > (current + step)) return current + step;
    if (requested < (current - step)) return current - step;
    return requested;
}

static float ApplyDeadband(float error)
{
    return (fabsf(error) <= RPM_ERROR_DEADBAND) ? 0.0f : error;
}

static float CalculateFeedForwardPwm(float target_rpm)
{
    if (target_rpm < 0.1f) return 0.0f;

    float controllable_rpm = ClampFloat(target_rpm, MIN_COMMAND_RPM,
                                        MAX_TARGET_RPM);
    float ratio = (controllable_rpm - MIN_COMMAND_RPM) /
                  (MAX_TARGET_RPM - MIN_COMMAND_RPM);
    return (float)MOTOR_PWM_MIN_RUNNING +
           ratio * ((float)MOTOR_PWM_MAX - (float)MOTOR_PWM_MIN_RUNNING);
}

static uint32_t CalculateDynamicEncoderFilterUs(float target_rpm, uint32_t ppr)
{
    if ((target_rpm <= 0.0f) || (ppr == 0U)) return ENCODER_FILTER_MAX_US;

    float interval_us = 60000000.0f / (target_rpm * (float)ppr);
    float filter_us = interval_us * ENCODER_FILTER_INTERVAL_RATIO;

    return (uint32_t)(ClampFloat(filter_us, (float)ENCODER_FILTER_MIN_US, (float)ENCODER_FILTER_MAX_US) + 0.5f);
}

static void UpdateDynamicFilters(float target_rpm)
{
    left_wheel.dynamic_filter_us = CalculateDynamicEncoderFilterUs(target_rpm, LEFT_ENCODER_PPR);
    right_wheel.dynamic_filter_us = CalculateDynamicEncoderFilterUs(target_rpm, RIGHT_ENCODER_PPR);
}

static float CalculateExponentialRamp(float start_val, float target_val, uint32_t elapsed_ms, uint32_t duration_ms)
{
    if (elapsed_ms >= duration_ms) return target_val;

    float t = ClampFloat((float)elapsed_ms / (float)duration_ms, 0.0f, 1.0f);
    float denominator = expf(EXP_RAMP_K) - 1.0f;
    float eased = (fabsf(denominator) < 0.0001f) ? t : ((expf(EXP_RAMP_K * t) - 1.0f) / denominator);

    return start_val + ((target_val - start_val) * eased);
}

static void UpdateWheelAccelerationRamp(WheelControl_t *wheel, uint32_t now_tick)
{
    if (wheel->ramp.active != 0U)
    {
        uint32_t elapsed_ms = now_tick - wheel->ramp.start_tick;
        wheel->ramp.current_rpm_ramp = CalculateExponentialRamp(wheel->ramp.start_rpm, wheel->ramp.target_rpm_cmd, elapsed_ms, RAMP_UP_DURATION_MS);

        if (elapsed_ms >= RAMP_UP_DURATION_MS)
        {
            wheel->ramp.current_rpm_ramp = wheel->ramp.target_rpm_cmd;
            wheel->ramp.active = 0U;
        }
    }
    else
    {
        wheel->ramp.current_rpm_ramp = wheel->ramp.target_rpm_cmd;
    }

    if (wheel->ramp.target_rpm_cmd <= 0.0f)
    {
        wheel->ramp.current_rpm_ramp = 0.0f;
        wheel->target_rpm = 0.0f;
        return;
    }

    wheel->target_rpm = ClampFloat(wheel->ramp.current_rpm_ramp, 0.0f, MAX_TARGET_RPM);
}

static void UpdateAccelerationRamp(uint32_t now_tick)
{
    UpdateWheelAccelerationRamp(&left_wheel, now_tick);
    UpdateWheelAccelerationRamp(&right_wheel, now_tick);

    UpdateDynamicFilters((left_wheel.target_rpm > right_wheel.target_rpm) ? left_wheel.target_rpm : right_wheel.target_rpm);
}

/* Hassas Pulse Aralığı Tabanlı RPM Hesaplama Motoru[cite: 1] */
static float CalculateRpmHighPrecision(WheelControl_t *wheel, uint32_t pulse_delta, uint32_t elapsed_ms, uint32_t ppr)
{
    if ((ppr == 0U) || (wheel == NULL)) return 0.0f;

    uint32_t delta_us = 0;
    __disable_irq();
    delta_us = wheel->current_delta_us;
    __enable_irq();

    if ((pulse_delta == 0U) || (delta_us == 0U))
    {
        wheel->zero_pulse_count++;
        if (wheel->zero_pulse_count >= ENCODER_TIMEOUT_COUNTS)
        {
            wheel->zero_pulse_count = ENCODER_TIMEOUT_COUNTS;
            return 0.0f;
        }
        return ((float)pulse_delta * 60000.0f) / ((float)ppr * (float)(elapsed_ms > 0 ? elapsed_ms : 1));
    }

    wheel->zero_pulse_count = 0U;

    float instant_rpm = 60000000.0f / ((float)delta_us * (float)ppr);
    return ClampFloat(instant_rpm, 0.0f, MAX_TARGET_RPM * 1.2f);
}

/* ------------------------------ Motor Sürücüsü ------------------------------ */

static void MotorDriver_Enable(void)
{
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
}

static void MotorDriver_Disable(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim5,  TIM_CHANNEL_4, 0U);
    __HAL_TIM_SET_COMPARE(&htim3,  TIM_CHANNEL_1, 0U);
}

static void LeftMotor_Apply(MotorDirection_t direction, uint16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;

    if (direction == MOTOR_DIR_FORWARD)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm);
    }
    else if (direction == MOTOR_DIR_REVERSE)
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    }
}

static void RightMotor_Apply(MotorDirection_t direction, uint16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) pwm = MOTOR_PWM_MAX;

    if (direction == MOTOR_DIR_FORWARD)
    {
        __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm);
    }
    else if (direction == MOTOR_DIR_REVERSE)
    {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, pwm);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, 0U);
    }
}

static void ResetWheelControl(WheelControl_t *wheel)
{
    wheel->target_rpm = 0.0f;
    wheel->p_term = 0.0f;
    wheel->integral_pwm = 0.0f;
    wheel->d_term = 0.0f;
    wheel->base_pwm = 0;
    wheel->applied_pwm = 0;
    wheel->current_delta_us = 0;
    wheel->ramp.start_rpm = 0.0f;
    wheel->ramp.current_rpm_ramp = 0.0f;
    wheel->ramp.target_rpm_cmd = 0.0f;
    wheel->ramp.active = 0U;
    wheel->ramp.stop_active = 0U;
}

static void FinishMotorStop(void)
{
    control_enabled = 0U;
    left_target_rpm_command = 0.0f;
    right_target_rpm_command = 0.0f;
    left_motor_direction = MOTOR_DIR_STOP;
    right_motor_direction = MOTOR_DIR_STOP;

    ResetWheelControl(&left_wheel);
    ResetWheelControl(&right_wheel);

    LeftMotor_Apply(MOTOR_DIR_STOP, 0U);
    RightMotor_Apply(MOTOR_DIR_STOP, 0U);
    MotorDriver_Disable();
}

static void FinishSingleWheelStop(WheelControl_t *wheel, volatile MotorDirection_t *dir, volatile float *target_cmd, void (*motor_apply)(MotorDirection_t, uint16_t))
{
    *target_cmd = 0.0f;
    *dir = MOTOR_DIR_STOP;
    ResetWheelControl(wheel);
    motor_apply(MOTOR_DIR_STOP, 0U);
}

static void ProcessStopRamp(uint32_t now_tick)
{
    if (left_wheel.ramp.stop_active != 0U)
    {
        uint32_t elapsed = now_tick - left_wheel.ramp.start_tick;
        float stop_rpm = CalculateExponentialRamp(left_wheel.ramp.start_rpm, 0.0f, elapsed, RAMP_DOWN_DURATION_MS);

        if ((elapsed >= RAMP_DOWN_DURATION_MS) || (stop_rpm < 0.1f))
        {
            FinishSingleWheelStop(&left_wheel, &left_motor_direction, &left_target_rpm_command, LeftMotor_Apply);
        }
        else
        {
            left_wheel.ramp.current_rpm_ramp = stop_rpm;
            left_wheel.target_rpm = stop_rpm;
            left_wheel.base_pwm = (int32_t)lroundf(CalculateFeedForwardPwm(stop_rpm));
            left_wheel.applied_pwm = SlewLimit(left_wheel.applied_pwm,
                                               left_wheel.base_pwm,
                                               PWM_SLEW_STEP);
            LeftMotor_Apply(left_motor_direction, (uint16_t)left_wheel.applied_pwm);
        }
    }

    if (right_wheel.ramp.stop_active != 0U)
    {
        uint32_t elapsed = now_tick - right_wheel.ramp.start_tick;
        float stop_rpm = CalculateExponentialRamp(right_wheel.ramp.start_rpm, 0.0f, elapsed, RAMP_DOWN_DURATION_MS);

        if ((elapsed >= RAMP_DOWN_DURATION_MS) || (stop_rpm < 2.0f))
        {
            FinishSingleWheelStop(&right_wheel, &right_motor_direction, &right_target_rpm_command, RightMotor_Apply);
        }
        else
        {
            right_wheel.ramp.current_rpm_ramp = stop_rpm;
            right_wheel.target_rpm = stop_rpm;
            right_wheel.base_pwm = (int32_t)lroundf(CalculateFeedForwardPwm(stop_rpm));
            right_wheel.applied_pwm = SlewLimit(right_wheel.applied_pwm,
                                                right_wheel.base_pwm,
                                                PWM_SLEW_STEP);
            RightMotor_Apply(right_motor_direction, (uint16_t)right_wheel.applied_pwm);
        }
    }

    if ((left_motor_direction == MOTOR_DIR_STOP) && (right_motor_direction == MOTOR_DIR_STOP))
    {
        FinishMotorStop();
    }
}

/* ------------------------------- Encoder İşlemi ----------------------------- */

static void Encoder_Reset(uint8_t clear_statistics)
{
    uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim2);

    __disable_irq();
    left_wheel.pulse_count = 0U;
    right_wheel.pulse_count = 0U;
    left_wheel.current_delta_us = 0U;
    right_wheel.current_delta_us = 0U;

    if (clear_statistics != 0U)
    {
        left_wheel.raw_count = 0U;
        left_wheel.rejected_count = 0U;
        right_wheel.raw_count = 0U;
        right_wheel.rejected_count = 0U;
    }

    left_wheel.last_encoder_us = now_us;
    right_wheel.last_encoder_us = now_us;
    __enable_irq();

    left_wheel.previous_control_count = 0U;
    right_wheel.previous_control_count = 0U;

    left_wheel.measured_rpm = 0.0f;
    right_wheel.measured_rpm = 0.0f;
    left_wheel.filtered_rpm = 0.0f;
    right_wheel.filtered_rpm = 0.0f;
    left_wheel.previous_filtered_rpm = 0.0f;
    right_wheel.previous_filtered_rpm = 0.0f;
    left_wheel.derivative_rpm_per_s = 0.0f;
    right_wheel.derivative_rpm_per_s = 0.0f;
    left_wheel.p_term = 0.0f;
    right_wheel.p_term = 0.0f;
    left_wheel.d_term = 0.0f;
    right_wheel.d_term = 0.0f;
}

void MotorControl_EncoderExtiCallback(uint16_t gpio_pin)
{
    uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim2);

    if (gpio_pin == LEFT_ENCODER_Pin)
    {
        left_wheel.raw_count++;
        uint32_t delta = now_us - left_wheel.last_encoder_us;

        if (delta < left_wheel.dynamic_filter_us)
        {
            left_wheel.rejected_count++;
            return;
        }

        left_wheel.current_delta_us = delta;
        left_wheel.last_encoder_us = now_us;
        left_wheel.pulse_count++;

        if (left_motor_direction == MOTOR_DIR_REVERSE) left_odo_ticks--;
        else left_odo_ticks++;
    }
    else if (gpio_pin == RIGHT_ENCODER_Pin)
    {
        right_wheel.raw_count++;
        uint32_t delta = now_us - right_wheel.last_encoder_us;

        if (delta < right_wheel.dynamic_filter_us)
        {
            right_wheel.rejected_count++;
            return;
        }

        right_wheel.current_delta_us = delta;
        right_wheel.last_encoder_us = now_us;
        right_wheel.pulse_count++;

        if (right_motor_direction == MOTOR_DIR_FORWARD) right_odo_ticks++;
        else right_odo_ticks--;
    }
}

/* ------------------------------- Başlatma ----------------------------------- */

void MotorControl_Init(void)
{
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();
    __HAL_TIM_SET_COUNTER(&htim2, 0U);

    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim5,  TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3,  TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    MotorDriver_Disable();
    UpdateDynamicFilters(0.0f);
    Encoder_Reset(1U);
}

/* ---------------------------- Güvenli Durdurma ----------------------------- */

void MotorControl_SafeStop(void)
{
    FinishMotorStop();
}

/* ---------------------------- Hedef Ayarlama ------------------------------- */

static void SetSingleWheelTarget(WheelControl_t *wheel,
                                 float signed_rpm_cmd,
                                 volatile MotorDirection_t *motor_direction,
                                 volatile float *target_rpm_command,
                                 void (*motor_apply)(MotorDirection_t, uint16_t),
                                 uint32_t now_tick)
{
    float original_rpm_cmd = signed_rpm_cmd;
    signed_rpm_cmd = ClampFloat(signed_rpm_cmd, -MAX_TARGET_RPM, MAX_TARGET_RPM);
    if (fabsf(signed_rpm_cmd - original_rpm_cmd) > 0.001f) command_clamped = 1U;
    uint8_t wheel_was_running = (*motor_direction != MOTOR_DIR_STOP) ? 1U : 0U;

    if (fabsf(signed_rpm_cmd) < 0.1f)
    {
        *target_rpm_command = 0.0f;

        if (wheel_was_running == 0U)
        {
            FinishSingleWheelStop(wheel, motor_direction, target_rpm_command, motor_apply);
            return;
        }

        if (wheel->ramp.stop_active == 0U)
        {
            wheel->ramp.start_rpm = wheel->target_rpm;
            wheel->ramp.start_tick = now_tick;
            wheel->ramp.active = 0U;
            wheel->ramp.stop_active = 1U;

            wheel->integral_pwm = 0.0f;
            wheel->p_term = 0.0f;
            wheel->d_term = 0.0f;
        }
        return;
    }

    if (fabsf(signed_rpm_cmd) < MIN_COMMAND_RPM)
    {
        signed_rpm_cmd = (signed_rpm_cmd > 0.0f) ? MIN_COMMAND_RPM : -MIN_COMMAND_RPM;
        command_clamped = 1U;
    }

    MotorDirection_t requested_direction = (signed_rpm_cmd > 0.0f) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
    float target_magnitude = fabsf(signed_rpm_cmd);
    uint8_t direction_changed = (requested_direction != *motor_direction) ? 1U : 0U;

    *target_rpm_command = signed_rpm_cmd;
    wheel->ramp.stop_active = 0U;

    if ((wheel_was_running == 0U) || (direction_changed != 0U))
    {
        if ((wheel_was_running != 0U) && (direction_changed != 0U))
        {
            motor_apply(MOTOR_DIR_STOP, 0U);
        }

        *motor_direction = requested_direction;
        wheel->integral_pwm = 0.0f;
        wheel->p_term = 0.0f;
        wheel->d_term = 0.0f;

        wheel->ramp.start_rpm = 0.0f;
        wheel->ramp.current_rpm_ramp = 0.0f;
        wheel->ramp.target_rpm_cmd = target_magnitude;
        wheel->ramp.start_tick = now_tick;
        wheel->ramp.active = 1U;

        wheel->applied_pwm = (int32_t)MOTOR_PWM_MIN_RUNNING;
        motor_apply(*motor_direction, (uint16_t)wheel->applied_pwm);
    }
    else
    {
        *motor_direction = requested_direction;
        wheel->ramp.start_rpm = wheel->target_rpm;
        wheel->ramp.target_rpm_cmd = target_magnitude;
        wheel->ramp.start_tick = now_tick;
        wheel->ramp.active = 1U;
    }
}

void MotorControl_SetTargetRpm(float left_signed_rpm, float right_signed_rpm)
{
    uint32_t now_tick = HAL_GetTick();
    uint8_t was_running = control_enabled;
    command_clamped = 0U;

    SetSingleWheelTarget(&left_wheel, left_signed_rpm, &left_motor_direction, &left_target_rpm_command, LeftMotor_Apply, now_tick);
    SetSingleWheelTarget(&right_wheel, right_signed_rpm, &right_motor_direction, &right_target_rpm_command, RightMotor_Apply, now_tick);

    if ((left_motor_direction == MOTOR_DIR_STOP) && (right_motor_direction == MOTOR_DIR_STOP) &&
        (left_wheel.ramp.stop_active == 0U) && (right_wheel.ramp.stop_active == 0U))
    {
        FinishMotorStop();
        return;
    }

    if (was_running == 0U) Encoder_Reset(0U);

    __disable_irq();
    left_sync_origin = left_wheel.pulse_count;
    right_sync_origin = right_wheel.pulse_count;
    __enable_irq();

    UpdateDynamicFilters((left_wheel.target_rpm > right_wheel.target_rpm) ? left_wheel.target_rpm : right_wheel.target_rpm);

    last_control_tick = now_tick;
    MotorDriver_Enable();
    control_enabled = 1U;
}

void MotorControl_Stop(void)
{
    MotorControl_SetTargetRpm(0.0f, 0.0f);
}

/* ----------------------------- PID Yardımcıları ----------------------------- */

static float UpdateIntegralWithAntiWindup(float current_integral_pwm, float error, float ki, float dt, float unsaturated_output, float output_min, float output_max)
{
    uint8_t saturated_high = (unsaturated_output >= output_max) ? 1U : 0U;
    uint8_t saturated_low  = (unsaturated_output <= output_min) ? 1U : 0U;

    if ((!saturated_high && !saturated_low) ||
        (saturated_high && (error < 0.0f)) ||
        (saturated_low && (error > 0.0f)))
    {
        current_integral_pwm += ki * error * dt;
    }

    return ClampFloat(current_integral_pwm, INTEGRAL_PWM_MIN, INTEGRAL_PWM_Max);
}

static void CalculateWheelPID(WheelControl_t *wheel, uint32_t pulse_delta,
                              uint32_t elapsed_ms, uint32_t ppr, float dt,
                              float control_target_rpm)
{
    wheel->measured_rpm = CalculateRpmHighPrecision(wheel, pulse_delta, elapsed_ms, ppr);

    if (wheel->zero_pulse_count >= ENCODER_TIMEOUT_COUNTS)
    {
        wheel->filtered_rpm = 0.0f;
        wheel->previous_filtered_rpm = 0.0f;
        wheel->derivative_rpm_per_s = 0.0f;
    }
    else
    {
        wheel->filtered_rpm += RPM_FILTER_ALPHA * (wheel->measured_rpm - wheel->filtered_rpm);
    }

    float raw_derivative = (wheel->filtered_rpm - wheel->previous_filtered_rpm) / dt;
    wheel->derivative_rpm_per_s += DERIVATIVE_FILTER_ALPHA * (raw_derivative - wheel->derivative_rpm_per_s);
    wheel->previous_filtered_rpm = wheel->filtered_rpm;

    float error = ApplyDeadband(control_target_rpm - wheel->filtered_rpm);
    wheel->p_term = wheel->kp * error;
    wheel->d_term = -wheel->kd * wheel->derivative_rpm_per_s;
}

/* ------------------------------ Kapalı Çevrim ------------------------------- */

static void ProcessClosedLoop(void)
{
    uint32_t now_tick = HAL_GetTick();

    if ((left_wheel.ramp.stop_active != 0U) || (right_wheel.ramp.stop_active != 0U))
    {
        ProcessStopRamp(now_tick);
    }

    uint32_t current_control_period = GetDynamicControlPeriod();

        uint32_t elapsed_ms = now_tick - last_control_tick;
        if (elapsed_ms < current_control_period) return; // Dinamik süre dolmadıysa çık

        float dt = (float)elapsed_ms / 1000.0f;

        if (control_enabled != 0U) {
            UpdateAccelerationRamp(now_tick);
        }

        if (dt <= 0.0f)
        {
            last_control_tick = now_tick;
            return;
        }

    __disable_irq();
    uint32_t left_count  = left_wheel.pulse_count;
    uint32_t right_count = right_wheel.pulse_count;
    __enable_irq();

    uint32_t left_delta  = left_count - left_wheel.previous_control_count;
    uint32_t right_delta = right_count - right_wheel.previous_control_count;

    left_wheel.previous_control_count  = left_count;
    right_wheel.previous_control_count = right_count;

    float magnitude_diff = fabsf(fabsf(left_target_rpm_command) -
                                 fabsf(right_target_rpm_command));

    float control_left_target_rpm  = left_wheel.target_rpm;
    float control_right_target_rpm = right_wheel.target_rpm;

    bool is_straight_driving = (magnitude_diff <= 5.0f) &&
                               (left_motor_direction != MOTOR_DIR_STOP) &&
                               (right_motor_direction != MOTOR_DIR_STOP) &&
                               (left_motor_direction != right_motor_direction);

    if (is_straight_driving)
    {
        uint32_t left_progress = left_count - left_sync_origin;
        uint32_t right_progress = right_count - right_sync_origin;
        int32_t count_diff = (int32_t)left_progress - (int32_t)right_progress;

        if (labs(count_diff) > SYNC_COUNT_DEADBAND)
        {
            float rpm_trim = (float)count_diff * SYNC_COUNT_KP;
            rpm_trim = ClampFloat(rpm_trim, -SYNC_CORRECTION_LIMIT, SYNC_CORRECTION_LIMIT);

            control_left_target_rpm  -= rpm_trim;
            control_right_target_rpm += rpm_trim;

            control_left_target_rpm = ClampFloat(control_left_target_rpm, 0.0f, MAX_TARGET_RPM);
            control_right_target_rpm = ClampFloat(control_right_target_rpm, 0.0f, MAX_TARGET_RPM);
        }
    }

    CalculateWheelPID(&left_wheel, left_delta, elapsed_ms, LEFT_ENCODER_PPR, dt,
                      control_left_target_rpm);
    CalculateWheelPID(&right_wheel, right_delta, elapsed_ms, RIGHT_ENCODER_PPR, dt,
                      control_right_target_rpm);

    if (control_enabled == 0U)
    {
        last_control_tick = now_tick;
        return;
    }

    float output_min = (float)MOTOR_PWM_MIN_RUNNING;
    float output_max = (float)MOTOR_PWM_MAX;

    left_wheel.base_pwm = (int32_t)lroundf(
        CalculateFeedForwardPwm(control_left_target_rpm));
    right_wheel.base_pwm = (int32_t)lroundf(
        CalculateFeedForwardPwm(control_right_target_rpm));

    float left_error  = ApplyDeadband(control_left_target_rpm - left_wheel.filtered_rpm);
    float right_error = ApplyDeadband(control_right_target_rpm - right_wheel.filtered_rpm);

    float left_unsaturated = (float)left_wheel.base_pwm + left_wheel.p_term +
                             left_wheel.integral_pwm + left_wheel.d_term;
    float right_unsaturated = (float)right_wheel.base_pwm + right_wheel.p_term +
                              right_wheel.integral_pwm + right_wheel.d_term;

    left_wheel.integral_pwm = UpdateIntegralWithAntiWindup(
        left_wheel.integral_pwm, left_error, left_wheel.ki, dt,
        left_unsaturated, output_min, output_max);
    right_wheel.integral_pwm = UpdateIntegralWithAntiWindup(
        right_wheel.integral_pwm, right_error, right_wheel.ki, dt,
        right_unsaturated, output_min, output_max);

    float left_pid_output  = left_wheel.p_term + left_wheel.integral_pwm + left_wheel.d_term;
    float right_pid_output = right_wheel.p_term + right_wheel.integral_pwm + right_wheel.d_term;

    int32_t left_requested_pwm  = (left_motor_direction == MOTOR_DIR_STOP || left_wheel.ramp.stop_active != 0U) ? left_wheel.applied_pwm : ClampInt32((int32_t)lroundf(left_pid_output + (float)left_wheel.base_pwm), MOTOR_PWM_MIN_RUNNING, MOTOR_PWM_MAX);
    int32_t right_requested_pwm = (right_motor_direction == MOTOR_DIR_STOP || right_wheel.ramp.stop_active != 0U) ? right_wheel.applied_pwm : ClampInt32((int32_t)lroundf(right_pid_output + (float)right_wheel.base_pwm), MOTOR_PWM_MIN_RUNNING, MOTOR_PWM_MAX);

    left_wheel.applied_pwm  = SlewLimit(left_wheel.applied_pwm, left_requested_pwm, PWM_SLEW_STEP);
    right_wheel.applied_pwm = SlewLimit(right_wheel.applied_pwm, right_requested_pwm, PWM_SLEW_STEP);

    if (left_wheel.ramp.stop_active == 0U)  LeftMotor_Apply(left_motor_direction, (uint16_t)left_wheel.applied_pwm);
    if (right_wheel.ramp.stop_active == 0U) RightMotor_Apply(right_motor_direction, (uint16_t)right_wheel.applied_pwm);

    last_control_tick = now_tick;
}

void MotorControl_Process(void)
{
    ProcessClosedLoop();
}

/* -------------------------------- Getter'lar ------------------------------- */

float MotorControl_GetLeftRpm(void)  { return left_wheel.filtered_rpm; }
float MotorControl_GetRightRpm(void) { return right_wheel.filtered_rpm; }

float MotorControl_GetTargetRpm(void)      { return left_wheel.target_rpm; }
float MotorControl_GetLeftTargetRpm(void)  { return left_wheel.target_rpm; }
float MotorControl_GetRightTargetRpm(void) { return right_wheel.target_rpm; }

uint16_t MotorControl_GetLeftAppliedPwm(void)  { return (left_wheel.applied_pwm > 0) ? (uint16_t)left_wheel.applied_pwm : 0U; }
uint16_t MotorControl_GetRightAppliedPwm(void) { return (right_wheel.applied_pwm > 0) ? (uint16_t)right_wheel.applied_pwm : 0U; }

uint32_t MotorControl_GetLeftRawCount(void)  { return left_wheel.raw_count; }
uint32_t MotorControl_GetRightRawCount(void) { return right_wheel.raw_count; }

uint32_t MotorControl_GetLeftRejectedCount(void)  { return left_wheel.rejected_count; }
uint32_t MotorControl_GetRightRejectedCount(void) { return right_wheel.rejected_count; }

float MotorControl_GetLeftTargetRpmCommand(void)  { return left_target_rpm_command; }
float MotorControl_GetRightTargetRpmCommand(void) { return right_target_rpm_command; }

void MotorControl_UpdatePID(char param, char side, float value)
{
    WheelControl_t *wheel = (side == 'L' || side == 'l') ? &left_wheel : ((side == 'R' || side == 'r') ? &right_wheel : NULL);
    if (wheel == NULL) return;

    if (param == 'P' || param == 'p')      wheel->kp = value;
    else if (param == 'I' || param == 'i') wheel->ki = value;
    else if (param == 'D' || param == 'd') wheel->kd = value;
}

int16_t MotorControl_GetLeftMeasuredMmPs(void)
{
    float rpm_mmps = ClampFloat(RPM_TO_MMPS(left_wheel.filtered_rpm), 0.0f, 32767.0f);
    int16_t measured_mmps = (int16_t)lroundf(rpm_mmps);
    if (left_motor_direction == MOTOR_DIR_REVERSE) {
        return -measured_mmps;
    }
    return measured_mmps;
}

int16_t MotorControl_GetRightMeasuredMmPs(void)
{
    float rpm_mmps = ClampFloat(RPM_TO_MMPS(right_wheel.filtered_rpm), 0.0f, 32767.0f);
    int16_t measured_mmps = (int16_t)lroundf(rpm_mmps);
    if (right_motor_direction == MOTOR_DIR_REVERSE) {
        return -measured_mmps;
    }
    return measured_mmps;
}

uint8_t MotorControl_IsEnabled(void) { return control_enabled; }
uint8_t MotorControl_WasCommandClamped(void) { return command_clamped; }

uint8_t MotorControl_HasEncoderFault(void)
{
    if (control_enabled == 0U) return 0U;

    uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim2);
    uint8_t left_fault = (left_motor_direction != MOTOR_DIR_STOP) &&
                         ((now_us - left_wheel.last_encoder_us) > 500000U);
    uint8_t right_fault = (right_motor_direction != MOTOR_DIR_STOP) &&
                          ((now_us - right_wheel.last_encoder_us) > 500000U);
    return (left_fault || right_fault) ? 1U : 0U;
}

int32_t MotorControl_GetLeftOdoTicks(void)  { return left_odo_ticks; }
int32_t MotorControl_GetRightOdoTicks(void) { return right_odo_ticks; }
