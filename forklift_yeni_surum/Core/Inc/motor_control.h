#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* Orange Pi/Bluetooth katmanlarindan gelen isaretli RPM hedefleri. */
extern volatile float left_target_rpm_command;
extern volatile float right_target_rpm_command;

/* -------------------- Sistem Başlatma ve Döngü Fonksiyonları -------------------- */
void MotorControl_Init(void);
void MotorControl_Process(void);

/* -------------------------- Sürüş Kontrol Fonksiyonları -------------------------- */
void MotorControl_SetTargetRpm(float left_signed_rpm, float right_signed_rpm);
void MotorControl_Stop(void);
void MotorControl_SafeStop(void);

/* ------------------------ Donanım Kesme & Canlı Ayarlar ------------------------- */
void MotorControl_EncoderExtiCallback(uint16_t gpio_pin);
void MotorControl_UpdatePID(char param, char side, float value);

/* --------------------------- Telemetri ve Getter'lar ---------------------------- */
float MotorControl_GetLeftRpm(void);
float MotorControl_GetRightRpm(void);
int16_t MotorControl_GetLeftMeasuredMmPs(void);
int16_t MotorControl_GetRightMeasuredMmPs(void);

/* Eski getter uyumluluğu için korunmuştur */
float MotorControl_GetTargetRpm(void);
float MotorControl_GetLeftTargetRpm(void);
float MotorControl_GetRightTargetRpm(void);

uint16_t MotorControl_GetLeftAppliedPwm(void);
uint16_t MotorControl_GetRightAppliedPwm(void);

uint32_t MotorControl_GetLeftRawCount(void);
uint32_t MotorControl_GetRightRawCount(void);
uint32_t MotorControl_GetLeftRejectedCount(void);
uint32_t MotorControl_GetRightRejectedCount(void);

float MotorControl_GetLeftTargetRpmCommand(void);
float MotorControl_GetRightTargetRpmCommand(void);

uint8_t MotorControl_IsEnabled(void);
uint8_t MotorControl_WasCommandClamped(void);
uint8_t MotorControl_HasEncoderFault(void);

int32_t MotorControl_GetLeftOdoTicks(void);
int32_t MotorControl_GetRightOdoTicks(void);

//void MotorControl_SetTargetMmPs(float left_mmps, float right_mmps);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
