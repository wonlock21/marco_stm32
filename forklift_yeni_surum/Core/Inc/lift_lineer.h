#ifndef LIFT_LINEER_H
#define LIFT_LINEER_H

#include "stm32f7xx_hal.h"

/* Lift için varsayılan PWM hızı (0-255 arası) */
#define LIFT_DEFAULT_SPEED 255U

/* Fonksiyon Prototipleri */
void LiftLineer_Init(void);
void LiftLineer_Up(uint16_t pwm_value);
void LiftLineer_Down(uint16_t pwm_value);
void LiftLineer_Stop(void);

#endif /* LIFT_LINEER_H */
