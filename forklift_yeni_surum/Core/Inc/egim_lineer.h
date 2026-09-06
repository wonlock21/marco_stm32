#ifndef EGIM_LINEER_H
#define EGIM_LINEER_H

#include "stm32f7xx_hal.h"

/* Eğim lineer motoru için varsayılan PWM hızı (0-255) */
#define EGIM_DEFAULT_SPEED 255U

/* Fonksiyon Prototipleri */
void EgimLineer_Init(void);
void EgimLineer_Up(uint16_t pwm_value);
void EgimLineer_Down(uint16_t pwm_value);
void EgimLineer_Stop(void);

#endif /* EGIM_LINEER_H */
