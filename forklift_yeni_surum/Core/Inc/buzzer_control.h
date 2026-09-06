#ifndef BUZZER_CONTROL_H_
#define BUZZER_CONTROL_H_

#include "stm32f7xx_hal.h"

/* Buzzer Pin Tanımlamaları (CubeMX'te PF7 seçildi) */
#define BUZZER_PORT     GPIOF
#define BUZZER_PIN      GPIO_PIN_7

/* Buzzer Çalışma Modları */
typedef enum {
    BUZZER_MODE_SILENT = 0,
    BUZZER_MODE_FORWARD,
    BUZZER_MODE_REVERSE,
    BUZZER_MODE_LOAD_PICKUP,
    BUZZER_MODE_LOAD_DROP
} BuzzerMode_t;

/* Fonksiyon Prototipleri */
void BuzzerControl_Init(void);
void BuzzerControl_SetMode(BuzzerMode_t mode);
void BuzzerControl_Process(void);

#endif /* BUZZER_CONTROL_H_ */
