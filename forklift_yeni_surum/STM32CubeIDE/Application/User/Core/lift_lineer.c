#include "lift_lineer.h"

/* GPIO Pin Tanımlamaları */
/* Lift ve Eğim Ortak Enable Pini (A3 -> PF8) */
#define LIFT_EN_PORT        GPIOF
#define LIFT_EN_PIN         GPIO_PIN_8

/* Lift Yukarı Pini (D6 -> PH6) - SADECE GPIO OUTPUT */
#define LIFT_UP_PORT        GPIOH
#define LIFT_UP_PIN         GPIO_PIN_6

/* Lift Aşağı Pini (D12 -> PB14) - SADECE GPIO OUTPUT */
#define LIFT_DOWN_PORT      GPIOB
#define LIFT_DOWN_PIN       GPIO_PIN_14

void LiftLineer_Init(void)
{
    /* Başlangıçta sistemi kapalı tut */
    LiftLineer_Stop();
}

void LiftLineer_Up(uint16_t pwm_value)
{
    /* 1. Güvenlik: Önce aşağı inme sinyalini tamamen sıfırla */
    HAL_GPIO_WritePin(LIFT_DOWN_PORT, LIFT_DOWN_PIN, GPIO_PIN_RESET);
    /* 2. Sürücüyü aktif et (Enable = 1) */
    HAL_GPIO_WritePin(LIFT_EN_PORT, LIFT_EN_PIN, GPIO_PIN_SET);
    /* 3. Yukarı pinine HIGH ver */
    HAL_GPIO_WritePin(LIFT_UP_PORT, LIFT_UP_PIN, GPIO_PIN_SET);
}

void LiftLineer_Down(uint16_t pwm_value)
{
    /* 1. Güvenlik: Önce yukarı çıkma sinyalini tamamen sıfırla */
    HAL_GPIO_WritePin(LIFT_UP_PORT, LIFT_UP_PIN, GPIO_PIN_RESET);
    /* 2. Sürücüyü aktif et (Enable = 1) */
    HAL_GPIO_WritePin(LIFT_EN_PORT, LIFT_EN_PIN, GPIO_PIN_SET);
    /* 3. Aşağı pinine HIGH ver */
    HAL_GPIO_WritePin(LIFT_DOWN_PORT, LIFT_DOWN_PIN, GPIO_PIN_SET);
}

void LiftLineer_Stop(void)
{
    /* Her iki yöndeki sinyalleri LOW yap */
    HAL_GPIO_WritePin(LIFT_UP_PORT, LIFT_UP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIFT_DOWN_PORT, LIFT_DOWN_PIN, GPIO_PIN_RESET);
}
