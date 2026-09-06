#include "egim_lineer.h"

/* GPIO Pin Tanımlamaları */
/* Eğim ve Lift Ortak Enable Pini (A3 -> PF8) */
#define EGIM_EN_PORT        GPIOF
#define EGIM_EN_PIN         GPIO_PIN_8

/* Eğim Yukarı Pini (D9 -> PA15) - SADECE GPIO OUTPUT */
#define EGIM_UP_PORT        GPIOA
#define EGIM_UP_PIN         GPIO_PIN_15

/* Eğim Aşağı Pini (D13 -> PI1) - SADECE GPIO OUTPUT */
#define EGIM_DOWN_PORT      GPIOI
#define EGIM_DOWN_PIN       GPIO_PIN_1

void EgimLineer_Init(void)
{
    /* Başlangıçta pinleri Low yaparak motoru kesin olarak durdur */
    EgimLineer_Stop();
}

/* pwm_value parametresi uartcom.c hata vermesin diye tutuldu, içeride kullanılmaz */
void EgimLineer_Up(uint16_t pwm_value)
{
    /* 1. Güvenlik: Önce aşağı yönü KES (Low yap) */
    HAL_GPIO_WritePin(EGIM_DOWN_PORT, EGIM_DOWN_PIN, GPIO_PIN_RESET);
    /* 2. Sürücüyü aktif et (Enable = 1) */
    HAL_GPIO_WritePin(EGIM_EN_PORT, EGIM_EN_PIN, GPIO_PIN_SET);
    /* 3. Yukarı yöne TAM GÜÇ ver (High yap) */
    HAL_GPIO_WritePin(EGIM_UP_PORT, EGIM_UP_PIN, GPIO_PIN_SET);
}

void EgimLineer_Down(uint16_t pwm_value)
{
    /* 1. Güvenlik: Önce yukarı yönü KES (Low yap) */
    HAL_GPIO_WritePin(EGIM_UP_PORT, EGIM_UP_PIN, GPIO_PIN_RESET);
    /* 2. Sürücüyü aktif et (Enable = 1) */
    HAL_GPIO_WritePin(EGIM_EN_PORT, EGIM_EN_PIN, GPIO_PIN_SET);
    /* 3. Aşağı yöne TAM GÜÇ ver (High yap) */
    HAL_GPIO_WritePin(EGIM_DOWN_PORT, EGIM_DOWN_PIN, GPIO_PIN_SET);
}

void EgimLineer_Stop(void)
{
    /* İki yönü de Low yaparak motoru elektriksel olarak frenle/durdur */
    HAL_GPIO_WritePin(EGIM_UP_PORT, EGIM_UP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EGIM_DOWN_PORT, EGIM_DOWN_PIN, GPIO_PIN_RESET);

    // Ortak Enable pinini (PF8) Lift de kullandığı için burada bilerek kapatmıyoruz.
}
