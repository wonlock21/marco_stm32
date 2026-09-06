#include "buzzer_control.h"

static BuzzerMode_t current_mode = BUZZER_MODE_SILENT;
static uint32_t mode_start_tick = 0;

void BuzzerControl_Init(void)
{
    /* Başlangıçta buzzerı sustur */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    current_mode = BUZZER_MODE_SILENT;
}

void BuzzerControl_SetMode(BuzzerMode_t mode)
{
    /* Eğer istenen mod zaten aktifse sistemi yorma */
    if (current_mode == mode)
    {
        return;
    }

    current_mode = mode;
    mode_start_tick = HAL_GetTick(); // Yeni modun başlangıç zamanını kaydet

    /* Mod değiştiğinde hemen sustur ki önceki moddan açık kalmasın */
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void BuzzerControl_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t cycle_time;

    switch (current_mode)
    {
        case BUZZER_MODE_SILENT:
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            break;

        case BUZZER_MODE_FORWARD:
            /* İLERİ: 500ms ON, 500ms OFF (Sakin, aralıklı uyarı) */
            cycle_time = (now - mode_start_tick) % 1000;
            if (cycle_time < 500) {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            }
            break;

        case BUZZER_MODE_REVERSE:
                    /* GERİ: Eskiden ileri sesi olan 500ms ON, 500ms OFF ritmi */
                    cycle_time = (now - mode_start_tick) % 1000;
                    if (cycle_time < 500) {
                        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
                    } else {
                        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
                    }
                    break;

        case BUZZER_MODE_LOAD_PICKUP:
            /* YÜK ALMA: Bip-Bip ... Bip-Bip (2 kısa, 1 uzun boşluk) */
            cycle_time = (now - mode_start_tick) % 1000;
            if (cycle_time < 100) {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);   // ON
            } else if (cycle_time < 200) {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET); // OFF
            } else if (cycle_time < 300) {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);   // ON
            } else {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET); // OFF (700ms)
            }
            break;

        case BUZZER_MODE_LOAD_DROP:
            /* YÜK BIRAKMA: Biiiip-Bip ... Biiiip-Bip (1 uzun, 1 kısa, boşluk) */
            cycle_time = (now - mode_start_tick) % 1000;
            if (cycle_time < 400) {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);   // UZUN ON
            } else if (cycle_time < 500) {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET); // OFF
            } else if (cycle_time < 600) {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);   // KISA ON
            } else {
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET); // OFF (400ms)
            }
            break;

        default:
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            break;
    }
}
