#include "acs712.h"

float ACS712_ReadCurrent(ADC_HandleTypeDef *hadc) {
    uint32_t total_adc = 0;
    uint32_t valid_samples = 0;
    const int num_samples = 100;

    // Arka arkaya 100 kez oku ve topla
    for(int i = 0; i < num_samples; i++) {
        if (HAL_ADC_Start(hadc) != HAL_OK) {
            break;
        }

        if (HAL_ADC_PollForConversion(hadc, 5U) != HAL_OK) {
            HAL_ADC_Stop(hadc);
            break;
        }

        total_adc += HAL_ADC_GetValue(hadc);
        valid_samples++;
    }

    if (valid_samples == 0U) {
        return 0.0f;
    }

    // Ortalamayı bul
    uint32_t adc_raw = total_adc / valid_samples;

    // Voltaj ve Akım Hesaplamaları
    float v_adc = (adc_raw * 3.3f) / 4095.0f;
    float v_sensor = v_adc * 1.56085f;

    // Yön düzeltmesi yapılmış formül
    float current = (2.5f - v_sensor) / 0.066f;

    // Gürültü bandını sıfırla
    if(current > -0.05f && current < 0.05f) {
        current = 0.0f;
    }

    // Sabit ofset düzeltmeniz varsa buraya ekleyebilirsiniz (örnek: current -= 0.20f;)

    return current;
}
