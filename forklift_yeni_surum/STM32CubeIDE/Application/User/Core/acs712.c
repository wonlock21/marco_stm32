#include "acs712.h"

float ACS712_ReadCurrent(ADC_HandleTypeDef *hadc) {
    uint32_t total_adc = 0;
    const int num_samples = 100;

    // Arka arkaya 100 kez oku ve topla
    for(int i = 0; i < num_samples; i++) {
        HAL_ADC_Start(hadc);
        HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
        total_adc += HAL_ADC_GetValue(hadc);
    }

    // Ortalamayı bul
    uint32_t adc_raw = total_adc / num_samples;

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
