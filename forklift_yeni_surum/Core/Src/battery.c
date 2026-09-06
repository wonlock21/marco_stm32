#include "battery.h"

#define SAMPLE_SIZE 100
#define TRIM_OFFSET 30
#define VALID_SAMPLES (SAMPLE_SIZE - (2 * TRIM_OFFSET))
#define EMA_ALPHA 0.15f
#define LUT_SIZE 9

static const float adc_table[LUT_SIZE] = {
    2019.0f, 2112.5f, 2201.5f, 2293.0f, 2385.5f, 2476.5f, 2567.5f, 2659.5f, 2751.0f
};

static const float volt_table[LUT_SIZE] = {
    11.0f, 11.5f, 12.0f, 12.5f, 13.0f, 13.5f, 14.0f, 14.5f, 15.0f
};

static void bubbleSort(uint32_t *array, uint16_t size) {
    for (uint16_t i = 0; i < size - 1; i++) {
        for (uint16_t j = 0; j < size - i - 1; j++) {
            if (array[j] > array[j + 1]) {
                uint32_t temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
}

static float calculate_voltage_from_lut(uint32_t adc_val) {
    float f_adc = (float)adc_val;

    if (f_adc <= adc_table[0]) return volt_table[0];
    if (f_adc >= adc_table[LUT_SIZE - 1]) return volt_table[LUT_SIZE - 1];

    for (int i = 0; i < LUT_SIZE - 1; i++) {
        if (f_adc >= adc_table[i] && f_adc <= adc_table[i + 1]) {
            float v_diff = volt_table[i + 1] - volt_table[i];
            float adc_diff = adc_table[i + 1] - adc_table[i];

            return volt_table[i] + (v_diff / adc_diff) * (f_adc - adc_table[i]);
        }
    }
    return 0.0f;
}

float Battery_GetVoltage(ADC_HandleTypeDef *hadc) {
    // Statik değişken, fonksiyondan çıkılsa bile eski değeri hafızada tutar
    static float smoothed_adc = 0.0f;

    // KRİTİK DÜZELTME: static eklenerek FreeRTOS Stack Overflow (Çökme) engellendi
    static uint32_t adc_samples[SAMPLE_SIZE];

    for(int i = 0; i < SAMPLE_SIZE; i++) {
        HAL_ADC_Start(hadc);
        if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
            adc_samples[i] = HAL_ADC_GetValue(hadc);
        } else {
            adc_samples[i] = 0;
        }
        HAL_ADC_Stop(hadc);
    }

    // ... Fonksiyonun geri kalanı tamamen aynı kalacak ...

    bubbleSort(adc_samples, SAMPLE_SIZE);

    // En düşük ve en yüksek 30'ar hatalı sıçramayı çöpe at
    uint32_t sum = 0;
    for(int i = TRIM_OFFSET; i < SAMPLE_SIZE - TRIM_OFFSET; i++) {
        sum += adc_samples[i];
    }

    // Kalan güvenilir 40 verinin ortalamasını al
    float current_avg = (float)sum / VALID_SAMPLES;

    // Zaman bazlı EMA (Low Pass) Filtresi
    if (smoothed_adc == 0.0f) {
        smoothed_adc = current_avg;
    } else {
        smoothed_adc = (EMA_ALPHA * current_avg) + ((1.0f - EMA_ALPHA) * smoothed_adc);
    }

    // Filtrelenmiş değere manuel eklediğiniz +200 ofseti ilave et
    uint32_t final_adc = (uint32_t)smoothed_adc + 200;

    return calculate_voltage_from_lut(final_adc);
}
