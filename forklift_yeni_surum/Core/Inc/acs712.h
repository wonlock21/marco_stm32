#ifndef ACS712_H
#define ACS712_H

#include "main.h" // İşlemci bağımsız çalışabilmesi için (F4 veya F7 fark etmez)

// Fonksiyon prototipi: Parametre olarak kullanılan ADC portunu alır
float ACS712_ReadCurrent(ADC_HandleTypeDef *hadc);

#endif /* ACS712_H */
