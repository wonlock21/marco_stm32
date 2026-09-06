#ifndef INC_UARTCOM_H_
#define INC_UARTCOM_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t right_pwm;
    int16_t left_pwm;
} MotorTargetCommand_t;

void UartCom_Init(UART_HandleTypeDef *huart);
void UartCom_BluetoothInit(UART_HandleTypeDef *huart);

bool UartCom_RxCallback(UART_HandleTypeDef *huart, MotorTargetCommand_t *command);
bool UartCom_BluetoothRxCallback(UART_HandleTypeDef *huart, MotorTargetCommand_t *command);

#endif /* INC_UARTCOM_H_ */
