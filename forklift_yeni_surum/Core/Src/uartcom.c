#include "uartcom.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "lift_lineer.h"
#include "motor_control.h"
#include "egim_lineer.h"
#include "buzzer_control.h"

#define BLUETOOTH_ILERI_L       45
#define BLUETOOTH_ILERI_R       45
#define BLUETOOTH_GERI_L        45
#define BLUETOOTH_GERI_R        45
#define BLUETOOTH_DONUS_L       15
#define BLUETOOTH_DONUS_R       15
#define UARTCOM_PWM_MAX         255

#define USB_UART_BUFFER_SIZE     32U
#define BLUETOOTH_BUFFER_SIZE    16U

static UART_HandleTypeDef *usb_uart_handle = NULL;
static UART_HandleTypeDef *bluetooth_uart_handle = NULL;

// Pointer uyarısı almamak için 1 elemanlı diziler (array) olarak tanımlandı
static uint8_t usb_rx_byte[1];
static uint8_t bluetooth_rx_byte[1];

static char usb_rx_buffer[USB_UART_BUFFER_SIZE];
static char bluetooth_rx_buffer[BLUETOOTH_BUFFER_SIZE];

static uint16_t usb_rx_index = 0U;
static uint16_t bluetooth_rx_index = 0U;

static MotorTargetCommand_t last_forwarded_command;
static bool last_forwarded_command_valid = false;

static int16_t ClampPwm(int32_t value);
static bool ParseUsbCommand(const char *text, MotorTargetCommand_t *command);
static bool ParseBluetoothCommand(const char *text, MotorTargetCommand_t *command);
static bool IsCommandChanged(const MotorTargetCommand_t *command);
static uint16_t NormalizeBluetoothCode(uint16_t code);
static void RestartUsbReception(void);
static void RestartBluetoothReception(void);

static int16_t ClampPwm(int32_t value)
{
    if (value > UARTCOM_PWM_MAX)
    {
        value = UARTCOM_PWM_MAX;
    }
    else if (value < -UARTCOM_PWM_MAX)
    {
        value = -UARTCOM_PWM_MAX;
    }

    return (int16_t)value;
}

static bool IsCommandChanged(const MotorTargetCommand_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    if (!last_forwarded_command_valid)
    {
        last_forwarded_command = *command;
        last_forwarded_command_valid = true;
        return true;
    }

    if ((command->right_pwm == last_forwarded_command.right_pwm) &&
        (command->left_pwm == last_forwarded_command.left_pwm))
    {
        return false;
    }

    last_forwarded_command = *command;

    return true;
}

/* USB formatı: sağ/sol, örnek: 127/-56 */
static bool ParseUsbCommand(const char *text, MotorTargetCommand_t *command)
{
    char *separator;
    char *right_end;
    char *left_end;
    long right_value;
    long left_value;

    if ((text == NULL) || (command == NULL))
    {
        return false;
    }

    separator = strchr(text, '/');

    if (separator == NULL)
    {
        return false;
    }

    right_value = strtol(text, &right_end, 10);

    if (right_end != separator)
    {
        return false;
    }

    left_value = strtol(separator + 1, &left_end, 10);

    while ((*left_end == ' ') || (*left_end == '\t'))
    {
        left_end++;
    }

    if (*left_end != '\0')
    {
        return false;
    }

    command->right_pwm = ClampPwm(right_value);
    command->left_pwm = ClampPwm(left_value);

    return true;
}

static uint16_t NormalizeBluetoothCode(uint16_t code)
{
    switch (code)
    {
        case 21:
            return 12;

        case 41:
            return 14;

        case 23:
            return 32;

        case 43:
            return 34;


        default:
            return code;
    }
}

/*
 * Bluetooth:
 * 1=W, 2=D, 3=S, 4=A
 * 12=W+D, 14=W+A, 23=S+D, 34=S+A
 */
static bool ParseBluetoothCommand(
    const char *text,
    MotorTargetCommand_t *command)
{
    char *end_pointer;
    long received_code;
    uint16_t command_code;

    const int16_t forward_pwm_l = BLUETOOTH_ILERI_L;
    const int16_t forward_pwm_r = BLUETOOTH_ILERI_R;
    const int16_t back_pwm_l = BLUETOOTH_GERI_L;
    const int16_t back_pwm_r = BLUETOOTH_GERI_R;
    const int16_t turn_pwm_l = BLUETOOTH_DONUS_L;
    const int16_t turn_pwm_r = BLUETOOTH_DONUS_R;

    if ((text == NULL) || (command == NULL))
    {
        return false;
    }

    /* ======================================================== */
    /* ---- YENİ EKLENEN: CANLI P.I.D KOMUT KONTROLÜ ----       */
    /* ======================================================== */
    const char *p = text;

    /* Gelen metnin başındaki olası boşlukları atla */
    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }

    /* Eğer komut P, I veya D harfiyle başlıyorsa bu bir PID komutudur */
    if (p[0] == 'P' || p[0] == 'p' ||
        p[0] == 'I' || p[0] == 'i' ||
        p[0] == 'D' || p[0] == 'd')
    {
        /* İkinci harf L veya R (Sol/Sağ) olmalı */
        if (p[1] == 'L' || p[1] == 'l' || p[1] == 'R' || p[1] == 'r')
        {
            char param = toupper((unsigned char)p[0]); // P, I veya D
            char side = toupper((unsigned char)p[1]);  // L veya R

            /* İlk iki harfi atlayıp kalan sayıyı virgüllü (float) olarak çek */
            float val = strtof(&p[2], NULL);

            /* Yeni değeri motor kontrolcüye gönder (Gücü kesmeden uygular) */
            MotorControl_UpdatePID(param, side, val);

            /* Sürüş hedeflerini (PWM'leri) bozmamak için false dönüyoruz */
            return false;
        }
    }
    /* ======================================================== */

    /* EĞER HARF YOKSA ESKİ SİSTEM GİBİ SAYI (SÜRÜŞ KOMUTU) OLARAK İŞLE */

    /*
     * DÜZELTME: Çapraz sürüş bozulmasın diye arkadaki çöpleri / boşlukları
     * reddeden o eski katı "if (*end_pointer != '\0')" kontrolünü sildik!
     */
    received_code = strtol(text, &end_pointer, 10);

    if ((received_code < 0) || (received_code > 99))
    {
        return false;
    }

    command_code = NormalizeBluetoothCode((uint16_t)received_code);

    switch (command_code)
            {
                case 0: /* Dur */
                    command->right_pwm = 0;
                    command->left_pwm = 0;
                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    break;

                case 1: /* W - ileri */
                    command->right_pwm = -forward_pwm_r;
                    command->left_pwm = forward_pwm_l;
                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    break;

                case 2: /* D - sağa dön */
                    command->right_pwm = turn_pwm_r;
                    command->left_pwm = turn_pwm_l;
                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    	break;

                case 12: /* W+D - Çapraz Sağ */
                    command->right_pwm = (-turn_pwm_r)/2;
                    command->left_pwm = forward_pwm_l;
                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    break;

                case 3: /* S - geri */
                    command->right_pwm = back_pwm_r;
                    command->left_pwm = -back_pwm_l;
                    BuzzerControl_SetMode(BUZZER_MODE_REVERSE);
                    break;

                case 4: /* A - sola dön */
                    command->right_pwm = -turn_pwm_r;
                    command->left_pwm = -turn_pwm_l;
                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    	break;

                case 14: /* W+A - Çapraz Sol */
                    command->right_pwm = -forward_pwm_r;
                    command->left_pwm = (turn_pwm_l )/2;
                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    break;

                case 32: /* S+D - Geri Sağ */
                    command->right_pwm = (turn_pwm_r)/2;
                    command->left_pwm = -back_pwm_l;
                    BuzzerControl_SetMode(BUZZER_MODE_REVERSE);
                    break;

                case 34: /* S+A - Geri Sol */
                    command->right_pwm = back_pwm_r;
                    command->left_pwm = (-turn_pwm_l)/2;
                    BuzzerControl_SetMode(BUZZER_MODE_REVERSE);
                    break;

                /* ---- LİFT KOMUTLARI ---- */
                case 9: /* Lift Yukarı */
                    LiftLineer_Up(LIFT_DEFAULT_SPEED);
                    BuzzerControl_SetMode(BUZZER_MODE_LOAD_PICKUP);
                    return false;

                case 10: /* Lift Aşağı */
                    LiftLineer_Down(LIFT_DEFAULT_SPEED);
                    BuzzerControl_SetMode(BUZZER_MODE_LOAD_DROP);
                    return false;

                case 11: /* Lift Dur */
                    LiftLineer_Stop();
                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    return false;

                /* ---- EĞİM KOMUTLARI ---- */
                case 17: /* Eğim Yukarı */
                    EgimLineer_Up(EGIM_DEFAULT_SPEED);
                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    return false;

                case 19: /* Eğim Dur */
                    EgimLineer_Stop();

                    BuzzerControl_SetMode(BUZZER_MODE_SILENT);
                    return false;

                case 18: /* Eğim Aşağı */
                    EgimLineer_Down(EGIM_DEFAULT_SPEED);
                    BuzzerControl_SetMode(BUZZER_MODE_LOAD_DROP);
                    return false;

                default:
                    return false;
            }

    command->right_pwm = ClampPwm(command->right_pwm);
    command->left_pwm = ClampPwm(command->left_pwm);

    return true;
}
static void RestartUsbReception(void)
{
    if (usb_uart_handle != NULL)
    {
        HAL_UART_Receive_IT(usb_uart_handle, usb_rx_byte, 1U);
    }
}

static void RestartBluetoothReception(void)
{
    if (bluetooth_uart_handle != NULL)
    {
        HAL_UART_Receive_IT(bluetooth_uart_handle, bluetooth_rx_byte, 1U);
    }
}

void UartCom_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    usb_uart_handle = huart;
    usb_rx_index = 0U;

    memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));

    RestartUsbReception();
}

void UartCom_BluetoothInit(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    bluetooth_uart_handle = huart;
    bluetooth_rx_index = 0U;

    memset(bluetooth_rx_buffer, 0, sizeof(bluetooth_rx_buffer));

    RestartBluetoothReception();
}

// ==========================================
// USB (PC) VERİLERİNİ DİNLEME VE AYIRMA KISMI
// ==========================================
bool UartCom_RxCallback(UART_HandleTypeDef *huart, MotorTargetCommand_t *command)
{
    bool command_ready = false;
    MotorTargetCommand_t parsed_command;

    if ((huart == NULL) || (command == NULL) || (usb_uart_handle == NULL))
    {
        return false;
    }

    if (huart->Instance != usb_uart_handle->Instance)
    {
        return false;
    }

    if ((usb_rx_byte[0] == '\r') || (usb_rx_byte[0] == '\n'))
    {
        if (usb_rx_index > 0U)
        {
            usb_rx_buffer[usb_rx_index] = '\0';

            if (ParseUsbCommand(usb_rx_buffer, &parsed_command) &&
                IsCommandChanged(&parsed_command))
            {
                *command = parsed_command;
                command_ready = true;
            }
        }

        usb_rx_index = 0U;
        memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));
    }
    else if (isdigit((unsigned char)usb_rx_byte[0]) ||
             (usb_rx_byte[0] == '-') ||
             (usb_rx_byte[0] == '+') ||
             (usb_rx_byte[0] == '/') ||
             (usb_rx_byte[0] == ' ') ||
             (usb_rx_byte[0] == '\t'))
    {
        if (usb_rx_index < (USB_UART_BUFFER_SIZE - 1U))
        {
            usb_rx_buffer[usb_rx_index++] = (char)usb_rx_byte[0];
        }
        else
        {
            usb_rx_index = 0U;
            memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));
        }
    }
    else
    {
        usb_rx_index = 0U;
        memset(usb_rx_buffer, 0, sizeof(usb_rx_buffer));
    }

    RestartUsbReception();

    return command_ready;
}

// ==========================================
// BLUETOOTH (HC-06) VERİLERİNİ DİNLEME VE AYIRMA KISMI
// ==========================================
// ==========================================
// BLUETOOTH (HC-06) VERİLERİNİ DİNLEME VE AYIRMA KISMI
// ==========================================
bool UartCom_BluetoothRxCallback(UART_HandleTypeDef *huart, MotorTargetCommand_t *command)
{
    bool command_ready = false;
    MotorTargetCommand_t parsed_command;

    if ((huart == NULL) || (command == NULL) || (bluetooth_uart_handle == NULL))
    {
        return false;
    }

    if (huart->Instance != bluetooth_uart_handle->Instance)
    {
        return false;
    }

    /* Satır sonu (Enter/CRLF) geldiyse paketi işlemeye başla */
    if ((bluetooth_rx_byte[0] == '\r') || (bluetooth_rx_byte[0] == '\n'))
    {
        if (bluetooth_rx_index > 0U)
        {
            bluetooth_rx_buffer[bluetooth_rx_index] = '\0';

            if (ParseBluetoothCommand(bluetooth_rx_buffer, &parsed_command) &&
                IsCommandChanged(&parsed_command))
            {
                *command = parsed_command;
                command_ready = true;
            }
        }

        bluetooth_rx_index = 0U;
        memset(bluetooth_rx_buffer, 0, sizeof(bluetooth_rx_buffer));
    }
    /*
     * YENİ GÜNCELLEME: isdigit yerine isalnum kullanıldı (A-Z, a-z, 0-9 kabul edilir).
     * Ayrıca virgüllü (float) PID değerleri için '.' (nokta) karakterine izin verildi.
     */
    else if (isalnum((unsigned char)bluetooth_rx_byte[0]) ||
             (bluetooth_rx_byte[0] == '.') ||
             (bluetooth_rx_byte[0] == ' ') ||
             (bluetooth_rx_byte[0] == '\t'))
    {
        if (bluetooth_rx_index < (BLUETOOTH_BUFFER_SIZE - 1U))
        {
            bluetooth_rx_buffer[bluetooth_rx_index++] = (char)bluetooth_rx_byte[0];
        }
        else
        {
            /* Buffer dolarsa (taşma riski varsa) çöpe at ve sıfırla */
            bluetooth_rx_index = 0U;
            memset(bluetooth_rx_buffer, 0, sizeof(bluetooth_rx_buffer));
        }
    }
    else
    {
        /* Tanımsız bir çöp karakter geldiyse tamponu tamamen sıfırla */
        bluetooth_rx_index = 0U;
        memset(bluetooth_rx_buffer, 0, sizeof(bluetooth_rx_buffer));
    }

    /* Kesmeyi (Interrupt) bir sonraki byte'ı dinlemek üzere tekrar kur */
    RestartBluetoothReception();

    return command_ready;
}
