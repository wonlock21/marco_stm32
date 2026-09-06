#ifndef UART_PROTOKOL_H
#define UART_PROTOKOL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

// Mesaj ID Tanımları
#define MSG_CMD_WHEEL_VELOCITY 0x01
#define MSG_CMD_FORK           0x02
#define MSG_CMD_SAFETY         0x03
#define MSG_CMD_HEARTBEAT      0x05

#define MSG_STATE_ODOMETRY     0x81
#define MSG_STATE_STATUS       0x82

// Watchdog zaman aşımı (spec §5)
#define WATCHDOG_TIMEOUT_MS    200

// CmdVelocity_t.flags biti (spec §3.1)
#define FLAG_MOTORS_ENABLED    (1 << 0)

// StateStatus_t.flags bitleri (spec §4.2)
#define STATUS_FLAG_ESTOP_ACTIVE        (1 << 0)
#define STATUS_FLAG_MODE_MANUAL         (1 << 1)
#define STATUS_FLAG_MOTORS_ENABLED      (1 << 2)
#define STATUS_FLAG_LIMIT_SWITCH_UP     (1 << 3)
#define STATUS_FLAG_LIMIT_SWITCH_DOWN   (1 << 4)
#define STATUS_FLAG_OVERCURRENT         (1 << 5)
#define STATUS_FLAG_WATCHDOG_TRIGGERED  (1 << 6)
#define STATUS_FLAG_CMD_CLAMPED         (1 << 7)
#define STATUS_FLAG_ENCODER_FAULT       (1 << 8)

// İletişim Modları
typedef enum {
  TX_MODE_ON_REQUEST = 0,
  TX_MODE_PERIODIC,
  TX_MODE_HYBRID
} TxMode_t;

typedef struct {
  float left_target;
  float right_target;
  uint8_t flags;
} __attribute__((packed)) CmdVelocity_t;

typedef struct {
  uint8_t action;
  uint16_t timeout_ms;
} __attribute__((packed)) CmdFork_t;

typedef struct {
  uint8_t command;
} __attribute__((packed)) CmdSafety_t;

typedef struct __attribute__((packed)) {
    uint64_t timestamp_us;
    int32_t  left_ticks;
    int32_t  right_ticks;
    int16_t  left_speed;
    int16_t  right_speed;
    float    imu_yaw;       /* Yaw verisi (derece cinsinden float)[cite: 3] */
} StateOdometry_t;

typedef struct {
  uint32_t timestamp_us;
  uint16_t flags;
  uint16_t battery_mv;
  int16_t current_ma_left;
  int16_t current_ma_right;
  int8_t temperature_c;
  uint8_t fork_state;
} __attribute__((packed)) StateStatus_t;

typedef enum {
  RX_STATE_SYNC_1 = 0,
  RX_STATE_SYNC_2,
  RX_STATE_LENGTH,
  RX_STATE_MSG_ID,
  RX_STATE_PAYLOAD,
  RX_STATE_CRC_L,
  RX_STATE_CRC_H
} RxState_t;

typedef struct {
  UART_HandleTypeDef *huart;
  RxState_t rx_state;
  uint8_t rx_length;
  uint8_t rx_msg_id;
  uint8_t rx_payload_buffer[64];
  uint8_t rx_payload_index;
  uint8_t rx_crc_buffer[2];

  // İletişim Modları
  TxMode_t odometry_tx_mode;
  TxMode_t status_tx_mode;

  CmdVelocity_t cmd_velocity;
  CmdFork_t cmd_fork;
  CmdSafety_t cmd_safety;

  volatile bool new_cmd_velocity_flag;
  volatile bool new_cmd_fork_flag;
  volatile bool new_cmd_safety_flag;
  volatile bool heartbeat_received_flag;

  uint32_t last_valid_packet_tick;
  uint32_t last_velocity_command_tick;
  volatile bool watchdog_triggered;
} ProtocolHandler_t;

#ifdef __cplusplus
static_assert(sizeof(CmdVelocity_t) == 9U, "CmdVelocity_t wire size must be 9 bytes");
static_assert(sizeof(StateOdometry_t) == 24U, "StateOdometry_t wire size must be 24 bytes");
static_assert(sizeof(StateStatus_t) == 14U, "StateStatus_t wire size must be 14 bytes");
#else
_Static_assert(sizeof(CmdVelocity_t) == 9U, "CmdVelocity_t wire size must be 9 bytes");
_Static_assert(sizeof(StateOdometry_t) == 24U, "StateOdometry_t wire size must be 24 bytes");
_Static_assert(sizeof(StateStatus_t) == 14U, "StateStatus_t wire size must be 14 bytes");
#endif

void Protocol_Init(ProtocolHandler_t *prot, UART_HandleTypeDef *huart);
void Protocol_UartRxHandler(ProtocolHandler_t *prot, uint8_t rx_byte);
void Protocol_UpdateWatchdog(ProtocolHandler_t *prot, uint32_t current_tick);

void Protocol_DirectSend_Odometry(ProtocolHandler_t *prot, const StateOdometry_t *odo);
void Protocol_DirectSend_Status(ProtocolHandler_t *prot, const StateStatus_t *status);

#endif // UART_PROTOKOL_H
