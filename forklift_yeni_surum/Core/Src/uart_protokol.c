#include "uart_protokol.h"
#include <string.h>

// Dahili CRC16 hesaplama fonksiyonu
static uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

void Protocol_Init(ProtocolHandler_t *prot, UART_HandleTypeDef *huart) {
  prot->huart = huart;
  prot->rx_state = RX_STATE_SYNC_1;
  prot->rx_length = 0;
  prot->rx_msg_id = 0;
  prot->rx_payload_index = 0;
  prot->new_cmd_velocity_flag = false;
  prot->new_cmd_fork_flag = false;
  prot->new_cmd_safety_flag = false;
  prot->heartbeat_received_flag = false;
  prot->last_valid_packet_tick = 0;
  prot->last_velocity_command_tick = HAL_GetTick();
  prot->watchdog_triggered = false;

  memset(&prot->cmd_velocity, 0, sizeof(CmdVelocity_t));
  memset(&prot->cmd_fork, 0, sizeof(CmdFork_t));
  memset(&prot->cmd_safety, 0, sizeof(CmdSafety_t));
}

void Protocol_UartRxHandler(ProtocolHandler_t *prot, uint8_t rx_byte) {
  switch (prot->rx_state) {
    case RX_STATE_SYNC_1:
      if (rx_byte == SYNC_BYTE_1) {
        prot->rx_state = RX_STATE_SYNC_2;
      }
      break;

    case RX_STATE_SYNC_2:
      if (rx_byte == SYNC_BYTE_2) {
        prot->rx_state = RX_STATE_LENGTH;
      } else {
        prot->rx_state = (rx_byte == SYNC_BYTE_1) ? RX_STATE_SYNC_2 : RX_STATE_SYNC_1;
      }
      break;

    case RX_STATE_LENGTH:
      if (rx_byte <= sizeof(prot->rx_payload_buffer)) {
        prot->rx_length = rx_byte;
        prot->rx_state = RX_STATE_MSG_ID;
      } else {
        prot->rx_state = RX_STATE_SYNC_1;
      }
      break;

    case RX_STATE_MSG_ID:
      prot->rx_msg_id = rx_byte;
      prot->rx_payload_index = 0;
      if (prot->rx_length > 0) {
        prot->rx_state = RX_STATE_PAYLOAD;
      } else {
        prot->rx_state = RX_STATE_CRC_L;
      }
      break;

    case RX_STATE_PAYLOAD:
      prot->rx_payload_buffer[prot->rx_payload_index++] = rx_byte;
      if (prot->rx_payload_index >= prot->rx_length) {
        prot->rx_state = RX_STATE_CRC_L;
      }
      break;

    case RX_STATE_CRC_L:
      prot->rx_crc_buffer[0] = rx_byte;
      prot->rx_state = RX_STATE_CRC_H;
      break;

    case RX_STATE_CRC_H:
      prot->rx_crc_buffer[1] = rx_byte;

      // CRC ve Paket Doğrulama Kontrolü
      {
        uint8_t header_check[2 + sizeof(prot->rx_payload_buffer)];
        header_check[0] = prot->rx_length;
        header_check[1] = prot->rx_msg_id;
        memcpy(&header_check[2], prot->rx_payload_buffer, prot->rx_length);

        uint16_t calculated_crc = Calculate_CRC16(header_check, prot->rx_length + 2);
        uint16_t received_crc = (uint16_t)prot->rx_crc_buffer[0] | ((uint16_t)prot->rx_crc_buffer[1] << 8);

        if (calculated_crc == received_crc) {
          prot->last_valid_packet_tick = HAL_GetTick();

          // Mesaj ID'sine göre gelen verileri işle
          switch (prot->rx_msg_id) {
            case MSG_CMD_WHEEL_VELOCITY: // 0x01
              if (prot->rx_length == sizeof(CmdVelocity_t)) {
                memcpy(&prot->cmd_velocity, prot->rx_payload_buffer, sizeof(CmdVelocity_t));
                prot->last_velocity_command_tick = HAL_GetTick();
                prot->watchdog_triggered = false;
                prot->new_cmd_velocity_flag = true;
              }
              break;

            case MSG_CMD_FORK: // 0x02
              if (prot->rx_length == sizeof(CmdFork_t)) {
                memcpy(&prot->cmd_fork, prot->rx_payload_buffer, sizeof(CmdFork_t));
                prot->new_cmd_fork_flag = true;
              }
              break;

            case MSG_CMD_SAFETY: // 0x03
              if (prot->rx_length == sizeof(CmdSafety_t)) {
                memcpy(&prot->cmd_safety, prot->rx_payload_buffer, sizeof(CmdSafety_t));
                prot->new_cmd_safety_flag = true;
              }
              break;

            case MSG_CMD_HEARTBEAT: // 0x05
              prot->heartbeat_received_flag = true;
              break;

            default:
              break;
          }
        }
      }
      prot->rx_state = RX_STATE_SYNC_1;
      break;

    default:
      prot->rx_state = RX_STATE_SYNC_1;
      break;
  }
}

void Protocol_UpdateWatchdog(ProtocolHandler_t *prot, uint32_t current_tick) {
  if (((current_tick - prot->last_velocity_command_tick) > WATCHDOG_TIMEOUT_MS) &&
      !prot->watchdog_triggered) {
    prot->cmd_velocity.left_target = 0.0f;
    prot->cmd_velocity.right_target = 0.0f;
    prot->cmd_velocity.flags &= ~FLAG_MOTORS_ENABLED;
    prot->watchdog_triggered = true;
    prot->new_cmd_velocity_flag = true;
  }
}

void Protocol_DirectSend_Odometry(ProtocolHandler_t *prot, const StateOdometry_t *odo) {
  uint8_t buffer[64];
  uint8_t length = sizeof(StateOdometry_t);

  buffer[0] = length;
  buffer[1] = MSG_STATE_ODOMETRY; // 0x81
  memcpy(&buffer[2], odo, sizeof(StateOdometry_t));

  uint16_t crc = Calculate_CRC16(buffer, length + 2);

  uint8_t frame_header[2] = {SYNC_BYTE_1, SYNC_BYTE_2};
  HAL_UART_Transmit(prot->huart, frame_header, 2, 10);
  HAL_UART_Transmit(prot->huart, buffer, length + 2, 50);
  uint8_t crc_bytes[2] = {(uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8)};
  HAL_UART_Transmit(prot->huart, crc_bytes, 2, 10);
}

void Protocol_DirectSend_Status(ProtocolHandler_t *prot, const StateStatus_t *status) {
  uint8_t buffer[64];
  uint8_t length = sizeof(StateStatus_t);

  buffer[0] = length;
  buffer[1] = MSG_STATE_STATUS; // 0x82
  memcpy(&buffer[2], status, sizeof(StateStatus_t));

  uint16_t crc = Calculate_CRC16(buffer, length + 2);

  uint8_t frame_header[2] = {SYNC_BYTE_1, SYNC_BYTE_2};
  HAL_UART_Transmit(prot->huart, frame_header, 2, 10);
  HAL_UART_Transmit(prot->huart, buffer, length + 2, 50);
  uint8_t crc_bytes[2] = {(uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8)};
  HAL_UART_Transmit(prot->huart, crc_bytes, 2, 10);
}
