/* CAN control implementation */
#include "can_control.h"
#include "servo.h"
#include "dc_motor.h"
#include <stdio.h>

// CAN関連ハンドル
static CAN_HandleTypeDef *hcan_ctrl = NULL;
static TIM_HandleTypeDef *htim_ctrl = NULL;

// CANフィルター設定用
static void can_filter_config(void);

void can_control_init(CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *htim) {
  hcan_ctrl = hcan;
  htim_ctrl = htim;

  // CANフィルター設定
  can_filter_config();

  // CAN受信割り込み開始
  HAL_CAN_Start(hcan_ctrl);
  HAL_CAN_ActivateNotification(hcan_ctrl, CAN_IT_RX_FIFO0_MSG_PENDING);
}

static void can_filter_config(void) {
  CAN_FilterTypeDef filter_config;

  // フィルター設定: ID 0x100〜0x102を受信
  // マスクモードでフィルタリング
  // ID: 0x100 (0001 0000 0000)
  // Mask: 0x7FC (0111 1111 1100) → 下位2ビット以外一致
  // これにより0x100, 0x101, 0x102を受信

  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterIdHigh = (CAN_ID_SERVO << 5);  // 標準ID (11bit) を上位にシフト
  filter_config.FilterIdLow = 0;
  filter_config.FilterMaskIdHigh = (0x7FC << 5);     // マスク: 0x100〜0x103を許可
  filter_config.FilterMaskIdLow = 0;
  filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter_config.FilterActivation = ENABLE;
  filter_config.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan_ctrl, &filter_config);
}

void can_control_rx_callback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // FIFO0からメッセージ取得
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
    return;
  }

  // CAN受信情報を出力
  printf("CAN RX | ID: 0x%03lX, DLC: %lu, Data: ", rx_header.StdId, rx_header.DLC);
  for (uint8_t i = 0; i < rx_header.DLC; i++) {
    printf("%02X ", rx_data[i]);
  }
  printf("\n");

  // 受信したCAN IDに応じて処理
  switch (rx_header.StdId) {
    case CAN_ID_SERVO:
      // サーボモーター制御
      // データ: [angle_high, angle_low]
      if (rx_header.DLC >= 2) {
        uint16_t angle_x10 = (rx_data[0] << 8) | rx_data[1];
        servo_set_angle(angle_x10);
      }
      break;

    case CAN_ID_MOTOR1:
      // DCモーター1制御
      // データ: [direction, duty]
      if (rx_header.DLC >= 2) {
        DcMotorDirection direction = (DcMotorDirection)rx_data[0];
        uint8_t duty = rx_data[1];
        dc_motor_set(DC_MOTOR_1, direction, duty);
      }
      break;

    case CAN_ID_MOTOR2:
      // DCモーター2制御
      // データ: [direction, duty]
      if (rx_header.DLC >= 2) {
        DcMotorDirection direction = (DcMotorDirection)rx_data[0];
        uint8_t duty = rx_data[1];
        dc_motor_set(DC_MOTOR_2, direction, duty);
      }
      break;

    default:
      // 未知のID (通常はフィルタで除外されるが念のため)
      break;
  }
}
