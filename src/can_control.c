/* CAN control implementation */
#include "can_control.h"
#include "servo.h"
#include "dc_motor.h"
#include "led.h"
#include <stdio.h>

#define DC_SPEED 100  // DCモーター速度 (%)

static CAN_HandleTypeDef *hcan_ctrl = NULL;
static TIM_HandleTypeDef *htim_ctrl = NULL;

static void can_filter_config(void);

// CAN受信初期化
void can_control_init(CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *htim) {
  hcan_ctrl = hcan;
  htim_ctrl = htim;

  // CANフィルター設定
  can_filter_config();

  // CAN受信割り込み開始
  HAL_CAN_Start(hcan_ctrl);
  HAL_CAN_ActivateNotification(hcan_ctrl, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// CANフィルター設定
static void can_filter_config(void) {
  CAN_FilterTypeDef filter_config;
  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterIdHigh = (CAN_ID_MOTOR << 5);
  filter_config.FilterIdLow = 0;
  filter_config.FilterMaskIdHigh = (0x7FF << 5);
  filter_config.FilterMaskIdLow = 0;
  filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter_config.FilterActivation = ENABLE;
  filter_config.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan_ctrl, &filter_config);
}

// CAN受信コールバック
void can_control_rx_callback(CAN_HandleTypeDef *hcan) {
  led_set(LED_COLOR_YELLOW, LED_STATE_ON);
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // FIFO0からメッセージ取得
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
    return;
  }

  // モーター駆動
  if (rx_header.StdId == CAN_ID_MOTOR) {
    // データ: [DCモーター1, DCモーター2, サーボモーター]
    if (rx_header.DLC >= 3) {
      // DCモーター1
      // 0: 逆転(引っ込む), 1: 正転(出っ張る), それ以外: 無視
      if (rx_data[0] == 0) {
        dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_REVERSE, DC_SPEED);
        printf("DC Motor 1: Reverse\n");
      } else if (rx_data[0] == 1) {
        dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_FORWARD, DC_SPEED);
        printf("DC Motor 1: Forward\n");
      }

      // DCモーター2
      // 0: 逆転(引っ込む), 1: 正転(出っ張る), それ以外: 無視
      if (rx_data[1] == 0) {
        dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_REVERSE, DC_SPEED);
        printf("DC Motor 2: Reverse\n");
      } else if (rx_data[1] == 1) {
        dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_FORWARD, DC_SPEED);
        printf("DC Motor 2: Forward\n");
      }

      // サーボモーター
      // 0: 角度増加(270度), 1: 現状維持, 2: 角度減少(0度)
      if (rx_data[2] == 0) {
        // 角度を増やす（270度方向へ）
        // servo_set_angle(2700);
        servo_control(SERVO_DIR_OPEN, SERVO_MODE_NORMAL);
        printf("Servo Motor: Open Gripper\n");
      } else if (rx_data[2] == 2) {
        // 角度を減らす（0度方向へ）
        // servo_set_angle(0);
        servo_control(SERVO_DIR_CLOSE, SERVO_MODE_NORMAL);
        printf("Servo Motor: Close Gripper\n");
      }
      // rx_data[2] == 1 の場合は何もしない（現在位置保持）
    }
  }
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
}
