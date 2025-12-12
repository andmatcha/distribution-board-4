/* CAN control implementation */
#include "can_control.h"
#include "servo.h"
#include "dc_motor.h"
#include "led.h"
#include <stdio.h>

// CAN関連ハンドル
static CAN_HandleTypeDef *hcan_ctrl = NULL;
static TIM_HandleTypeDef *htim_ctrl = NULL;

// サーボモーター現在角度 (angle_x10単位: 0〜2700)
static uint16_t servo_current_angle = 1350;  // 初期値135度

// サーボモーター角度変化量 (angle_x10単位)
// サーボ速度: 0.14秒で60度 → 428.6度/秒
// CAN受信間隔: 0.01秒 → 4.29度/受信 ≒ 4度
#define SERVO_ANGLE_STEP  40  // 4度 (angle_x10単位)

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

  // フィルター設定: ID 0x208を受信
  // マスクモードでフィルタリング
  // ID: 0x208
  // Mask: 0x7FF (完全一致)

  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterIdHigh = (CAN_ID_CONTROL << 5);  // 標準ID (11bit) を上位にシフト
  filter_config.FilterIdLow = 0;
  filter_config.FilterMaskIdHigh = (0x7FF << 5);       // マスク: 完全一致
  filter_config.FilterMaskIdLow = 0;
  filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter_config.FilterActivation = ENABLE;
  filter_config.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan_ctrl, &filter_config);
}

void can_control_rx_callback(CAN_HandleTypeDef *hcan) {
  led_set(LED_COLOR_YELLOW, LED_STATE_ON);
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // FIFO0からメッセージ取得
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
    return;
  }

  // 受信したCAN IDに応じて処理
  if (rx_header.StdId == CAN_ID_CONTROL) {
    // CAN ID 0x208: モーター制御
    // データ: [DCモーター1, DCモーター2, サーボモーター]
    if (rx_header.DLC >= 3) {
      // DCモーター1制御 (rx_data[0])
      // 0: 逆転(引っ込む), 1: 正転(出っ張る), それ以外: 無視
      if (rx_data[0] == 0) {
        dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_REVERSE, 50);
        printf("DC Motor 1: Reverse (50%%)\n");
      } else if (rx_data[0] == 1) {
        dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_FORWARD, 50);
        printf("DC Motor 1: Forward (50%%)\n");
      }

      // DCモーター2制御 (rx_data[1])
      // 0: 逆転(引っ込む), 1: 正転(出っ張る), それ以外: 無視
      if (rx_data[1] == 0) {
        dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_REVERSE, 50);
        printf("DC Motor 2: Reverse (50%%)\n");
      } else if (rx_data[1] == 1) {
        dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_FORWARD, 50);
        printf("DC Motor 2: Forward (50%%)\n");
      }

      // サーボモーター制御 (rx_data[2])
      // 0: 角度を増やす(270度方向), 1: 停止(現在位置保持), 2: 角度を減らす(0度方向)
      if (rx_data[2] == 0) {
        // 角度を増やす（270度方向へ）
        if (servo_current_angle < 2700) {
          servo_current_angle += SERVO_ANGLE_STEP;
          if (servo_current_angle > 2700) {
            servo_current_angle = 2700;  // 上限リミット
          }
          servo_set_angle(servo_current_angle);
          printf("Servo: %u.%u deg (increasing)\n", servo_current_angle / 10, servo_current_angle % 10);
        }
      } else if (rx_data[2] == 2) {
        // 角度を減らす（0度方向へ）
        if (servo_current_angle > 0) {
          if (servo_current_angle >= SERVO_ANGLE_STEP) {
            servo_current_angle -= SERVO_ANGLE_STEP;
          } else {
            servo_current_angle = 0;  // 下限リミット
          }
          servo_set_angle(servo_current_angle);
          printf("Servo: %u.%u deg (decreasing)\n", servo_current_angle / 10, servo_current_angle % 10);
        }
      }
      // rx_data[2] == 1 の場合は何もしない（現在位置保持）
    }
  }
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
}
