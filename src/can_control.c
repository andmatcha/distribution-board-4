/* CAN control implementation */
#include "can_control.h"
#include "servo.h"
#include "dc_motor.h"
#include "led.h"

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

// CANフィルター設定 (0x208, 0x1FFのみ受信)
static void can_filter_config(void) {
  CAN_FilterTypeDef filter_config;
  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDLIST;
  filter_config.FilterScale = CAN_FILTERSCALE_16BIT;
  filter_config.FilterIdHigh = (CAN_ID_DC << 5);
  filter_config.FilterIdLow = (CAN_ID_SERVO << 5);
  filter_config.FilterMaskIdHigh = 0;
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

  // 受信IDによって処理を分岐
  if (rx_header.StdId == CAN_ID_DC) {
    // ニョッキDCモーター (0x208)
    // データ長チェック（rx_data[4]までアクセスするため最低5バイト必要）
    if (rx_header.DLC < 5) {
      return;
    }

    // DCモーター1: キーボードニョッキ (rx_data[0])
    if (rx_data[0] == 1) {
      dc_motor_push();
    } else {
      dc_motor_set(DC_MOTOR_1, DC_MOTOR_DIR_STOP, 0);
    }

    // DCモーター2: USBニョッキ (rx_data[3]=正転, rx_data[4]=逆転)
    if (rx_data[3] == 1) {
      dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_FORWARD, DC_SPEED);
    } else if (rx_data[4] == 1) {
      dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_REVERSE, DC_SPEED);
    } else {
      dc_motor_set(DC_MOTOR_2, DC_MOTOR_DIR_STOP, 0);
    }
  } else if (rx_header.StdId == CAN_ID_SERVO) {
    // 把持サーボモーター (0x1FF)
    // データ長チェック（rx_data[5]までアクセスするため最低6バイト必要）
    if (rx_header.DLC < 6) {
      return;
    }

    // 符号付き整数として解釈
    int16_t data = (int16_t)(rx_data[4]<<8 | rx_data[5]);
    if (data < 0) {
      servo_control(SERVO_DIR_OPEN, SERVO_MODE_NORMAL);
      printf("SERVO OPEN\n");
    } else if (data == 0) {
      servo_control(SERVO_DIR_STOP, SERVO_MODE_NORMAL);
      printf("SERVO STOP\n");
    } else {
      servo_control(SERVO_DIR_CLOSE, SERVO_MODE_NORMAL);
      printf("SERVO CLOSE\n");
    }
  }
  led_set(LED_COLOR_YELLOW, LED_STATE_OFF);
}
