/* CAN control public API */
#ifndef CAN_CONTROL_H
#define CAN_CONTROL_H

#include "stm32f1xx_hal.h"

// CAN ID定義
#define CAN_ID_DC   0x208  // ニョッキDCモーター
#define CAN_ID_SERVO  0x1FF  // 把持サーボモーター

/**
 * @brief CAN受信初期化
 * @param hcan CANハンドル
 * @param htim TIM3ハンドル (モーター制御用)
 */
void can_control_init(CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *htim);

/**
 * @brief CAN受信コールバック (HAL_CAN_RxFifo0MsgPendingCallbackから呼ばれる)
 * @param hcan CANハンドル
 */
void can_control_rx_callback(CAN_HandleTypeDef *hcan);

#endif /* CAN_CONTROL_H */
