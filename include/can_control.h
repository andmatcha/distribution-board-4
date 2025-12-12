/* CAN control public API */
#ifndef CAN_CONTROL_H
#define CAN_CONTROL_H

#include "stm32f1xx_hal.h"

// CAN ID定義
#define CAN_ID_SERVO     0x100  // サーボモーター制御用
#define CAN_ID_MOTOR1    0x101  // DCモーター1制御用
#define CAN_ID_MOTOR2    0x102  // DCモーター2制御用

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
