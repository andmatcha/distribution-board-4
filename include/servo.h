/* Servo motor control public API */
#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"

/**
 * @brief サーボモーター初期化
 * @param htim TIM3ハンドル
 */
void servo_init(TIM_HandleTypeDef *htim);

/**
 * @brief サーボモーター角度設定
 * @param angle_x10 角度×10 (0〜1800で0.0〜180.0度を表す)
 */
void servo_set_angle(uint16_t angle_x10);

#endif /* SERVO_H */
