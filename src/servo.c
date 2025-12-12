/* Servo motor control implementation */
#include "servo.h"

// サーボモーター用タイマーハンドル
static TIM_HandleTypeDef *htim_servo = NULL;

// サーボPWM周波数: 50Hz (20ms周期)
// TIM3設定: 32MHz / (prescaler+1) / (period+1) = PWM周波数
// 現在の設定: prescaler=31, period=19999 → 32MHz/32/20000 = 50Hz
//
// サーボモーター仕様:
// - 制御角: 270度
// - パルス幅範囲: 0.5ms ～ 2.5ms
//
// 50Hz PWMの場合、1周期 = 20ms
// サーボ制御パルス幅:
// - 0度:   0.5ms → CCR = (0.5ms / 20ms) × 20000 = 500
// - 135度: 1.5ms → CCR = (1.5ms / 20ms) × 20000 = 1500
// - 270度: 2.5ms → CCR = (2.5ms / 20ms) × 20000 = 2500

#define SERVO_MIN_CCR  500   // 0.5ms パルス幅 (0度)
#define SERVO_MAX_CCR  2500  // 2.5ms パルス幅 (270度)

void servo_init(TIM_HandleTypeDef *htim) {
  htim_servo = htim;

  // TIM3 CH3 (PB0)をPWM出力として開始
  HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_3);

  // 初期位置: 135度 (中央)
  servo_set_angle(1350);
}

void servo_set_angle(uint16_t angle_x10) {
  if (htim_servo == NULL) {
    return;
  }

  // 角度範囲チェック (0〜2700)
  if (angle_x10 > 2700) {
    angle_x10 = 2700;
  }

  // CCR値計算: 角度に応じたデューティ比
  // angle_x10が0〜2700の範囲を、CCRのSERVO_MIN_CCR〜SERVO_MAX_CCRに変換
  uint32_t ccr = SERVO_MIN_CCR + ((angle_x10 * (SERVO_MAX_CCR - SERVO_MIN_CCR)) / 2700);

  // CH3のCCR値を設定
  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_3, ccr);
}
