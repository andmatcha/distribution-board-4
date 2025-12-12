/* Servo motor control implementation */
#include "servo.h"

// サーボモーター用タイマーハンドル
static TIM_HandleTypeDef *htim_servo = NULL;

// サーボPWM周波数: 50Hz (20ms周期)
// TIM3設定: 32MHz / (prescaler+1) / (period+1) = PWM周波数
// 現在の設定: prescaler=0, period=999なので32MHz/1/1000=32kHz
// サーボ用には50Hzが必要なため、CH3のみ別途パルス幅で制御
// ※実際にはCubeMXでTIM3を50Hzに再設定することを推奨しますが、
// 　今回はDCモーター用32kHzと共用のため、パルス幅計算で対応

// 32kHz PWMの場合、1周期 = 1/32000 = 31.25us
// サーボ制御: 20ms周期(50Hz)で1.0ms〜2.0msのパルス幅
// しかし32kHzでは周期が短すぎるため、サーボ制御には不適切
//
// 本来のサーボ制御:
// - PWM周波数: 50Hz (周期20ms)
// - パルス幅: 1.0ms (0度) 〜 1.5ms (90度) 〜 2.0ms (180度)
//
// 今回は簡易的に、TIM3_CH3のCCRを直接制御
// 32kHzで動作させる場合は以下の近似を使用:
// CCR = (angle_x10 / 1800) * (max_ccr - min_ccr) + min_ccr

#define SERVO_MIN_CCR  50   // 最小パルス幅相当
#define SERVO_MAX_CCR  100  // 最大パルス幅相当

void servo_init(TIM_HandleTypeDef *htim) {
  htim_servo = htim;

  // TIM3 CH3 (PB0)をPWM出力として開始
  HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_3);

  // 初期位置: 90度 (中央)
  servo_set_angle(900);
}

void servo_set_angle(uint16_t angle_x10) {
  if (htim_servo == NULL) {
    return;
  }

  // 角度範囲チェック (0〜1800)
  if (angle_x10 > 1800) {
    angle_x10 = 1800;
  }

  // CCR値計算: 角度に応じたデューティ比
  // angle_x10が0〜1800の範囲を、CCRのSERVO_MIN_CCR〜SERVO_MAX_CCRに変換
  uint32_t ccr = SERVO_MIN_CCR + ((angle_x10 * (SERVO_MAX_CCR - SERVO_MIN_CCR)) / 1800);

  // CH3のCCR値を設定
  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_3, ccr);
}
