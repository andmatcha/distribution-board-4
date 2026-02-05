/* Servo motor control implementation */
#include "servo.h"
#include <stdio.h>

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
#define ANGLE_MIN     0     // 最小角度 (度)
#define ANGLE_MAX     270   // 最大角度 (度)

uint16_t current_angle = 0; // 初期角度

void servo_set_angle(uint16_t angle);

void servo_init(TIM_HandleTypeDef *htim) {
  htim_servo = htim;

  printf("[SERVO_INIT] TIM Instance: 0x%08lX\n", (uint32_t)htim->Instance);
  printf("[SERVO_INIT] TIM3 prescaler: %lu\n", htim->Instance->PSC);
  printf("[SERVO_INIT] TIM3 ARR: %lu\n", htim->Instance->ARR);
  printf("[SERVO_INIT] TIM3 CR1: 0x%04X\n", htim->Instance->CR1);

  // TIM3 CH3 (PB0)をPWM出力として開始
  // HAL_TIM_PWM_Startはタイマーを自動的に有効化する
  HAL_StatusTypeDef status = HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_3);
  printf("[SERVO_INIT] HAL_TIM_PWM_Start status: %d (0=OK)\n", status);
  printf("[SERVO_INIT] TIM3 CR1 after PWM_Start: 0x%04X (bit0 should be 1 for counter enable)\n", htim_servo->Instance->CR1);

  // CCER register check (capture/compare enable register)
  printf("[SERVO_INIT] TIM3 CCER: 0x%04X (bit8 should be 1 for CH3 enable)\n", htim_servo->Instance->CCER);

  // CH3が有効化されていない場合、手動で有効化
  if (!(htim_servo->Instance->CCER & TIM_CCER_CC3E)) {
    printf("[SERVO_INIT] WARNING: CH3 not enabled, manually enabling...\n");
    htim_servo->Instance->CCER |= TIM_CCER_CC3E;
    printf("[SERVO_INIT] TIM3 CCER after manual enable: 0x%04X\n", htim_servo->Instance->CCER);
  }

  // カウンターが有効化されていない場合、手動で有効化
  if (!(htim_servo->Instance->CR1 & TIM_CR1_CEN)) {
    printf("[SERVO_INIT] WARNING: Counter not enabled, manually enabling...\n");
    htim_servo->Instance->CR1 |= TIM_CR1_CEN;
    printf("[SERVO_INIT] TIM3 CR1 after manual enable: 0x%04X\n", htim_servo->Instance->CR1);
  }

  // CCMR2 register check (capture/compare mode register for CH3/CH4)
  printf("[SERVO_INIT] TIM3 CCMR2: 0x%04X (bits 4-6 should be 110 for PWM mode 1 on CH3)\n", htim_servo->Instance->CCMR2);

  // CCR3 register check
  printf("[SERVO_INIT] TIM3 CCR3: %lu\n", htim_servo->Instance->CCR3);

  // 初期位置
  servo_set_angle(current_angle);

  // Force update event to apply CCR changes immediately
  htim_servo->Instance->EGR = TIM_EGR_UG;
  printf("[SERVO_INIT] Update event generated\n");

  // Verify CCR3 after update
  printf("[SERVO_INIT] TIM3 CCR3 after update: %lu\n", htim_servo->Instance->CCR3);

  // Additional register verification
  printf("[SERVO_INIT] TIM3 CNT (counter): %lu\n", htim_servo->Instance->CNT);
  printf("[SERVO_INIT] TIM3 SR (status): 0x%04X\n", htim_servo->Instance->SR);

  // GPIO register verification (GPIOB)
  printf("[SERVO_INIT] GPIOB CRL (config low): 0x%08lX\n", GPIOB->CRL);
  printf("[SERVO_INIT] GPIOB ODR (output): 0x%04X\n", GPIOB->ODR);

  // AFIO register verification (remap settings)
  printf("[SERVO_INIT] AFIO MAPR (remap): 0x%08lX\n", AFIO->MAPR);
  printf("[SERVO_INIT] AFIO MAPR TIM3_REMAP bits (10-11): %lu (should be 2 for partial remap)\n",
         (AFIO->MAPR >> 10) & 0x3);
}

void servo_set_angle(uint16_t angle) {
  printf("[SERVO] servo_set_angle called: angle=%u\n", angle);

  if (htim_servo == NULL) {
    printf("[SERVO] ERROR: htim_servo is NULL!\n");
    return;
  }

  // 角度範囲チェック (0〜270)
  if (angle < ANGLE_MIN) {
    angle = ANGLE_MIN;
  }
  if (angle > ANGLE_MAX) {
    angle = ANGLE_MAX;
  }

  // CCR値計算: 角度に応じたデューティ比
  // angleが0〜270の範囲を、CCRのSERVO_MIN_CCR〜SERVO_MAX_CCRに変換
  uint32_t ccr = SERVO_MIN_CCR + ((angle * (SERVO_MAX_CCR - SERVO_MIN_CCR)) / ANGLE_MAX);

  // CH3のCCR値を設定
  __HAL_TIM_SET_COMPARE(htim_servo, TIM_CHANNEL_3, ccr);

  // 設定後の確認
  uint32_t actual_ccr = __HAL_TIM_GET_COMPARE(htim_servo, TIM_CHANNEL_3);
  printf("[SERVO] CCR set to %lu, readback=%lu\n", ccr, actual_ccr);
}

// サーボモーター制御 呼ばれるたびに角度を少しずつ変化させる
void servo_control(ServoDirection direction, ServoMode mode) {
  uint16_t angle_step = (mode == SERVO_MODE_FAST) ? 10 : 5; // 高速モード : 通常モード

  if (direction == SERVO_DIR_OPEN) {
    current_angle = current_angle + angle_step;
    if (current_angle > ANGLE_MAX) {
      current_angle = ANGLE_MAX;
    }
  } else if (direction == SERVO_DIR_CLOSE) {
    if (current_angle < angle_step) {
      current_angle = ANGLE_MIN;
    } else {
      current_angle = current_angle - angle_step;
    }
  } else {
    // SERVO_DIR_STOP: 現在の角度を保持（何もしない）
    return;
  }

  servo_set_angle(current_angle);
}
