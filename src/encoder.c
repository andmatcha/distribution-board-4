/**
 ******************************************************************************
 * @file           : encoder.c
 * @brief          : Encoder reading module implementation
 ******************************************************************************
 */

#include "encoder.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define ENCODER_CMD 0x54
#define ENCODER_BUFFER_SIZE 2
#define ENCODER_RS485_DE_PIN GPIO_PIN_8
#define ENCODER_RS485_DE_PORT GPIOA

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *encoder_huart = NULL;
static uint8_t encoder_rx_buf[ENCODER_BUFFER_SIZE];
static uint16_t encoder_position = 0;
static volatile bool encoder_data_ready = false;
static volatile uint32_t encoder_uart_error_count = 0;
static volatile uint32_t encoder_checksum_error_count = 0;

/* Private function prototypes -----------------------------------------------*/
static void encoder_rs485_tx_enable(void);
static void encoder_rs485_rx_enable(void);
static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Enable RS485 transmit mode
 */
static void encoder_rs485_tx_enable(void)
{
  HAL_GPIO_WritePin(ENCODER_RS485_DE_PORT, ENCODER_RS485_DE_PIN, GPIO_PIN_SET);
}

/**
 * @brief Enable RS485 receive mode
 */
static void encoder_rs485_rx_enable(void)
{
  HAL_GPIO_WritePin(ENCODER_RS485_DE_PORT, ENCODER_RS485_DE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Decode encoder position from 2 bytes with checksum validation
 * @param lsb LSB byte
 * @param msb MSB byte
 * @param position Pointer to store decoded 14-bit position
 * @retval true if checksum is valid, false otherwise
 */
static bool encoder_decode_position(uint8_t lsb, uint8_t msb, uint16_t *position)
{
  if (position == NULL) {
    return false;
  }

  uint16_t w = (uint16_t)lsb | ((uint16_t)msb << 8);

  /* Calculate checksum (2-bit) */
  uint16_t cs = 0x3;
  for (int i = 0; i < 14; i += 2) {
    cs ^= (w >> i) & 0x3U;
  }

  /* Verify checksum */
  if (cs == (w >> 14)) {
    *position = (w & 0x3FFFU);
    return true;
  }

  return false;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize encoder module
 */
void encoder_init(UART_HandleTypeDef *huart)
{
  if (huart == NULL) {
    return;
  }

  encoder_huart = huart;
  encoder_data_ready = false;
  encoder_position = 0;
  encoder_uart_error_count = 0;
  encoder_checksum_error_count = 0;

  /* Ensure RX mode */
  encoder_rs485_rx_enable();
}

/**
 * @brief Request encoder position data
 */
void encoder_request_position(void)
{
  if (encoder_huart == NULL) {
    printf("[ENC] ERROR: UART handle is NULL\n");
    return;
  }

  static uint8_t cmd = ENCODER_CMD;

  encoder_rs485_tx_enable();
  HAL_UART_Receive_DMA(encoder_huart, encoder_rx_buf, ENCODER_BUFFER_SIZE);
  HAL_UART_Transmit(encoder_huart, &cmd, 1, 10);
  while (__HAL_UART_GET_FLAG(encoder_huart, UART_FLAG_TC) == RESET) {}
  encoder_rs485_rx_enable();
}

/**
 * @brief Get latest encoder position data
 */
bool encoder_get_position(uint16_t *position)
{
  if (position == NULL) {
    return false;
  }

  /* Critical section to prevent data race */
  __disable_irq();
  if (encoder_data_ready) {
    encoder_data_ready = false;
    *position = encoder_position;
    __enable_irq();
    return true;
  }
  __enable_irq();
  return false;
}

/**
 * @brief UART RX complete callback
 */
void encoder_rx_complete_callback(UART_HandleTypeDef *huart)
{
  HAL_UART_DMAStop(huart);

  if (huart->Instance != encoder_huart->Instance) {
    return;
  }

  printf("[ENC] RX complete: LSB=0x%02X, MSB=0x%02X\n", encoder_rx_buf[0], encoder_rx_buf[1]);

  uint16_t w = (encoder_rx_buf[0] | encoder_rx_buf[1] << 8);

  /* Calculate checksum */
  uint16_t cs = 0x3;
  for (int i = 0; i < 14; i += 2) {
    cs ^= (w >> i) & 0x3;
  }

  /* Verify checksum */
  if (cs == (w >> 14)) {
    encoder_position = (w & 0x3FFF);
    encoder_data_ready = true;
  } else {
    /* Invalid checksum - discard data */
    encoder_checksum_error_count++;
  }
}

/**
 * @brief UART error callback
 */
void encoder_error_callback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != encoder_huart->Instance) {
    return;
  }

  uint32_t err = HAL_UART_GetError(huart);
  encoder_uart_error_count++;
  printf("[USART1] Error: 0x%08lX\n", (unsigned long)err);

  /* Clear all error flags */
  if (err & HAL_UART_ERROR_ORE) {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }
  if (err & HAL_UART_ERROR_FE) {
    __HAL_UART_CLEAR_FEFLAG(huart);
  }
  if (err & HAL_UART_ERROR_NE) {
    __HAL_UART_CLEAR_NEFLAG(huart);
  }
  if (err & HAL_UART_ERROR_PE) {
    __HAL_UART_CLEAR_PEFLAG(huart);
  }

  /* Flush data register */
  (void)__HAL_UART_FLUSH_DRREGISTER(huart);

  /* Ensure RX mode */
  encoder_rs485_rx_enable();

  /* Additional DR clear */
  if (encoder_huart->Instance->DR) {
    (void)encoder_huart->Instance->DR;
  }

  /* Restart DMA reception */
  if (HAL_UART_Receive_DMA(encoder_huart, encoder_rx_buf, ENCODER_BUFFER_SIZE) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Get checksum error count
 */
uint32_t encoder_get_checksum_error_count(void)
{
  return encoder_checksum_error_count;
}

/**
 * @brief Get UART error count
 */
uint32_t encoder_get_uart_error_count(void)
{
  return encoder_uart_error_count;
}
