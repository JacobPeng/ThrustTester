// ----------------------------------------------------------------------------
// debug.c/h
// ----------------------------------------------------------------------------
// Copyright (c) 2017-2018, Jacob Peng.
// 
// Application Description:
// ========================
// Thrust Tester.
// Sample voltage, current, thrust, ppm, rpm and vibration;
// Calculate thrust efficiency and power.
//
// MCU Resources:
// ==============
// STM32F103C8T6-48pin, ARM Cortex M3, 72MHz SYSCLK, 64K ROM, 20K RAM, 37 GPIOs, 
// 16 12-bit ADCs, 3 USARTs, 2 SPIs, 2 I2Cs, 4 timers, 1 iwdg, 1 wwdg, swd supported.
// 
// File Description:
// =================
// Debug gpio driver.
//
// Revision:
// =========
// Release A0.0.1 - 2018/01/26 - Jacob Peng
//  - Add: Initialized debug gpio.
//

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "debug.h"

#include "stm32f1xx_hal.h"

#include "usart.h"

// ----------------------------------------------------------------------------
// Initialization Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void DEBUG_Init(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Output debug information.
// ----------------------------------------------------------------------------
void DEBUG_Init(void) {
  MX_USART3_UART_Init();
#if 0
  if (HAL_UART_Receive_DMA(uartHandle, g_uart3.buf_rx, UART3_LEN_RX)) {
    _Error_Handler(__FILE__, __LINE__);
  } // if 
  __HAL_UART_ENABLE_IT(uartHandle, UART_IT_IDLE);
#endif
  if (HAL_UART_Receive_DMA(&huart3, g_uart3.buf_rx, 1) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  } // if
} // DEBUG_Init()

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

void DEBUG_GpioOn(void) {
  HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
} // DEBUG_GpioOn()
void DEBUG_GpioOff(void) {
  HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
} // DEBUG_GpioOff()
void DEBUG_GpioToggle(void) {
  HAL_GPIO_TogglePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
} // DEBUG_GpioToggle()

void DEBUG_LedOn(void) {
  HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_RESET);
} // DEBUG_LedOn()
void DEBUG_LedOff(void) {
  HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
} // DEBUG_LedOff()
void DEBUG_LedToggle(void) {
  HAL_GPIO_TogglePin(LED_OUT_GPIO_Port, LED_OUT_Pin);
} // DEBUG_GpioToggle()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
