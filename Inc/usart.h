/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */

#define UART_PRINTF     huart3
#define UART3_LEN_RX    64

typedef struct STRUCT_UartInfo {
  bool     cplt_rx;
  uint16_t len_rx;
  uint8_t  buf_rx[UART3_LEN_RX];
} STRUCT_UartInfo;

extern STRUCT_UartInfo g_uart3;

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

// UARTx idle interrupt handler.
void UARTx_IDLE_IRQHandler(UART_HandleTypeDef* p_uart_id);
// Print char data with UARTx.
void UARTx_Putc(UART_HandleTypeDef* p_uart_id, uint8_t ch);
// Print buffer with UARTx.
void UARTx_PutBuffer(UART_HandleTypeDef* p_uart_id, const uint8_t* buf, 
                     uint16_t len);
// // Read char data with UARTx.
// void UARTx_Getc(UART_HandleTypeDef* p_uart_id, uint8_t ch);
// // Read buffer with UARTx.
// void UARTx_GetBuffer(UART_HandleTypeDef* p_uart_id, uint8_t* buf, uint16_t len);
// Echo input data with UART3.
void UART3_Echo(uint8_t dat);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
