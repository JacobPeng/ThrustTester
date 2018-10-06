/**
  ******************************************************************************
  * File Name          : USART.c
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include "terminal.h"
#include "debug.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = PRINT_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PRINT_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PRINT_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PRINT_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, PRINT_TX_Pin|PRINT_RX_Pin);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

STRUCT_UartInfo g_uart3 = {0};

#if 0
// ----------------------------------------------------------------------------
// void UARTx_IDLE_IRQHandler(UART_HandleTypeDef*)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : huart - pointer of uart
// Description  : UARTx idle interrupt handler, receive variable lenghth data
//  with DMA.
// ----------------------------------------------------------------------------
void UARTx_IDLE_IRQHandler(UART_HandleTypeDef* p_uart_id) {
  if (__HAL_UART_GET_FLAG(p_uart_id, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(p_uart_id);

    // Need not __HAL_DMA_DISABLE(), hdmarx was disabled implicitly here.
    // If use HAL_UART_DMAStop(), it will abort tx and rx.
    // Here we only need to abort rx.
    (void)HAL_UART_AbortReceive(p_uart_id);  
    g_uart3.len_rx = 
      (uint16_t)(UART3_LEN_RX - __HAL_DMA_GET_COUNTER(p_uart_id->hdmarx));
    // Signal rx was completed.
    g_uart3.cplt_rx = true;
    // Need not __HAL_DMA_ENABLE(), hdmarx was enabled implicitly here.
    // Receive again from the beginning.
    (void)HAL_UART_Receive_DMA(p_uart_id, g_uart3.buf_rx, UART3_LEN_RX);
  } // if
} // UART1_Idle_IRQHandler()
#endif

// ----------------------------------------------------------------------------
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef*)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : huart - pointer of uart
// Description  : Rx Transfer completed callbacks, it's called by hal library.
// ----------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (USART3 == huart->Instance) {
    static uint16_t s_index_rx = 0;

    // Increment index for the next reception.
    ++s_index_rx;
    // Check the border of buffer and newline character.
    if ('\n' == g_uart3.buf_rx[s_index_rx - 1] ||
        UART3_LEN_RX == s_index_rx) {
      g_uart3.len_rx = s_index_rx;
      s_index_rx = 0;
      g_uart3.cplt_rx = true;
    } // if
    // Start another reception.
    (void)HAL_UART_Receive_DMA(huart, g_uart3.buf_rx + s_index_rx, 1);
  } // if
} // HAL_UART_RxCpltCallback()

// ----------------------------------------------------------------------------
// void UARTx_Putc(UART_HandleTypeDef*, uint8_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : 
//  p_uart_id - UARTx id
//  ch      - byte to send
// Description  : Print char data with UARTx.
// ----------------------------------------------------------------------------
void UARTx_Putc(UART_HandleTypeDef* p_uart_id, uint8_t ch) { 
  while (p_uart_id->gState != HAL_UART_STATE_READY) continue;
  (void)HAL_UART_Transmit_DMA(p_uart_id, (uint8_t *)&ch, 1);
} // UARTx_Putc()

// ----------------------------------------------------------------------------
// void UARTx_PutBuffer(UART_HandleTypeDef*, const uint8_t*, uint16_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : 
//  p_uart_id - UARTx id
//  buf     - start address of buffer
//  len     - send bytes size
// Description  : Print buffer with UARTx.
// ----------------------------------------------------------------------------
void UARTx_PutBuffer(UART_HandleTypeDef* p_uart_id, const uint8_t* buf, 
                     uint16_t len) {
  while (p_uart_id->gState != HAL_UART_STATE_READY) continue;
  (void)HAL_UART_Transmit_DMA(p_uart_id, (uint8_t*)buf, len);
} // UARTx_PutBuffer()

// ----------------------------------------------------------------------------
// void UARTx_Getc(UART_HandleTypeDef, uint8_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : 
//  p_uart_id - UARTx id
//  ch      - read byte
// Description  : Read char data with UARTx.
// ----------------------------------------------------------------------------
// void UARTx_Getc(UART_HandleTypeDef* p_uart_id, uint8_t ch) {   
//   (void)HAL_UART_Receive_DMA(p_uart_id, (uint8_t *)&ch, 1);
// } // UARTx_Getc()

// ----------------------------------------------------------------------------
// void UARTx_GetBuffer(UART_HandleTypeDef, const uint8_t*, uint16_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : 
//  p_uart_id - UARTx id
//  buf     - start address of buffer
//  len     - read bytes size
// Description  : Read buffer with UARTx.
// ----------------------------------------------------------------------------
// void UARTx_GetBuffer(UART_HandleTypeDef* p_uart_id, uint8_t* buf, uint16_t len) {  
//   (void)HAL_UART_Receive_DMA(p_uart_id, (uint8_t*)buf, len);
//   // while (p_uart_id->RxState != HAL_UART_STATE_READY) continue;
// } // UARTx_GetBuffer()

// ----------------------------------------------------------------------------
// void UART3_Echo(uint8_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : 
//  dat - input data
// Description  : Echo input data with UART3.
// ----------------------------------------------------------------------------
void UART3_Echo(uint8_t dat) {
  UARTx_Putc(&huart3, dat);
} // UART3_Echo()

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
