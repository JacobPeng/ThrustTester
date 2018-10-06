/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include <string.h>

#include "rtc.h"
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = MPU6050_SCL_Pin|MPU6050_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  
    /* I2C1 DMA Init */
    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Channel6;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Channel7;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, MPU6050_SCL_Pin|MPU6050_SDA_Pin);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmatx);
    HAL_DMA_DeInit(i2cHandle->hdmarx);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

#define I2C_TIMEOUT_SHORT 5   // ms
#define I2C_TIMEOUT_LONG  100 

#if 1
// ----------------------------------------------------------------------------
// int8_t I2Cx_Write(uint8_t, uint8_t, const uint8_t*, uint8_t)
// ------------------------------------------------------------
// Parameters   : 
//  slave_addr  - i2c bus address of slave device
//  reg_addr    - resiter address of slave device
//  buf         - start address of data
//  len         - data size
// Description  : Write data to slave device.
// ----------------------------------------------------------------------------
int8_t I2Cx_Write(uint8_t slave_addr, uint8_t reg_addr,
                  const uint8_t* buf, uint8_t len) {
  if (!buf) return -1;
  if (!len) return -1;

  uint8_t buf_tmp[len + 1];

  buf_tmp[0] = reg_addr;
  memcpy(buf_tmp + 1, buf, len);
  // Use I2C with DMA, ensure data size >= 2!!!
  // Send register address and data together.
  while (HAL_I2C_Master_Transmit_DMA(&hi2c1, slave_addr, 
                                     (uint8_t*)buf_tmp, len + 1) != HAL_OK) {
    // Error_Handler() function is called when Timeout error occurs.
    // When Acknowledge failure occurs (Slave don't acknowledge its address),
    // Master restarts communication.
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
      Error_Handler();
      return -1;
    } // if
  } // while
  // Before starting a new communication transfer, you need to check the 
  // current state of the peripheral; if it’s busy you need to wait for the 
  // end of current transfer before starting a new one.
  STRUCT_RtcTimerMs i2c_timer;

  // While the bus is busy.
  RTC_SetMs(&i2c_timer, I2C_TIMEOUT_LONG);
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    if (RTC_ExpiredMs(&i2c_timer)) {
      Error_Handler();
      return -1;
    } // if
  } // while

  // All operations are ok.
  return 0;
} // I2Cx_Write()

// ----------------------------------------------------------------------------
// int8_t I2Cx_Read(uint8_t, uint8_t, uint8_t*, uint8_t)
// ----------------------------------------------------------------------------
// Return Value : 0, successful; others, failed.
// Parameters   : 
//  slave_addr  - i2c bus address of slave device
//  reg_addr    - resiter address of slave device
//  buf         - start address of data
//  len         - data size
// Description  : Read data from slave device.
// ----------------------------------------------------------------------------
int8_t I2Cx_Read(uint8_t slave_addr, uint8_t reg_addr,
                 uint8_t* buf, uint8_t len) {
  if (!buf) return -1;
  if (!len) return -1;

  // Send register address first.
  do {
    if (HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, 
                                   (uint8_t*)&reg_addr, 1)!= HAL_OK) {
      Error_Handler();
      return -1;
    } // if
    // Before starting a new communication transfer, you need to check the 
    // current state of the peripheral; if it’s busy you need to wait for the 
    // end of current transfer before starting a new one.
    STRUCT_RtcTimerMs i2c_timer;

    // While the bus is busy.
    RTC_SetMs(&i2c_timer, I2C_TIMEOUT_SHORT);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
      if (RTC_ExpiredMs(&i2c_timer)) {
        Error_Handler();
        return -1;
      } // if
    } // while    
    // When Acknowledge failure occurs (Slave don't acknowledge its address),
    // Master restarts communication.    
  } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

  // Then read data.
  if (len < 2) {
    do {
      if (HAL_I2C_Master_Receive_IT(&hi2c1, slave_addr, 
                                    (uint8_t*)buf, len)!= HAL_OK) {
        Error_Handler();
        return -1;
      } // if
      // Before starting a new communication transfer, you need to check the 
      // current state of the peripheral; if it’s busy you need to wait for the 
      // end of current transfer before starting a new one.
      STRUCT_RtcTimerMs i2c_timer;

      // While the bus is busy.
      RTC_SetMs(&i2c_timer, I2C_TIMEOUT_SHORT);
      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        if (RTC_ExpiredMs(&i2c_timer)) {
          Error_Handler();
          return -1;
        } // if
      } // while      
      // When Acknowledge failure occurs (Slave don't acknowledge its address),
      // Master restarts communication.    
    } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);  
  } else {
    // Use I2C with DMA, ensure data size >= 2!!!
    // Read data.
    while (HAL_I2C_Master_Receive_DMA(&hi2c1, slave_addr, 
                                      (uint8_t*)buf, len) != HAL_OK) {
      // Error_Handler() function is called when Timeout error occurs.
      // When Acknowledge failure occurs (Slave don't acknowledge its address),
      // Master restarts communication.
      if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
        Error_Handler();
        return -1;
      } // if
    } // while
    // Before starting a new communication transfer, you need to check the 
    // current state of the peripheral; if it’s busy you need to wait for the 
    // end of current transfer before starting a new one.
    STRUCT_RtcTimerMs i2c_timer;

    // While the bus is busy.
    RTC_SetMs(&i2c_timer, I2C_TIMEOUT_LONG);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
      if (RTC_ExpiredMs(&i2c_timer)) {
        Error_Handler();
        return -1;
      } // if
    } // while    
  } // if else  

  // All operations are ok.
  return 0;
} // I2Cx_Read()
#endif 

#if 0
// ----------------------------------------------------------------------------
// int8_t I2Cx_Write(uint8_t, uint8_t, const uint8_t*, uint8_t)
// ------------------------------------------------------------
// Parameters   : 
//  slave_addr  - i2c bus address of slave device
//  reg_addr    - resiter address of slave device
//  buf         - start address of data
//  len         - data size
// Description  : Write data to slave device.
// ----------------------------------------------------------------------------
int8_t I2Cx_Write(uint8_t slave_addr, uint8_t reg_addr,
                  const uint8_t* buf, uint8_t len) {
  if (!buf) return -1;
  if (!len) return -1;

  uint8_t buf_tmp[len + 1];

  buf_tmp[0] = reg_addr;
  memcpy(buf_tmp + 1, buf, len);
  // Send register address and data together.
  do {
    if (HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, (uint8_t*)buf_tmp, 
                                   len + 1)!= HAL_OK) {
      Error_Handler();
      return -1;
    } // if
    // Before starting a new communication transfer, you need to check the 
    // current state of the peripheral; if it’s busy you need to wait for the 
    // end of current transfer before starting a new one.
    STRUCT_RtcTimerMs i2c_timer;

    // While the bus is busy.
    RTC_SetMs(&i2c_timer, I2C_TIMEOUT_LONG);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
      if (RTC_ExpiredMs(&i2c_timer)) {
        Error_Handler();
        return -1;
      } // if
    } // while    
    // When Acknowledge failure occurs (Slave don't acknowledge its address),
    // Master restarts communication.    
  } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

  // All operations are ok.
  return 0;
} // I2Cx_Write()

// ----------------------------------------------------------------------------
// int8_t I2Cx_Read(uint8_t, uint8_t, uint8_t*, uint8_t)
// ----------------------------------------------------------------------------
// Return Value : 0, successful; others, failed.
// Parameters   : 
//  slave_addr  - i2c bus address of slave device
//  reg_addr    - resiter address of slave device
//  buf         - start address of data
//  len         - data size
// Description  : Read data from slave device.
// ----------------------------------------------------------------------------
int8_t I2Cx_Read(uint8_t slave_addr, uint8_t reg_addr,
                 uint8_t* buf, uint8_t len) {
  if (!buf) return -1;
  if (!len) return -1;

  // Send register address first.
  do {
    if (HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, 
                                   (uint8_t*)&reg_addr, 1)!= HAL_OK) {
      Error_Handler();
      return -1;
    } // if
    // Before starting a new communication transfer, you need to check the 
    // current state of the peripheral; if it’s busy you need to wait for the 
    // end of current transfer before starting a new one.
    STRUCT_RtcTimerMs i2c_timer;

    // While the bus is busy.
    RTC_SetMs(&i2c_timer, I2C_TIMEOUT_SHORT);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
      if (RTC_ExpiredMs(&i2c_timer)) {
        Error_Handler();
        return -1;
      } // if
    } // while      
    // When Acknowledge failure occurs (Slave don't acknowledge its address),
    // Master restarts communication.    
  } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

  // Then read data.
  do {
    if (HAL_I2C_Master_Receive_IT(&hi2c1, slave_addr, (uint8_t*)buf, 
                                  len)!= HAL_OK) {
      Error_Handler();
      return -1;
    } // if
    // Before starting a new communication transfer, you need to check the 
    // current state of the peripheral; if it’s busy you need to wait for the 
    // end of current transfer before starting a new one.
    STRUCT_RtcTimerMs i2c_timer;

    // While the bus is busy.
    RTC_SetMs(&i2c_timer, I2C_TIMEOUT_LONG);
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
      if (RTC_ExpiredMs(&i2c_timer)) {
        Error_Handler();
        return -1;
      } // if
    } // while    
    // When Acknowledge failure occurs (Slave don't acknowledge its address),
    // Master restarts communication.    
  } while (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);

  // All operations are ok.
  return 0;
} // I2Cx_Read()
#endif

#if 0
// ----------------------------------------------------------------------------
// int8_t I2Cx_Write(uint8_t, uint8_t, const uint8_t*, uint8_t)
// ------------------------------------------------------------
// Parameters   : 
//  slave_addr  - i2c bus address of slave device
//  reg_addr    - resiter address of slave device
//  buf         - start address of data
//  len         - data size
// Description  : Write data to slave device.
// ----------------------------------------------------------------------------
int8_t I2Cx_Write(uint8_t slave_addr, uint8_t reg_addr,
                  const uint8_t* buf, uint8_t len) {
  if (!buf) return -1;
  if (!len) return -1;

  uint8_t buf_tmp[len + 1];

  buf_tmp[0] = reg_addr;
  memcpy(buf_tmp + 1, buf, len);
  // Send register address and data together.
  while (HAL_I2C_Master_Transmit(&hi2c1, slave_addr, (uint8_t*)buf_tmp, 
                                 len + 1, I2C_TIMEOUT_LONG)!= HAL_OK) {
    // Error_Handler() function is called when Timeout error occurs.
    // When Acknowledge failure occurs (Slave don't acknowledge its address),
    // Master restarts communication.
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
      Error_Handler();
      return -1;
    } // if
  } // while

  // All operations are ok.
  return 0;
} // I2Cx_Write()

// ----------------------------------------------------------------------------
// int8_t I2Cx_Read(uint8_t, uint8_t, uint8_t*, uint8_t)
// ----------------------------------------------------------------------------
// Return Value : 0, successful; others, failed.
// Parameters   : 
//  slave_addr  - i2c bus address of slave device
//  reg_addr    - resiter address of slave device
//  buf         - start address of data
//  len         - data size
// Description  : Read data from slave device.
// ----------------------------------------------------------------------------
int8_t I2Cx_Read(uint8_t slave_addr, uint8_t reg_addr,
                 uint8_t* buf, uint8_t len) {
  if (!buf) return -1;
  if (!len) return -1;

  // Send register address first.
  while (HAL_I2C_Master_Transmit(&hi2c1, slave_addr, (uint8_t*)&reg_addr, 
                                 1, I2C_TIMEOUT_SHORT)!= HAL_OK) {
    // Error_Handler() function is called when Timeout error occurs.
    // When Acknowledge failure occurs (Slave don't acknowledge its address),
    // Master restarts communication.
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
      Error_Handler();
      return -1;
    } // if
  } // while

  // Then read data.
  while (HAL_I2C_Master_Receive(&hi2c1, slave_addr, (uint8_t*)buf, 
                                len, I2C_TIMEOUT_LONG)!= HAL_OK) {
    // Error_Handler() function is called when Timeout error occurs.
    // When Acknowledge failure occurs (Slave don't acknowledge its address),
    // Master restarts communication.
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
      Error_Handler();
      return -1;
    } // if
  } // while

  // All operations are ok.
  return 0;
} // I2Cx_Read()
#endif

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
