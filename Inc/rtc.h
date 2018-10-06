/**
  ******************************************************************************
  * File Name          : RTC.h
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
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
#ifndef __rtc_H
#define __rtc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdint.h>
#include <stdbool.h>
  
/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------
#define RTC_MS_MAX 65536  // ms
#define RTC_S_MAX  86400  // s

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
typedef struct {
  uint16_t start;
  uint16_t interval;
} STRUCT_RtcTimerMs;

typedef struct {
  uint32_t start;
  uint32_t interval;
} STRUCT_RtcTimerS;

typedef struct {
  uint16_t  millisecond;
  uint8_t   second;
  uint8_t   minute;
  uint8_t   hour;
  uint8_t   day;
  uint8_t   month;
  uint8_t   year;
} STRUCT_RtcInfo;

extern STRUCT_RtcInfo g_rtc;

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */

// ----------------------------------------------------------------------------
// High Layer Function
// ----------------------------------------------------------------------------
// Initialize RTC.
void RTC_Init(void);
// RTC service routine. 
void HAL_SYSTICK_Callback(void);
// RTC timer ticks per ms.
uint16_t RTC_TickMs(void);
// Set the initial value of RTC timer, ms.
void RTC_SetMs(STRUCT_RtcTimerMs* t, uint16_t interval);
// Check whether RTC timer is expired or not, ms.
bool RTC_ExpiredMs(const STRUCT_RtcTimerMs* t);
// RTC timer ticks per s.
uint32_t RTC_TickS(void);
// Set the initial value of RTC timer, s.
void RTC_SetS(STRUCT_RtcTimerS* t, uint32_t interval);
// Check whether RTC timer is expired or not, s.
bool RTC_ExpiredS(const STRUCT_RtcTimerS* t);

// ----------------------------------------------------------------------------
// Medium Layer Function
// ----------------------------------------------------------------------------
// Calculate days in current month.
static uint8_t RTC_CalculateDays(void);

// ----------------------------------------------------------------------------
// Low Layer Function
// ----------------------------------------------------------------------------

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ rtc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
