/**
  ******************************************************************************
  * File Name          : RTC.c
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

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */
#include <string.h>
/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 3 */

  /* USER CODE END RTC_Init 3 */

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 4 */

  /* USER CODE END RTC_Init 4 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
STRUCT_RtcInfo g_rtc = {0};

// ----------------------------------------------------------------------------
// Initialization Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void RTC_Init(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Initialize RTC.
// ----------------------------------------------------------------------------
void RTC_Init(void) {
  MX_RTC_Init();
  if (HAL_RTCEx_SetSecond_IT(&hrtc) != HAL_OK) {
    Error_Handler();
  } // if
  // Initialize all variables.
  memset(&g_rtc, 0, sizeof(g_rtc));
} // RTC_Init()

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void RTC_CalculateDays(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Calculate days in current month.
// ----------------------------------------------------------------------------
static uint8_t RTC_CalculateDays(void) {
  return g_rtc.month == 2 ? 
    28 + (1 >> (g_rtc.year & 3)) : 
    30 + ((0x15AA >> g_rtc.month) & 1);
} // RTC_CalculateDays()

// ----------------------------------------------------------------------------
// void HAL_SYSTICK_Callback(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : RTC ms service routine. 
//  User application should call this function per ms.
//  It's called by SysTick_Handler().
// ----------------------------------------------------------------------------
void HAL_SYSTICK_Callback(void) {
  ++g_rtc.millisecond;
} // HAL_SYSTICK_Callback()

// ----------------------------------------------------------------------------
// void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : hrtc - handler of RTC
// Description  : RTC s service routine. 
//  User application should call this function per s.
//  It's called by HAL_RTCEx_RTCIRQHandler().
// ----------------------------------------------------------------------------
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc) {
  UNUSED(hrtc);

  ++g_rtc.second;
  if (g_rtc.second >= 60) {
    g_rtc.second = 0;
    ++g_rtc.minute;
    if (g_rtc.minute >= 60) {
      g_rtc.minute = 0;
      ++g_rtc.hour;
      if (g_rtc.hour >= 24) {
        g_rtc.hour = 0;
        ++g_rtc.day;
        if (RTC_CalculateDays() == g_rtc.day) {
          g_rtc.day = 1;
          ++g_rtc.month;
          if (g_rtc.month >= 13) {
            g_rtc.month = 1;
            ++g_rtc.year;
          } // if
        } // if
      } // if
    } // if
  } // if   
} // HAL_RTCEx_RTCEventCallback()

// ----------------------------------------------------------------------------
// uint16_t RTC_TickMs(void)
// ----------------------------------------------------------------------------
// Return Value : ms, max 60000-1
// Parameters   : None
// Description  : RTC timer ticks per ms.
// ----------------------------------------------------------------------------
uint16_t RTC_TickMs(void) {
  // Suspend Tick ms increment.
  HAL_SuspendTick();
  uint16_t tick = g_rtc.millisecond;
  // Resume Tick ms increment.
  HAL_ResumeTick();

  return tick;
} // RTC_TickMs()

// ----------------------------------------------------------------------------
// void RTC_SetMs(STRUCT_RtcTimerMs*, uint16_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   :
//  t         - pointer of RTC timer
//  interval  - delay time
// Description  : Set the initial value of RTC timer, ms.
// ----------------------------------------------------------------------------
void RTC_SetMs(STRUCT_RtcTimerMs* t, uint16_t interval) {
  if (interval >= RTC_MS_MAX) return;
  t->interval = interval;
  t->start = RTC_TickMs();
} // RTC_SetMs()

// ----------------------------------------------------------------------------
// bool RTC_ExpiredMs(STRUCT_RtcTimerMs*)
// ----------------------------------------------------------------------------
// Return Value : 1, expired; 0, not expired
// Parameters   : t - pointer of RTC timer
// Description  : Check whether RTC timer is expired or not, ms.
// ----------------------------------------------------------------------------
bool RTC_ExpiredMs(const STRUCT_RtcTimerMs* t) {
  uint16_t tick = RTC_TickMs();

  return tick >= t->start ?
    tick - t->start >= t->interval :
    tick + (RTC_MS_MAX - t->start) >= t->interval;
} // RTC_ExpiredMs()

// ----------------------------------------------------------------------------
// uint32_t RTC_TickS(void)
// ----------------------------------------------------------------------------
// Return Value : s, max 86400-1
// Parameters   : None
// Description  : RTC timer ticks per s.
// ----------------------------------------------------------------------------
uint32_t RTC_TickS(void) {
  // Suspend Tick s increment.
  HAL_RTCEx_DeactivateSecond(&hrtc);
  uint32_t tick = (uint32_t)(g_rtc.hour*3600 + g_rtc.minute*60 + g_rtc.second);
  // Resume Tick s increment.
  HAL_RTCEx_SetSecond_IT(&hrtc);

  return tick;
} // RTC_TickS()

// ----------------------------------------------------------------------------
// void RTC_SetS(STRUCT_RtcTimerS*, uint32_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   :
//  t         - pointer of RTC timer
//  interval  - delay time
// Description  : Set the initial value of RTC timer, s.
// ----------------------------------------------------------------------------
void RTC_SetS(STRUCT_RtcTimerS* t, uint32_t interval) {
  if (interval >= RTC_S_MAX) return;
  t->interval = interval;
  t->start = RTC_TickS();
} // RTC_SetS()

// ----------------------------------------------------------------------------
// bool RTC_ExpiredS(STRUCT_RtcTimerS*)
// ----------------------------------------------------------------------------
// Return Value : 1, expired; 0, not expired
// Parameters   : t - pointer of RTC timer
// Description  : Check whether RTC timer is expired or not, s.
// ----------------------------------------------------------------------------
bool RTC_ExpiredS(const STRUCT_RtcTimerS* t) {
  uint32_t tick = RTC_TickS();

  return tick >= t->start ?
    tick - t->start >= t->interval :
    tick + (RTC_S_MAX - t->start) >= t->interval;
} // RTC_ExpiredS()

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
