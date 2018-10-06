// ----------------------------------------------------------------------------
// rpm.c/h
// ----------------------------------------------------------------------------
// Copyright (c) 2017-2018, Jacob Peng.
// 
// Application Description:
// ========================
// Thrust Tester.
// Sample voltage, current, thrust, rpm, rpm and vibration;
// Calculate thrust efficiency and power.
//
// MCU Resources:
// ==============
// STM32F103C8T6-48pin, ARM Cortex M3, 72MHz SYSCLK, 64K ROM, 20K RAM, 37 GPIOs, 
// 16 12-bit ADCs, 3 USARTs, 2 SPIs, 2 I2Cs, 4 timers, 1 iwdg, 1 wwdg, swd supported.
// 
// File Description:
// =================
// Capture rpm application.
//
// Revision:
// =========
// Release B0.0.1 - 2018/03/23 - Jacob Peng
//  - Mod: Calculated rpm with laser sensor.
// Release A0.0.1 - 2018/01/29 - Jacob Peng
//  - Add: Captures rpm.
//  - Add: Uses protothread.
//

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "rpm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"

#include "debug.h"
#include "rtc.h"
#include "tim.h"
#include "protocol.h"

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
static bool s_rpm_is_updated = false;
static uint16_t s_rpm = 0;
static uint32_t s_rpm_interval = 0;

// ----------------------------------------------------------------------------
// Interrupt Service Routines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void RPM_IRQHandler(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Captures rpm input.
// ----------------------------------------------------------------------------
#define INTERVAL_MIN 500

void RPM_IRQHandler(void) {
  uint32_t new_time = TIM_Get32BitCounter();  
  static bool s_first_sample = true;
  static uint32_t s_last_time = 0;

  if (!HAL_GPIO_ReadPin(RPM_IN_GPIO_Port, RPM_IN_Pin)) {
    if (s_first_sample) {
      s_first_sample = false;
      s_last_time = new_time;
    } else {
      s_first_sample = true;
      uint32_t interval = (new_time > s_last_time) ? 
        new_time - s_last_time :
        (uint32_t)(new_time + (4294967296 - s_last_time));
      s_last_time = new_time;

      if (interval < INTERVAL_MIN) {
        interval = INTERVAL_MIN;
      } // else if

      s_rpm_interval = interval;
      s_rpm_is_updated = true;
    } // if else 
  } // if
} // PPM_IRQHandler()

// ----------------------------------------------------------------------------
// Initialization Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void RPM_Init(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Initialize rpm input.
// ----------------------------------------------------------------------------
void RPM_Init(void) {
  MX_TIM4_Init();
  if (HAL_TIM_Base_Start_IT(&RPM_TIM) != HAL_OK) {
    Error_Handler();
  } // if 
} // RPM_Init()

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

static bool RPM_IsUpdated(void) {
  if (s_rpm_is_updated) {
    s_rpm_is_updated = false;
    return true;
  } else {
    return false;
  } // if else
} // RPM_IsUpdated()

// ----------------------------------------------------------------------------
// PT_THREAD(RPM_Thread(struct pt*))
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : pt - the pointer of thread
// Description  : Rpm thread. Updates rpm input per 1s.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
#define RPM_UPDATE_PERIOD 100 // ms
#define RPM_TIMEOUT 1000 // ms

PT_THREAD(RPM_Thread(struct pt* pt)) {
  static STRUCT_RtcTimerMs s_rpm_timer;

  PT_BEGIN(pt);

  // Check rpm is updated or zero.
  RTC_SetMs(&s_rpm_timer, RPM_TIMEOUT);
  // Implicitly return here.
  PT_WAIT_UNTIL(pt, RPM_IsUpdated() || RTC_ExpiredMs(&s_rpm_timer));
  // Count interval us between two close falling edges.
  // When propellor rotates 180°, it takes time t us, t = interval.
  // When propellor rotates 360°, it takes time 2t us.
  // r1 rpm/s = 1/2t rpm/us.
  // r2 rpm/min = r1*60*10^6 rpm/min = (30*10^6)/t rpm/min.
  // rpm_min = 30 rpm/min, rpm_max = 60000 rpm/min.
  if (RTC_ExpiredMs(&s_rpm_timer)) {
    s_rpm = 0;
  } else {
    s_rpm = (30*1000000)/s_rpm_interval;
  } // if else
#if (defined(DEBUG_ALL) || defined(DEBUG_RPM))
  RTC_SetMs(&s_rpm_timer, RPM_UPDATE_PERIOD);
  PT_WAIT_UNTIL(pt, RTC_ExpiredMs(&s_rpm_timer));
  printf("%d\n", s_rpm);
#endif

  // Update protocol data.
  g_pro.data.rpm = s_rpm; 
  // Update protocol signal.
  g_pro.signal.updated[kProIndexRpm]  = true; 

  PT_END(pt);
} // RPM_Thread()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
