// ----------------------------------------------------------------------------
// pwm.c/h
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
// Outputs pwm application.
//
// Revision:
// =========
// Release B0.0.1 - 2018/06/26 - Jacob Peng
//  - Mod: Outputted pwm of ppm only.
// Release A0.0.1 - 2018/01/29 - Jacob Peng
//  - Add: Outputted pwm.
//  - Add: Uses protothread.
//

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "pwm.h"

#include <stdio.h>
#include <string.h>

#include "tim.h"

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
typedef struct {
  bool      updated;
  uint16_t  duty_x200;
} STRUCT_PwmInfo;

static STRUCT_PwmInfo s_pwm = {0};

// ----------------------------------------------------------------------------
// Initialization Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void PWM_Init(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Initialize pwm output, 50Hz.
// ----------------------------------------------------------------------------
void PWM_Init(void) {
  MX_TIM1_Init();
  if (HAL_TIM_Base_Start(&PWM_TIM) != HAL_OK) {
    Error_Handler();
  } // if
  // Initialize all variables.
  memset(&s_pwm, 0, sizeof(s_pwm));
} // PWM_Init()

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

void PWM_Update(uint16_t duty_x200) {
  s_pwm.duty_x200 = duty_x200;
  s_pwm.updated   = true;
} // PPM_Update()

static bool PWM_IsUpdated(void) {
  if (s_pwm.updated) {
    s_pwm.updated = false;
    return true;
  } else {
    return false;
  } // if else
} // PWM_IsUpdated()

// ----------------------------------------------------------------------------
// void PWM_ChangeDuty(uint16_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : 
//  duty_x200 - 200 times of the actual duty ratio.
// Description  : Changes pwm duty ratio. 
//  duty ratio = (TIMx_CCRx/TIMx_ARR)*100 = (duty_x200/20000)*100.
//  PPM duty ratio ranges from 5% to 10%, 1ms/20ms to 2ms/20ms.
//  PPM duty_x200 ranges from 1000 to 2000.
// ----------------------------------------------------------------------------
void PWM_ChangeDuty(uint16_t duty_x200) {
  PWM_TIM.Instance->CCR1 = duty_x200;
} // PWM_ChangeDuty()

// ----------------------------------------------------------------------------
// void PWM_Enable(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Enable pwm output forcibly. 
//  You must call it if you want to enable pwm output again when 
//  PWM_Disable() was called before.
// ----------------------------------------------------------------------------
void PWM_Enable(void) {
  if (HAL_TIM_PWM_Start(&PWM_TIM, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  } // if
} // PWM_Enable()

// ----------------------------------------------------------------------------
// void PWM_Disable(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Diable pwm output forcibly. 
//  You can also disable pwm by setting duty ratio to 0.
// ----------------------------------------------------------------------------
void PWM_Disable(void) {
  if (HAL_TIM_PWM_Stop(&PWM_TIM, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  } // if
} // PWM_Disable()

// ----------------------------------------------------------------------------
// PT_THREAD(PWM_Thread(struct pt*))
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : pt - the pointer of thread
// Description  : PWM thread. Updates pwm output in real time.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
PT_THREAD(PWM_Thread(struct pt* pt)) {
  // If PT_YIELD() wasn't used, the execution sequence is like below:
  // s_pwm.duty_x200[0] was changed!
  // s_pwm.duty_x200[1] was changed!
  // s_pwm.duty_x200[2] was changed!
  // main loop count 1
  // If PT_YIELD() was used, the execution sequence is like below:
  // s_pwm.duty_x200[0] was changed!
  // main loop count 1
  // s_pwm.duty_x200[1] was changed!
  // main loop count 2
  // s_pwm.duty_x200[2] was changed!
  // main loop count 3
  // main loop count 4
  // You must enable pwm before change duty.
  // PWM_Enable(); 
  PT_BEGIN(pt);

  // Implicitly return here.
  PT_WAIT_UNTIL(pt, PWM_IsUpdated());
  
  PWM_ChangeDuty(s_pwm.duty_x200);

  PT_END(pt);
} // PWM_Thread()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
