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
// Release A0.0.1 - 2018/01/29 - Jacob Peng
//  - Add: Outputs pwm.
//  - Add: Uses protothread.
//

#ifndef PWM_H_
#define PWM_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

#include "graham-pt.h"

// ----------------------------------------------------------------------------
// High layer function
// ----------------------------------------------------------------------------
// Initialize pwm output.
void PWM_Init(void);
void PWM_Update(uint16_t duty_x200);
// Changes pwm duty ratio.
void PWM_ChangeDuty(uint16_t duty_x200);
// Enable pwm output forcibly.
void PWM_Enable(void);
// Diable pwm output forcibly.
void PWM_Disable(void);
// PWM thread. Updates pwm output in real time.
PT_THREAD(PWM_Thread(struct pt* pt));

// ----------------------------------------------------------------------------
// Medium layer function
// ----------------------------------------------------------------------------
static bool PWM_IsUpdated(void);

// ----------------------------------------------------------------------------
// Low layer function
// ----------------------------------------------------------------------------

#endif  // #ifndef PWM_H_

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
