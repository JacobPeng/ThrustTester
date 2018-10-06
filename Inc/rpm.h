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

#ifndef RPM_H_
#define RPM_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

#include "graham-pt.h"

// ----------------------------------------------------------------------------
// High layer function
// ----------------------------------------------------------------------------
// Captures rpm input.
void RPM_IRQHandler(void);
// Initialize rpm input.
void RPM_Init(void);
// Rpm thread. Updates rpm input per 1s.
PT_THREAD(RPM_Thread(struct pt* pt));

// ----------------------------------------------------------------------------
// Medium layer function
// ----------------------------------------------------------------------------
static bool RPM_IsUpdated(void);

// ----------------------------------------------------------------------------
// Low layer function
// ----------------------------------------------------------------------------

#endif  // #ifndef RPM_H_

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
