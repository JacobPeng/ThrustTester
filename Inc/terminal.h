// ----------------------------------------------------------------------------
// terminal.c/h
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
// Process terminal commands.
//
// Revision:
// =========
// Release A0.0.1 - 2018/05/09 - Jacob Peng
//  - Add: some basic commands.
//

#ifndef TERMINAL_H_
#define TERMINAL_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdbool.h>

#include "graham-pt.h"

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// High layer function
// ----------------------------------------------------------------------------
// Terminal thread. Communicates with PC.
PT_THREAD(TER_Thread(struct pt* pt));

// ----------------------------------------------------------------------------
// Medium layer function
// ----------------------------------------------------------------------------
static bool TERMINAL_CmdIsUpdated(void);
// Process terminal commands.
static void TERMINAL_ProcessString(char* str);

// ----------------------------------------------------------------------------
// Low layer function
// ----------------------------------------------------------------------------

#endif  // #ifndef TERMINAL_H_

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
