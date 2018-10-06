// ----------------------------------------------------------------------------
// ppm.c/h
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
// Captures ppm application.
//
// Revision:
// =========
// Release B0.0.1 - 2018/03/26 - Jacob Peng
//  - Add: Checked whether ppm is lost or not.
// Release A0.0.1 - 2018/01/29 - Jacob Peng
//  - Add: Captured ppm.
//

#ifndef PPM_H_
#define PPM_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

#include "graham-pt.h"

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------
#define PPM_CAPTURE_NOISE 0     // >= 4 us
#define PPM_CAPTURE_MAX   2300  // us
#define PPM_CAPTURE_MIN   700   // us
#define PPM_CAPTURE_1850  1850  // us
#define PPM_VALID_MAX     2000  // us
#define PPM_VALID_MED     1500  // us median
#define PPM_VALID_MIN     1000  // us

// ----------------------------------------------------------------------------
// High layer function
// ----------------------------------------------------------------------------
// Captures ppm input.
void PPM_IRQHandler(void);
// Initialize ppm input.
void PPM_Init(void);
void PPM_Update(uint16_t ppm);
// Throttle behavior, output thrust in the specified throttle range.
bool throttle_behavior(uint8_t throttle_start, uint8_t throttle_end, 
                       uint8_t throttle_step);
// Flight time, Output flight time of drone with battery.
bool flight_time(uint16_t drone_weight, uint8_t battery_cell, float kp, float ki);
// PPM thread. Updates ppm input in real time.
PT_THREAD(PPM_Thread(struct pt* pt));
// Throttle behavior thread. Changes ppm in the specified way.
PT_THREAD(THB_Thread(struct pt* pt));
// Flight time thread. Changes ppm in the specified way.
PT_THREAD(FLG_Thread(struct pt* pt));

// ----------------------------------------------------------------------------
// Medium layer function
// ----------------------------------------------------------------------------
static bool PPM_IsUpdated(void);
static bool THB_IsEnabled(void);
static bool FLG_IsEnabled(void);
static bool thrust_is_updated(void);

// ----------------------------------------------------------------------------
// Low layer function
// ----------------------------------------------------------------------------

#endif  // #ifndef PPM_H_

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
