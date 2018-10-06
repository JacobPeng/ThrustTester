// ----------------------------------------------------------------------------
// debug.c/h
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
// Debug gpio driver.
//
// Revision:
// =========
// Release A0.0.1 - 2018/01/26 - Jacob Peng
//  - Add: Initialized debug gpio.
//

#ifndef DEBUG_H_
#define DEBUG_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------
// #define DEBUG_ALL
// #define DEBUG_SYSTICK
// #define DEBUG_WATCHDOG
// #define DEBUG_KEY
// #define DEBUG_RPM
// #define DEBUG_ADC
// #define DEBUG_MPU
// #define DEBUG_PROTOCOL

// Help language, english or chinese.
// #define LANGUAGE_EN
#define LANGUAGE_CN

// Suppress multiple byte warning.
#ifdef __CC_ARM
#pragma diag_suppress 870 
#endif

// ----------------------------------------------------------------------------
// High Layer Function
// ----------------------------------------------------------------------------
void DEBUG_Init(void);
void DEBUG_GpioOn(void);
void DEBUG_GpioOff(void);
void DEBUG_GpioToggle(void);
void DEBUG_LedOn(void);
void DEBUG_LedOff(void);
void DEBUG_LedToggle(void);

// ----------------------------------------------------------------------------
// Medium Layer Function
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Low Layer Function
// ----------------------------------------------------------------------------

#endif  // #ifndef DEBUG_H_

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
