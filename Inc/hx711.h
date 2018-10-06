// ----------------------------------------------------------------------------
// hx711.c/h
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
// HX711 driver.
//
// Revision:
// =========
// Release A0.0.1 - 2018/03/12 - Jacob Peng
//  - Add: Basic driver.
//

#ifndef HX711_H_
#define HX711_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------
#define AMP_RATE_A_128  128
#define AMP_RATE_A_64   64
#define AMP_RATE_B_32   32

enum ENUM_Hx711Index {
  kHx711Index1 = 0,
  kHx711Index2,
  kHx711Index3,
  kHx711Count
};

#define HX711_INDEX_THRUST0 kHx711Index1
#define HX711_INDEX_THRUST1 kHx711Index2
#define HX711_INDEX_TORQUE  kHx711Index3

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
extern float    g_coefficient[kHx711Count];
extern int32_t  g_offset[kHx711Count];

// ----------------------------------------------------------------------------
// High layer function
// ----------------------------------------------------------------------------
// Initialize HX711 IC.
void HX711_Init(void);
bool HX711_IsFound(uint8_t index);
void HX711_SetCoefficient(uint8_t index, float coefficient);
void HX711_SetOffset(uint8_t index, int32_t offset);
float HX711_ReadCoefficient(uint8_t index);
int32_t HX711_ReadOffset(uint8_t index);
// Read HX711 ad value, max 10 or 80 Hz.
int32_t HX711_ReadAd(uint8_t index, bool* ready_or_not, uint8_t amp_rate);
// Calibrate HX711, calculate coefficient and offset.
bool calibrate_thrust_sensor(uint8_t hx711_index, uint16_t weight);

// ----------------------------------------------------------------------------
// Medium layer function
// ----------------------------------------------------------------------------
static void HX711_SetParameter(void);
static void HX711_PulseScl(uint8_t index);
static void HX711_SetAmp(uint8_t index, uint8_t amp_rate);
static bool HX711_IsReady(uint8_t index);

// ----------------------------------------------------------------------------
// Low layer function
// ----------------------------------------------------------------------------

#endif  // #ifndef HX711_H_

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
