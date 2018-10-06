// ----------------------------------------------------------------------------
// mpu6050.c/h
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
// Read mpu6050 applicaiton.
//
// Revision:
// =========
// Release B0.0.1 - 2018/03/05 - Jacob Peng
//  - Mod: Deprecated dmp engine, used fifo only.
// Release A0.0.1 - 2018/02/01 - Jacob Peng
//  - Add: Initializes mpu6050.
//  - Add: Uses DMP engine.
//

#ifndef MPU6050_H_
#define MPU6050_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

#include "graham-pt.h"

// ----------------------------------------------------------------------------
// High Layer Function
// ----------------------------------------------------------------------------
// Initialize mpu6050.
void MPU6050_Init(void);
// MPU6050 thread. Updates gyro and accel periodically.
PT_THREAD(MPU_Thread(struct pt* pt));

// ----------------------------------------------------------------------------
// Medium Layer Function
// ----------------------------------------------------------------------------
static void run_self_test(void);

// ----------------------------------------------------------------------------
// Low Layer Function
// ----------------------------------------------------------------------------

#endif  // #ifndef MPU6050_H_

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
