// ----------------------------------------------------------------------------
// protocol.c/h
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
// Communication protocol with PC.
//
// Revision:
// =========
// Release A0.0.1 - 2018/03/27 - Jacob Peng
//  - Add: Output raw data.
//

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

#include "graham-pt.h"

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
typedef struct {  
  float     voltage;
  float     current;
  int16_t   thrust;
  float     torque;
  float     throttle;
  uint16_t  rpm;
  float     thrust_efficiency_gw;
  float     thrust_efficiency_ga;
  float     vibration_x;
  float     vibration_y;
  float     vibration_z;
  float     power;
} STRUCT_ProData;

enum ENUM_ProIndex {
  kProIndexVoltage = 0,
  kProIndexCurrent,
  kProIndexThrust,
  kProIndexTorque,
  kProIndexThrottle,
  kProIndexRpm,
  kProIndexTegw,
  kProIndexTega,
  kProIndexVibrationX,
  kProIndexVibrationY,
  kProIndexVibrationZ,
  kProIndexPower,
  kProIndexCount
};

typedef struct {
  uint8_t cmd;
  bool    en_thrust1;
  bool    output_pro;
  bool    abort_routine;
  bool    updated[kProIndexCount];
} STRUCT_ProSignal;

enum ENUM_ProCmd {
  kProCmdUnknown = 0,
  kProCmdThb,
  kProCmdFlg,
  kProCmdCOunt
};

typedef struct  {
  STRUCT_ProData    data;
  STRUCT_ProSignal  signal;
} STRUCT_Pro;

extern STRUCT_Pro g_pro;

// ----------------------------------------------------------------------------
// High layer function
// ----------------------------------------------------------------------------
// Protocol thread. Output thrust information to PC.
PT_THREAD(PRO_Thread(struct pt* pt));

// ----------------------------------------------------------------------------
// Medium layer function
// ----------------------------------------------------------------------------
static bool PRO_IsUpdated(void);

// ----------------------------------------------------------------------------
// Low layer function
// ----------------------------------------------------------------------------

#endif  // #ifndef PROTOCOL_H_

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
