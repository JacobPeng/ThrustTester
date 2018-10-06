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

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "protocol.h"

#include <stdio.h>
#include <string.h>

#include "rtc.h"
#include "debug.h"
#include "terminal.h"
#include "usart.h"

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
STRUCT_Pro g_pro = {0};

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

static bool PRO_IsUpdated(void) {
  if (g_pro.signal.output_pro) {
    g_pro.signal.output_pro = false;
    return true;
  } else {
    return false;
  } // if else
} // PRO_IsUpdated()

// ----------------------------------------------------------------------------
// PT_THREAD(PRO_Thread(struct pt*))
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : pt - the pointer of thread
// Description  : Protocol thread. Output thrust information to PC.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
PT_THREAD(PRO_Thread(struct pt* pt)) {
  PT_BEGIN(pt);

  // Implicitly return here.
  PT_WAIT_UNTIL(pt, PRO_IsUpdated());  

  printf("%.1f, %.2f, %d, %.2f, %.1f, %d, %.2f, %.1f, %.2f, %.2f, %.2f, %.1f\n",
    g_pro.data.voltage, g_pro.data.current, g_pro.data.thrust, g_pro.data.torque, 
    g_pro.data.throttle, g_pro.data.rpm, g_pro.data.thrust_efficiency_gw, 
    g_pro.data.thrust_efficiency_ga, g_pro.data.vibration_x, 
    g_pro.data.vibration_y, g_pro.data.vibration_z, g_pro.data.power);
  // printf("%02dh %02dm %02ds %02dms\n", 
  //         g_rtc.hour, g_rtc.minute, g_rtc.second, g_rtc.millisecond);

  PT_END(pt);
} // PRO_Thread()


// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
