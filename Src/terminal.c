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

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "terminal.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "usart.h"
#include "hx711.h"
#include "ppm.h"
#include "protocol.h"
#include "debug.h"

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------
#define ARG_NUM 12

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

static bool TERMINAL_CmdIsUpdated(void) {
  if (g_uart3.cplt_rx) {
    g_uart3.cplt_rx = false;
    return true;
  } else {
    return false;
  } // if else
} // TERMINAL_CmdIsUpdated()

// ----------------------------------------------------------------------------
// void CRT_ProcessString(char*)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : str - input string
// Description  : Process terminal commands.
// ----------------------------------------------------------------------------
static void TERMINAL_ProcessString(char* str) {
  uint8_t argc = 0;
  char* argv[ARG_NUM] = {0};
  char* save_ptr = 0;
  const char delimit_buf[] = "(), \r\n";

  // Separate string by tokens, such as comma and space.
  char* p_tmp = strtok_r(str, delimit_buf, &save_ptr);
  while (p_tmp && (argc < ARG_NUM)) {
    argv[argc++] = p_tmp;
    p_tmp = strtok_r((void*)0, delimit_buf, &save_ptr);
  } // while

  if (!argc) {
    printf("<-");
    return;
  } // if

  if (!strcmp(argv[0], "help")) {
#ifdef LANGUAGE_EN
    printf("Valid commands are:\n");
    printf("help\n");
    printf("  Show all commands.\n");
    printf("ping\n");
    printf("  Print pong here to see if the reply works.\n");
    printf("abort\n");
    printf("  Abort the running routine.\n");
    printf("reboot\n");
    printf("  Reboot MCU.\n");
    printf("version\n");
    printf("  Print the current firmware version.\n");
    printf("set_thrust1(enable_or_not)\n");
    printf("  Only THRUST-0 sensor is used in default for thrust, "
              "if you want to use THRUST-0 and THRUST-1 at the same time, "
              "please [set_thrust1(1)] !\n");
    printf("  enable_or_not is from 0 - 1.\n");
    printf("  Example: set_thrust1(1)\n");
    printf("set_ppm(ppm)\n");
    printf("  Set ppm manually.\n");
    printf("  ppm is from 0 - 100, unit of ppm is 1%%.\n");
    printf("  Example: set_ppm(20)\n");
    printf("calibrate_thrust_sensor(sensor_index, weight)\n");
    printf("  Calibrate thrust sensor, get coefficient and offset.\n");
    printf("  sensor_index is from 0 - 2, unit of weight is g.\n");
    printf("  Example: calibrate_thrust_sensor(0, 2000)\n");
    printf("throttle_behavior(throttle_start, throttle_end, "
            "throttle_step)\n");
    printf("  Output raw data about throttle curve per 2 seconds.\n");
    printf("  throttle_xxx are from 0 - 100.\n");
    printf("  Example: throttle_behavior(0, 100, 1)\n");
    printf("flight_time(drone_weight, battery_cell, kp, ki)\n");
    printf("  Output flight time of drone with battery.\n");
    printf("  drone_weight is the weight of the whole drone, unit is g.\n");
    printf("  battery_cell is from 2 - 12, number of battery cells.\n");
    printf("  kp, ki are pid parameters.\n");
    printf("  kp recommend 0; ki, 100g recommend 0.1 and 1000g recommend 0.01.\n");
    printf("  Warning! ki recommend a small value, otherwise ppm will ramp up "
              "rapidly, it will scare you (*Φ皿Φ*)!\n");
    printf("  If you were scared, use [abort] to stop test! "
              "Reduce ki, try again.\n");
    printf("  Example: flight_time(100,  3, 0.0, 0.1)\n");
    printf("  Example: flight_time(1000, 3, 0.0, 0.01)\n");

    printf("\n");
    printf("Invalid commands are (developing):\n");
    printf("set_limit(voltage_max, current_max, power_max)\n");
    printf("  Set maximum limits, protect PCB and other devices.\n");
    printf("  voltage_max, allowed max voltage, unit is V.\n");
    printf("  current_max, allowed max current, unit is A.\n");
    printf("  power_max,   allowed max power,   unit is W.\n");
    printf("  Example: set_limit(27, 40, 1000)\n");
#else
    printf("有效命令:\n");
    printf("help\n");
    printf("  显示所有命令.\n");
    printf("ping\n");
    printf("  回复pong, 用来检查通信是否正常.\n");
    printf("abort\n");
    printf("  终止当前运行中的例程.\n");
    printf("reboot\n");
    printf("  重启MCU.\n");
    printf("version\n");
    printf("  查看当前固件版本.\n");
    printf("set_thrust1(enable_or_not)\n");
    printf("  测量推力时, 默认只使用THRUST-0传感器, "
              "如果想要同时使用THRUST-0和THRUST-1, "
              "请使用 [set_thrust1(1)] 命令!\n");
    printf("  enable_or_not取值0 - 1.\n");
    printf("  示例: set_thrust1(1)\n");
    printf("set_ppm(ppm)\n");
    printf("  手动给定 ppm 油门值.\n");
    printf("  ppm 取值0 - 100, 单位是1%%.\n");
    printf("  示例: set_ppm(20)\n");
    printf("calibrate_thrust_sensor(sensor_index, weight)\n");
    printf("  校准推力传感器, 获取系数和偏移量.\n");
    printf("  sensor_index 取值 0 - 2, weight 单位是g.\n");
    printf("  示例: calibrate_thrust_sensor(0, 2000)\n");
    printf("throttle_behavior(throttle_start, throttle_end, "
            "throttle_step)\n");
    printf("  每2s输出油门曲线原始数据.\n");
    printf("  throttle_xxx 取值 0 - 100.\n");
    printf("  示例: throttle_behavior(0, 100, 1)\n");
    printf("flight_time(drone_weight, battery_cell, kp, ki)\n");
    printf("  测量飞机的整机悬停续航时间.\n");
    printf("  drone_weight 表示整机重量, 单位是g.\n");
    printf("  battery_cell 取值 2 - 12, 电池电芯数量.\n");
    printf("  kp, ki 是pid参数.\n");
    printf("  kp, 建议取0; ki, 100g建议取0.1, 1000g建议取0.01f.\n");
    printf("  警告! 建议ki一开始取一个较小的值，否者油门会上升得特别块, "
              "会吓到你的 (*Φ皿Φ*)!\n");
    printf("  如果吓到你了, 使用 [abort] 命令立即停止测试! 减少ki再次测试.\n");
    printf("  示例: flight_time(100,  3, 0.0, 0.1)\n");
    printf("  示例: flight_time(1000, 3, 0.0, 0.01)\n");

    printf("\n");
    printf("无效命令(开发中):\n");
    printf("set_limit(voltage_max, current_max, power_max)\n");
    printf("  设置最大限制, 保护PCB和其他设备.\n");
    printf("  voltage_max, 允许最大电压, 单位是V.\n");
    printf("  current_max, 允许最大电流, 单位是A.\n");
    printf("  power_max,   允许最大功率, 单位是W.\n");
    printf("  示例: set_limit(27, 40, 1000)\n");
#endif
    printf("\n");
    printf("<-");
  } else if (!strcmp(argv[0], "ping")) {
    printf("pong\n");
    printf("\n");
    printf("<-");
  } else if (!strcmp(argv[0], "abort")) {
    g_pro.signal.abort_routine = true;
#ifdef LANGUAGE_EN
    printf("The running routine will be aborted!\n");
#else
    printf("运行中的例程即将停止!\n");
#endif
  } else if (!strcmp(argv[0], "reboot")) {
#ifdef LANGUAGE_EN
    printf("MCU will reboot now!\n");
#else
    printf("MCU即将重启!\n");
#endif
    printf("\n");
    HAL_NVIC_SystemReset();
  } else if (!strcmp(argv[0], "version")) {
    printf(FIRMWARE_VERSION);
    printf("\n");
    printf("<-");
  } else if (!strcmp(argv[0], "set_thrust1")) {
    if (argc != 2) {
#ifdef LANGUAGE_EN
      printf("Error input! This command need 1 argument.\n");
      printf("set_thrust1(enable_or_not)\n");
      printf("  Example: set_thrust1(1)\n");
#else
      printf("输入错误! 当前命令需要1个参数.\n");
      printf("set_thrust1(enable_or_not)\n");
      printf("  示例: set_thrust1(1)\n");
#endif
      printf("\n");
      printf("<-");
      return;
    } // if
    bool enable_or_not = false;
    sscanf(argv[1], "%u", (unsigned int*)&enable_or_not);
    g_pro.signal.en_thrust1 = enable_or_not;
#ifdef LANGUAGE_EN
    if (enable_or_not)
      printf("THRUST-0 and THRUST-1 are used for thrust at the same time.\n");
    else
      printf("Only THRUST-0 is used for thrust.\n");
#else
    if (enable_or_not)
      printf("THRUST-0 和 THRUST-1 同时用于推力测量.\n");
    else
      printf("只有 THRUST-0 用于推力测量.\n");
#endif
    printf("\n");
    printf("<-");
  } else if (!strcmp(argv[0], "set_ppm")) {
    if (argc != 2) {
#ifdef LANGUAGE_EN
      printf("Error input! This command need 1 argument.\n");
      printf("set_ppm(ppm)\n");
      printf("  Example: set_ppm(20)\n");
#else
      printf("输入错误! 当前命令需要1个参数.\n");
      printf("set_ppm(ppm)\n");
      printf("  示例: set_ppm(20)\n");
#endif
      printf("\n");
      printf("<-");
      return;
    } // if
    uint16_t ppm = 0;
    sscanf(argv[1], "%u", (unsigned int*)&ppm);
    PPM_Update(PPM_VALID_MIN + ppm*10);
    printf("\n");
    printf("<-");
  } else if (!strcmp(argv[0], "calibrate_thrust_sensor")) {
    if (argc != 3) {
#ifdef LANGUAGE_EN
      printf("Error input! This command need 2 arguments.\n");
      printf("calibrate_thrust_sensor(sensor_index, weight)\n");
      printf("  Example: calibrate_thrust_sensor(0, 2000)\n");
#else
      printf("输入错误! 当前命令需要2个参数.\n");
      printf("calibrate_thrust_sensor(sensor_index, weight)\n");
      printf("  示例: calibrate_thrust_sensor(0, 2000)\n");
#endif
      printf("\n");
      printf("<-");
      return;
    } // if
#ifdef LANGUAGE_EN
    printf("Calibrate thrust sensor, get coefficient and offset.\n");
#else
    printf("校准推力传感器, 获取系数和偏移量.\n");
#endif
    uint8_t   sensor_index = 0;
    uint16_t  weight = 0;
    sscanf(argv[1], "%u", (unsigned int*)&sensor_index);
    sscanf(argv[2], "%u", (unsigned int*)&weight);
    if (!calibrate_thrust_sensor(sensor_index, weight)) {
      printf("Sorry /(ToT)/~~!\n");
#ifdef LANGUAGE_EN
      printf("Please check input arguments.\n");
      printf("Try again.\n");
#else
      printf("请检查输入参数.\n");
      printf("再试一次.\n");
#endif
    } // if else
    printf("\n");
    printf("<-");
  } else if (!strcmp(argv[0], "throttle_behavior")) {
    if (argc != 4) {
#ifdef LANGUAGE_EN
      printf("Error input! This command need 3 arguments.\n");
      printf("throttle_behavior(voltage, throttle_start, throttle_end, "
              "throttle_step)\n");
      printf("  Example: throttle_behavior(0, 100, 1)\n");
#else
      printf("输入错误! 当前命令需要3个参数.\n");
      printf("throttle_behavior(voltage, throttle_start, throttle_end, "
              "throttle_step)\n");
      printf("  示例: throttle_behavior(0, 100, 1)\n");
#endif
      printf("\n");
      printf("<-");
      return;
    } // if
    uint8_t throttle_start = 0;
    uint8_t throttle_end = 0;
    uint8_t throttle_step = 0;
    g_pro.signal.cmd = kProCmdThb;
    sscanf(argv[1], "%u", (unsigned int*)&throttle_start);
    sscanf(argv[2], "%u", (unsigned int*)&throttle_end);
    sscanf(argv[3], "%u", (unsigned int*)&throttle_step);
    if (!throttle_behavior(throttle_start, throttle_end, throttle_step)) {
      printf("Sorry /(ToT)/~~!\n");
#ifdef LANGUAGE_EN
      printf("Please check input arguments.\n");
      printf("Try again.\n");
#else
      printf("请检查输入参数.\n");
      printf("再试一次.\n");
#endif
      printf("\n");
      printf("<-");
    } // if
    g_pro.signal.cmd = kProCmdUnknown;
  } else if (!strcmp(argv[0], "flight_time")) {
    if (argc != 5) {
#ifdef LANGUAGE_EN
      printf("Error input! This command need 4 arguments.\n");
      printf("flight_time(drone_weight, battery_cell, kp, ki)\n");
      printf("  Example: flight_time(100,  3, 0.0, 0.1)\n");
      printf("  Example: flight_time(1000, 3, 0.0, 0.01)\n");
#else
      printf("输入错误! 当前命令需要4个参数.\n");
      printf("flight_time(drone_weight, battery_cell, kp, ki)\n");
      printf("  示例: flight_time(100,  3, 0.0, 0.1)\n");
      printf("  示例: flight_time(1000, 3, 0.0, 0.01)\n");
#endif
      printf("\n");
      printf("<-");
      return;
    } // if
    uint16_t drone_weight = 0;
    uint8_t  battery_cell = 0;
    float    kp = 0.0f;
    float    ki = 0.0f;
    g_pro.signal.cmd = kProCmdFlg;
    sscanf(argv[1], "%u", (unsigned int*)&drone_weight);
    sscanf(argv[2], "%u", (unsigned int*)&battery_cell);
    sscanf(argv[3], "%f", &kp);
    sscanf(argv[4], "%f", &ki);
    if (!flight_time(drone_weight, battery_cell, kp, ki)) {
      printf("Sorry /(ToT)/~~!\n");
#ifdef LANGUAGE_EN
      printf("Please check input arguments.\n");
      printf("Try again.\n");
#else
      printf("请检查输入参数.\n");
      printf("再试一次.\n");
#endif
      printf("\n");
      printf("<-");
    } // if else
    g_pro.signal.cmd = kProCmdUnknown;
  } else if (!strcmp(argv[0], "set_limit")) {
#ifdef LANGUAGE_EN
    printf("Invalid command (developing).\n");
#else
    printf("无效命令 (开发中).\n");
#endif
    printf("\n");
    printf("<-");
  } else {
#ifdef LANGUAGE_EN
    printf("Unknown command, try [help].\n");
#else
    printf("未知命令, 试一下 [help].\n");
#endif
    printf("\n");
    printf("<-");
  } // if else
} // TERMINAL_ProcessString()

// ----------------------------------------------------------------------------
// PT_THREAD(TER_Thread(struct pt*))
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : pt - the pointer of thread
// Description  : Terminal thread. Communicates with PC.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
PT_THREAD(TER_Thread(struct pt* pt)) {
 // static STRUCT_RtcTimerMs s_prot_timer;

  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, TERMINAL_CmdIsUpdated());

  // UARTx_PutBuffer(&UART_PRINTF, g_uart3.buf_rx, g_uart3.len_rx);
  TERMINAL_ProcessString((char*)g_uart3.buf_rx);
  memset(g_uart3.buf_rx, 0, sizeof(g_uart3.buf_rx));

  PT_END(pt);
} // TER_Thread()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
