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

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "ppm.h"

#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"

#include "tim.h"
#include "pwm.h"
#include "rtc.h"
#include "protocol.h"
#include "adc.h"
#include "main.h"
#include "debug.h"

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------
#define PROTOCOL_TITLE_EN "Voltage(V),Current(A),Thrust(g),Torque(N.m)," \
                          "Throttle(%%),Rpm(rpm/min),T.E.1(g/W),T.E.2(g/A)," \
                          "X Vibr.(g0),Y Vibr.(g0),Z Vibr.(g0),Power(W)\n"
#define PROTOCOL_TITLE_CN "电压(V),电流(A),推力(g),力矩(N.m)," \
                          "油门(%%),转速(rpm/min),力效1(g/W),力效2(g/A)," \
                          "X 振动.(g0),Y 振动.(g0),Z 振动.(g0),功率(W)\n"                          

#define DURATION_TIME_THROTTLE_PRESTART 10  // s
#define DURATION_TIME_THROTTLE          2   // s

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
typedef struct {
  bool      enabled;
  uint16_t  throttle;       // 1000-2000us
  uint16_t  throttle_start; // 1000-2000us
  uint16_t  throttle_end;   // 1000-2000us
  uint16_t  throttle_step;  // 0-1000us
} STRUCT_ThrottleBehavior;

typedef struct {
  bool      enabled;
  bool      voltage_less_than_3_5V;
  bool      voltage_less_than_3_0V;
  bool      disarmed;
  uint8_t   battery_cell;
  uint16_t  thrust_equals_weight_cnt;
  uint16_t  thrust_less_than_weight_cnt;
  uint16_t  battery_3_5v_cnt;
  uint16_t  battery_3_0v_cnt;
  int16_t   thrust_error;
  int32_t   thrust_integral;
  uint16_t  drone_weight;
  int16_t   throttle;
  uint16_t  throttle_hover_full_voltage;
  uint32_t  time;
  uint32_t  time_start;
  uint32_t  time_end_3_5V;
  uint32_t  time_end_3_0V;
  float     kp;
  float     ki;
} STRUCT_FlightTime;

static STRUCT_ThrottleBehavior s_thr_beh = {0};
STRUCT_FlightTime g_flight_time = {0};

static bool s_ppm_is_updated = false;
static uint16_t s_ppm = 0;

// ----------------------------------------------------------------------------
// Initialization Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void PPM_Init(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Initialize ppm input.
// ----------------------------------------------------------------------------
void PPM_Init(void) {
  PWM_Enable();
} // PPM_Init()

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

void PPM_Update(uint16_t ppm) {
  s_ppm = ppm;
  s_ppm_is_updated = true;  
} // PPM_Update()

static bool PPM_IsUpdated(void) {
  if (s_ppm_is_updated) {
    s_ppm_is_updated = false;
    return true;
  } else {
    return false;
  } // if else
} // PPM_IsUpdated()

static bool THB_IsEnabled(void) {
  if (s_thr_beh.enabled) {
    s_thr_beh.enabled = false;
    return true;
  } else {
    return false;
  } // if else  
} // THB_IsEnabled()

static bool FLG_IsEnabled(void) {
  if (g_flight_time.enabled) {
    g_flight_time.enabled = false;
    return true;
  } else {
    return false;
  } // if else
} // FLG_IsEnabled()

static bool thrust_is_updated(void) {
  if (g_pro.signal.updated[kProIndexThrust]) {
    g_pro.signal.updated[kProIndexThrust] = false;
    return true;
  } else {
    return false;
  } // if else 
} // thrust_is_updated()

// ----------------------------------------------------------------------------
// bool throttle_behavior(uint8_t, uint8_t, uint8_t)
// ----------------------------------------------------------------------------
// Return Value : 0, failed; 1, successfully.
// Parameters   : 
//  throttle_start  - start throttle, 0 to 100
//  throttle_end    - end throttle, 0 to 100
//  throttle_step   - step, 0 to 100
// Description  : Throttle behavior, output thrust in the specified throttle range.
// ----------------------------------------------------------------------------
#define THROTTLE_MAX  100   // %

bool throttle_behavior(uint8_t throttle_start, uint8_t throttle_end, 
                       uint8_t throttle_step) {
  if (throttle_start > THROTTLE_MAX) return false;
  if (throttle_end   > THROTTLE_MAX) return false;
  if (throttle_step  > THROTTLE_MAX) return false;
  if (throttle_start + throttle_step > throttle_end) return false;

  memset(&s_thr_beh, 0, sizeof(s_thr_beh));
  // Get inputs.
  s_thr_beh.throttle        = PPM_VALID_MIN;  // 1000-2000us  
  s_thr_beh.throttle_start  = throttle_start*10 + PPM_VALID_MIN;  // 1000-2000us  
  s_thr_beh.throttle_end    = throttle_end*10 + PPM_VALID_MIN;  // 1000-2000us    
  s_thr_beh.throttle_step   = throttle_step*10; // 0-1000us 
  // Set flag.
  s_thr_beh.enabled = true;

  return true;
} // throttle_behavior()

// ----------------------------------------------------------------------------
// bool throttle_behavior(uint16_t)
// ----------------------------------------------------------------------------
// Return Value : 0, failed; 1, successfully.
// Parameters   : 
//  drone_weight - weight of the whole drone
//  battery_cell - number of battery cells
//  kp           - p of pid
//  ki           - i of pid
// Description  : Flight time, Output flight time of drone with battery.
// ----------------------------------------------------------------------------
bool flight_time(uint16_t drone_weight, uint8_t battery_cell, float kp, float ki) {
  if (battery_cell > 12 || battery_cell < 2) return false;

  memset(&g_flight_time, 0, sizeof(g_flight_time));
  // Get inputs.
  g_flight_time.drone_weight = drone_weight;
  g_flight_time.battery_cell = battery_cell;
  g_flight_time.kp = kp;
  g_flight_time.ki = ki;
  // Set flag.
  g_flight_time.enabled = true;

  return true;
} // throttle_behavior()

// ----------------------------------------------------------------------------
// PT_THREAD(PPM_Thread(struct pt*))
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : pt - the pointer of thread
// Description  : PPM thread. Updates ppm input in real time.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
PT_THREAD(PPM_Thread(struct pt* pt)) {
  PT_BEGIN(pt);

  // Implicitly return here.
  PT_WAIT_UNTIL(pt, PPM_IsUpdated());

  uint16_t ppm_to_pwm = 0;
  // PPM duty_x200 ranges from 500 to 1000.
  // PWM for ppm is 50Hz, 1ms to 2ms.
  if ((s_ppm >= PPM_VALID_MIN) &&
      (s_ppm <= PPM_VALID_MAX)) {  
    ppm_to_pwm = s_ppm;
    // PWM_Enable(kPwmIndexPpm);
  } else if (s_ppm < PPM_VALID_MIN) {
    // PWM_Disable(kPwmIndexPpm);
    ppm_to_pwm = 0;
  } else if (s_ppm > PPM_VALID_MAX) {
    ppm_to_pwm = PPM_VALID_MAX;
  } // else if
  
  PWM_Update(ppm_to_pwm);
  // Update protocol data.
  if (!ppm_to_pwm) {
    g_pro.data.throttle = 0.0f;
  } else {
    g_pro.data.throttle = (ppm_to_pwm - 1000)/10.0f;
  } // if else
  // Update protocol signal.
  g_pro.signal.updated[kProIndexThrottle] = true;  

  PT_END(pt);
} // PPM_Thread()

// ----------------------------------------------------------------------------
// PT_THREAD(THB_Thread(struct pt*))
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : pt - the pointer of thread
// Description  : Throttle behavior thread. Changes ppm in the specified way.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
PT_THREAD(THB_Thread(struct pt* pt)) {
  static STRUCT_RtcTimerS s_thr_timer;

  PT_BEGIN(pt);

  // Implicitly return here.
  PT_WAIT_UNTIL(pt, THB_IsEnabled());

#ifdef LANGUAGE_EN 
  printf("[ 0s]: throttle_behavior will start soon.\n");
#else
  printf("[ 0s]: 油门行为测试即将开始.\n");
#endif
  // printf("%02dh %02dm %02ds\n", g_rtc.hour, g_rtc.minute, g_rtc.second);  
  // First, throttle_zero for 10s, at 0s. 
  RTC_SetS(&s_thr_timer, DURATION_TIME_THROTTLE_PRESTART >> 1); 
  PPM_Update(PPM_VALID_MIN);
  // Implicitly return here.
  PT_WAIT_UNTIL(pt, RTC_ExpiredS(&s_thr_timer));
  // Wait another 5s.
  RTC_SetS(&s_thr_timer, DURATION_TIME_THROTTLE_PRESTART >> 1);  
  // Get sensor start_offset, at 5s.
#ifdef LANGUAGE_EN 
  printf("[%2ds]: Read sensor start offset now.\n",
          DURATION_TIME_THROTTLE_PRESTART >> 1);
#else
  printf("[%2ds]: 读取传感器起始偏移量.\n",
          DURATION_TIME_THROTTLE_PRESTART >> 1);
#endif  
  // printf("%02dh %02dm %02ds\n", g_rtc.hour, g_rtc.minute, g_rtc.second);
  for (uint8_t i = 0; i < (uint8_t)kHx711Count; ++i) {
    g_adc_signal.read_start_offset[i] = true;
  } // for
  // Implicitly return here.
  PT_WAIT_UNTIL(pt, RTC_ExpiredS(&s_thr_timer));
  // Second, throttle_start for 2s, at 10s.
#ifdef LANGUAGE_EN 
  printf("[%2ds]: throttle_behavior starts now.\n",
          DURATION_TIME_THROTTLE_PRESTART);
  printf("\n");
  printf(PROTOCOL_TITLE_EN);  
#else
  printf("[%2ds]: 油门行为测试现在开始.\n",
          DURATION_TIME_THROTTLE_PRESTART);
  printf("\n");
  printf(PROTOCOL_TITLE_CN);  
#endif  
  // printf("%02dh %02dm %02ds\n", g_rtc.hour, g_rtc.minute, g_rtc.second);
  for (s_thr_beh.throttle = s_thr_beh.throttle_start; 
       s_thr_beh.throttle <= s_thr_beh.throttle_end; 
       s_thr_beh.throttle += s_thr_beh.throttle_step) {
    if (g_pro.signal.abort_routine) break;
    RTC_SetS(&s_thr_timer, DURATION_TIME_THROTTLE >> 1); 
    PPM_Update(s_thr_beh.throttle);
    // Implicitly return here.
    PT_WAIT_UNTIL(pt, RTC_ExpiredS(&s_thr_timer));
    RTC_SetS(&s_thr_timer, DURATION_TIME_THROTTLE >> 1);
    // Output protocol data, at 11s.
    g_pro.signal.output_pro = true;     
    // Implicitly return here.
    PT_WAIT_UNTIL(pt, RTC_ExpiredS(&s_thr_timer));   
  } // for
  // Third, everything was done.
  PPM_Update(0);
  if (g_pro.signal.abort_routine) {
    g_pro.signal.abort_routine = false;
#ifdef LANGUAGE_EN 
    printf("throttle_behavior was aborted.\n");
#else
    printf("油门行为终止.\n");
#endif
  } else {
#ifdef LANGUAGE_EN 
    printf("Manufacturer\n");
    printf("Comment\n");
    printf("Motor Model\n");
    printf("ESC Model\n");
    printf("Propeller\n");
    printf("Tester\n");
    printf("Date\n");
    printf("Test Item,throttle_behavior\n");
    printf("Test Parameters,\"throttle_start(%%): %d, "
            "throttle_end(%%): %d, throttle_step(%%): %d\"\n", 
            (s_thr_beh.throttle_start-PPM_VALID_MIN)/10, 
            (s_thr_beh.throttle_end-PPM_VALID_MIN)/10, 
            s_thr_beh.throttle_step/10);
    printf("\"Thrust Tester. Copyright (c) 2017-2018, Jacob Peng.\"\n");   
    printf("firmware version:");
    printf(FIRMWARE_VERSION);
    printf("https://github.com/JacobPeng/ThrustTester\n"); 
    printf("\n");
    printf("Please save thrust data to a file "
            "(1806M_HG_throttle_behavior_20180516.csv)!\n");
    printf("Good job (*^_^*)!\n");
#else
    printf("制造商\n");
    printf("备注\n");
    printf("电机型号\n");
    printf("电调型号\n");
    printf("桨叶\n");
    printf("测试者\n");
    printf("日期\n");
    printf("测试项, throttle_behavior\n");
    printf("测试参数,\"throttle_start(%%): %d, "
            "throttle_end(%%): %d, throttle_step(%%): %d\"\n", 
            (s_thr_beh.throttle_start-PPM_VALID_MIN)/10, 
            (s_thr_beh.throttle_end-PPM_VALID_MIN)/10, 
            s_thr_beh.throttle_step/10);
    printf("\"推力测试仪. 版权所有 (c) 2017-2018, Jacob Peng.\"\n");   
    printf("固件版本:");
    printf(FIRMWARE_VERSION);
    printf("\nhttps://github.com/JacobPeng/ThrustTester\n"); 
    printf("\n");
    printf("请保存推力数据到文件 (2312M-DJI-throttle-behavior-20180516.csv)!\n");
    printf("好样的 (*^_^*)!\n");
#endif    
  } // if else
  printf("\n");
  printf("<-");

  PT_END(pt);
} // THB_Thread()

// ----------------------------------------------------------------------------
// PT_THREAD(FLG_Thread(struct pt*))
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : pt - the pointer of thread
// Description  : Flight time thread. Changes ppm in the specified way.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
PT_THREAD(FLG_Thread(struct pt* pt)) {
  PT_YIELDING();
  static STRUCT_RtcTimerS s_flg_timer;
  static STRUCT_RtcTimerS s_timer;

  PT_BEGIN(pt);

  // Implicitly return here.
  PT_WAIT_UNTIL(pt, FLG_IsEnabled());

#ifdef LANGUAGE_EN 
  printf("[ 0s]: flight_time will start soon.\n");
#else
  printf("[ 0s]: 飞行时间测试即将开始.\n");
#endif
  // printf("%02dh %02dm %02ds\n", g_rtc.hour, g_rtc.minute, g_rtc.second);  
  // First, throttle_zero for 10s, at 0s. 
  RTC_SetS(&s_flg_timer, DURATION_TIME_THROTTLE_PRESTART >> 1); 
  PPM_Update(PPM_VALID_MIN);
  // Implicitly return here.
  PT_WAIT_UNTIL(pt, RTC_ExpiredS(&s_flg_timer));
  // Wait another 5s.
  RTC_SetS(&s_flg_timer, DURATION_TIME_THROTTLE_PRESTART >> 1);  
  // Get sensor start_offset, at 5s.
#ifdef LANGUAGE_EN 
  printf("[%2ds]: Read sensor start offset now.\n",
          DURATION_TIME_THROTTLE_PRESTART >> 1);
#else
  printf("[%2ds]: 读取传感器起始偏移量.\n",
          DURATION_TIME_THROTTLE_PRESTART >> 1);
#endif 
  // printf("%02dh %02dm %02ds\n", g_rtc.hour, g_rtc.minute, g_rtc.second);
  for (uint8_t i = 0; i < (uint8_t)kHx711Count; ++i) {
    g_adc_signal.read_start_offset[i] = true;
  } // for
  // Implicitly return here.
  PT_WAIT_UNTIL(pt, RTC_ExpiredS(&s_flg_timer));
  // Second, throttle_pid for variable time, at 10s.
#ifdef LANGUAGE_EN 
  printf("[%2ds]: flight_time starts now.\n",
          DURATION_TIME_THROTTLE_PRESTART);
  printf("\n");
  printf(PROTOCOL_TITLE_EN);  
#else
  printf("[%2ds]: 飞行时间测试现在开始.\n",
          DURATION_TIME_THROTTLE_PRESTART);
  printf("\n");
  printf(PROTOCOL_TITLE_CN); 
#endif  

#define FLG_OUTPUT_INTERVAL 10  // s
  RTC_SetS(&s_timer, FLG_OUTPUT_INTERVAL); 
#define KP 0.0f
#define KI 0.1f // 100g, 0.1f; 1000g, 0.01f.
  // Run pid for ppm. Increment type pid.
  while (!g_flight_time.voltage_less_than_3_0V) {
    if (g_pro.signal.abort_routine) break;
    PT_WAIT_UNTIL(pt, thrust_is_updated());
    g_flight_time.thrust_error = g_flight_time.drone_weight - g_pro.data.thrust;
    g_flight_time.thrust_integral += g_flight_time.thrust_error;
    g_flight_time.throttle = PPM_VALID_MIN + 
      g_flight_time.kp*g_flight_time.thrust_error + 
      g_flight_time.ki*g_flight_time.thrust_integral;
    // Limit pid throttle.
    if (g_flight_time.throttle > PPM_VALID_MAX)
      g_flight_time.throttle = PPM_VALID_MAX;
    else if (g_flight_time.throttle < PPM_VALID_MIN)
      g_flight_time.throttle = PPM_VALID_MIN;    
    PPM_Update(g_flight_time.throttle);
    // Output protocol data.
    if (RTC_ExpiredS(&s_timer)) {
      RTC_SetS(&s_timer, FLG_OUTPUT_INTERVAL); 
      g_pro.signal.output_pro = true;
      PT_YIELD(pt);
    } // if
#define MAX(a, b) (((a) > (b)) ? (a) : (b)) 
#define THRUST_EQUALS_WEIGHT_ERROR_MIN_VALUE    10   // g
#define THRUST_EQUALS_WEIGHT_ERROR_MIN_PERCENT  0.05f
#define THRUST_EQUALS_WEIGHT_CNT                20 // 1 count, 0.1s.
    // Determine whether thrust == drone_weight.
    // Drone was not disarmed here.
    if (!g_flight_time.disarmed) {
      if (abs(g_flight_time.thrust_error) <= 
          MAX(THRUST_EQUALS_WEIGHT_ERROR_MIN_VALUE, 
              THRUST_EQUALS_WEIGHT_ERROR_MIN_PERCENT*g_flight_time.drone_weight)) {
        if (g_flight_time.thrust_equals_weight_cnt < THRUST_EQUALS_WEIGHT_CNT)
          ++g_flight_time.thrust_equals_weight_cnt;
        else
          g_flight_time.thrust_equals_weight_cnt = THRUST_EQUALS_WEIGHT_CNT;
        if (THRUST_EQUALS_WEIGHT_CNT == g_flight_time.thrust_equals_weight_cnt) {
          g_flight_time.thrust_equals_weight_cnt = 0;
          g_flight_time.throttle_hover_full_voltage = g_flight_time.throttle;
          // Drone was disarmed, record time.
          g_flight_time.time_start = RTC_TickS();    
          g_flight_time.disarmed = true;
#ifdef LANGUAGE_EN 
          printf("Drone was armed.\n");
#else
          printf("飞机已经解锁.\n");
#endif
          // Output protocol data.
          g_pro.signal.output_pro = true;
          PT_YIELD(pt);
        } // if
      } else {
        g_flight_time.thrust_equals_weight_cnt = 0;        
      } // if else
    } else {
#define BATTERY_3_5V_CNT  20 // 1 count, 0.1s.      
      // Drone was disarmed here.
      // Battery cell voltage was greater than 3.5V.  
      if (!g_flight_time.voltage_less_than_3_5V) {            
        if (g_pro.data.voltage <= g_flight_time.battery_cell*3.5f) {
          if (g_flight_time.battery_3_5v_cnt < BATTERY_3_5V_CNT)
            ++g_flight_time.battery_3_5v_cnt;
          else
            g_flight_time.battery_3_5v_cnt = BATTERY_3_5V_CNT;
          if (BATTERY_3_5V_CNT == g_flight_time.battery_3_5v_cnt) {
            g_flight_time.battery_3_5v_cnt = 0;
            // Battery cell voltage was less than 3.5V, record time.
            g_flight_time.time_end_3_5V = RTC_TickS();    
            g_flight_time.voltage_less_than_3_5V = true;
            g_flight_time.time = (g_flight_time.time_end_3_5V > 
                                  g_flight_time.time_start) ?
              (g_flight_time.time_end_3_5V - g_flight_time.time_start) :
              g_flight_time.time_end_3_5V + (RTC_S_MAX - g_flight_time.time_start);
#ifdef LANGUAGE_EN 
            printf("Cell voltage 3.5V: "
                   "the safe  flight time is %02dh %02dm %02ds.\n", 
                    (uint8_t)(g_flight_time.time/3600), 
                    (uint8_t)((g_flight_time.time%3600)/60),
                    (uint8_t)((g_flight_time.time%3600)%60)); 
#else
            printf("单节电芯电压 3.5V: "
                   "安全飞行时间 %02d小时 %02d分钟 %02d秒.\n", 
                    (uint8_t)(g_flight_time.time/3600), 
                    (uint8_t)((g_flight_time.time%3600)/60),
                    (uint8_t)((g_flight_time.time%3600)%60)); 
#endif                         
            // Output protocol data.
            g_pro.signal.output_pro = true;
            PT_YIELD(pt);
          } // if
        } else {
          g_flight_time.battery_3_5v_cnt = 0;
        } // if else
      } else {
#if 0
#define BATTERY_3_0V_CNT  200 // 1 count, 0.01s.
        // Battery cell voltage was less than 3.5V.
        if (g_pro.data.voltage <= g_flight_time.battery_cell*3.0f) {
          if (g_flight_time.battery_3_0v_cnt < BATTERY_3_0V_CNT)
            ++g_flight_time.battery_3_0v_cnt;
          else
            g_flight_time.battery_3_0v_cnt = BATTERY_3_0V_CNT;
          if (BATTERY_3_0V_CNT == g_flight_time.battery_3_0v_cnt) {
            g_flight_time.battery_3_0v_cnt = 0;
            // Battery cell voltage was less than 3.0V, record time.
            g_flight_time.time_end_3_0V = RTC_TickS();
            g_flight_time.voltage_less_than_3_0V = true;
#ifdef LANGUAGE_EN
            printf("Cell voltage 3.0V, determined by voltage.\n");
#else
            printf("单节电芯电压3.0V, 根据电压判断.\n");
#endif
            // Output protocol data.
            g_pro.signal.output_pro = true;
            PT_YIELD(pt);
          } // if
        } else {
          g_flight_time.battery_3_0v_cnt = 0;        
        } // if else
#endif
      } // if else
#define THRUST_LESS_THAN_WEIGHT_ERROR_MIN_VALUE   25  // g
#define THRUST_LESS_THAN_WEIGHT_ERROR_MIN_PERCENT 0.1f
#define THRUST_LESS_THAN_WEIGHT_CNT               100 // 1 count, 0.1s.       
      if (abs(g_flight_time.thrust_error) > 
          MAX(THRUST_LESS_THAN_WEIGHT_ERROR_MIN_VALUE, 
              THRUST_LESS_THAN_WEIGHT_ERROR_MIN_PERCENT*
              g_flight_time.drone_weight)) {
        if (g_flight_time.thrust_less_than_weight_cnt < 
            THRUST_LESS_THAN_WEIGHT_CNT)
          ++g_flight_time.thrust_less_than_weight_cnt;
        else
          g_flight_time.thrust_less_than_weight_cnt = THRUST_LESS_THAN_WEIGHT_CNT;
        if (THRUST_LESS_THAN_WEIGHT_CNT == 
            g_flight_time.thrust_less_than_weight_cnt) {
          g_flight_time.thrust_less_than_weight_cnt = 0;
          // Battery cell voltage was less than 3.0V, record time.
          g_flight_time.time_end_3_0V = RTC_TickS() - 8;
          g_flight_time.voltage_less_than_3_0V = true;            
#ifdef LANGUAGE_EN
          printf("Cell voltage 3.0V, determined by thrust.\n");
#else
          printf("单节电芯电压 3.0V, 根据推力判断.\n");
#endif
          // Output protocol data.
          g_pro.signal.output_pro = true;
          PT_YIELD(pt);
        } // if
      } else {
        g_flight_time.thrust_less_than_weight_cnt = 0;        
      } // if else
    } // if else
  } // while
  // Third, everything was done.
  // Drone was armed, record time.
  // HAL_Delay(10000);  
  PPM_Update(0);
  if (g_pro.signal.abort_routine) {
    g_pro.signal.abort_routine = false;
#ifdef LANGUAGE_EN 
    printf("flight_time was aborted.\n");
#else
    printf("飞行时间测试终止.\n");
#endif
  } else {
    g_flight_time.time = (g_flight_time.time_end_3_5V > g_flight_time.time_start) ?
      (g_flight_time.time_end_3_5V - g_flight_time.time_start) :
      g_flight_time.time_end_3_5V + (RTC_S_MAX - g_flight_time.time_start);
#ifdef LANGUAGE_EN 
    printf("Cell voltage 3.5V: the safe  flight time is %02dh %02dm %02ds.\n", 
            (uint8_t)(g_flight_time.time/3600), 
            (uint8_t)((g_flight_time.time%3600)/60),
            (uint8_t)((g_flight_time.time%3600)%60));
#else
    printf("单节电芯电压 3.5V: 安全飞行时间 %02d小时 %02d分钟 %02d秒.\n", 
            (uint8_t)(g_flight_time.time/3600), 
            (uint8_t)((g_flight_time.time%3600)/60),
            (uint8_t)((g_flight_time.time%3600)%60));
#endif
    g_flight_time.time = (g_flight_time.time_end_3_0V > g_flight_time.time_start) ?
      (g_flight_time.time_end_3_0V - g_flight_time.time_start) :
      g_flight_time.time_end_3_0V + (RTC_S_MAX - g_flight_time.time_start);
#ifdef LANGUAGE_EN 
    printf("Cell voltage 3.0V: the whole flight time is %02dh %02dm %02ds.\n", 
            (uint8_t)(g_flight_time.time/3600), 
            (uint8_t)((g_flight_time.time%3600)/60),
            (uint8_t)((g_flight_time.time%3600)%60));
    printf("Good job (*^_^*)!\n");
#else
    printf("单节电芯电压 3.0V: 极限飞行时间 %02d小时 %02d分钟 %02d秒.\n", 
            (uint8_t)(g_flight_time.time/3600), 
            (uint8_t)((g_flight_time.time%3600)/60),
            (uint8_t)((g_flight_time.time%3600)%60));
    printf("好样的 (*^_^*)!\n");
#endif      
  } // if else  
  printf("\n");
  printf("<-");  

  PT_END(pt);
} // FLG_Thread()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
