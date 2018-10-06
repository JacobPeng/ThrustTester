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
// Release B0.0.1 - 2018/05/03 - Jacob Peng
//  - Add: 2 more channels input.
// Release A0.0.1 - 2018/03/12 - Jacob Peng
//  - Add: Basic driver.
//

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "hx711.h"

#include "stm32f1xx_hal.h"

#include "rtc.h"
#include "protocol.h"
#include "stm_flash.h"
#include "debug.h"

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------
#define DEFAULT_COEFFICIENT 0.00466f 
#define DEFAULT_OFFSET      0

#define SAVE_PARA_SIZE  32
#define PARA_IS_SAVED   0x01

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
static uint8_t s_amp_cnt[kHx711Count] = {0};
float   g_coefficient[kHx711Count] = {0};
int32_t g_offset[kHx711Count] = {0};

struct STRUCT_Hx711 {
  float   coefficient;
  int32_t offset;
};

struct STRUCT_ParaInFlash {  
  bool   saved[kHx711Count];
  struct STRUCT_Hx711 hx711[kHx711Count];
};

union UNION_ParaInFlash {
  uint16_t data_u16[SAVE_PARA_SIZE];
  struct STRUCT_ParaInFlash data_st;
};

static union UNION_ParaInFlash s_para_in_flash = {.data_st = {0}};
static bool s_sensor_is_found[kHx711Count] = {0};

// ----------------------------------------------------------------------------
// Initialization Subroutines
// ----------------------------------------------------------------------------

static void HX711_SetParameter(void) {
  for (uint8_t i = 0; i < (uint8_t)kHx711Count; ++i) {
    if (PARA_IS_SAVED == s_para_in_flash.data_st.saved[i]) {
      HX711_SetCoefficient(i, s_para_in_flash.data_st.hx711[i].coefficient);
      HX711_SetOffset(i, s_para_in_flash.data_st.hx711[i].offset);     
    } else {
      HX711_SetCoefficient(i, DEFAULT_COEFFICIENT);
      HX711_SetOffset(i, DEFAULT_OFFSET);
    } // if else
  } // for 
} // HX711_SetParameter()

// ----------------------------------------------------------------------------
// void HX711_Init(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Initialize HX711 IC.
// ----------------------------------------------------------------------------
void HX711_Init(void) {
  STMFLASH_ReadBuffer(FLASH_USER_START_ADDR, 
                      s_para_in_flash.data_u16, 
                      SAVE_PARA_SIZE);  
  HX711_SetParameter();
  HAL_GPIO_WritePin(HX711_SCL1_GPIO_Port, HX711_SCL1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HX711_SCL2_GPIO_Port, HX711_SCL2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HX711_SCL3_GPIO_Port, HX711_SCL3_Pin, GPIO_PIN_RESET);

  // Determine sensors are plugged in or not.
#define RETRY_TIMES 5
  for (uint8_t i = 0; i < (uint8_t)kHx711Count; ++i) {
    for (uint8_t retry_cnt = 0; retry_cnt < RETRY_TIMES; ++retry_cnt) {    
      (void)HX711_ReadAd(i, &s_sensor_is_found[i], AMP_RATE_A_128);
      if (s_sensor_is_found[i]) break;
      HAL_Delay(100);
    } // for
  } // for
} // HX711_Init()

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------
bool HX711_IsFound(uint8_t index) {
  if (index >= kHx711Count) return false;

  return s_sensor_is_found[index];
} // HX711_IsFound()

static void HX711_PulseScl(uint8_t index) {
  switch (index) {
    case kHx711Index1:
      HAL_GPIO_WritePin(HX711_SCL1_GPIO_Port, HX711_SCL1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(HX711_SCL1_GPIO_Port, HX711_SCL1_Pin, GPIO_PIN_RESET);
      break;
    case kHx711Index2:
      HAL_GPIO_WritePin(HX711_SCL2_GPIO_Port, HX711_SCL2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(HX711_SCL2_GPIO_Port, HX711_SCL2_Pin, GPIO_PIN_RESET);
      break;
    case kHx711Index3:
      HAL_GPIO_WritePin(HX711_SCL3_GPIO_Port, HX711_SCL3_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(HX711_SCL3_GPIO_Port, HX711_SCL3_Pin, GPIO_PIN_RESET);
      break;      
    default:
      break;      
  } // switch()
} // HX711_Pulse()

static void HX711_SetAmp(uint8_t index, uint8_t amp_rate) {
  if (index >= (uint8_t)kHx711Count) return;

  switch (amp_rate) {
    case 32:
      s_amp_cnt[index] = 2;
      break;
    case 64:
      s_amp_cnt[index] = 3;
      break;
    case 128:
      s_amp_cnt[index] = 1;
      break;
    default:
      s_amp_cnt[index] = 1;
      break;
  } // switch()
} // HX711_SetAmp()

static bool HX711_IsReady(uint8_t index) {
  if (index >= (uint8_t)kHx711Count) return false;
  bool ready_or_not = false;

  switch (index) {
    case kHx711Index1:
      ready_or_not = 
      (HAL_GPIO_ReadPin(HX711_SDA1_GPIO_Port, HX711_SDA1_Pin) == GPIO_PIN_RESET);
      break;
    case kHx711Index2:
      ready_or_not = 
      (HAL_GPIO_ReadPin(HX711_SDA2_GPIO_Port, HX711_SDA2_Pin) == GPIO_PIN_RESET);
      break;
    case kHx711Index3:
      ready_or_not = 
      (HAL_GPIO_ReadPin(HX711_SDA3_GPIO_Port, HX711_SDA3_Pin) == GPIO_PIN_RESET);
      break;      
    default:
      break;      
  } // switch()  

  return ready_or_not;
} // HX711_IsReady()

void HX711_SetCoefficient(uint8_t index, float coefficient) {
  if (index >= (uint8_t)kHx711Count) return;

  g_coefficient[index] = coefficient;
} // HX711_SetCoefficient()

void HX711_SetOffset(uint8_t index, int32_t offset) {
  if (index >= (uint8_t)kHx711Count) return;

  g_offset[index] = offset;
} // HX711_SetOffset()

float HX711_ReadCoefficient(uint8_t index) {
  if (index >= (uint8_t)kHx711Count) return 0.0f;

  return g_coefficient[index];
} // HX711_ReadCoefficient()

int32_t HX711_ReadOffset(uint8_t index) {
  if (index >= (uint8_t)kHx711Count) return 0;

  return g_offset[index];
} // HX711_ReadOffset()

// ----------------------------------------------------------------------------
// int32_t HX711_ReadAd(uint8_t, bool*)
// ----------------------------------------------------------------------------
// Return Value : hx711 ad value, 24-bit valid.
// Parameters   : 
//  index         - index of hx711
//  ready_or_not  - status of hx711
//  amp_rate      - amp rate of hx711
// Description  : Read HX711 ad value, max 80Hz.
// ----------------------------------------------------------------------------
int32_t HX711_ReadAd(uint8_t index, bool* ready_or_not, uint8_t amp_rate) {
  if (index >= (uint8_t)kHx711Count) {
    *ready_or_not = false;
    return 0;
  } // if
  int32_t result = 0;
  STRUCT_RtcTimerMs hx711_timer;

  // While hx711 is busy.
  RTC_SetMs(&hx711_timer, 10);
  while (!HX711_IsReady(index)) {
    if (RTC_ExpiredMs(&hx711_timer)) {
      // Error_Handler();
      *ready_or_not = false;
      return 0;
    } // if
  } // while
  // Read 24-bit ad value.
  for (uint8_t i = 0; i < 24; ++i) {
    HX711_PulseScl(index);
    result = (int32_t)((uint32_t)result << 1);
    switch (index) {
      case kHx711Index1:
        if (HAL_GPIO_ReadPin(HX711_SDA1_GPIO_Port, HX711_SDA1_Pin)) ++result;
        break;
      case kHx711Index2:
        if (HAL_GPIO_ReadPin(HX711_SDA2_GPIO_Port, HX711_SDA2_Pin)) ++result;
        break;
      case kHx711Index3:
        if (HAL_GPIO_ReadPin(HX711_SDA3_GPIO_Port, HX711_SDA3_Pin)) ++result;
        break;        
      default:
        break;
    } // switch()
  } // for
  HX711_SetAmp(index, amp_rate);
  for (uint8_t i = 0; i < s_amp_cnt[index]; ++i) {
    HX711_PulseScl(index);
  } // for

  *ready_or_not = true;
  return (result & (1L << 23)) ? (result | (-1L << 24)) : result;
} // HX711_ReadAd()

// ----------------------------------------------------------------------------
// bool calibrate_thrust_sensor(uint8_t, uint16_t)
// ----------------------------------------------------------------------------
// Return Value : 0, failed; 1, successfully.
// Parameters   : 
//  hx711_index - index of hx711
//  weight      - weight of counterbalance
// Description  : Calibrate HX711, calculate coefficient and offset.
// ----------------------------------------------------------------------------
// HX711_ad = thrust/coefficient + offset.
// thrust = (HX711_ad - offset)*coefficient.
// When thrust = 0g, offset = HX711_ad0.
// When thrust = 20000g, coefficient = 20000.0f/(HX711_ad1 - offset).
bool calibrate_thrust_sensor(uint8_t hx711_index, uint16_t weight) {
  if (hx711_index >= kHx711Count) return false;
  if (!HX711_IsFound(hx711_index)) {
#ifdef LANGUAGE_EN     
    printf("Sorry, thrust sensor %d was not found.\n", hx711_index);
#else
    printf("抱歉, 推力传感器 %d 未找到.\n", hx711_index);
#endif
    return false;
  } // if
  
  printf("\n");
#ifdef LANGUAGE_EN   
  printf("First, put nothing on thrust sensor, get offset.\n");
  printf("Please do it within 10 seconds.\n");    
#else
  printf("第一步, 移除推力传感器上的砝码, 获取偏移量.\n");
  printf("请在10s内完成.\n");    
#endif  
  for (uint8_t i = 10; i > 0; i -= 2) {
    printf("%2d s\n", i);
    HAL_Delay(2000);
  } // for
#ifdef LANGUAGE_EN   
  printf("Start to get offset.\n");
  printf("This will be done within 1 seconds.\n");
#else
  printf("开始读取偏移量.\n");
  printf("将在1s内完成.\n");
#endif
  bool hx711_is_read_or_not = false;
  int32_t ad_tmp = 0;
  int32_t offset = 0;
  float  coefficient = 0.0f;
  uint8_t retry_cnt;
  for (retry_cnt = 0; retry_cnt < RETRY_TIMES; ++retry_cnt) {    
    ad_tmp = HX711_ReadAd(hx711_index, &hx711_is_read_or_not, AMP_RATE_A_128);
    if (hx711_is_read_or_not) break;
  } // for
  if (RETRY_TIMES == retry_cnt) {
#ifdef LANGUAGE_EN      
    printf("Failed to read sensor %d.\n", hx711_index);
    printf("Please make sure thrust sensors are connected reliably!\n");
#else
    printf("读取传感器 %d 失败.\n", hx711_index);
    printf("请确保传感器可靠连接!\n");
#endif
    return false;
  } else {
    offset = ad_tmp;
    s_para_in_flash.data_st.hx711[hx711_index].offset = offset;
#ifdef LANGUAGE_EN 
    printf("Succeed to get offset!\n");
#else
    printf("成功获取偏移量!\n");
#endif
  } // if else

  printf("\n");
#ifdef LANGUAGE_EN 
  printf("Second, put weight (such as 2kg) on thrust sensor, get coefficient.\n");
  printf("Please do it within 30 seconds.\n");
#else
  printf("第二步, 在传感器上放置砝码 (比如2kg), 获取系数.\n");
  printf("请在30s内完成.\n");  
#endif  
  for (uint8_t i = 30; i > 0; i -= 5) {
    printf("%2d s\n", i);
    HAL_Delay(5000);
  } // for
#ifdef LANGUAGE_EN   
  printf("Start to get coefficient.\n");
  printf("This will be done within 1 seconds.\n");
#else
  printf("开始读取系数.\n");
  printf("将在1s内完成.\n"); 
#endif
  for (retry_cnt = 0; retry_cnt < RETRY_TIMES; ++retry_cnt) {    
    ad_tmp = HX711_ReadAd(hx711_index, &hx711_is_read_or_not, AMP_RATE_A_128);
    if (hx711_is_read_or_not) break;
    HAL_Delay(100);
  } // for
  if (RETRY_TIMES == retry_cnt) {
#ifdef LANGUAGE_EN
    printf("Failed to read sensor %d.\n", hx711_index);
    printf("Please make sure thrust sensors are connected reliably!\n");
#else
    printf("读取传感器 %d 失败.\n", hx711_index);
    printf("请确保传感器可靠连接!\n");
#endif
    return false;
  } else {
    coefficient = 
      (weight*1.0f)/(ad_tmp - s_para_in_flash.data_st.hx711[hx711_index].offset);
    s_para_in_flash.data_st.hx711[hx711_index].coefficient = coefficient;
#ifdef LANGUAGE_EN 
    printf("Succeed to get coefficient!\n");
#else
    printf("成功获取系数!\n");
#endif
  } // if else

  printf("\n");
  printf("coefficient = %f\n", 
          s_para_in_flash.data_st.hx711[hx711_index].coefficient);
  printf("offset      = %d\n", 
          s_para_in_flash.data_st.hx711[hx711_index].offset);
  s_para_in_flash.data_st.saved[hx711_index] = PARA_IS_SAVED;
  STMFLASH_WriteBuffer(FLASH_USER_START_ADDR, 
                      s_para_in_flash.data_u16, 
                      SAVE_PARA_SIZE);
#ifdef LANGUAGE_EN 
  printf("Parameters were saved in flash.\n\n");
  printf("Read parameters from flash, make sure they are the same.\n");
#else
  printf("参数已经保存在flash中.\n\n");
  printf("从flash中读取参数, 确保flash中的参数和需要保存的参数一致.\n");  
#endif  
  
  STMFLASH_ReadBuffer(FLASH_USER_START_ADDR, 
                      s_para_in_flash.data_u16, 
                      SAVE_PARA_SIZE);
  printf("coefficient = %f\n", 
          s_para_in_flash.data_st.hx711[hx711_index].coefficient);
  printf("offset      = %d\n", 
          s_para_in_flash.data_st.hx711[hx711_index].offset);  
  if (coefficient != s_para_in_flash.data_st.hx711[hx711_index].coefficient ||
      offset      != s_para_in_flash.data_st.hx711[hx711_index].offset) {
#ifdef LANGUAGE_EN 
    printf("Failed to save parameters!\n");
#else
    printf("参数保存失败!\n");
#endif
    return false;
  } else {
#ifdef LANGUAGE_EN 
    printf("Succeed to save parameters.\n");
    printf("You need not to restart MCU.\n");
    printf("Good job (*^_^*)!\n");
#else
    printf("成功保存参数.\n");
    printf("不必重启MCU.\n");
    printf("好样的 (*^_^*)!\n");
#endif

    HX711_SetParameter();
  } // if else

  return true;
} // calibrate_thrust_sensor()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
