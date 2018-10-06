/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
#include <stdlib.h>
#include <string.h>

#include "fixmath.h"
#include "rtc.h"
#include "debug.h"
#include "protocol.h"
#include "ppm.h"
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    */
    GPIO_InitStruct.Pin = VOL_AD_Pin|CUR_AD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */
    /* Run the ADC calibration */
    if (HAL_ADCEx_Calibration_Start(adcHandle) != HAL_OK) {
      /* Calibration Error */
      Error_Handler();
    }
  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    */
    HAL_GPIO_DeInit(GPIOA, VOL_AD_Pin|CUR_AD_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
enum ENUM_AdcIndex {
  kAdcIndexVol = 0,
  kAdcIndexCur,
  kAdcCount
};

typedef struct {
  bool      error[kHx711Count];
  uint16_t  value[kAdcCount];
  int32_t   hx711[kHx711Count];
  float     start_offset[kHx711Count];
} adc_data_s;

enum {
  kCurDataSize = 128
};

typedef struct {
  bool      finished;
  uint8_t   index;
  uint16_t  adc_buf[kCurDataSize];
  float     value[kCurDataSize];
} cur_s;

static uint16_t s_adc1_buf[kAdcCount] = {0};
static adc_data_s s_ad = {0};
static cur_s      s_cur = {0};
adc_signal_s g_adc_signal = {0};

// ----------------------------------------------------------------------------
// Interruption Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : hadc - handler of adc
// Description  : Conversion complete callback in non blocking mode.
//  It was excuted by 146KHz.
// ----------------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (&hadc1 == hadc) {
    DEBUG_GpioToggle();
    // if (!s_cur.finished)
    {
      s_cur.adc_buf[s_cur.index++] = s_adc1_buf[kAdcIndexCur];
      if (s_cur.index >= kCurDataSize) {
        s_cur.index = 0;
        s_cur.finished = true;
      } // if
    } // if
  } // if
} // HAL_ADC_ConvCpltCallback()

// ----------------------------------------------------------------------------
// Initialization Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// void ADCx_Init(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Initialize ADC, use DMA.
// ----------------------------------------------------------------------------
void ADCx_Init(void) {
  MX_ADC1_Init();
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc1_buf,
                        (uint8_t)kAdcCount) != HAL_OK) {
    Error_Handler();
  } // if
  HX711_Init();
  // Initialize all variables.
  memset(&s_ad, 0, sizeof(s_ad));
  memset(s_adc1_buf, 0, sizeof(s_adc1_buf));
} // ADCx_Init()

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// int32_t Filter_EliminateJitter(uint8_t)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : index - adc index
// Description  : Eliminate jitter of ad value.
// ----------------------------------------------------------------------------
#define JITTER_TIMES 5

#if JITTER_TIMES
// recommended value:
// THRUST_NOISE =  100, JITTER_TIMES >= 10.
// THRUST_NOISE =  200, JITTER_TIMES >= 5.
// THRUST_NOISE =  0,   JITTER_TIMES >  100.
// THRUST_NOISE >= 500, JITTER_TIMES =  0.

static int32_t Filter_EliminateJitter(uint8_t index) {
  static uint16_t s_jitter_cnt[kHx711Count] = {0};
  static int32_t  s_ad_old[kHx711Count] = {0};

  if (s_ad.hx711[index] != s_ad_old[index]) {
    ++s_jitter_cnt[index];
    if (s_jitter_cnt[index] >= JITTER_TIMES) {
      s_jitter_cnt[index] = 0;
      s_ad_old[index] = s_ad.hx711[index];
    } // if
  } else {
    s_jitter_cnt[index] = 0;
  } // if else

  return s_ad_old[index];
} // Filter_EliminateJitter()
#endif

// ----------------------------------------------------------------------------
// void ADC_Thread(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Adc thread. Updates all used adc channels per 1s.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
// max vol = 35V, ad_vol = 35V/VOL_DIVIDER  = 3182mV.
// full range voltage = 3300mV, ad_value = 2^12 - 1 = 4095.
// When ad_vol_high = 3182mV, ad_value = (3182mV/3300mV)*4095 = 3949.
// When ad_value = 1,
// ad_vol = 3182mV/3949 = 0.806mV.
// Resistor divider.
#define VOL_RES1          47.0f // kΩ
#define VOL_RES2          4.7f  // kΩ
// ADC reference voltage.
#define VOL_REFER         3.3f  // V
#define VOL_REFER_FACTOR  (float)(VOL_REFER/4095.0f)
// ADC sampling divider of the battery.
#define VOL_DIVIDER       (float)((VOL_RES1 + VOL_RES2)/VOL_RES2)
//                        Vref (mV)
//  measurement (mV) = --------------- * res_divider * ad_value
//                     (2^12)-1 (bits)
#define VOL_FACTOR  (float)(VOL_REFER_FACTOR*VOL_DIVIDER) // 8.864mV
#define VOL_NOISE   4 // >= 4, 35.456mV

// curr_res = 2.5mΩ, curr_max = 57A, max curr_vol = curr_res*curr_max = 142.5mV.
// When ad_value = 1, curr = 57A/4095 = 13.919mA.
#define CUR_NOISE   4 // >= 4, 55.678mA
static const float kCurMin = 0.6f;
// Board #1,  1 - 10A, y = 0.0139x + 0.3985. AD = 691.  max error = 1.01%
// Board #1, 11 - 20A, y = 0.0139x + 0.4304. AD = 1412. max error = 0.35%
// Board #1, 21 - 30A, y = 0.0139x + 0.4151. AD = 2133. max error = 0.24%
// Board #1, 31 - 40A, y = 0.0138x + 0.5781. AD = 2857. max error = 0.06%
// Board #1, 41 - 50A, y = 0.0138x + 0.5248. AD = 3580. max error = 0.16%
// Board #1, 51 - 57A, y = 0.0139x + 0.1996. AD = 4095. max error = 0.41%
enum ENUM_CurIndex {
  kCurIndex10A = 0,
  kCurIndex20A,
  kCurIndex30A,
  kCurIndex40A,
  kCurIndex50A,
  kCurIndex60A,
  kCurIndexCount,
};
enum ENUM_CurFactorIndex {
  kCurFactorIndexA = 0,
  kCurFactorIndexB,
  kCurFactorIndexCount
};
const float cur_factor[kCurIndexCount][2] = {
  {0.0139, 0.3985},
  {0.0139, 0.4304},
  {0.0139, 0.4151},
  {0.0138, 0.5781},
  {0.0138, 0.5248},
  {0.0139, 0.1996},
};
const uint16_t cur_ad_boundary[kCurIndexCount] =
  {691, 1412, 2133, 2857, 3580, 4095};

// thrust sensor output 2mV/V, max output = 2mV/V*AVDD = 2*4.4 = 8.8mV.
// full range voltage = 0.5(AVDD/GAIN) = 0.5*(4400/128) = 17.1875mV,
// ad_value = 2^(24-1) - 1 = 8388607.
// When thrust sensor output 8.8mV,
// ad_value = (8.8mV/17.1875mV)*8388607 = 4294967.
// When ad_value = 1, thrust = 20kg/4294967 = 4.66mg.
#define THRUST_NOISE  200 // >= 200, 0.932g

#define TORQUE_FACTOR 0.0f  // fake
#define TORQUE_NOISE  200

#define HX711_ERR -1  // Erroneous data, what happened?

#define ADC_UPDATE_PERIOD 90 // Actually 10Hz here.

PT_THREAD(ADC_Thread(struct pt* pt)) {
  static STRUCT_RtcTimerMs s_adc_timer;

  PT_BEGIN(pt);

  // Update adc in a fixed period.
  RTC_SetMs(&s_adc_timer, ADC_UPDATE_PERIOD);
  // Implicitly return here.
  PT_WAIT_UNTIL(pt, RTC_ExpiredMs(&s_adc_timer));

  // Read all HX711 ad value, 10-80Hz.
  bool hx711_is_read_or_not = false;
  float thrust  = 0.0f;
  float torque  = 0.0f;
  for (uint8_t i = 0; i < (uint8_t)kHx711Count; ++i) {
    if (!HX711_IsFound(i)) continue;
    int32_t hx711_tmp;
    hx711_tmp = HX711_ReadAd(i, &hx711_is_read_or_not, AMP_RATE_A_128);
    if (!hx711_is_read_or_not) continue;
    // Error data, what happened?
    if (HX711_ERR == hx711_tmp) {
      s_ad.error[i] = true;
      continue;
    } else {
      s_ad.error[i] = false;
    } // if else
    // Schmitt trigger filter.
    if (abs(hx711_tmp - s_ad.hx711[i]) > THRUST_NOISE) {
      // Updating saved value.
      s_ad.hx711[i] = hx711_tmp;
    } // if
#if JITTER_TIMES
    if (g_pro.signal.cmd != kProCmdFlg)
    {
      // Eliminate jitter of ad value.
      s_ad.hx711[i] = Filter_EliminateJitter(i);
    } // if
#endif
    // Read start offset only once.
    if (g_adc_signal.read_start_offset[i]) {
      g_adc_signal.read_start_offset[i] = false;
      s_ad.start_offset[i] = (s_ad.hx711[i]-HX711_ReadOffset(i))*
        HX711_ReadCoefficient(i);
    } // if
    if ((uint8_t)HX711_INDEX_THRUST0 == i) {
      thrust = (s_ad.hx711[i]-HX711_ReadOffset(i))*HX711_ReadCoefficient(i);
      thrust -= s_ad.start_offset[i];
    } else if ((uint8_t)HX711_INDEX_THRUST1 == i) {
      // THRUST-1 sensor must be enabled manually!
      if (!g_pro.signal.en_thrust1) continue;
      thrust += (s_ad.hx711[i]-HX711_ReadOffset(i))*HX711_ReadCoefficient(i);
      thrust -= s_ad.start_offset[i];
    } else {
      torque = (s_ad.hx711[i]-HX711_ReadOffset(i))*HX711_ReadCoefficient(i)*
        TORQUE_FACTOR;
      torque -= s_ad.start_offset[i]*TORQUE_FACTOR;
    } // if else
  } // for
  // Read all inner ADC.
  // Get voltage adc conversion result.
  uint16_t ad_tmp = s_adc1_buf[kAdcIndexVol];
  // Schmitt trigger filter.
  if (abs((int)(ad_tmp - s_ad.value[kAdcIndexVol])) > VOL_NOISE) {
    // Updating saved value.
    s_ad.value[kAdcIndexVol] = ad_tmp;
  } // if
  float voltage = s_ad.value[kAdcIndexVol]*VOL_FACTOR;
#if 0
  // Get current adc conversion result.
  ad_tmp = s_adc1_buf[kAdcIndexCur];
  // Schmitt trigger filter.
  if (abs((int)(ad_tmp - s_ad.value[kAdcIndexCur])) > CUR_NOISE) {
    // Updating saved value.
    s_ad.value[kAdcIndexCur] = ad_tmp;
  } // if
  float current = 0.0f;
  if (s_ad.value[kAdcIndexCur] <= cur_ad_boundary[kCurIndex10A]) {
    current = cur_factor[kCurIndex10A][kCurFactorIndexA]*s_ad.value[kAdcIndexCur]
      + cur_factor[kCurIndex10A][kCurFactorIndexB];
  } else if (s_ad.value[kAdcIndexCur] <= cur_ad_boundary[kCurIndex20A]) {
    current = cur_factor[kCurIndex20A][kCurFactorIndexA]*s_ad.value[kAdcIndexCur]
      + cur_factor[kCurIndex20A][kCurFactorIndexB];
  } else if (s_ad.value[kAdcIndexCur] <= cur_ad_boundary[kCurIndex30A]) {
    current = cur_factor[kCurIndex30A][kCurFactorIndexA]*s_ad.value[kAdcIndexCur]
      + cur_factor[kCurIndex30A][kCurFactorIndexB];
  } else if (s_ad.value[kAdcIndexCur] <= cur_ad_boundary[kCurIndex40A]) {
    current = cur_factor[kCurIndex40A][kCurFactorIndexA]*s_ad.value[kAdcIndexCur]
      + cur_factor[kCurIndex40A][kCurFactorIndexB];
  } else if (s_ad.value[kAdcIndexCur] <= cur_ad_boundary[kCurIndex50A]) {
    current = cur_factor[kCurIndex50A][kCurFactorIndexA]*s_ad.value[kAdcIndexCur]
      + cur_factor[kCurIndex50A][kCurFactorIndexB];
  } else if (s_ad.value[kAdcIndexCur] <= cur_ad_boundary[kCurIndex60A]) {
    current = cur_factor[kCurIndex60A][kCurFactorIndexA]*s_ad.value[kAdcIndexCur]
      + cur_factor[kCurIndex60A][kCurFactorIndexB];
  } // else if
  if (current < kCurMin) current = 0.0f;
#else
//-----------------------------------------------------------------------------
// Calculate RMS of current. Execute time: 2.4ms.
//-----------------------------------------------------------------------------
  // Irms = sqrtf((I1^2 + I2^2 + ... +In^2)/n)
  float current = 0.0f;
  // if (s_cur.finished)
  {
    fix16_t fix_cur = 0;
    int64_t fix_sum = 0;
    // Disable update current's adc value when it is calculating RMS.
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    for (uint8_t i = 0; i < kCurDataSize; ++i) {
      if (s_cur.adc_buf[i] <= cur_ad_boundary[kCurIndex10A]) {
        current = cur_factor[kCurIndex10A][kCurFactorIndexA]*
          s_cur.adc_buf[i] + cur_factor[kCurIndex10A][kCurFactorIndexB];
      } else if (s_cur.adc_buf[i] <= cur_ad_boundary[kCurIndex20A]) {
        current = cur_factor[kCurIndex20A][kCurFactorIndexA]*
          s_cur.adc_buf[i] + cur_factor[kCurIndex20A][kCurFactorIndexB];
      } else if (s_cur.adc_buf[i] <= cur_ad_boundary[kCurIndex30A]) {
        current = cur_factor[kCurIndex30A][kCurFactorIndexA]*
          s_cur.adc_buf[i] + cur_factor[kCurIndex30A][kCurFactorIndexB];
      } else if (s_cur.adc_buf[i] <= cur_ad_boundary[kCurIndex40A]) {
        current = cur_factor[kCurIndex40A][kCurFactorIndexA]*
          s_cur.adc_buf[i] + cur_factor[kCurIndex40A][kCurFactorIndexB];
      } else if (s_cur.adc_buf[i] <= cur_ad_boundary[kCurIndex50A]) {
        current = cur_factor[kCurIndex50A][kCurFactorIndexA]*
          s_cur.adc_buf[i] + cur_factor[kCurIndex50A][kCurFactorIndexB];
      } else if (s_cur.adc_buf[i] <= cur_ad_boundary[kCurIndex60A]) {
        current = cur_factor[kCurIndex60A][kCurFactorIndexA]*
          s_cur.adc_buf[i] + cur_factor[kCurIndex60A][kCurFactorIndexB];
      } // else if
      fix_cur  = fix16_from_float(current);
      fix_sum += fix16_mul(fix_cur, fix_cur);
    } // for
    // Allow to read current's adc value again.
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    // s_cur.finished = false;
    fix_sum >>= 7;
    // current = fix16_to_float(fix16_sqrt(fix_sum));
    // f_cutoff = factor/(2*pi*dt), dt = 7us, sample rate.
    // When factor = 0.1, f_cutoff =  2.27KHz.
    // When factor = 0.9, f_cutoff = 20.46KHz.
    static const uint8_t kFactor = 0.1*256;
    static fix16_t s_old_fix_cur = 0;
    s_old_fix_cur = fix16_lerp8(s_old_fix_cur, fix16_sqrt(fix_sum), kFactor);
    current = fix16_to_float(s_old_fix_cur);
    if (current < kCurMin) current = 0.0f;
  } // if
#endif
  // Update protocol data.
  g_pro.data.voltage  = voltage;
  // Update protocol signal.
  g_pro.signal.updated[kProIndexVoltage]  = true;
  g_pro.data.current  = current;
  g_pro.signal.updated[kProIndexCurrent]  = true;
  g_pro.data.power    = g_pro.data.voltage*g_pro.data.current;
  g_pro.signal.updated[kProIndexPower]    = true;
  // if (!s_ad.error[HX711_INDEX_THRUST0])
  {
    g_pro.data.thrust   = (int16_t)(-thrust);
    g_pro.signal.updated[kProIndexThrust] = true;
  } // if
  if (!s_ad.error[HX711_INDEX_TORQUE]) {
    g_pro.data.torque   = torque;
    g_pro.signal.updated[kProIndexTorque] = true;
  } // if
#if (defined(DEBUG_ALL) || defined(DEBUG_ADC))
  // printf("%4d, %.1f\n", s_ad.value[kAdcIndexVol], g_pro.data.voltage);
  // printf("%8d\n", s_ad.value[kAdcIndexCur]);
  printf("%.2f\n", g_pro.data.current);
  // printf("%8d, %.2f\n", s_adc1_buf[kAdcIndexCur], g_pro.data.current);
  // printf("%8d, %8d\n", s_ad.hx711[HX711_INDEX_THRUST0], g_pro.data.thrust);
#endif
  if (!s_ad.error[HX711_INDEX_THRUST0]) {
    if (g_pro.data.power > 0)
      g_pro.data.thrust_efficiency_gw = g_pro.data.thrust/g_pro.data.power;
    else
      g_pro.data.thrust_efficiency_gw = 0;
    g_pro.signal.updated[kProIndexTegw] = true;
    if (g_pro.data.current > 0)
      g_pro.data.thrust_efficiency_ga = g_pro.data.thrust/g_pro.data.current;
    else
      g_pro.data.thrust_efficiency_ga = 0;
    g_pro.signal.updated[kProIndexTega] = true;
  } // if

  PT_END(pt);
} // ADC_Thread()

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
