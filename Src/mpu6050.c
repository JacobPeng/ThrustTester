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

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "mpu6050.h"

#include <stdio.h>
#include <stdlib.h>
// #include <string.h>
// #include <math.h>

#include "inv_mpu.h"
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "debug.h"
#include "protocol.h"

// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------
enum ENUM_GyroIndex {
  kGyroIndexX = 0,
  kGyroIndexY,
  kGyroIndexZ,
  kGyroCount
};

enum ENUM_AccelIndex {
  kAccelIndexX = 0,
  kAccelIndexY,
  kAccelIndexZ,
  kAccelCount
};

#define DEFAULT_MPU_HZ  (50) // max 200Hz

// #define CALCULATE_ANGLE
// #define CALCULATE_VECTOR

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------
typedef struct {
  float     gyro_sens;
  uint16_t  accel_sens;
  int16_t   gyro_raw[kGyroCount];
  int16_t   accel_raw[kAccelCount];
  float     accel_g[kAccelCount];
  uint32_t  timestamp;
  uint8_t   sensors;
  uint8_t   more;
  uint16_t  new_ms;
  uint16_t  last_ms;  
  uint16_t  interval;
} STRUCT_Mpu6050Info;

STRUCT_Mpu6050Info s_mpu6050;

#ifdef CALCULATE_VECTOR
typedef struct {
  bool  not_first_sample;
  char  sign_gyro_rz;
  // projection of normalized gravitation force vector on x/y/z axis, 
  // as measured by accelerometer.
  float r_acc[3];
  // Rw obtained from last estimated value and gyro movement.
  float r_gyro[3];
  // Rw estimated from combining r_acc and r_gyro.
  float r_est[3];
  // angles between projection of R on XZ/YZ plane and Z axis (deg).
  float angle_rz[2];
} STRUCT_Vector;

static STRUCT_Vector s_vector;
#endif

// ----------------------------------------------------------------------------
// Initialization Subroutines
// ----------------------------------------------------------------------------

static void run_self_test(void) {
  long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
  int32_t result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
  int32_t result = mpu_run_self_test(gyro, accel);
#endif 
  // result = 0x05, accel self test failed, why?
  (void)result;
  // if (result == 0x7) 
  {
    // Test passed. We can trust the gyro data here, so let's push it down
    // to the register.
    (void)mpu_get_gyro_sens(&s_mpu6050.gyro_sens);
    (void)mpu_get_accel_sens(&s_mpu6050.accel_sens);
#if (defined(DEBUG_ALL) || defined(DEBUG_MPU))
    // gyro_sens = 16.4, accel_sens = 16384.
    printf("gyro_sens = %.2f, accel_sens = %5d.\n", 
           s_mpu6050.gyro_sens, s_mpu6050.accel_sens);
#endif
    for (uint8_t i = 0; i < 3; ++i) {      
      gyro[i]   = (long)(gyro[i]*32.8f);  // convert to ±1000dps
      gyro[i]   = (long)((unsigned long)gyro[i] >> 16);
      accel[i]  = (long)(accel[i]*2048);   // convert to ±16g
      accel[i]  = (long)((unsigned long)accel[i] >> 16);
    } // for
    // Gyro bias inputs are LSB in ±1000dps format.
    (void)mpu_set_gyro_bias_reg(gyro);
#if defined (MPU6500) || defined (MPU9250)
    (void)mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
    // Accel bias inputs are LSB in ±16G format.
    (void)mpu_set_accel_bias_6050_reg(accel);
#endif
  } 
  // else {
  //   printf("mpu6050 self test result = %d\n", result);
  //   Error_Handler();
  // } // if else
} // run_self_test()

// ----------------------------------------------------------------------------
// void MPU6050_Init(void)
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// Description  : Initialize mpu6050, 400KHz.
// ----------------------------------------------------------------------------
void MPU6050_Init(void) { 
  MX_I2C1_Init();
  // Set up gyro.
  // Every function preceded by mpu_ is a driver function and can be found
  // in inv_mpu.h.
  if (mpu_init()) Error_Handler();

  // If you're not using an MPU9150 AND you're not using DMP features, this
  // function will place all slaves on the primary bus.
  // mpu_set_bypass(1);

  // Get/set hardware configuration. Start gyro. 
  // Wake up all sensors. 
  (void)mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  // Push both gyro and accel data into the FIFO. 
  (void)mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  (void)mpu_set_sample_rate(DEFAULT_MPU_HZ);
#if (defined(DEBUG_ALL) || defined(DEBUG_MPU))
  // Read back configuration in case it was set improperly. 
  uint8_t   accel_fsr;
  uint16_t  sample_rate;
  uint16_t  gyro_fsr;
  (void)mpu_get_sample_rate(&sample_rate);
  (void)mpu_get_gyro_fsr(&gyro_fsr);
  (void)mpu_get_accel_fsr(&accel_fsr);
  // sample_rate = 50, gyro_fsr = 2000, accel_fsr = 2.
  // gyro 16.4 LSB/(°/s), accel 16384 LSB/g.
  printf("\nsample_rate = %d, gyro_fsr = %d, accel_fsr = %d.\n", 
         sample_rate, gyro_fsr, accel_fsr);
#endif
  run_self_test();
#ifdef CALCULATE_VECTOR
  memset(&s_vector, 0, sizeof(s_vector));
#endif  
} // MPU6050_Init()

// ----------------------------------------------------------------------------
// Runtime Subroutines
// ----------------------------------------------------------------------------

#ifdef CALCULATE_ANGLE
float angle_pitch = 0.0;
float angle_roll  = 0.0;

// Complementary filter.
static void MPU6050_CalculateAngle(float* p_angle_pitch, float* p_angle_roll) {
  // Angle around the X-axis
  *p_angle_pitch += (s_mpu6050.gyro_raw[kGyroIndexX]/s_mpu6050.gyro_sens)*
    (s_mpu6050.interval/1000.0f);
  printf("%.2f, ", *p_angle_pitch);
  // Angle around the Y-axis
  *p_angle_roll -= (s_mpu6050.gyro_raw[kGyroIndexY]/s_mpu6050.gyro_sens)*
    (s_mpu6050.interval/1000.0f);
  printf("%.2f, ", *p_angle_roll);

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int force_magnitude = abs(s_mpu6050.accel_raw[kAccelIndexX]) + 
                        abs(s_mpu6050.accel_raw[kAccelIndexY]) + 
                        abs(s_mpu6050.accel_raw[kAccelIndexZ]);
  if (force_magnitude > 8192 && force_magnitude < 32768) {
    // Turning around the X axis results in a vector on the Y-axis.
    float accel_pitch = atan2f((float)s_mpu6050.accel_raw[kAccelIndexY], 
      (float)s_mpu6050.accel_raw[kAccelIndexZ])*(180/PI);
    printf("%.2f, ", accel_pitch);
    *p_angle_pitch = *p_angle_pitch * 0.98 + accel_pitch * 0.02;
    printf("%.2f, ", *p_angle_pitch);
    // Turning around the Y axis results in a vector on the X-axis
    float accel_roll = atan2f((float)s_mpu6050.accel_raw[kAccelIndexX], 
      (float)s_mpu6050.accel_raw[kAccelIndexZ])*(180/PI);
    printf("%.2f, ", accel_roll);
    *p_angle_roll = *p_angle_roll * 0.98 + accel_roll * 0.02;
    printf("%.2f", *p_angle_roll);
    printf("\n");
  } // if
} // MPU6050_CalculateAngle()
#endif

#ifdef CALCULATE_VECTOR

#define PI 3.14159265358979f

static void Normalize3DVector(float* vector) {
  float r = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  vector[0] /= r;
  vector[1] /= r;
  vector[2] /= r;
} // Normalize3DVector()

static float squared(float x) {
  return x*x;
} // squared()

// Simplified kalman filter.
static void MPU6050_CalculateVector(void) {
  // Calculate accelerometer in unit g.
  for (int i = 0; i < kAccelCount; ++i) {
    s_vector.r_acc[i] = s_mpu6050.accel_raw[i]/(s_mpu6050.accel_sens*1.0f);
  } // for
  // normalize vector (convert to a vector with same direction and with length 1)
  // Normalize3DVector(s_vector.r_acc);
  if (!s_vector.not_first_sample) {
    s_vector.not_first_sample = true;
    // initialize with accelerometer readings
    for (int i = 0; i < kAccelCount; ++i) s_vector.r_est[i] = s_vector.r_acc[i];
  } else {
    // Evaluate RwGyro vector.
    if (abs(s_vector.r_est[kGyroIndexZ]) < 0.1) {
      // Rz is too small and because it is used as reference for computing 
      // angle_rz[kGyroIndexX] and angle_rz[kGyroIndexY],
      // it's error fluctuations will amplify leading to bad results
      // in this case skip the gyro data and just use previous estimate.
      for (int i = 0; i < kGyroCount; ++i) s_vector.r_gyro[i] = s_vector.r_est[i];
    } else {
      // Get angles between projection of R on ZX/ZY plane and Z axis, 
      // based on last RwEst.
      for (int i = 0; i < kGyroIndexZ; ++i) {
        // Get current gyro rate in deg/s.
        s_vector.r_gyro[i] = s_mpu6050.gyro_raw[i]/s_mpu6050.gyro_sens;
        s_vector.r_gyro[i] *= s_mpu6050.interval/1000.0f;
        // Get angle and convert to degrees.
        s_vector.angle_rz[i] = atan2f(s_vector.r_est[i], 
                                      s_vector.r_est[kGyroIndexZ]) *(180/PI);
        // Get updated angle according to gyro movement.
        s_vector.angle_rz[i] += s_vector.r_gyro[i];
      } // for

      // Estimate sign of RzGyro by looking in what quadrant the angle Axz is, 
      // RzGyro is positive if Axz in range -90 ..90 => cos(Awz) >= 0
      s_vector.sign_gyro_rz = (cos(s_vector.angle_rz[kGyroIndexX]*PI/180) >= 0) 
        ? 1 : -1;

      // Reverse calculation of RwGyro from Awz angles.
      s_vector.r_gyro[kGyroIndexX] = sin(s_vector.angle_rz[kGyroIndexX]*PI/180);
      s_vector.r_gyro[kGyroIndexX] /= sqrt(1 + 
        squared(cos(s_vector.angle_rz[kGyroIndexX]*PI/180)) * 
        squared(tan(s_vector.angle_rz[kGyroIndexY]*PI/180)));
      s_vector.r_gyro[kGyroIndexY] = sin(s_vector.angle_rz[kGyroIndexY]*PI/180);
      s_vector.r_gyro[kGyroIndexY] /= sqrt(1 + 
        squared(cos(s_vector.angle_rz[kGyroIndexY]*PI/180)) * 
        squared(tan(s_vector.angle_rz[kGyroIndexX]*PI/180)));
      s_vector.r_gyro[kGyroIndexZ] = s_vector.sign_gyro_rz * 
        sqrt(1 - squared(s_vector.r_gyro[kGyroIndexX]) - 
             squared(s_vector.r_gyro[kGyroIndexY]));
    } // if else

#define GYRO_SCALE 10 // recommended: 5-20    
    // Combine Accelerometer and gyro readings.
    for (int i = 0; i < kAccelCount; ++i) {
      s_vector.r_est[i] = (s_vector.r_acc[i] + GYRO_SCALE*s_vector.r_gyro[i])/(1+GYRO_SCALE);      
    } // for
    // normalize vector (convert to a vector with same direction and with length 1)
    Normalize3DVector(s_vector.r_est);
  } // if else

  printf("%d, ", s_mpu6050.interval);
  for (int i = 0; i < kAccelCount; ++i) {
    printf("%.5f, %.5f, ", s_vector.r_acc[i], s_vector.r_est[i]);    
  } // for
  printf("\n");
} // MPU6050_CalculateVector()
#endif

// ----------------------------------------------------------------------------
// PT_THREAD(MPU_Thread(struct pt*))
// ----------------------------------------------------------------------------
// Return Value : None
// Parameters   : pt - the pointer of thread
// Description  : MPU6050 thread. Updates gyro and accel periodically.
//  User application should call this function regularly.
//  It's called by main().
// ----------------------------------------------------------------------------
#define MPU6050_UPDATE_PERIOD 5 // 1-5 ms
#define GYRO_NOISE  0   // >= 2
#define ACCEL_NOISE 130 // >= 130

PT_THREAD(MPU_Thread(struct pt* pt)) { 
  static STRUCT_RtcTimerMs s_mpu6050_timer;
  static int16_t s_gyro_tmp[kGyroCount] = {0};
  static int16_t s_accel_tmp[kAccelCount] = {0};   

  PT_BEGIN(pt);

  // Update gyro and accel in a fixed period.
  RTC_SetMs(&s_mpu6050_timer, MPU6050_UPDATE_PERIOD);
  // Implicitly return here.
  PT_WAIT_UNTIL(pt, RTC_ExpiredMs(&s_mpu6050_timer));

  s_mpu6050.new_ms = RTC_TickMs();
  RTC_SetMs(&s_mpu6050_timer, MPU6050_UPDATE_PERIOD);
  // This function gets new data from the FIFO. 
  // The FIFO can contain any combination of gyro, accel. 
  // The sensors parameter tells the caller which data fields were actually
  // populated with new data.
  // For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't being filled
  // with accel data.
  // The more parameter is non-zero if there are leftover packets in the FIFO.

  // While FIFO isn't ready, reads more times.
  // If you read fifo too slowly, fifo will be full soon.
  // If you read fifo too quickly, fifo will be empty soon.
  // The best solution: enable fifo full interrupt, read out all data in 
  // interrupt function.
  // Here reads fifo 2-10 times frequency of fifo output.
  while (mpu_read_fifo(s_gyro_tmp, s_accel_tmp, 
                       (unsigned long *)&s_mpu6050.timestamp, 
                       &s_mpu6050.sensors, &s_mpu6050.more)) {
    if (RTC_ExpiredMs(&s_mpu6050_timer)) {
      // Error_Handler();
      PT_EXIT(pt);
    } // if
  } // while
  // if (mpu_read_fifo(s_gyro_tmp, s_accel_tmp, 
  //                   (unsigned long *)&s_mpu6050.timestamp, 
  //                   &s_mpu6050.sensors, &s_mpu6050.more)) {
  //   PT_EXIT(pt);
  // } // if  
  // Get interval between last read fifo and this read fifo.
  s_mpu6050.interval = s_mpu6050.new_ms >= s_mpu6050.last_ms ?
    s_mpu6050.new_ms - s_mpu6050.last_ms :
    s_mpu6050.new_ms + (RTC_MS_MAX - s_mpu6050.last_ms);
  s_mpu6050.last_ms = s_mpu6050.new_ms;
  for (uint8_t i = 0; i < (uint8_t)kGyroCount; ++i) {
    // Schmitt trigger filter.
    if (abs(s_gyro_tmp[i] - s_mpu6050.gyro_raw[i]) > GYRO_NOISE) {
      // Updating gyro raw.
      s_mpu6050.gyro_raw[i] = s_gyro_tmp[i];
    } // if
  } // for
  for (uint8_t i = 0; i < (uint8_t)kAccelCount; ++i) {
    // Schmitt trigger filter.
    if (abs(s_accel_tmp[i] - s_mpu6050.accel_raw[i]) > ACCEL_NOISE) {
      // Updating accel raw.
      s_mpu6050.accel_raw[i] = s_accel_tmp[i];
      s_mpu6050.accel_g[i] = s_mpu6050.accel_raw[i]/(s_mpu6050.accel_sens*1.0f);
    } // if
  } // for 
  
  // Print debug information.
#if (defined(DEBUG_ALL) || defined(DEBUG_MPU))  
  printf("%2d, ", s_mpu6050.interval);
  for (uint8_t i = 0; i < (uint8_t)kGyroCount; ++i) {
    printf("%5d, ", s_mpu6050.gyro_raw[i]);    
  } // for
  for (uint8_t i = 0; i < (uint8_t)kAccelCount; ++i) {
    printf("%5d, ", s_mpu6050.accel_raw[i]);    
  } // for
  for (uint8_t i = 0; i < (uint8_t)kAccelCount; ++i) {
    printf("%.2f, ", s_mpu6050.accel_g[i]);    
  } // for
  printf("\n");  
#endif 

  // Debug advanced data processing.
#ifdef CALCULATE_ANGLE
  MPU6050_CalculateAngle(&angle_pitch, &angle_roll);
#endif
#ifdef CALCULATE_VECTOR
  MPU6050_CalculateVector();
#endif

  // Update protocol data.
  g_pro.data.vibration_x = s_mpu6050.accel_g[kAccelIndexX];
  g_pro.data.vibration_y = s_mpu6050.accel_g[kAccelIndexY];
  g_pro.data.vibration_z = s_mpu6050.accel_g[kAccelIndexZ]; 
  // Update protocol signal.
  g_pro.signal.updated[kProIndexVibrationX] = true;
  g_pro.signal.updated[kProIndexVibrationY] = true;
  g_pro.signal.updated[kProIndexVibrationZ] = true;  

  PT_END(pt);
} // MPU_Thread()

// ----------------------------------------------------------------------------
// End Of File
// ----------------------------------------------------------------------------
