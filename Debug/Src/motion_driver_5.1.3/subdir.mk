################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/motion_driver_5.1.3/inv_mpu.c \
../Src/motion_driver_5.1.3/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Src/motion_driver_5.1.3/inv_mpu.o \
./Src/motion_driver_5.1.3/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Src/motion_driver_5.1.3/inv_mpu.d \
./Src/motion_driver_5.1.3/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Src/motion_driver_5.1.3/%.o: ../Src/motion_driver_5.1.3/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DARM_MATH_CM3 -D_DEFAULT_SOURCE -DSTM32F103xB -DUSE_HAL_DRIVER -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/libfixmath" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/motion_driver_5.1.3" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/pt-1.5" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/CMSIS/Include" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


