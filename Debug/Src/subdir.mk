################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/debug.c \
../Src/dma.c \
../Src/gpio.c \
../Src/hx711.c \
../Src/i2c.c \
../Src/main.c \
../Src/mpu6050.c \
../Src/ppm.c \
../Src/protocol.c \
../Src/pwm.c \
../Src/retarget_io.c \
../Src/rpm.c \
../Src/rtc.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/stm_flash.c \
../Src/system_stm32f1xx.c \
../Src/terminal.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/adc.o \
./Src/debug.o \
./Src/dma.o \
./Src/gpio.o \
./Src/hx711.o \
./Src/i2c.o \
./Src/main.o \
./Src/mpu6050.o \
./Src/ppm.o \
./Src/protocol.o \
./Src/pwm.o \
./Src/retarget_io.o \
./Src/rpm.o \
./Src/rtc.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/stm_flash.o \
./Src/system_stm32f1xx.o \
./Src/terminal.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/adc.d \
./Src/debug.d \
./Src/dma.d \
./Src/gpio.d \
./Src/hx711.d \
./Src/i2c.d \
./Src/main.d \
./Src/mpu6050.d \
./Src/ppm.d \
./Src/protocol.d \
./Src/pwm.d \
./Src/retarget_io.d \
./Src/rpm.d \
./Src/rtc.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/stm_flash.d \
./Src/system_stm32f1xx.d \
./Src/terminal.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DARM_MATH_CM3 -D_DEFAULT_SOURCE -DSTM32F103xB -DUSE_HAL_DRIVER -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/libfixmath" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/motion_driver_5.1.3" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/pt-1.5" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/CMSIS/Include" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


