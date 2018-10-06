################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/libfixmath/fix16.c \
../Src/libfixmath/fix16_exp.c \
../Src/libfixmath/fix16_sqrt.c \
../Src/libfixmath/fix16_str.c \
../Src/libfixmath/fix16_trig.c \
../Src/libfixmath/fract32.c \
../Src/libfixmath/uint32.c 

OBJS += \
./Src/libfixmath/fix16.o \
./Src/libfixmath/fix16_exp.o \
./Src/libfixmath/fix16_sqrt.o \
./Src/libfixmath/fix16_str.o \
./Src/libfixmath/fix16_trig.o \
./Src/libfixmath/fract32.o \
./Src/libfixmath/uint32.o 

C_DEPS += \
./Src/libfixmath/fix16.d \
./Src/libfixmath/fix16_exp.d \
./Src/libfixmath/fix16_sqrt.d \
./Src/libfixmath/fix16_str.d \
./Src/libfixmath/fix16_trig.d \
./Src/libfixmath/fract32.d \
./Src/libfixmath/uint32.d 


# Each subdirectory must supply rules for building sources it contributes
Src/libfixmath/%.o: ../Src/libfixmath/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DARM_MATH_CM3 -D_DEFAULT_SOURCE -DSTM32F103xB -DUSE_HAL_DRIVER -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/libfixmath" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/motion_driver_5.1.3" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Inc/pt-1.5" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/CMSIS/Include" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


