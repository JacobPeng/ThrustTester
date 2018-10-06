################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/motion_driver_5.1.3/subdir.mk
-include Src/libfixmath/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/TransformFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/SupportFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/StatisticsFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/MatrixFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/FilteringFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/FastMathFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/ControllerFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/ComplexMathFunctions/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/CommonTables/subdir.mk
-include Drivers/CMSIS/DSP_Lib/Source/BasicMathFunctions/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
Thrust\ Tester.hex \

SECONDARY_SIZE += \
Thrust\ Tester.siz \


# All Target
all: Thrust\ Tester.elf secondary-outputs

# Tool invocations
Thrust\ Tester.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM Cross C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O3 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "/Users/Jacob/Documents/My Documents/Eclipse/workspace/Thrust Tester/STM32F103C8Tx_FLASH.ld" -Xlinker --gc-sections -Wl,-Map,"Thrust Tester.map" --specs=nano.specs -u _printf_float --specs=nosys.specs -o "Thrust Tester.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Thrust\ Tester.hex: Thrust\ Tester.elf
	@echo 'Invoking: GNU ARM Cross Create Flash Image'
	arm-none-eabi-objcopy -O ihex "Thrust Tester.elf"  "Thrust Tester.hex"
	@echo 'Finished building: $@'
	@echo ' '

Thrust\ Tester.siz: Thrust\ Tester.elf
	@echo 'Invoking: GNU ARM Cross Print Size'
	arm-none-eabi-size --format=berkeley "Thrust Tester.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) "Thrust Tester.elf"
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
