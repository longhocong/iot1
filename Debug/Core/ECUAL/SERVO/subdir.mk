################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/ECUAL/SERVO/SERVO.c \
../Core/ECUAL/SERVO/SERVO_cfg.c 

OBJS += \
./Core/ECUAL/SERVO/SERVO.o \
./Core/ECUAL/SERVO/SERVO_cfg.o 

C_DEPS += \
./Core/ECUAL/SERVO/SERVO.d \
./Core/ECUAL/SERVO/SERVO_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/ECUAL/SERVO/%.o Core/ECUAL/SERVO/%.su: ../Core/ECUAL/SERVO/%.c Core/ECUAL/SERVO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-ECUAL-2f-SERVO

clean-Core-2f-ECUAL-2f-SERVO:
	-$(RM) ./Core/ECUAL/SERVO/SERVO.d ./Core/ECUAL/SERVO/SERVO.o ./Core/ECUAL/SERVO/SERVO.su ./Core/ECUAL/SERVO/SERVO_cfg.d ./Core/ECUAL/SERVO/SERVO_cfg.o ./Core/ECUAL/SERVO/SERVO_cfg.su

.PHONY: clean-Core-2f-ECUAL-2f-SERVO

