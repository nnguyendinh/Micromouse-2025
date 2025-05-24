################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/controller.c \
../Core/Src/delay.c \
../Core/Src/encoders.c \
../Core/Src/gyro.c \
../Core/Src/main.c \
../Core/Src/motors.c \
../Core/Src/pid3.c \
../Core/Src/stm32f2xx_hal_msp.c \
../Core/Src/stm32f2xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f2xx.c \
../Core/Src/utility.c 

OBJS += \
./Core/Src/controller.o \
./Core/Src/delay.o \
./Core/Src/encoders.o \
./Core/Src/gyro.o \
./Core/Src/main.o \
./Core/Src/motors.o \
./Core/Src/pid3.o \
./Core/Src/stm32f2xx_hal_msp.o \
./Core/Src/stm32f2xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f2xx.o \
./Core/Src/utility.o 

C_DEPS += \
./Core/Src/controller.d \
./Core/Src/delay.d \
./Core/Src/encoders.d \
./Core/Src/gyro.d \
./Core/Src/main.d \
./Core/Src/motors.d \
./Core/Src/pid3.d \
./Core/Src/stm32f2xx_hal_msp.d \
./Core/Src/stm32f2xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f2xx.d \
./Core/Src/utility.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F205xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/controller.cyclo ./Core/Src/controller.d ./Core/Src/controller.o ./Core/Src/controller.su ./Core/Src/delay.cyclo ./Core/Src/delay.d ./Core/Src/delay.o ./Core/Src/delay.su ./Core/Src/encoders.cyclo ./Core/Src/encoders.d ./Core/Src/encoders.o ./Core/Src/encoders.su ./Core/Src/gyro.cyclo ./Core/Src/gyro.d ./Core/Src/gyro.o ./Core/Src/gyro.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motors.cyclo ./Core/Src/motors.d ./Core/Src/motors.o ./Core/Src/motors.su ./Core/Src/pid3.cyclo ./Core/Src/pid3.d ./Core/Src/pid3.o ./Core/Src/pid3.su ./Core/Src/stm32f2xx_hal_msp.cyclo ./Core/Src/stm32f2xx_hal_msp.d ./Core/Src/stm32f2xx_hal_msp.o ./Core/Src/stm32f2xx_hal_msp.su ./Core/Src/stm32f2xx_it.cyclo ./Core/Src/stm32f2xx_it.d ./Core/Src/stm32f2xx_it.o ./Core/Src/stm32f2xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f2xx.cyclo ./Core/Src/system_stm32f2xx.d ./Core/Src/system_stm32f2xx.o ./Core/Src/system_stm32f2xx.su ./Core/Src/utility.cyclo ./Core/Src/utility.d ./Core/Src/utility.o ./Core/Src/utility.su

.PHONY: clean-Core-2f-Src

