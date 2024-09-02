################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/algorithms.c \
../Core/Src/bme280.c \
../Core/Src/bmi088.c \
../Core/Src/dataPacking.c \
../Core/Src/externalPins.c \
../Core/Src/lora.c \
../Core/Src/main.c \
../Core/Src/queternion.c \
../Core/Src/reset_detect.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/strain_gauge.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/usr_gnss_l86_parser.c 

OBJS += \
./Core/Src/algorithms.o \
./Core/Src/bme280.o \
./Core/Src/bmi088.o \
./Core/Src/dataPacking.o \
./Core/Src/externalPins.o \
./Core/Src/lora.o \
./Core/Src/main.o \
./Core/Src/queternion.o \
./Core/Src/reset_detect.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/strain_gauge.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/usr_gnss_l86_parser.o 

C_DEPS += \
./Core/Src/algorithms.d \
./Core/Src/bme280.d \
./Core/Src/bmi088.d \
./Core/Src/dataPacking.d \
./Core/Src/externalPins.d \
./Core/Src/lora.d \
./Core/Src/main.d \
./Core/Src/queternion.d \
./Core/Src/reset_detect.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/strain_gauge.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/usr_gnss_l86_parser.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/algorithms.d ./Core/Src/algorithms.o ./Core/Src/algorithms.su ./Core/Src/bme280.d ./Core/Src/bme280.o ./Core/Src/bme280.su ./Core/Src/bmi088.d ./Core/Src/bmi088.o ./Core/Src/bmi088.su ./Core/Src/dataPacking.d ./Core/Src/dataPacking.o ./Core/Src/dataPacking.su ./Core/Src/externalPins.d ./Core/Src/externalPins.o ./Core/Src/externalPins.su ./Core/Src/lora.d ./Core/Src/lora.o ./Core/Src/lora.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/queternion.d ./Core/Src/queternion.o ./Core/Src/queternion.su ./Core/Src/reset_detect.d ./Core/Src/reset_detect.o ./Core/Src/reset_detect.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/strain_gauge.d ./Core/Src/strain_gauge.o ./Core/Src/strain_gauge.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/usr_gnss_l86_parser.d ./Core/Src/usr_gnss_l86_parser.o ./Core/Src/usr_gnss_l86_parser.su

.PHONY: clean-Core-2f-Src

