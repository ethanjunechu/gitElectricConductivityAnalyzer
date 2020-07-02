################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/AD5933.c \
../Src/adc.c \
../Src/bsp_rtc.c \
../Src/bsp_spi_flash.c \
../Src/dac.c \
../Src/delay.c \
../Src/dma.c \
../Src/gpio.c \
../Src/main.c \
../Src/rtc.c \
../Src/spi.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f1xx.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/AD5933.o \
./Src/adc.o \
./Src/bsp_rtc.o \
./Src/bsp_spi_flash.o \
./Src/dac.o \
./Src/delay.o \
./Src/dma.o \
./Src/gpio.o \
./Src/main.o \
./Src/rtc.o \
./Src/spi.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f1xx.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/AD5933.d \
./Src/adc.d \
./Src/bsp_rtc.d \
./Src/bsp_spi_flash.d \
./Src/dac.d \
./Src/delay.d \
./Src/dma.d \
./Src/gpio.d \
./Src/main.d \
./Src/rtc.d \
./Src/spi.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f1xx.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F103xE -I"/Users/ethanchu/Desktop/5000-01/Inc" -I"/Users/ethanchu/Desktop/5000-01/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/Users/ethanchu/Desktop/5000-01/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/Users/ethanchu/Desktop/5000-01/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/Users/ethanchu/Desktop/5000-01/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


