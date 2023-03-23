################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/cBot/button.c \
../Core/Src/cBot/buzzer.c \
../Core/Src/cBot/cBot.c \
../Core/Src/cBot/fifo.c \
../Core/Src/cBot/marioSong.c \
../Core/Src/cBot/sercom.c \
../Core/Src/cBot/ws2812b.c 

OBJS += \
./Core/Src/cBot/button.o \
./Core/Src/cBot/buzzer.o \
./Core/Src/cBot/cBot.o \
./Core/Src/cBot/fifo.o \
./Core/Src/cBot/marioSong.o \
./Core/Src/cBot/sercom.o \
./Core/Src/cBot/ws2812b.o 

C_DEPS += \
./Core/Src/cBot/button.d \
./Core/Src/cBot/buzzer.d \
./Core/Src/cBot/cBot.d \
./Core/Src/cBot/fifo.d \
./Core/Src/cBot/marioSong.d \
./Core/Src/cBot/sercom.d \
./Core/Src/cBot/ws2812b.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/cBot/%.o Core/Src/cBot/%.su: ../Core/Src/cBot/%.c Core/Src/cBot/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc/cBot -I../Core/Inc/u8g2 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-cBot

clean-Core-2f-Src-2f-cBot:
	-$(RM) ./Core/Src/cBot/button.d ./Core/Src/cBot/button.o ./Core/Src/cBot/button.su ./Core/Src/cBot/buzzer.d ./Core/Src/cBot/buzzer.o ./Core/Src/cBot/buzzer.su ./Core/Src/cBot/cBot.d ./Core/Src/cBot/cBot.o ./Core/Src/cBot/cBot.su ./Core/Src/cBot/fifo.d ./Core/Src/cBot/fifo.o ./Core/Src/cBot/fifo.su ./Core/Src/cBot/marioSong.d ./Core/Src/cBot/marioSong.o ./Core/Src/cBot/marioSong.su ./Core/Src/cBot/sercom.d ./Core/Src/cBot/sercom.o ./Core/Src/cBot/sercom.su ./Core/Src/cBot/ws2812b.d ./Core/Src/cBot/ws2812b.o ./Core/Src/cBot/ws2812b.su

.PHONY: clean-Core-2f-Src-2f-cBot

