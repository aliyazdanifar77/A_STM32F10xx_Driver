################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../deriver/src/A_gpio_f10xx_drv.c 

OBJS += \
./deriver/src/A_gpio_f10xx_drv.o 

C_DEPS += \
./deriver/src/A_gpio_f10xx_drv.d 


# Each subdirectory must supply rules for building sources it contributes
deriver/src/A_gpio_f10xx_drv.o: ../deriver/src/A_gpio_f10xx_drv.c deriver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"D:/STM32F103_GPIO_DERIVER/deriver/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"deriver/src/A_gpio_f10xx_drv.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

