################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MPR121.c \
../Core/Src/app_freertos.c \
../Core/Src/lsm303c.c \
../Core/Src/main.c \
../Core/Src/ms5837_lib.c \
../Core/Src/ssd1306.c \
../Core/Src/ssd1306_fonts.c \
../Core/Src/stm32wbxx_hal_msp.c \
../Core/Src/stm32wbxx_hal_timebase_tim.c \
../Core/Src/stm32wbxx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wbxx.c 

OBJS += \
./Core/Src/MPR121.o \
./Core/Src/app_freertos.o \
./Core/Src/lsm303c.o \
./Core/Src/main.o \
./Core/Src/ms5837_lib.o \
./Core/Src/ssd1306.o \
./Core/Src/ssd1306_fonts.o \
./Core/Src/stm32wbxx_hal_msp.o \
./Core/Src/stm32wbxx_hal_timebase_tim.o \
./Core/Src/stm32wbxx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wbxx.o 

C_DEPS += \
./Core/Src/MPR121.d \
./Core/Src/app_freertos.d \
./Core/Src/lsm303c.d \
./Core/Src/main.d \
./Core/Src/ms5837_lib.d \
./Core/Src/ssd1306.d \
./Core/Src/ssd1306_fonts.d \
./Core/Src/stm32wbxx_hal_msp.d \
./Core/Src/stm32wbxx_hal_timebase_tim.d \
./Core/Src/stm32wbxx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wbxx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MPR121.cyclo ./Core/Src/MPR121.d ./Core/Src/MPR121.o ./Core/Src/MPR121.su ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/lsm303c.cyclo ./Core/Src/lsm303c.d ./Core/Src/lsm303c.o ./Core/Src/lsm303c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ms5837_lib.cyclo ./Core/Src/ms5837_lib.d ./Core/Src/ms5837_lib.o ./Core/Src/ms5837_lib.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/ssd1306_fonts.cyclo ./Core/Src/ssd1306_fonts.d ./Core/Src/ssd1306_fonts.o ./Core/Src/ssd1306_fonts.su ./Core/Src/stm32wbxx_hal_msp.cyclo ./Core/Src/stm32wbxx_hal_msp.d ./Core/Src/stm32wbxx_hal_msp.o ./Core/Src/stm32wbxx_hal_msp.su ./Core/Src/stm32wbxx_hal_timebase_tim.cyclo ./Core/Src/stm32wbxx_hal_timebase_tim.d ./Core/Src/stm32wbxx_hal_timebase_tim.o ./Core/Src/stm32wbxx_hal_timebase_tim.su ./Core/Src/stm32wbxx_it.cyclo ./Core/Src/stm32wbxx_it.d ./Core/Src/stm32wbxx_it.o ./Core/Src/stm32wbxx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32wbxx.cyclo ./Core/Src/system_stm32wbxx.d ./Core/Src/system_stm32wbxx.o ./Core/Src/system_stm32wbxx.su

.PHONY: clean-Core-2f-Src

