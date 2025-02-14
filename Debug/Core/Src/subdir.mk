################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/HMC5883L.c \
../Core/Src/I2Cdev.c \
../Core/Src/JDY-18.c \
../Core/Src/PID.c \
../Core/Src/boatCompass.c \
../Core/Src/boatEngine.c \
../Core/Src/boatServo.c \
../Core/Src/data_filter_service.c \
../Core/Src/location_service.c \
../Core/Src/main.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/HMC5883L.o \
./Core/Src/I2Cdev.o \
./Core/Src/JDY-18.o \
./Core/Src/PID.o \
./Core/Src/boatCompass.o \
./Core/Src/boatEngine.o \
./Core/Src/boatServo.o \
./Core/Src/data_filter_service.o \
./Core/Src/location_service.o \
./Core/Src/main.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/HMC5883L.d \
./Core/Src/I2Cdev.d \
./Core/Src/JDY-18.d \
./Core/Src/PID.d \
./Core/Src/boatCompass.d \
./Core/Src/boatEngine.d \
./Core/Src/boatServo.d \
./Core/Src/data_filter_service.d \
./Core/Src/location_service.d \
./Core/Src/main.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/HMC5883L.cyclo ./Core/Src/HMC5883L.d ./Core/Src/HMC5883L.o ./Core/Src/HMC5883L.su ./Core/Src/I2Cdev.cyclo ./Core/Src/I2Cdev.d ./Core/Src/I2Cdev.o ./Core/Src/I2Cdev.su ./Core/Src/JDY-18.cyclo ./Core/Src/JDY-18.d ./Core/Src/JDY-18.o ./Core/Src/JDY-18.su ./Core/Src/PID.cyclo ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/PID.su ./Core/Src/boatCompass.cyclo ./Core/Src/boatCompass.d ./Core/Src/boatCompass.o ./Core/Src/boatCompass.su ./Core/Src/boatEngine.cyclo ./Core/Src/boatEngine.d ./Core/Src/boatEngine.o ./Core/Src/boatEngine.su ./Core/Src/boatServo.cyclo ./Core/Src/boatServo.d ./Core/Src/boatServo.o ./Core/Src/boatServo.su ./Core/Src/data_filter_service.cyclo ./Core/Src/data_filter_service.d ./Core/Src/data_filter_service.o ./Core/Src/data_filter_service.su ./Core/Src/location_service.cyclo ./Core/Src/location_service.d ./Core/Src/location_service.o ./Core/Src/location_service.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

