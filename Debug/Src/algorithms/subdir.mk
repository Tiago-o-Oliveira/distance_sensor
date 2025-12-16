################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/algorithms/acc_algorithm.c 

OBJS += \
./Src/algorithms/acc_algorithm.o 

C_DEPS += \
./Src/algorithms/acc_algorithm.d 


# Each subdirectory must supply rules for building sources it contributes
Src/algorithms/%.o Src/algorithms/%.su Src/algorithms/%.cyclo: ../Src/algorithms/%.c Src/algorithms/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I/home/tiago/Workspace/STM32CubeIDE/distance_sensor/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-algorithms

clean-Src-2f-algorithms:
	-$(RM) ./Src/algorithms/acc_algorithm.cyclo ./Src/algorithms/acc_algorithm.d ./Src/algorithms/acc_algorithm.o ./Src/algorithms/acc_algorithm.su

.PHONY: clean-Src-2f-algorithms

