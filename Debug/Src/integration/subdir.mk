################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/integration/acc_hal_integration_stm32cube_xm.c \
../Src/integration/acc_integration_cortex.c \
../Src/integration/acc_integration_log.c \
../Src/integration/acc_integration_stm32.c \
../Src/integration/acc_wrap_printf.c \
../Src/integration/printf.c 

OBJS += \
./Src/integration/acc_hal_integration_stm32cube_xm.o \
./Src/integration/acc_integration_cortex.o \
./Src/integration/acc_integration_log.o \
./Src/integration/acc_integration_stm32.o \
./Src/integration/acc_wrap_printf.o \
./Src/integration/printf.o 

C_DEPS += \
./Src/integration/acc_hal_integration_stm32cube_xm.d \
./Src/integration/acc_integration_cortex.d \
./Src/integration/acc_integration_log.d \
./Src/integration/acc_integration_stm32.d \
./Src/integration/acc_wrap_printf.d \
./Src/integration/printf.d 


# Each subdirectory must supply rules for building sources it contributes
Src/integration/%.o Src/integration/%.su Src/integration/%.cyclo: ../Src/integration/%.c Src/integration/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu99 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I/home/tiago/Workspace/STM32CubeIDE/distance_sensor/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-integration

clean-Src-2f-integration:
	-$(RM) ./Src/integration/acc_hal_integration_stm32cube_xm.cyclo ./Src/integration/acc_hal_integration_stm32cube_xm.d ./Src/integration/acc_hal_integration_stm32cube_xm.o ./Src/integration/acc_hal_integration_stm32cube_xm.su ./Src/integration/acc_integration_cortex.cyclo ./Src/integration/acc_integration_cortex.d ./Src/integration/acc_integration_cortex.o ./Src/integration/acc_integration_cortex.su ./Src/integration/acc_integration_log.cyclo ./Src/integration/acc_integration_log.d ./Src/integration/acc_integration_log.o ./Src/integration/acc_integration_log.su ./Src/integration/acc_integration_stm32.cyclo ./Src/integration/acc_integration_stm32.d ./Src/integration/acc_integration_stm32.o ./Src/integration/acc_integration_stm32.su ./Src/integration/acc_wrap_printf.cyclo ./Src/integration/acc_wrap_printf.d ./Src/integration/acc_wrap_printf.o ./Src/integration/acc_wrap_printf.su ./Src/integration/printf.cyclo ./Src/integration/printf.d ./Src/integration/printf.o ./Src/integration/printf.su

.PHONY: clean-Src-2f-integration

