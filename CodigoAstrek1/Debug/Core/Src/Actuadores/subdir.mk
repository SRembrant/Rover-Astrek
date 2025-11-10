################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Actuadores/Control_Kinematics.c \
../Core/Src/Actuadores/Control_PWM.c \
../Core/Src/Actuadores/Control_Pose.c \
../Core/Src/Actuadores/Control_Rover.c \
../Core/Src/Actuadores/Motor.c 

OBJS += \
./Core/Src/Actuadores/Control_Kinematics.o \
./Core/Src/Actuadores/Control_PWM.o \
./Core/Src/Actuadores/Control_Pose.o \
./Core/Src/Actuadores/Control_Rover.o \
./Core/Src/Actuadores/Motor.o 

C_DEPS += \
./Core/Src/Actuadores/Control_Kinematics.d \
./Core/Src/Actuadores/Control_PWM.d \
./Core/Src/Actuadores/Control_Pose.d \
./Core/Src/Actuadores/Control_Rover.d \
./Core/Src/Actuadores/Motor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Actuadores/%.o Core/Src/Actuadores/%.su Core/Src/Actuadores/%.cyclo: ../Core/Src/Actuadores/%.c Core/Src/Actuadores/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Inc/Actuadores" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Src/Actuadores" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Inc/Sensores" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Inc/Transmision" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Src/Sensores" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Src/Transmision" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Inc/Nav" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Src/Nav" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Inc/Herramientas" -I"C:/Users/joadj/OneDrive/Escritorio/Universidad/Proyecto Astrek/RepoAstrek/Rover-Astrek/CodigoAstrek1/Core/Src/Herramientas" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Actuadores

clean-Core-2f-Src-2f-Actuadores:
	-$(RM) ./Core/Src/Actuadores/Control_Kinematics.cyclo ./Core/Src/Actuadores/Control_Kinematics.d ./Core/Src/Actuadores/Control_Kinematics.o ./Core/Src/Actuadores/Control_Kinematics.su ./Core/Src/Actuadores/Control_PWM.cyclo ./Core/Src/Actuadores/Control_PWM.d ./Core/Src/Actuadores/Control_PWM.o ./Core/Src/Actuadores/Control_PWM.su ./Core/Src/Actuadores/Control_Pose.cyclo ./Core/Src/Actuadores/Control_Pose.d ./Core/Src/Actuadores/Control_Pose.o ./Core/Src/Actuadores/Control_Pose.su ./Core/Src/Actuadores/Control_Rover.cyclo ./Core/Src/Actuadores/Control_Rover.d ./Core/Src/Actuadores/Control_Rover.o ./Core/Src/Actuadores/Control_Rover.su ./Core/Src/Actuadores/Motor.cyclo ./Core/Src/Actuadores/Motor.d ./Core/Src/Actuadores/Motor.o ./Core/Src/Actuadores/Motor.su

.PHONY: clean-Core-2f-Src-2f-Actuadores

