@echo off
setlocal
set "GCC=C:\ST\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe"
set "FLAGS=-std=c11 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F407xx -Wall -Wextra -Wshadow -Werror -fsyntax-only"
set "INCS=-I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../MODULE/algorithm -I../MODULE/motor -I../MODULE/motor/DJImotor -I../MODULE/motor/DMmotor -I../BSP/CAN -I../BSP/DWT -I../BSP/USART -I../APPLICATION -I../APPLICATION/arm -I../APPLICATION/chassis -I../MODULE/imu -I../MODULE/daemon -I../MODULE -I../MODULE/usb -I../MODULE/protocol -I../MODULE/servo -I../MODULE/buzzer -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/ST/ARM/DSP/Inc -I./Engineer -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc"
for %%F in (../Core/Src/tim.c ../Core/Src/gpio.c ../Core/Src/stm32f4xx_it.c ../MODULE/servo/mg995_servo.c ../APPLICATION/chassis/chassis.c ../APPLICATION/arm/arm_tool.c ../APPLICATION/arm/arm_kinematics.c ../APPLICATION/arm/arm_trajectory.c ../APPLICATION/arm/arm.c ../APPLICATION/app_arm_command_id.c ../APPLICATION/app_arm_flow.c ../APPLICATION/app_fruit_task.c ../APPLICATION/app_arm_side_pick_place.c ../APPLICATION/app_runtime.c) do (
  echo Checking %%F
  "%GCC%" %FLAGS% %INCS% %%F || exit /b 1
)
exit /b 0
