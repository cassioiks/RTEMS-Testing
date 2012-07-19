################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../imu/imu-1d-fixed.c \
../imu/imu-1d.c 

OBJS += \
./imu/imu-1d-fixed.o \
./imu/imu-1d.o 

C_DEPS += \
./imu/imu-1d-fixed.d \
./imu/imu-1d.d 


# Each subdirectory must supply rules for building sources it contributes
imu/%.o: ../imu/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: RTEMS C Compiler'
	/opt/rtems-4.11/bin/sparc-rtems4.11-gcc -B/opt/rtems-4.11/sparc-rtems4.11/sis/lib/ -specs bsp_specs -qrtems -mcpu=cypress -Os -g -Wall -c -fmessage-length=0 -pipe -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


