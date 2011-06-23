################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../rtmath-console-test.cpp 

OBJS += \
./rtmath-console-test.o 

CPP_DEPS += \
./rtmath-console-test.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -D_DEBUG -DHEAP_CHECK -I"/mnt/ubuntu/home/rhoneyag/workspace/rtmath" -O0 -fopenmp -g3 -ggdb -p -pg -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


