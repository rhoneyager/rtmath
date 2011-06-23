################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Stdafx.cpp \
../atmos.cpp \
../damatrix.cpp \
../damatrix_quad.cpp \
../debug.cpp \
../debug_mem.cpp \
../error.cpp \
../layer.cpp \
../lbl.cpp \
../matrixop.cpp \
../phaseFunc.cpp \
../polynomial.cpp \
../quadrature.cpp \
../rtmath-base.cpp \
../rtmath.cpp \
../zeros.cpp 

OBJS += \
./Stdafx.o \
./atmos.o \
./damatrix.o \
./damatrix_quad.o \
./debug.o \
./debug_mem.o \
./error.o \
./layer.o \
./lbl.o \
./matrixop.o \
./phaseFunc.o \
./polynomial.o \
./quadrature.o \
./rtmath-base.o \
./rtmath.o \
./zeros.o 

CPP_DEPS += \
./Stdafx.d \
./atmos.d \
./damatrix.d \
./damatrix_quad.d \
./debug.d \
./debug_mem.d \
./error.d \
./layer.d \
./lbl.d \
./matrixop.d \
./phaseFunc.d \
./polynomial.d \
./quadrature.d \
./rtmath-base.d \
./rtmath.d \
./zeros.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -D_DEBUG -O0 -fopenmp -g3 -ggdb -p -pg -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


