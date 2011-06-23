################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: rtmath-console-test

dependents:
	-cd /mnt/ubuntu/home/rhoneyag/workspace/rtmath/Release && $(MAKE) all

# Tool invocations
rtmath-console-test: $(OBJS) $(USER_OBJS) /mnt/ubuntu/home/rhoneyag/workspace/rtmath/Release/rtmath.a
	@echo 'Building target: $@'
	@echo 'Invoking: Cross G++ Linker'
	g++ -L"/mnt/ubuntu/home/rhoneyag/workspace/rtmath/Release" -L./ -fopenmp -p -pg -o"rtmath-console-test" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C++_DEPS)$(OBJS)$(C_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(CXX_DEPS)$(C_UPPER_DEPS) rtmath-console-test
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
/mnt/ubuntu/home/rhoneyag/workspace/rtmath/Release/rtmath.a:

-include ../makefile.targets
