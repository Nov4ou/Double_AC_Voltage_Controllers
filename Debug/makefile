################################################################################
# Automatically-generated file. Do not edit!
################################################################################

CG_TOOL_ROOT := /Applications/ti/ti-cgt-c2000_22.6.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./app/adc/adc.obj" \
"./app/epwm/epwm.obj" \
"./app/key/key.obj" \
"./app/oled/oled.obj" \
"./app/spi/spi.obj" \
"./app/timer/timer.obj" \
"./f2806x_libraries/F2806x_Adc.obj" \
"./f2806x_libraries/F2806x_CodeStartBranch.obj" \
"./f2806x_libraries/F2806x_CpuTimers.obj" \
"./f2806x_libraries/F2806x_DefaultIsr.obj" \
"./f2806x_libraries/F2806x_EPwm.obj" \
"./f2806x_libraries/F2806x_GlobalVariableDefs.obj" \
"./f2806x_libraries/F2806x_Gpio.obj" \
"./f2806x_libraries/F2806x_I2C.obj" \
"./f2806x_libraries/F2806x_PieCtrl.obj" \
"./f2806x_libraries/F2806x_PieVect.obj" \
"./f2806x_libraries/F2806x_Spi.obj" \
"./f2806x_libraries/F2806x_SysCtrl.obj" \
"./f2806x_libraries/F2806x_usDelay.obj" \
"./user/main.obj" \
"../f2806x_libraries/F28069.cmd" \
"../f2806x_libraries/F2806x_Headers_nonBIOS.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"/Applications/ti/solar/v1.2/float/lib/Solar_Lib_Float.lib" \

-include ../makefile.init

RM := rm -rf
RMDIR := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include app/adc/subdir_vars.mk
-include app/epwm/subdir_vars.mk
-include app/key/subdir_vars.mk
-include app/oled/subdir_vars.mk
-include app/spi/subdir_vars.mk
-include app/timer/subdir_vars.mk
-include f2806x_libraries/subdir_vars.mk
-include user/subdir_vars.mk
-include app/adc/subdir_rules.mk
-include app/epwm/subdir_rules.mk
-include app/key/subdir_rules.mk
-include app/oled/subdir_rules.mk
-include app/spi/subdir_rules.mk
-include app/timer/subdir_rules.mk
-include f2806x_libraries/subdir_rules.mk
-include user/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Double_AC_Voltage_Controllers.out 

EXE_OUTPUTS__QUOTED += \
"Double_AC_Voltage_Controllers.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "Double_AC_Voltage_Controllers.out"

# Tool invocations
Double_AC_Voltage_Controllers.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"/Applications/ti/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi -z -m"Double_AC_Voltage_Controllers.map" --stack_size=0x300 --warn_sections -i"/Applications/ti/ti-cgt-c2000_22.6.1.LTS/lib" -i"/Applications/ti/ti-cgt-c2000_22.6.1.LTS/include" --reread_libs --diag_wrap=off --display_error_number --xml_link_info="Double_AC_Voltage_Controllers_linkInfo.xml" --rom_model -o "Double_AC_Voltage_Controllers.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "app/adc/adc.obj" "app/epwm/epwm.obj" "app/key/key.obj" "app/oled/oled.obj" "app/spi/spi.obj" "app/timer/timer.obj" "f2806x_libraries/F2806x_Adc.obj" "f2806x_libraries/F2806x_CodeStartBranch.obj" "f2806x_libraries/F2806x_CpuTimers.obj" "f2806x_libraries/F2806x_DefaultIsr.obj" "f2806x_libraries/F2806x_EPwm.obj" "f2806x_libraries/F2806x_GlobalVariableDefs.obj" "f2806x_libraries/F2806x_Gpio.obj" "f2806x_libraries/F2806x_I2C.obj" "f2806x_libraries/F2806x_PieCtrl.obj" "f2806x_libraries/F2806x_PieVect.obj" "f2806x_libraries/F2806x_Spi.obj" "f2806x_libraries/F2806x_SysCtrl.obj" "f2806x_libraries/F2806x_usDelay.obj" "user/main.obj" 
	-$(RM) "app/adc/adc.d" "app/epwm/epwm.d" "app/key/key.d" "app/oled/oled.d" "app/spi/spi.d" "app/timer/timer.d" "f2806x_libraries/F2806x_Adc.d" "f2806x_libraries/F2806x_CpuTimers.d" "f2806x_libraries/F2806x_DefaultIsr.d" "f2806x_libraries/F2806x_EPwm.d" "f2806x_libraries/F2806x_GlobalVariableDefs.d" "f2806x_libraries/F2806x_Gpio.d" "f2806x_libraries/F2806x_I2C.d" "f2806x_libraries/F2806x_PieCtrl.d" "f2806x_libraries/F2806x_PieVect.d" "f2806x_libraries/F2806x_Spi.d" "f2806x_libraries/F2806x_SysCtrl.d" "user/main.d" 
	-$(RM) "f2806x_libraries/F2806x_CodeStartBranch.d" "f2806x_libraries/F2806x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

