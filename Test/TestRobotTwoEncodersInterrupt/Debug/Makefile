################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../Visual\ Micro/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += 


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS += 

OBJS_AS_ARGS += 

C_DEPS += 

C_DEPS_AS_ARGS += 

OUTPUT_FILE_PATH +=TestRobotTwoEncodersInterrupt.elf

OUTPUT_FILE_PATH_AS_ARGS +=TestRobotTwoEncodersInterrupt.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

# AVR32/GNU C Compiler



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 0.0.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="TestRobotTwoEncodersInterrupt.map" -Wl,--start-group  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega2560  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "TestRobotTwoEncodersInterrupt.elf" "TestRobotTwoEncodersInterrupt.hex"
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "TestRobotTwoEncodersInterrupt.elf" "TestRobotTwoEncodersInterrupt.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "TestRobotTwoEncodersInterrupt.elf" > "TestRobotTwoEncodersInterrupt.lss"
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "TestRobotTwoEncodersInterrupt.elf" "TestRobotTwoEncodersInterrupt.srec"
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-size.exe" "TestRobotTwoEncodersInterrupt.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "TestRobotTwoEncodersInterrupt.elf" "TestRobotTwoEncodersInterrupt.a" "TestRobotTwoEncodersInterrupt.hex" "TestRobotTwoEncodersInterrupt.lss" "TestRobotTwoEncodersInterrupt.eep" "TestRobotTwoEncodersInterrupt.map" "TestRobotTwoEncodersInterrupt.srec" "TestRobotTwoEncodersInterrupt.usersignatures"
	