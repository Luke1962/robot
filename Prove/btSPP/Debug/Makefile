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
C_SRCS +=  \
../BTD32.cpp \
../BTHID.cpp \
../SPP.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
BTD32.o \
BTHID.o \
SPP.o

OBJS_AS_ARGS +=  \
BTD32.o \
BTHID.o \
SPP.o

C_DEPS +=  \
BTD32.d \
BTHID.d \
SPP.d

C_DEPS_AS_ARGS +=  \
BTD32.d \
BTHID.d \
SPP.d

OUTPUT_FILE_PATH +=btSPP.elf

OUTPUT_FILE_PATH_AS_ARGS +=btSPP.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

# AVR32/GNU C Compiler







./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 0.0.0
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -D__SAM3X8E__ -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON -DUSB_MANUFACTURER="\"Unknown\"" -DUSB_PRODUCT="\"Arduino Due\"" -D_VMDEBUG=1 -DARDUINO=101 -DARDUINO_MAIN -DF_CPU=84000000L -Dprintf=iprintf -D__SAM__ -D__cplusplus  -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\cores\arduino" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\variants\arduino_due_x" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\libraries\SPI" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\libraries\SPI\utility" -I"C:\Program Files (x86)\Arduino\libraries\USBHost\src" -I"C:\Program Files (x86)\Arduino\libraries\USBHost\src\utility" -I"C:\Program Files (x86)\Arduino\libraries" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\libraries" -I"C:\Program Files (x86)\Visual Micro\Visual Micro for Arduino\Micro Platforms\default\debuggers" -I"C:\Users\Luca\Documents\Arduino\SW\libraries" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\system\CMSIS\Device\ATMEL" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\system\libsam" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\system\libsam\include" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\system\CMSIS\CMSIS\Include" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\system\CMSIS\CMSIS\Include" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\system\CMSIS\Device\ATMEL" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\system\libsam" -I"C:\Program Files (x86)\Arduino\hardware\arduino\sam\system\libsam\include" -I"C:\Program Files (x86)\Arduino/hardware/tools/g++_arm_none_eabi\arm-none-eabi\include" -I"C:\Program Files (x86)\Arduino/hardware/tools/g++_arm_none_eabi\arm_none_eabi\include" -I"C:\Program Files (x86)\Arduino/hardware/tools/g++_arm_none_eabi\lib\gcc\arm-none-eabi\4.4.1\include" -I"C:\Program Files (x86)\Arduino\hardware\tools\g++_arm_none_eabi\lib\gcc\arm-none-eabi\4.4.1\include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega2560 -c -include=arduino.h -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



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
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="btSPP.map" -Wl,--start-group  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega2560  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "btSPP.elf" "btSPP.hex"
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "btSPP.elf" "btSPP.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "btSPP.elf" > "btSPP.lss"
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "btSPP.elf" "btSPP.srec"
	"C:\Program Files (x86)\Atmel\Atmel Studio 6.2\extensions\Atmel\AtmelToolchainProvider\6.2.666\avr8-gnu-toolchain\bin\avr-size.exe" "btSPP.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "btSPP.elf" "btSPP.a" "btSPP.hex" "btSPP.lss" "btSPP.eep" "btSPP.map" "btSPP.srec" "btSPP.usersignatures"
	