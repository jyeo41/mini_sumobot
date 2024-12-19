TARGET = sumobot_mini
BUILD_DIR = build

# Add list of all necessary directories
PROJECT_DIR = .
PROJECT_INC = $(PROJECT_DIR)/Inc
PROJECT_SRC = $(PROJECT_DIR)/Src
PROJECT_ASM = $(PROJECT_DIR)/Startup

# CMSIS header directories
CMSIS_CORE_INC = $(PROJECT_DIR)/external/STM32CubeF4/Drivers/CMSIS/Core/Include
CMSIS_DEVICE_INC = $(PROJECT_DIR)/external/cmsis_device_f4/Include

# MPaland printf implementation directory, needed for build target
PRINTF_DIR = $(PROJECT_DIR)/external/printf

# ARM GNU Toolchain GCC binary path
ARM_NONE_EABI_BIN = /opt/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi/bin

# Toolchain
CC = $(ARM_NONE_EABI_BIN)/arm-none-eabi-gcc

# Target Architecture and other flags, for assembler, compiler, linker
# CPU is target architecture.
# FPU is floating point unit, both the -mfpu AND -mfloat have to be specified together to make use of hardware FPU.
# SPECS is to use a more lightweight implementation of Newlib C library (which is also a lightweight version of the Std Lib C.
#	This flag is used to reduce code size by a considerable amount
CPU = -mcpu=cortex-m4 -mthumb
FPU = -mfpu=fpv4-sp-d16 -mfloat-abi=hard
SPECS = --specs=nano.specs

# C Compiling
MCU = -DSTM32F407xx # needed when compiling .c to .o files to specify the header to use our target MCU
C_INCLUDES = $(PROJECT_INC) $(CMSIS_CORE_INC) $(CMSIS_DEVICE_INC) $(PRINTF_DIR)
W_FLAGS = -Wall -Werror -Wextra
OPT_FLAGS = -O0

# Use compiler generated dependency ".d" files to recompile with "make"
# even when editing header ".h" files
DEP_FLAGS = -MMD -MP

# maximum debug level, needed to not hang during code stepping in GDB CLI
DEBUG_LEVEL = -g3
C_FLAGS = $(CPU) $(MCU) $(FPU) $(SPECS) $(DEP_FLAGS) $(addprefix -I, $(C_INCLUDES)) $(W_FLAGS) $(OPT_FLAGS) $(DEBUG_LEVEL)

# ASM Compiling
ASM_SOURCES = $(foreach dir, $(PROJECT_ASM), $(wildcard $(dir)/*.s))
ASM_FLAGS = $(CPU) $(FPU) $(SPECS) $(DEBUG_LEVEL)

# Linking
LD_SCRIPT = STM32F407VGTX_FLASH.ld
LD_FLAGS = $(CPU) $(FPU) $(SPECS) -T$(LD_SCRIPT)

# Find all the .c files and generate .o files
# create a list of all source files prepended with their directory tree
C_SOURCES = $(foreach dir, $(PROJECT_SRC) $(PRINTF_DIR), $(wildcard $(dir)/*.c)) 

# create a list of all the target object files for build/*.o
OBJECTS = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))

# do the same thing for the startup .s file
OBJECTS += $(addprefix $(BUILD_DIR)/, $(notdir $(ASM_SOURCES:.s=.o))) 
DEP_SOURCES = $(patsubst %.o, %.d, $(OBJECTS))

# Debugging lines to check the built paths from above commands
#$(info C_SOURCES: $(C_SOURCES))
#$(info OBJECTS: $(OBJECTS))

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(LD_FLAGS) $(OBJECTS) -o $@

$(BUILD_DIR)/%.o: $(PROJECT_SRC)/%.c | $(BUILD_DIR)
	$(CC) $(C_FLAGS) -c -o $@ $<

# Build target to add the printf external directory as an additional build target.
# Not the most elegant solution, but it works. The build target will also check this directory
#	as well as the PROJECT_SRC directory when looking for source files.
$(BUILD_DIR)/%.o: $(PRINTF_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(C_FLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(PROJECT_ASM)/%.s | $(BUILD_DIR)
	$(CC) $(ASM_FLAGS) -c -o $@ $<

$(BUILD_DIR):
	@mkdir $@

-include $(DEP_SOURCES)

.PHONY: all clean flash print-% cppcheck docker-clean size picocom

all: $(BUILD_DIR)/$(TARGET).elf

# Flashing from CLI using STM32 Cube Programmer
# port = SWD because we are using ST-Link
# freq = 4000 which is default, but explicitly type it for clarify
# mode = UR means under reset, good to hold the MCU in reset state when flashing
# reset = HWrst means hardware reset after flashing, this is the default
# erase 0 means only erase the memory in sector 0, can use all but takes longer
# --download to flash the input
# --verify to check it was flashed properly
# --go to actually start the program after flashing
flash: $(BUILD_DIR)/$(TARGET).elf
	STM32_Programmer_CLI --connect port=SWD freq=4000 mode=UR reset=HWrst --erase 0 --download $^ --verify --go

# Ignore the syscalls and sysmem files because they're just used to fill stub functions called by new/nanolibc.
# --inline-suppr is useful to suppress certain functions I won't need
# --supress=missingIncludeSystem is needed in conjunction with setting vendor header files as <> instead of ""
#	otherwise cppcheck will keep complaining about missing "stm32f4xx.h" headers.
# The cppcheck.report shows a list of all the available checks and whether it ran them or not. The F2P version runs less.
# --suppress=unusedFunction \ this is an unused flag for cppcheck
CPPCHECK = cppcheck
CPPCHECK_IGNORE = $(PROJECT_SRC)/syscalls.c $(PROJECT_SRC)/sysmem.c
cppcheck:
	@$(CPPCHECK) --quiet --enable=all --inline-suppr \
	--error-exitcode=1 --suppress=missingIncludeSystem \
	--checkers-report=cppcheck.report \
	--suppress=checkersReport --check-level=exhaustive \
	$(PROJECT_SRC) -I$(PROJECT_INC) $(addprefix -i, $(CPPCHECK_IGNORE))

# -% is a wildcard that represents any string. When you type the command 
#  make print-BASE_DIR, this matches the print-% rule and replaces % with BASE_DIR.
#  Next, $* will expand to the stem of the target which is the text matching the %.
#  Running make print-BASE_DIR would expand $* to BASE_DIR.
#  $($*) would expand to $(BASE_DIR) which would echo out the VALUE of BASE_DIR
print-%:
	@echo $*=$($*)

# Delete all containers and volume use first then delete all the images after.
# Need to check to make sure containers and images exist first using if then syntax,
#	otherwise docker complains and says theres an error if you try to run the rm or rmi command
#	with no existing containers/images.
docker-clean:
	@if [ -n "$$(docker ps -aq)" ]; then \
		docker rm -vf $$(docker ps -aq); \
	fi;
	@if [ -n "$$(docker images -aq)" ]; then \
		docker rmi -f $$(docker images -aq); \
	fi

# Phony target to check the file size of the target elf file. the arm-none-eabi-size binary splits the sizes into
# their respective sections:
#	- .text (contains program code in flash memory)
#	- .data (initialized globals and static variables, stored in flash and to be copied to RAM)
#	- .bss (uninitialized globals and statics)
size: $(BUILD_DIR)/$(TARGET).elf
	$(ARM_NONE_EABI_BIN)/arm-none-eabi-size $^

picocom:
	@picocom -b 115200 /dev/ttyUSB0

clean:
	@rm -rf build
