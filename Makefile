TARGET = sumobot_mini
BUILD_DIR = build

# Add list of all necessary directories
BASE_DIR = $(HOME)/devel/embedded_programming/ST
PROJECT_DIR = $(BASE_DIR)/STM32CubeIDE/stm32_workspace/sumobot_mini
PROJECT_INC = $(PROJECT_DIR)/Inc
PROJECT_SRC = $(PROJECT_DIR)/Src
PROJECT_ASM = $(PROJECT_DIR)/Startup

# CMSIS header directories
CMSIS_CORE_INC = $(BASE_DIR)/CMSIS/Core/Include
CMSIS_DEVICE_INC = $(BASE_DIR)/CMSIS/Device/ST/STM32F4xx/Include

# Toolchain
CC = arm-none-eabi-gcc

# Target Architecture
CPU = -mcpu=cortex-m4 -mthumb

# C Compiling
MCU = -DSTM32F407xx # needed when compiling .c to .o files to specify the header to use our target MCU
C_INCLUDES = $(PROJECT_INC) $(CMSIS_CORE_INC) $(CMSIS_DEVICE_INC)
W_FLAGS = -Wall -Werror -Wextra
OPT_FLAGS = -O0

# Use compiler generated dependency ".d" files to recompile with "make"
# even when editing header ".h" files
DEP_FLAGS = -MMD -MP

# maximum debug level, needed to not hang during code stepping in GDB CLI
DEBUG_LEVEL = -g3
C_FLAGS = $(CPU) $(MCU) $(DEP_FLAGS) $(addprefix -I, $(C_INCLUDES)) $(W_FLAGS) $(OPT_FLAGS) $(DEBUG_LEVEL)

# ASM Compiling
ASM_SOURCES = $(foreach dir, $(PROJECT_ASM), $(wildcard $(dir)/*.s))
ASM_FLAGS = $(CPU) $(DEBUG_LEVEL)

# Linking
LD_SCRIPT = STM32F407VGTX_FLASH.ld
LD_FLAGS = $(CPU) -T$(LD_SCRIPT)

# Find all the .c files and generate .o files
C_SOURCES = $(foreach dir, $(PROJECT_SRC), $(wildcard $(dir)/*.c)) # create a list of all source files prepended with their directory tree
OBJECTS = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o))) # create a list of all the target object files for build/*.o
OBJECTS += $(addprefix $(BUILD_DIR)/, $(notdir $(ASM_SOURCES:.s=.o))) # do the same thing for the startup .s file
DEP_SOURCES = $(patsubst %.o, %.d, $(OBJECTS))
# Debugging lines to check the built paths from above commands
#$(info C_SOURCES: $(C_SOURCES))
#$(info OBJECTS: $(OBJECTS))

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(LD_FLAGS) $(OBJECTS) -o $@

$(BUILD_DIR)/%.o: $(PROJECT_SRC)/%.c | $(BUILD_DIR)
	$(CC) $(C_FLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(PROJECT_ASM)/%.s | $(BUILD_DIR)
	$(CC) $(ASM_FLAGS) -c -o $@ $<

$(BUILD_DIR):
	@mkdir $@

-include $(DEP_SOURCES)

.PHONY: all clean flash debug

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

debug:
	@echo "Starting st-util..."
	st-util & \ # run st-util GDB server in the background
	ST_UTIL_PID=$$!; \ #save the PID of the background process
	echo "Waiting for st-util to listen on port 4242..."; \
	while ! lsof -i :4242; do \ # while loop syntax to check if GDB server is ready
		sleep 1; \ # keep sleeping until it is ready
	done; \
	echo "st-util is now listening on port 4242."; \
	echo "Starting GDB..."; \
	arm-none-eabi-gdb $(BUILD_DIR)/$(TARGET).elf --eval-command="target extended-remote :4242"; \ # eval command to connect to our target immediately after
	kill $$ST_UTIL_PID # kill the GDB server after quitting the GDB session

clean:
	@rm -rf build
