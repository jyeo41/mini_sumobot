TARGET = sumobot_mini
BUILD_DIR = build

BASE_DIR = /home/joon/devel/embedded_programming/ST
PROJECT_DIR = $(BASE_DIR)/STM32CubeIDE/stm32_workspace/sumobot_mini
PROJECT_INC = $(PROJECT_DIR)/Inc
PROJECT_SRC = $(PROJECT_DIR)/Src
PROJECT_ASM = $(PROJECT_DIR)/Startup

CMSIS_CORE_INC = $(BASE_DIR)/CMSIS/Core/Include
CMSIS_DEVICE_INC = $(BASE_DIR)/CMSIS/Device/ST/STM32F4xx/Include

# Toolchain
CC = arm-none-eabi-gcc

# Target Architecture
CPU = -mcpu=cortex-m4 -mthumb

# C Compiling
MCU = -DSTM32F407xx
C_INCLUDES = $(PROJECT_INC) $(CMSIS_CORE_INC) $(CMSIS_DEVICE_INC)
W_FLAGS = -Wall -Werror -Wextra
OPT_FLAGS = -O0
DEBUG_LEVEL = -g3
C_FLAGS = $(CPU) $(MCU) $(addprefix -I, $(C_INCLUDES)) $(W_FLAGS) $(OPT_FLAGS) $(DEBUG_LEVEL)

# ASM Compiling
ASM_SOURCES = $(foreach dir, $(PROJECT_ASM), $(wildcard $(dir)/*.s))
ASM_FLAGS = $(CPU) $(DEBUG_LEVEL)

# Linking
LD_SCRIPT = STM32F407VGTX_FLASH.ld
LD_FLAGS = $(CPU) -T$(LD_SCRIPT)

# Find all the .c files and generate .o files
C_SOURCES = $(foreach dir, $(PROJECT_SRC), $(wildcard $(dir)/*.c))
OBJECTS = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/, $(notdir $(ASM_SOURCES:.s=.o)))

#$(info C_SOURCES: $(C_SOURCES))
#$(info OBJECTS: $(OBJECTS))

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(LD_FLAGS) $(OBJECTS) -o $@

$(BUILD_DIR)/%.o: $(PROJECT_SRC)/%.c | $(BUILD_DIR)
	$(CC) $(C_FLAGS) -c -o $@ $^

$(BUILD_DIR)/%.o: $(PROJECT_ASM)/%.s | $(BUILD_DIR)
	$(CC) $(ASM_FLAGS) -c -o $@ $^


$(BUILD_DIR):
	mkdir $@


.PHONY: all clean flash debug

all: $(BUILD_DIR)/$(TARGET).elf

flash: $(BUILD_DIR)/$(TARGET).elf
	STM32_Programmer_CLI --connect port=SWD freq=4000 mode=UR reset=HWrst --erase 0 --download $^ --verify --go

debug:
	@echo "Starting st-util..."
	st-util & \
	ST_UTIL_PID=$$!; \
	echo "Waiting for st-util to listen on port 4242..."; \
	while ! lsof -i :4242; do \
		sleep 1; \
	done; \
	echo "st-util is now listening on port 4242."; \
	echo "Starting GDB..."; \
	arm-none-eabi-gdb $(BUILD_DIR)/$(TARGET).elf --eval-command="target extended-remote :4242"; \
	kill $$ST_UTIL_PID

clean:
	rm -rf build
