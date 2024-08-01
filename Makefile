-include ../makefile.init

RM := rm -rf

# define src dir and files
SRC_DIRS := Src Drivers/STM32L4xx_HAL_Driver/Src /Startup
SRCS := $(wildcard $(addsuffix /*.c,$(SRC_DIRS)))
OBJS := $(SRCS:.c=.o)

# Define the include directories
INC_DIRS := Inc Drivers/STM32L4xx_HAL_Driver/Inc Drivers/STM32L4xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32L4xx/Include Drivers/CMSIS/Include 
INCLUDES := $(addprefix -I,$(INC_DIRS))

# Define the preprocessor definitions
DEFINES := -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx

# Compiler and linker settings
CC := arm-none-eabi-gcc
CFLAGS := -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wall -Wextra -O2 $(INCLUDES) $(DEFINES)
LDFLAGS := -T"/home/dirk/Gsystem/STM32L452RETXP_FLASH.ld" --specs=nosys.specs -Wl,-Map="Gsystem.map" -Wl,--gc-sections -static --specs=nano.specs -Wl,--start-group -lc -lm -Wl,--end-group

# All Target
all: main-build

# Main-build Target
main-build: Gsystem.elf secondary-outputs

# Tool invocations
Gsystem.elf: $(OBJS) $(USER_OBJS) /home/dirk/Gsystem/STM32L452RETXP_FLASH.ld
	$(CC) -o $@ $(OBJS) $(USER_OBJS) $(LDFLAGS)
	@echo 'Finished building target: $@'
	@echo ' '

# Compile source files into object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

default.size.stdout: Gsystem.elf
	arm-none-eabi-size Gsystem.elf
	@echo 'Finished building: $@'
	@echo ' '

Gsystem.list: Gsystem.elf
	arm-none-eabi-objdump -h -S Gsystem.elf > $@
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS) Gsystem.elf Gsystem.list Gsystem.map default.size.stdout
	-@echo ' '

secondary-outputs: default.size.stdout Gsystem.list

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets