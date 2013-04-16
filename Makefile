###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the baseflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
# 

###############################################################################
# Things that the user might override on the commandline
#

# The target to build, must be one of NAZE, FY90Q OR OLIMEXINO
TARGET		?= ESC32

# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG ?=

###############################################################################
# Things that need to be maintained as the source changes
#

VALID_TARGETS	 = ESC32

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)/src
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/bin

# Source files common to all targets
ESC32_SRC	 = startup_stm32f10x_md_gcc.S adc.c binary.c cli.c config.c digital.c fet.c getbuildnum.c main.c misc.c ow.c pwm.c rcc.c run.c serial.c \
		   stm32f10x_adc.c stm32f10x_dbgmcu.c stm32f10x_dma.c stm32f10x_exti.c stm32f10x_flash.c stm32f10x_gpio.c stm32f10x_iwdg.c \
		   stm32f10x_pwr.c stm32f10x_rcc.c stm32f10x_tim.c stm32f10x_usart.c system_stm32f10x.c timer.c syscalls.c

# Search path for baseflight sources
VPATH		:= $(SRC_DIR)

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 = arm-none-eabi-gcc
OBJCOPY		 = arm-none-eabi-objcopy

#
# Tool options.
#
INCLUDE_DIRS	 = $(SRC_DIR)

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
BASE_CFLAGS		 = $(ARCH_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -Wall \
		   -ffunction-sections \
		   -fdata-sections \
		   -DSTM32F10X_MD \
		   -DUSE_STDPERIPH_DRIVER \
		   -D$(TARGET)

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LD_SCRIPT	 = $(ROOT)/stm32_flash.ld
LDFLAGS		 = -lm \
		   $(ARCH_FLAGS) \
		   -static \
		   -Wl,-gc-sections \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

ifeq ($(DEBUG),GDB)
CFLAGS = $(BASE_CFLAGS) \
	-ggdb \
	-O0
else
CFLAGS = $(BASE_CFLAGS) \
	-Os
endif


TARGET_HEX	 = $(BIN_DIR)/hyon_$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/hyon_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 
$(OBJECT_DIR)/$(TARGET)/%.o): %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

clean:
	rm -f $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS)

help:
	@echo ""
	@echo "Makefile for the baseflight firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
