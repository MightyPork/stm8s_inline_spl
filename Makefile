#######
# makefile for STM8S_StdPeriph_Lib and SDCC compiler
#
# Customized by MightyPork 1/2017
#
# usage:
#   1. if SDCC not in PATH set path -> CC_ROOT
#   2. set correct STM8 device -> DEVICE
#   3. set project paths -> PRJ_ROOT, PRJ_SRC_DIR, PRJ_INC_DIR
#   4. set SPL root path -> SPL_ROOT
#   5. include required SPL modules -> SPL_SOURCE
#
#######

# STM8 device (default is STM8 discovery board)
DEVICE=STM8S103
DEVICE_FLASH=stm8s103f3

# set compiler path & parameters
CC_ROOT =
CC      = sdcc
CFLAGS  = -mstm8 -lstm8 --opt-code-size --disable-warning 126 --disable-warning 110
# -DUSE_FULL_ASSERT=1

# set output folder and target name
OUTPUT_DIR = ./Build
TARGET     = $(OUTPUT_DIR)/$(DEVICE).hex

# set project folder and files (all *.c)
PRJ_ROOT    = .
PRJ_SRC_DIR = $(PRJ_ROOT)/User
PRJ_INC_DIR = $(PRJ_ROOT)/User
# all project sources included by default
PRJ_SOURCE  = $(notdir $(wildcard $(PRJ_SRC_DIR)/*.c))
PRJ_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(PRJ_SOURCE:.c=.rel))

# set SPL paths
#SPL_SRC_DIR = /usr/share/sdcc/lib/src/stm8/
#SPL_INC_DIR = /usr/share/sdcc/include/stm8/

LIB_INC_DIR = /usr/share/sdcc/include/

#SPL_SRC_DIR = Libraries/SPL/src/
SPL_INC_DIR = Library/SPL/
# add all library sources used here
SPL_SOURCE  =
#stm8s_flash.c
SPL_OBJECTS := $(addprefix $(OUTPUT_DIR)/, $(SPL_SOURCE:.c=.rel))

# collect all include folders
INCLUDE = -I$(PRJ_SRC_DIR) -I$(SPL_INC_DIR) -I$(LIB_INC_DIR)

# collect all source directories
VPATH=$(PRJ_SRC_DIR):$(SPL_SRC_DIR)

.PHONY: clean

all: $(TARGET)

$(OUTPUT_DIR)/%.rel: %.c
	$(CC) $(CFLAGS) -D$(DEVICE) $(INCLUDE) -c $?

$(OUTPUT_DIR)/%.rel: %.c
	$(CC) $(CFLAGS) -D$(DEVICE) $(INCLUDE) -c $? -o $@

$(TARGET): $(PRJ_OBJECTS) $(SPL_OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $^

flash: $(TARGET)
	stm8flash -c stlinkv2 -p $(DEVICE_FLASH) -s flash -w $(TARGET)

clean:
	rm $(OUTPUT_DIR)/*
