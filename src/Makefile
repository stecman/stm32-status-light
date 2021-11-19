# Serial number for USB interface
SERIALNUM ?= DEVELOPMENT
CDEFS = -DUSB_SERIALNUM="\"$(SERIALNUM)\""

# Parameters for firmware size script
FLASH_SIZE_BYTES = 32768
SRAM_SIZE_BYTES = 6144

# Base OpenCM3 build
PROJECT = status-light
BUILD_DIR = build

SHARED_DIR = ../common
CFILES = main.c \
  delay.c \
  sk6812.c \
  usb_control.c \

SERIALNUM ?= DEVELOPMENT

CSTD = -std=c11
OPT = -Os

DEVICE = stm32f070f6

VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

# GDB config
GDB ?=

# Use gdb-multiarch if it's present
ifndef GDB
	GDB = $(shell command -v gdb-multiarch 2> /dev/null)
endif

# Otherwise assume prefixed gdb
ifndef GDB
	GDB = $(PREFIX)gdb
endif

# OpenOCD config
OOCD ?= openocd

# Programmer to use (name of interface config in OpenOCD)
# For ST-Link v1 or v2 programmers, use "stlink"
# See the interfaces/*.cfg files installed by openocd for options
PROGRAMMER ?= jlink

# Transport to use for programming
ifeq ($(PROGRAMMER), stlink)
	TRANSPORT ?= hla_swd
else
	TRANSPORT ?= swd
endif

# Note we're using a local stm32g0x config here
# This has been modified to allow attaching in low power mode with a slow clock
OOCD_FLAGS = -f interface/$(PROGRAMMER).cfg \
             -c "transport select $(TRANSPORT)" \
             -f ../openocd-jlink.cfg

# Flag to attach without resetting the device
HALT ?=


reset:
	$(OOCD) $(OOCD_FLAGS) -c $(if $(HALT), 'reset_halt ()', 'reset_run ()')

flash: all
	BINARY=$(PROJECT).elf $(OOCD) $(OOCD_FLAGS) -c $(if $(HALT), 'program_and_halt ()', 'program_and_run ()')

# Launch a split window with OpenOCD and GDB
# OpenOCD is automatically killed when GDB is closed
attach:
	@tmux new-session \
			'make -s attach_openocd' \; \
		split-window \
			'echo -e "Waiting for OpenOCD port to open...\n\n" \
				"If the top pane closes immediately, run make attach_openocd\n" \
				"to find what the problem is). Ctrl-C to abort waiting.\n"; \
			until nc -z localhost 4444; do sleep 0.1; done; \
			make -s attach_gdb; \
			kill $$(pgrep -P $$PPID -f openocd)' \; \
		resize-pane -t 1 -y 10

attach_openocd:
	$(OOCD) $(OOCD_FLAGS) -c $(if $(HALT), 'attach ()', 'init')

attach_gdb:
	$(GDB) $(PROJECT).elf -ex 'target extended-remote :3333' -ex 'set remotetimeout 1000' -ex 'monitor adapter speed 2000'


disasm: $(PROJECT).elf
	$(OBJDUMP) -d $(PROJECT).elf

.PHONY: flash reset debug_gdb debug_server disasm