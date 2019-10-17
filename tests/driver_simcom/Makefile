APPLICATION = simcom_test

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-railwaycar

USEMODULE += lptimer
USEMODULE += simcom
USEMODULE += od

FEATURES_REQUIRED += periph_rtt

#CFLAGS+=-g -ggdb3
CFLAGS += -DNO_RIOT_BANNER
CFLAGS += -DSTDIO_UART_BAUDRATE=115200
CFLAGS += -DRECEIVE_MAX_LEN=64

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include

.PHONY: all
all: 
	@echo "File $(APPLICATION).elf copy to flasher"
	@cp $(ELFFILE) flasher/firmware.elf
	@echo "File $(APPLICATION).hex copy to flasher"
	@cp $(HEXFILE) flasher/firmware.hex
