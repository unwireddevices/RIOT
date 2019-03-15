# include ../Makefile.tests_common

# name of your application
APPLICATION = test-nrf-nfc

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-nrf

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

CFLAGS += -DNO_RIOT_BANNER
CFLAGS += -DCONFIG_GPIO_AS_PINRESET

# USEMODULE += periph_nfc
FEATURES_REQUIRED = periph_nfc

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include

.PHONY: all
all: 
	@$(OBJCOPY) $(OFLAGS) -Oihex $(ELFFILE) bin/$(BOARD)/nrf-nfc.hex