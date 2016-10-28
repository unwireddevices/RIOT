# name of your application
APPLICATION = lora-ptp

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

CFLAGS += -DDEVELHELP

FEATURES_REQUIRED = periph_spi periph_gpio

DRIVER ?= sx1276
QUIET ?= 1

# Modules to include:
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += od
USEMODULE += xtimer
USEMODULE += crypto
USEMODULE += cipher_modes
USEMODULE += random
USEMODULE += hashes
USEMODULE += checksum

USEMODULE += l1-nvram-eeprom
USEMODULE += sx1276

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += lora-star-mac
USEMODULE += lora-ptp

INCLUDES += -I../../unwired-modules/lora-star-mac/include/
INCLUDES += -I../../unwired-modules/lora-ptp/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

include $(RIOTBASE)/Makefile.include
