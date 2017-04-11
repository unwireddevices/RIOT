# name of your application
APPLICATION = lora-star-uni-gate

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r3

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

CFLAGS += -DDEVELHELP
CFLAGS += -DGATE_USE_WATCHDOG

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

USEMODULE += sx1276

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += lora-star-mac
USEMODULE += lora-star-gate

INCLUDES += -I../../unwired-modules/lora-star-mac/include/
INCLUDES += -I../../unwired-modules/lora-star-gate/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

CFLAGS += -DNO_RIOT_BANNER

include $(RIOTBASE)/Makefile.include
