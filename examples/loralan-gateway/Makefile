# name of your application
APPLICATION = loralan-gateway

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-relay

CFLAGS += -DHD44780_CYRILLIC
CFLAGS += -DHD44780_PCF8574
USEMODULE += hd44780

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

CFLAGS += -DDEVELHELP
CFLAGS += -DGATE_USE_WATCHDOG

CFLAGS += -Wno-unused

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
USEMODULE += rtctimers

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += lora-star-mac
USEMODULE += lora-star-gate
USEMODULE += unwds-common

INCLUDES += -I../../unwired-modules/lora-star-mac/include/
INCLUDES += -I../../unwired-modules/lora-star-gate/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

CFLAGS += -DNO_RIOT_BANNER

include $(RIOTBASE)/Makefile.include
