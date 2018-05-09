# name of your application
APPLICATION = loralan-gateway

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r3

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

QUIET ?= 1

CFLAGS += -DDEVELHELP
CFLAGS += -DDISPLAY_JOINKEY_2BYTES
CFLAGS += -DCRYPTO_AES
CFLAGS += -DNO_RIOT_BANNER

FEATURES_REQUIRED += periph_gpio
FEATURES_REQUIRED += periph_rtc
FEATURES_REQUIRED += periph_spi
FEATURES_REQUIRED += periph_eeprom
FEATURES_REQUIRED += periph_uart
FEATURES_REQUIRED += periph_pm
FEATURES_REQUIRED += periph_iwdg
FEATURES_REQUIRED += periph_cpuid

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
USEMODULE += sx127x
USEMODULE += rtctimers-millis

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += loralan-mac
USEMODULE += loralan-gateway
USEMODULE += loralan-common
USEMODULE += unwds-common

INCLUDES += -I../../unwired-modules/loralan-mac/include/
INCLUDES += -I../../unwired-modules/loralan-gateway/include/
INCLUDES += -I../../unwired-modules/loralan-common/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

FEATURES_OPTIONAL += config

include $(RIOTBASE)/Makefile.include
