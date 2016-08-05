# name of your application
APPLICATION = lora-star-simple

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
USEMODULE += config
USEMODULE += od
USEMODULE += xtimer
USEMODULE += crypto
USEMODULE += cipher_modes
USEMODULE += random
USEMODULE += hashes

USEMODULE += sx1276

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += lora-star-mac
USEMODULE += lora-star-end-device
USEMODULE += unwds-common

INCLUDES += -I../../unwired-modules/lora-star-mac/include/
INCLUDES += -I../../unwired-modules/lora-star-end-device/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

############ UNWD MODULES USED #####################

#USEMODULE += unwds-gpio
#CFLAGS += -DUNWDS_GPIO
INCLUDES += -I../../unwired-modules/unwds-gpio/include/

#USEMODULE += umdk-4btn
#CFLAGS += -DUMDK_4BTN
INCLUDES += -I../../unwired-modules/umdk-4btn/include/

USEMODULE += umdk-gps
CFLAGS += -DUMDK_GPS
INCLUDES += -I../../unwired-modules/umdk-gps/include/

USEMODULE += umdk-temp
USEMODULE += printf_float
CFLAGS += -DUMDK_TEMP
INCLUDES += -I../../unwired-modules/umdk-temp/include/



####################################################

CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

include $(RIOTBASE)/Makefile.include
