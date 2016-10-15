# name of your application
APPLICATION = lora-star-uni

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
USEMODULE += lora-star-end-device
USEMODULE += unwds-common

INCLUDES += -I../../unwired-modules/lora-star-mac/include/
INCLUDES += -I../../unwired-modules/lora-star-end-device/include/
INCLUDES += -I../../unwired-modules/lora-star-gate/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

############ UNWD MODULES USED #####################

USEMODULE += unwds-gpio
INCLUDES += -I../../unwired-modules/unwds-gpio/include/

USEMODULE += umdk-4btn
INCLUDES += -I../../unwired-modules/umdk-4btn/include/

USEMODULE += umdk-gps
INCLUDES += -I../../unwired-modules/umdk-gps/include/

#USEMODULE += umdk-temp
#USEMODULE += lm75a_new
USEMODULE += printf_float
INCLUDES += -I../../unwired-modules/umdk-temp/include/

#USEMODULE += umdk-acc
INCLUDES += -I../../unwired-modules/umdk-acc/include/
USEMODULE += printf_float

USEMODULE += umdk-lmt01
INCLUDES += -I../../unwired-modules/umdk-lmt01/include/

USEMODULE += umdk-uart
INCLUDES += -I../../unwired-modules/umdk-uart/include/

USEMODULE += umdk-sht21
USEMODULE += sht21
INCLUDES += -I../../unwired-modules/umdk-sht21/include/

USEMODULE += umdk-pir
USEMODULE += pir
INCLUDES += -I../../unwired-modules/umdk-pir/include/

USEMODULE += umdk-6adc
INCLUDES += -I../../unwired-modules/umdk-6adc/include/


####################################################


CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

include $(RIOTBASE)/Makefile.include
