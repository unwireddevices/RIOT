# name of your application
APPLICATION = lora-star-uni

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r2

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

USEMODULE += sx1276
USEMODULE += rtctimers

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += lora-star-mac
USEMODULE += lora-star-end-device
USEMODULE += unwds-common

INCLUDES += -I../../unwired-modules/lora-star-mac/include/
INCLUDES += -I../../unwired-modules/lora-star-end-device/include/
INCLUDES += -I../../unwired-modules/lora-star-gate/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

############ UNWD MODULES USED #####################

UNWD_GPIO	  =	1
UNWD_4BTN	  =	1
UNWD_GPS	  =	1
UNWD_LM75	  =	1
UNWD_LSM6DS3  =	1
UNWD_LMT01	  =	1
UNWD_UART	  =	1
UNWD_SHT21	  =	1
UNWD_PIR	  =	1
UNWD_6ADC	  =	1
UNWD_LPS331   =	1
UNWD_4COUNTER =	1
UNWD_RSSIECHO =	1
UNWD_PWM	  =	1
UNWD_OPT3001  =	1
UNWD_DALI	  =	1
UNWD_BME280   = 1

####### UNWD MMODULES' includes & flags ############
ifeq ($(UNWD_GPIO),1)
USEMODULE += umdk-gpio
endif

ifeq ($(UNWD_4BTN),1)
USEMODULE += umdk-4btn
endif

ifeq ($(UNWD_GPS),1)
USEMODULE += umdk-gps
endif

ifeq ($(UNWD_LM75),1)
USEMODULE += umdk-lm75
endif

ifeq ($(UNWD_LSM6DS3),1)
USEMODULE += umdk-lsm6ds3
endif

ifeq ($(UNWD_LMT01),1)
USEMODULE += umdk-lmt01
endif

ifeq ($(UNWD_UART),1)
USEMODULE += umdk-uart
endif

ifeq ($(UNWD_SHT21),1)
USEMODULE += umdk-sht21
endif

ifeq ($(UNWD_PIR),1)
USEMODULE += umdk-pir
endif

ifeq ($(UNWD_6ADC),1)
USEMODULE += umdk-6adc
endif

ifeq ($(UNWD_LPS331),1)
USEMODULE += umdk-lps331
endif

ifeq ($(UNWD_4COUNTER),1)
USEMODULE += umdk-4counter
endif

ifeq ($(UNWD_RSSIECHO),1)
USEMODULE += umdk-rssiecho
endif

ifeq ($(UNWD_PWM),1)
USEMODULE += umdk-pwm
endif

ifeq ($(UNWD_OPT3001),1)
USEMODULE += umdk-opt3001
endif

ifeq ($(UNWD_DALI),1)
USEMODULE += umdk-dali
endif

ifeq ($(UNWD_BME280),1)
USEMODULE += umdk-bme280
endif

UMDK-MODULES-ENABLED = $(addsuffix /Makefile.include, $(addprefix ../../unwired-modules/, $(filter umdk-%, ${USEMODULE})))

define umdk_include_makefile
ifneq ("$(wildcard $(FILE))","")
	include $(FILE)
endif
endef

$(foreach FILE, $(UMDK-MODULES-ENABLED), $(eval $(call umdk_include_makefile, $FILE)))

####################################################


CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

include $(RIOTBASE)/Makefile.include
