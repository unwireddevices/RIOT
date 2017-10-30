# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-relay

CFLAGS += -DHD44780_CYRILLIC
CFLAGS += -DHD44780_PCF8574
USEMODULE += hd44780

############ UMDK MODULES USED #####################
# variable in the list below must match modules'
# subdirectory name in ../../unwired-modules/
# and no other variable should have umdk-% name

umdk-gpio	  = 0
umdk-4btn	  = 0
umdk-gps	  = 0
umdk-lm75	  = 0
umdk-lsm6ds3  = 0
umdk-lmt01	  = 0
umdk-uart	  = 0
umdk-sht21	  = 0
umdk-pir	  = 0
umdk-adc	  = 0
umdk-lps331   = 0
umdk-counter  = 0
umdk-rssiecho = 0
umdk-pwm	  = 0
umdk-opt3001  = 0
umdk-dali	  = 0
umdk-bme280   = 0
umdk-config   = 1
umdk-mhz19    = 0
umdk-range    = 0
umdk-hd44780  = 0

####### UMDK MODULES defines and Makefile.include inclusion ############

define umdk_add_module
ifeq ($($(UMDK_MODULE_NAME)),1)
    USEMODULE += $(UMDK_MODULE_NAME)
endif
endef

$(foreach UMDK_MODULE_NAME, $(filter umdk-%, ${.VARIABLES}), $(eval $(call umdk_add_module, $UMDK_MODULE_NAME)))

UMDK_MODULES_MAKEFILES = $(addsuffix /Makefile.include, $(addprefix ../../unwired-modules/, $(filter umdk-%, ${USEMODULE})))

define umdk_include_makefile
ifneq ("$(wildcard $(FILE))","")
	include $(FILE)
endif
endef

$(foreach FILE, $(UMDK_MODULES_MAKEFILES), $(eval $(call umdk_include_makefile, $FILE)))

################ Other modules to include ################
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

####################################################

# name of your application
APPLICATION = loralan-public

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

FEATURES_REQUIRED = periph_spi periph_gpio

DRIVER ?= sx1276
QUIET ?= 1

CFLAGS += -DDEVELHELP
CFLAGS += -DNO_RIOT_BANNER

####################################################

CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

include $(RIOTBASE)/Makefile.include
