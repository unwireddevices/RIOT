# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r3

############ UMDK MODULES USED #####################
# variable in the list below must match modules'
# subdirectory name in ../../unwired-modules/
# and no other variable should have umdk-% name

umdk-gpio	  = 1
umdk-4btn	  = 1
umdk-gps	  = 1
umdk-lm75	  = 1
umdk-lsm6ds3  = 1
umdk-lmt01	  = 1
umdk-uart	  = 1
umdk-sht21	  = 1
umdk-pir	  = 1
umdk-adc	  = 1
umdk-lps331   = 1
umdk-counter  = 1
umdk-rssiecho = 1
umdk-pwm	  = 1
umdk-opt3001  = 1
umdk-dali	  = 1
umdk-bme280   = 1
umdk-config   = 1
umdk-mhz19    = 1

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
APPLICATION = lora-star-uni

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
