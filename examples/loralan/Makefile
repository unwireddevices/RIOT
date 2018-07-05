# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r3#unwd-range-l4-r3

############ UMDK MODULES USED #####################
# variable in the list below must match modules'
# subdirectory name in ../../unwired-modules/
# and no other variable should have umdk-% name

umdk-gpio	   = 0
umdk-4btn	   = 1
umdk-gps	   = 0
umdk-lm75	   = 0
umdk-lsm6ds3   = 0
umdk-lmt01	   = 0
umdk-uart	   = 0
umdk-sht21	   = 0
umdk-pir	   = 0
umdk-adc	   = 0
umdk-lps331    = 0
umdk-counter   = 0
umdk-rssiecho  = 0
umdk-pwm	   = 0
umdk-opt3001   = 0
umdk-bme280    = 0
umdk-config    = 0
umdk-mhz19     = 0
umdk-usound    = 0
umdk-hd44780   = 0
umdk-ibutton   = 0
umdk-irblaster = 0
umdk-hx711     = 0
umdk-fdc1004   = 0
umdk-adxl345   = 0
umdk-modbus	   = 0

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
USEMODULE += rtctimers-millis

USEMODULE += sx127x

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += loralan-mac
USEMODULE += loralan-device
USEMODULE += loralan-common
USEMODULE += unwds-common

INCLUDES += -I../../unwired-modules/loralan-mac/include/
INCLUDES += -I../../unwired-modules/loralan-device/include/
INCLUDES += -I../../unwired-modules/loralan-common/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

####################################################

# name of your application
APPLICATION = loralan-public

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

FEATURES_REQUIRED += periph_adc
FEATURES_REQUIRED += periph_dac
FEATURES_REQUIRED += periph_gpio
FEATURES_REQUIRED += periph_i2c
FEATURES_REQUIRED += periph_pwm
FEATURES_REQUIRED += periph_rtc
FEATURES_REQUIRED += periph_spi
FEATURES_REQUIRED += periph_eeprom
FEATURES_REQUIRED += periph_uart
FEATURES_REQUIRED += periph_pm
FEATURES_REQUIRED += periph_iwdg
FEATURES_REQUIRED += periph_cpuid

QUIET ?= 1

CFLAGS += -DDEVELHELP
CFLAGS += -DNO_RIOT_BANNER

####################################################

CFLAGS += -DAES_NO_DECRYPTION
CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

include $(RIOTBASE)/Makefile.include
