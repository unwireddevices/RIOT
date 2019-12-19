# name of your application
APPLICATION = loralan-gateway

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r3

# Current firmware version
CFLAGS += -DFIRMWARE_VERSION=\"1.99\"

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

QUIET ?= 1

CFLAGS += -DDEVELHELP
CFLAGS += -DDISPLAY_JOINKEY_2BYTES
CFLAGS += -DCRYPTO_AES
CFLAGS += -DNO_RIOT_BANNER
CFLAGS += -DISR_STACKSIZE=1024

FEATURES_REQUIRED += periph_gpio
FEATURES_REQUIRED += periph_spi
FEATURES_REQUIRED += periph_eeprom
FEATURES_REQUIRED += periph_uart
FEATURES_REQUIRED += periph_pm
FEATURES_REQUIRED += periph_iwdg
FEATURES_REQUIRED += periph_cpuid
FEATURES_REQUIRED += periph_status
FEATURES_REQUIRED += periph_status_extended

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
USEMODULE += lptimer

####### Empty modules list as we don't need any modules for the gateway ############

SHELL := /bin/bash
UMDK_MODULES_LIST = $(RIOTBASE)/unwired-modules/include/umdk-modules.h

all: create_empty_module_list

create_empty_module_list:
	@echo "/* DO NOT EDIT! FILE IS AUTO GENERATED */" > $(UMDK_MODULES_LIST); \
	echo "#ifndef UMDK_MODULES_H_" >> $(UMDK_MODULES_LIST); \
	echo -e "#define UMDK_MODULES_H_\n" >> $(UMDK_MODULES_LIST); \
	printf "\nstatic const unwd_module_t modules[] = {\n" >> $(UMDK_MODULES_LIST); \
	printf "{ 0, \"\", NULL, NULL, NULL } };\n" >> $(UMDK_MODULES_LIST); \
	echo -e "\n#endif" >> $(UMDK_MODULES_LIST); \

INCLUDES += -I$(RIOTBASE)/unwired-modules/include/

#################################

DIRS += $(RIOTBASE)/apps/unwds-common/loralan-mac/
DIRS += $(RIOTBASE)/apps/unwds-common/loralan-common/
DIRS += $(RIOTBASE)/apps/unwds-common/loralan-gateway/
DIRS += $(RIOTBASE)/apps/unwds-common/unwds-common/

USEMODULE += loralan-gateway
USEMODULE += loralan-mac
USEMODULE += loralan-common
USEMODULE += unwds-common

INCLUDES += -I$(RIOTBASE)/apps/unwds-common/loralan-mac/include/
INCLUDES += -I$(RIOTBASE)/apps/unwds-common/loralan-common/include/
INCLUDES += -I$(RIOTBASE)/apps/unwds-common/loralan-gateway/include/
INCLUDES += -I$(RIOTBASE)/apps/unwds-common/unwds-common/include/

FEATURES_OPTIONAL += config

include $(RIOTBASE)/Makefile.include
