# name of your application
APPLICATION = soilsensor

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-soilsensor-f070

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

CFLAGS += -DNO_RIOT_BANNER
CFLAGS += -DTHREAD_STACKSIZE_MAIN=1536
# CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

FEATURES_REQUIRED += periph_cpuid
FEATURES_REQUIRED += periph_gpio
FEATURES_REQUIRED += periph_timer
FEATURES_REQUIRED += periph_uart
FEATURES_REQUIRED += periph_adc
FEATURES_REQUIRED += periph_flashpage
FEATURES_REQUIRED += periph_status

USEMODULE += xtimer
USEMODULE += checksum
USEMODULE += random
USEMODULE += shell
USEMODULE += shell_commands

include $(RIOTBASE)/Makefile.include
