# name of your application
APPLICATION = pawn-test

# If no BOARD is found in the environment, use this default:
BOARD ?= native

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += config
USEMODULE += od
USEMODULE += xtimer

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

############ UNWD MODULES USED #####################
EXTERNAL_MODULE_DIRS += ../../unwired-modules/

USEMODULE += unwd-pawn
INCLUDES += -I../../unwired-modules/unwd-pawn/

####################################################

include $(RIOTBASE)/Makefile.include
