# name of your application
APPLICATION = skald_eddystone

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-beacon-C2M-r100

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../..

# include Skald
USEMODULE += skald_eddystone

# - advertising interval is configured during compile time, override by setting
#   `CFLAGS+=-DSKALD_INTERVAL=xxx` in microseconds
CFLAGS += -DSKALD_INTERVAL=10000

# - advertising channels are configured during compile time, override by
#   setting `CFLAGS+=-DSKALD_ADV_CHAN={37,39}`
# CFLAGS+=-DSKALD_ADV_CHAN={37,39}


# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
# CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include
