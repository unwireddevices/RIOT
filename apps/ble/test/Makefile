# name of your application
APPLICATION = test

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-beacon-C2M-r100

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../..

FEATURES_REQUIRED += periph_pwm

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

CFLAGS += -DNO_RIOT_BANNER

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include
