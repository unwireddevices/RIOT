# name of your application
APPLICATION = mirea

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r3

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

USEMODULE += rtctimers-millis

FEATURES_REQUIRED += periph_rtc
FEATURES_REQUIRED += periph_gpio

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include
