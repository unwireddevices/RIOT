APPLICATION = lptimer
BOARD = unwd-range-l1-r3

include ../Makefile.tests_common

#DISABLE_MODULE += auto_init

USEMODULE += od
USEMODULE += ps
USEMODULE += lptimer
USEMODULE += xtimer

FEATURES_REQUIRED += periph_rtt
FEATURES_REQUIRED += periph_rtc

#CFLAGS += -DDEVELHELP

include $(RIOTBASE)/Makefile.include
