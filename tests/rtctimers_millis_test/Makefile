APPLICATION = rtctimers_millis_test
BOARD = unwd-range-l1-r3

include ../Makefile.tests_common

#DISABLE_MODULE += auto_init

USEMODULE += od
USEMODULE += ps
USEMODULE += rtctimers
USEMODULE += rtctimers-millis
USEMODULE += xtimer
USEMODULE += pm_layered

FEATURES_REQUIRED += periph_rtt
FEATURES_REQUIRED += periph_rtc
FEATURES_REQUIRED += periph_pm

#CFLAGS += -DDEVELHELP

include $(RIOTBASE)/Makefile.include
