APPLICATION = xtimer_rtc_test
include ../Makefile.tests_common

#DISABLE_MODULE += auto_init

USEMODULE += od
USEMODULE += ps
USEMODULE += rtc-timers
USEMODULE += xtimer

USEMODULE += sx1276

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += rtc-timers
INCLUDES += -I../../unwired-modules/rtc-timers/include/

CFLAGS += -DDEVELHELP

include $(RIOTBASE)/Makefile.include
