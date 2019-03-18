include ../Makefile.tests_common

USEMODULE += dsp
USEMODULE += ps
CFLAGS += -DFFT_SAMPLE=128


FEATURES_REQUIRED += periph_cpuid
FEATURES_REQUIRED += periph_status
FEATURES_REQUIRED += periph_status_extended
FEATURES_REQUIRED += periph_rtc

include $(RIOTBASE)/Makefile.include
