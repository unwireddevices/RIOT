include ../Makefile.tests_common

USEMODULE += dsp_fft
USEMODULE += dsp_math
USEMODULE += ps
CFLAGS += -DFFT_SAMPLE=256


FEATURES_REQUIRED += periph_cpuid
FEATURES_REQUIRED += periph_status
FEATURES_REQUIRED += periph_status_extended

include $(RIOTBASE)/Makefile.include
