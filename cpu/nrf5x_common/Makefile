MODULE = cpu_common
DIRS = periph

ifneq (,$(filter periph_eeprom,$(FEATURES_REQUIRED)))
  USEMODULE += periph_flashpage
endif

# build one of the radio drivers, if enabled
ifneq (,$(filter nrfble,$(USEMODULE)))
  DIRS += radio/nrfble
endif

ifneq (,$(filter nrfmin,$(USEMODULE)))
  DIRS += radio/nrfmin
endif

ifneq (,$(filter nrfmax,$(USEMODULE)))
  DIRS += radio/nrfmax
endif

include $(RIOTBASE)/Makefile.base
