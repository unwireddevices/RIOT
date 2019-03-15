# define the module that is build
MODULE = cpu

# add a list of subdirectories, that should also be build
DIRS = periph $(RIOTCPU)/cortexm_common $(RIOTCPU)/nrf5x_common

# (file triggers compiler bug. see #5775)
SRC_NOLTO += vectors.c

ifneq (,$(filter nrf52_nfc,$(USEMODULE)))
  DIRS += nfc/
endif


include $(RIOTBASE)/Makefile.base
