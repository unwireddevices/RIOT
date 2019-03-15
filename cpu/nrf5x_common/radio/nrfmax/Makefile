MODULE = nrfmax

SRC = nrfmax.c
SRC += nrfmax_getset.c

ifneq (,$(filter gnrc_netdev_default,$(USEMODULE)))
  SRC += nrfmax_gnrc.c
endif

include $(RIOTBASE)/Makefile.base
