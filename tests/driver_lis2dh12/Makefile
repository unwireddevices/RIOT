include ../Makefile.tests_common

# as default we use the SPI mode for now, as the I2C mode is not supported, yet
DRIVER ?= lis2dh12_i2c


FEATURES_PROVIDED += lptimer

USEMODULE += lptimer
USEMODULE += lis2dh12
USEMODULE += $(DRIVER)

include $(RIOTBASE)/Makefile.include
