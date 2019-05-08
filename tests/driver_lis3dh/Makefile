include ../Makefile.tests_common

# as default we use the I2C mode
DRIVER ?= lis3dh_i2c

USEMODULE += lis3dh_i2c

FEATURES_REQUIRED += periph_gpio_irq
USEMODULE += lis3dh
USEMODULE += xtimer

FEATURES_REQUIRED += periph_cpuid
FEATURES_REQUIRED += periph_status
FEATURES_REQUIRED += periph_status_extended
FEATURES_REQUIRED += periph_rtc


#QUIET = 0
include $(RIOTBASE)/Makefile.include
