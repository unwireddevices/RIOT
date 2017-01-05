# name of your application
APPLICATION = lora-star-uni

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1-r2

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

CFLAGS += -DDEVELHELP

FEATURES_REQUIRED = periph_spi periph_gpio

DRIVER ?= sx1276
QUIET ?= 1

# Modules to include:
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += od
USEMODULE += xtimer
USEMODULE += crypto
USEMODULE += cipher_modes
USEMODULE += random
USEMODULE += hashes
USEMODULE += checksum

USEMODULE += l1-nvram-eeprom
USEMODULE += sx1276

EXTERNAL_MODULE_DIRS += ../../unwired-modules/
USEMODULE += lora-star-mac
USEMODULE += lora-star-end-device
USEMODULE += unwds-common
USEMODULE += rtc-timers

INCLUDES += -I../../unwired-modules/lora-star-mac/include/
INCLUDES += -I../../unwired-modules/lora-star-end-device/include/
INCLUDES += -I../../unwired-modules/lora-star-gate/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/
INCLUDES += -I../../unwired-modules/rtc-timers/include/

############ UNWD MODULES USED #####################

UNWD_GPIO     = 1
UNWD_4BTN     = 1
UNWD_GPS	  =	1
UNWD_LM75     =	1
UNWD_LSM6DS3  =	1
UNWD_LMT01    =	1
UNWD_UART	  =	1
UNWD_SHT21    =	1
UNWD_PIR	  =	1
UNWD_6ADC	  =	1
UNWD_LPS331   = 1
UNWD_4COUNTER = 1
UNWD_RSSIECHO = 1
UNWD_PWM      = 1
UNWD_OPT3001  = 1

####### UNWD MMODULES' includes & flags ############
ifeq ($(UNWD_GPIO),1)
USEMODULE += unwds-gpio
CFLAGS += -D umdk_gpio
endif
INCLUDES += -I../../unwired-modules/unwds-gpio/include/

ifeq ($(UNWD_4BTN),1)
USEMODULE += umdk-4btn
CFLAGS += -D umdk_4btn
endif
INCLUDES += -I../../unwired-modules/umdk-4btn/include/

ifeq ($(UNWD_GPS),1)
USEMODULE += umdk-gps
CFLAGS += -D umdk_gps
endif
INCLUDES += -I../../unwired-modules/umdk-gps/include/

ifeq ($(UNWD_LM75),1)
USEMODULE += umdk-lm75
USEMODULE += lm75a_new
CFLAGS += -D umdk_lm75
endif
INCLUDES += -I../../unwired-modules/umdk-lm75/include/

ifeq ($(UNWD_LSM6DS3),1)
USEMODULE += umdk-lsm6ds3
USEMODULE += printf_float
CFLAGS += -D umdk_lsm6ds3
endif
INCLUDES += -I../../unwired-modules/umdk-lsm6ds3/include/

ifeq ($(UNWD_LMT01),1)
USEMODULE += umdk-lmt01
CFLAGS += -D umdk_lmt01
endif
INCLUDES += -I../../unwired-modules/umdk-lmt01/include/

ifeq ($(UNWD_UART),1)
USEMODULE += umdk-uart
CFLAGS += -D umdk_uart
endif
INCLUDES += -I../../unwired-modules/umdk-uart/include/

ifeq ($(UNWD_SHT21),1)
USEMODULE += umdk-sht21
USEMODULE += sht21
CFLAGS += -D umdk_sht21
endif
INCLUDES += -I../../unwired-modules/umdk-sht21/include/

ifeq ($(UNWD_PIR),1)
USEMODULE += umdk-pir
USEMODULE += pir
CFLAGS += -D umdk_pir
endif
INCLUDES += -I../../unwired-modules/umdk-pir/include/

ifeq ($(UNWD_6ADC),1)
USEMODULE += umdk-6adc
CFLAGS += -D umdk_6adc
endif
INCLUDES += -I../../unwired-modules/umdk-6adc/include/

ifeq ($(UNWD_LPS331),1)
USEMODULE += umdk-lps331
USEMODULE += lps331ap
CFLAGS += -D umdk_lps331
endif
INCLUDES += -I../../unwired-modules/umdk-lps331/include/

ifeq ($(UNWD_4COUNTER),1)
USEMODULE += umdk-4counter
CFLAGS += -D umdk_4counter
endif
INCLUDES += -I../../unwired-modules/umdk-4counter/include/

ifeq ($(UNWD_RSSIECHO),1)
USEMODULE += umdk-rssiecho
CFLAGS += -D umdk_rssiecho
endif
INCLUDES += -I../../unwired-modules/umdk-rssiecho/include/

ifeq ($(UNWD_PWM),1)
USEMODULE += umdk-pwm
CFLAGS += -D umdk_pwm
endif
INCLUDES += -I../../unwired-modules/umdk-pwm/include/

ifeq ($(UNWD_OPT3001),1)
USEMODULE += umdk-opt3001
USEMODULE += opt3001
CFLAGS += -D umdk_opt3001
endif
INCLUDES += -I../../unwired-modules/umdk-opt3001/include/
####################################################


CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

include $(RIOTBASE)/Makefile.include
