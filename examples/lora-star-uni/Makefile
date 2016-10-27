# name of your application
APPLICATION = lora-star-uni

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-range-l1

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

INCLUDES += -I../../unwired-modules/lora-star-mac/include/
INCLUDES += -I../../unwired-modules/lora-star-end-device/include/
INCLUDES += -I../../unwired-modules/lora-star-gate/include/
INCLUDES += -I../../unwired-modules/unwds-common/include/

############ UNWD MODULES USED #####################

UNWD_GPIO   =   1
UNWD_4BTN   =   1
UNWD_GPS	=	1
UNWD_TEMP	=	0
UNWD_LM75A  =	0
UNWD_ACC	=	0
UNWD_LMT01  =	1
UNWD_UART	=	1
UNWD_SHT21  =	1
UNWD_PIR	=	1
UNWD_6ADC	=	1
UNWD_LPS331 =   1

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

ifeq ($(UNWD_TEMP),1)
USEMODULE += umdk-temp
CFLAGS += -D umdk_temp
endif

ifeq ($(UNWD_LM75A),1)
USEMODULE += lm75a_new
USEMODULE += printf_float
CFLAGS += -D umdk_lm75a
endif
INCLUDES += -I../../unwired-modules/umdk-temp/include/

ifeq ($(UNWD_ACC),1)
USEMODULE += umdk-acc
USEMODULE += printf_float
CFLAGS += -D umdk_acc
endif
INCLUDES += -I../../unwired-modules/umdk-acc/include/

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


####################################################


CFLAGS += -DCRYPTO_AES

FEATURES_OPTIONAL += config
FEATURES_OPTIONAL += periph_rtc

include $(RIOTBASE)/Makefile.include
