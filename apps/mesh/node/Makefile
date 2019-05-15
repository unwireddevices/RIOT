# name of your application
APPLICATION = unwds_dag
CFLAGS += -DUNWDS_DAG
CFLAGS += -DUMDK_LIT

CFLAGS += -DNO_RIOT_BANNER
CFLAGS += -DCONFIG_GPIO_AS_PINRESET

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-nrf

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../../..

# Include common files
INCLUDES += -I../
DIRS += ../
USEMODULE += unwds_networking_common

CFLAGS += -DAES_NO_DECRYPTION
CFLAGS += -DCRYPTO_AES
USEMODULE += crypto
USEMODULE += cipher_modes
USEMODULE += checksum

FEATURES_REQUIRED += periph_rtt
FEATURES_REQUIRED += periph_rtc
							 
############ UMDK MODULES USED #####################
# variable in the list below must match modules'
# subdirectory name in ../../unwired-modules/
# and no other variable should have umdk-% name

# umdk-gpio	   = 0 
# umdk-4btn	   = 0 
# umdk-gps	   = 0 
# umdk-lm75	   = 0 
# umdk-lsm6ds3   = 0 
# umdk-lmt01	   = 0 
# umdk-uart	   = 0 
# umdk-sht21	   = 0 
# umdk-pir	   = 0 
# umdk-adc	   = 0 
# umdk-lps331    = 0 
# umdk-counter   = 0 
# umdk-rssiecho  = 0 
# umdk-pwm	   = 0 
# umdk-opt3001   = 1 
# umdk-bme280    = 0 
# umdk-config    = 0 
# umdk-mhz19     = 0 
# umdk-usound    = 0 
# umdk-hd44780   = 0 
# umdk-ibutton   = 0 
# umdk-irblaster = 0 
# umdk-hx711     = 0 
# umdk-fdc1004   = 0 
# umdk-adxl345   = 0 
# umdk-modbus	   = 0 

####### UMDK MODULES defines and Makefile.include inclusion ############

# define umdk_add_module
# ifeq ($($(UMDK_MODULE_NAME)),1)
    # USEMODULE += $(UMDK_MODULE_NAME)
# endif
# endef

# $(foreach UMDK_MODULE_NAME, $(filter umdk-%, ${.VARIABLES}), $(eval $(call umdk_add_module, $UMDK_MODULE_NAME)))

# UMDK_MODULES_MAKEFILES = $(addsuffix /Makefile.include, $(addprefix ../../../unwired-modules/, $(filter umdk-%, ${USEMODULE})))

# define umdk_include_makefile
# ifneq ("$(wildcard $(FILE))","")
	# include $(FILE)
# endif
# endef

# $(foreach FILE, $(UMDK_MODULES_MAKEFILES), $(eval $(call umdk_include_makefile, $FILE)))










# Include packages that pull up and auto-init the link layer.
# NOTE: 6LoWPAN will be included if IEEE802.15.4 devices are present
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_udp
# Add a routing protocol
USEMODULE += gnrc_rpl
USEMODULE += auto_init_gnrc_rpl
# This application dumps received packets to STDIO using the pktdump module
USEMODULE += gnrc_pktdump
# Additional networking modules that can be dropped if not needed
USEMODULE += gnrc_icmpv6_echo
# Add also the shell, some shell commands
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += netstats_l2
USEMODULE += netstats_ipv6
USEMODULE += netstats_rpl

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

INCLUDES += -I$(RIOTBASE)/unwired-modules/include/

# Uncomment the following 2 lines to specify static link lokal IPv6 address
# this might be useful for testing, in cases where you cannot or do not want to
# run a shell with ifconfig to get the real link lokal address.
#IPV6_STATIC_LLADDR ?= '"fe80::cafe:cafe:cafe:1"'
#CFLAGS += -DGNRC_IPV6_STATIC_LLADDR=$(IPV6_STATIC_LLADDR)

# Uncomment this to join RPL DODAGs even if DIOs do not contain
# DODAG Configuration Options (see the doc for more info)
# CFLAGS += -DGNRC_RPL_DODAG_CONF_OPTIONAL_ON_JOIN

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include

# Set a custom channel if needed
ifneq (,$(filter cc110x,$(USEMODULE)))          # radio is cc110x sub-GHz
  DEFAULT_CHANNEL ?= 0
  CFLAGS += -DCC110X_DEFAULT_CHANNEL=$(DEFAULT_CHANNEL)
else
  ifneq (,$(filter at86rf212b,$(USEMODULE)))    # radio is IEEE 802.15.4 sub-GHz
    DEFAULT_CHANNEL ?= 5
    FLAGS += -DIEEE802154_DEFAULT_SUBGHZ_CHANNEL=$(DEFAULT_CHANNEL)
  else                                          # radio is IEEE 802.15.4 2.4 GHz
    DEFAULT_CHANNEL ?= 26
    CFLAGS += -DIEEE802154_DEFAULT_CHANNEL=$(DEFAULT_CHANNEL)
  endif
endif

.PHONY: all
all: 
	@echo "File $(APPLICATION).elf copy to ../flasher"
	@cp $(ELFFILE) ../flasher
	@$(OBJCOPY) $(OFLAGS) -Oihex $(ELFFILE) ../flasher/unwds_dag.hex