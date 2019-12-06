# name of your application
APPLICATION = lorawan

# application-specific modules
USEMODULE += gnrc_netdev_default
USEMODULE += gnrc_lorawan
USEMODULE += gnrc_netapi_callbacks

CFLAGS += -DUNWDS_MAC_LORAWAN

# LoRaWAN packet buffer size
CFLAGS += -DGNRC_PKTBUF_SIZE=512

include ../unwds-common/Makefile.include
