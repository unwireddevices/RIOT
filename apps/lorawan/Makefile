# name of the application
APPLICATION = lorawan

# application-specific modules
USEMODULE += gnrc_netdev_default
USEMODULE += gnrc_lorawan
USEMODULE += gnrc_netapi_callbacks

# needed by unwds-common
CFLAGS += -DUNWDS_MAC_LORAWAN

# GNRC LoRaWAN packet buffer size
CFLAGS += -DGNRC_PKTBUF_SIZE=512

# other application settings here:
include ../unwds-common/Makefile.include
