APPLICATION = tele2med_wolf

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# If no BOARD is found in the environment, use this default:
BOARD ?= unwd-railwaycar

# Include packages that pull up and auto-init the link layer.
# NOTE: 6LoWPAN will be included if IEEE802.15.4 devices are present
# USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_ipv6_default
USEMODULE += gnrc_sock_udp

# Add also the shell, some shell commands
USEMODULE += shell
USEMODULE += shell_commands

USEMODULE += simcom

USEMODULE += od

USEPKG += wolfssl
USEMODULE += wolfcrypt
USEMODULE += wolfssl
USEMODULE += wolfcrypt-test
USEMODULE += wolfcrypt_ecc
USEMODULE += wolfssl_dtls

FEATURES_REQUIRED += periph_status
FEATURES_REQUIRED += periph_status_extended

# FIX
FEATURES_REQUIRED += periph_rtt
USEMODULE += lptimer

#CFLAGS+=-g -ggdb3
CFLAGS += -DNO_RIOT_BANNER
CFLAGS += -DDTLS_DEFAULT_PORT=$(DTLS_PORT) -DDTLS_WOLFSSL
CFLAGS += -DSTDIO_UART_BAUDRATE=115200
CFLAGS += -DRECEIVE_MAX_LEN=64

CFLAGS += -DDEBUG_WOLFSSL
CFLAGS += -DMODULE_WOLFCRYPT_RSA
CFLAGS += -DHAVE_APPLICATION_USER_SETTINGS_H

# FIXME: This is a temporary patch taken from the TinyDTLS example to ensure proper stack allocation
CFLAGS += -DTHREAD_STACKSIZE_MAIN=\(3*THREAD_STACKSIZE_DEFAULT\)

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

# May need these for seeding wolfCrypt DRBG
# USEMODULE += random
# USEMODULE += prng_minstd

# USEPKG += wolfssl
# USEMODULE += wolfssl_socket wolfcrypt_rsa wolfcrypt_dh
USEMODULE += wolfcrypt_rsa

include $(RIOTBASE)/Makefile.include

.PHONY: all
all: 
	@echo "File $(APPLICATION).elf copy to flasher"
	@cp $(ELFFILE) flasher/firmware.elf
	@echo "File $(APPLICATION).hex copy to flasher"
	@cp $(HEXFILE) flasher/firmware.hex
