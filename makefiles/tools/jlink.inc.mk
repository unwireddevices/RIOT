export FLASHER = $(RIOTTOOLS)/jlink/jlink.sh
export DEBUGGER = $(RIOTTOOLS)/jlink/jlink.sh
export DEBUGSERVER = $(RIOTTOOLS)/jlink/jlink.sh
export RESET = $(RIOTTOOLS)/jlink/jlink.sh

JLINKHEXFILE = $(BINFILE)

export FFLAGS ?= flash $(JLINKHEXFILE)
export DEBUGGER_FLAGS ?= debug $(ELFFILE)
export DEBUGSERVER_FLAGS ?= debug-server
export RESET_FLAGS ?= reset
