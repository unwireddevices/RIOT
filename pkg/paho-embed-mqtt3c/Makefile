PKG_NAME=paho-embed-mqtt3c
PKG_URL=https://github.com/eclipse/paho.mqtt.embedded-c.git
PKG_VERSION=1e7c3371142985a282bc1089ce4385297eb53640
PKG_LICENSE=EDL

.PHONY: all

all: git-download
	"$(MAKE)" -C $(PKG_BUILDDIR)/MQTTPacket/src -f $(CURDIR)/Makefile.paho-embed-mqtt3c

include $(RIOTBASE)/pkg/pkg.mk