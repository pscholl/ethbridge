CONTIKI_PROJECT = ethbridge
all: $(CONTIKI_PROJECT)

APPS = ethbridge
UIP_CONF_IPV6=1

CFLAGS+=-DJENNIC_CONF_COORDINATOR

CONTIKI = ../contiki/
include $(CONTIKI)/Makefile.include