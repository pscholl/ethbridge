CONTIKI_PROJECT = ethbridge
all: $(CONTIKI_PROJECT)

CONTIKI_TARGET_SOURCEFILES = cdc_dev.c bridge_aux.c

UIP_CONF_IPV6=1
#CFLAGS+=-DJENNIC_CONF_COORDINATOR

CONTIKI = ../contiki/
include $(CONTIKI)/Makefile.include
