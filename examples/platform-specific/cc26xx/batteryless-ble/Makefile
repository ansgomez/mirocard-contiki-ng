CONTIKI ?= ../../../../

CONTIKI_PROJECT = beaconing

TARGET=cc26x0-cc13x0
BOARD=miromico-batteryless/cc2650
CFLAGS += -g

CONTIKI_SOURCEFILES += data.c

PLATFORMS_ONLY = cc26x0-cc13x0

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MAKE_ROUTING = MAKE_ROUTING_NULLROUTING

# BSL_FLAGS += -b 115200

all: $(CONTIKI_PROJECT)

info: $(CONTIKI_PROJECT).elf
	arm-none-eabi-size --format=sysv --radix=10 --totals --common $<

# .PHONY: clean 

include $(CONTIKI)/Makefile.include
