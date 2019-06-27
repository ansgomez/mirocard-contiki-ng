CONTIKI_PROJECT = transient

CONTIKI_SOURCEFILES += policy.c

PLATFORMS_ONLY = srf06-cc26xx

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MAKE_ROUTING = MAKE_ROUTING_NULLROUTING

# BSL_FLAGS += -b 115200

all: $(CONTIKI_PROJECT)

info: $(CONTIKI_PROJECT).elf
	arm-none-eabi-size --format=sysv --radix=10 --totals --common $<

policy-pool.h:
	python3 generate_policy.py $(POLICY)

policy-clean:
	rm -f policy-pool.h

clean: policy-clean

.PHONY: policy-pool.h

include $(CONTIKI)/Makefile.include
