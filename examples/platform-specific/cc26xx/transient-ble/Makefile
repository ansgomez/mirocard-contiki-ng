CONTIKI ?= ../../../../

CONTIKI_PROJECT = transient

CONTIKI_SOURCEFILES += policy.c data.c haar.c

PLATFORMS_ONLY = srf06-cc26xx

MAKE_MAC = MAKE_MAC_NULLMAC
MAKE_NET = MAKE_NET_NULLNET
MAKE_ROUTING = MAKE_ROUTING_NULLROUTING

# BSL_FLAGS += -b 115200

POLICY := policies/pool_deterministic10.pkl

all: policy-pool $(CONTIKI_PROJECT)

info: $(CONTIKI_PROJECT).elf
	arm-none-eabi-size --format=sysv --radix=10 --totals --common $<

policy-pool:
	python3 generate_policy.py $(POLICY)

policy-clean:
	rm -f policy-pool.h

clean: policy-clean

# .PHONY: clean policy-clean policy-pool

include $(CONTIKI)/Makefile.include
