VERILATOR ?= verilator
BUILD_ROOT ?= $(CURDIR)/build

RTL_SRCS = ../r4000_top.v \
           ../r4000_core.v \
           ../r4000_alu.v \
           ../r4000_muldiv.v \
           ../r4000_fpu.v \
           ../r4000_tlb.v \
           ../r4000_icache.v \
           ../r4000_dcache.v \
           ../r4000_scache.v

SMOKE_DIR = $(BUILD_ROOT)/smoke
SMOKE_EXE = $(SMOKE_DIR)/Vr4000_top
COMMON_FLAGS = --cc --exe --build -Wall -Wno-fatal \
	-Wno-UNUSED -Wno-UNDRIVEN --x-initial 0 -I..

.PHONY: all smoke clean

all: smoke

$(SMOKE_EXE): $(RTL_SRCS) ../r4000_defs.vh tb_smoke.cpp
	mkdir -p $(SMOKE_DIR)
	$(VERILATOR) $(COMMON_FLAGS) --top-module r4000_top \
		--Mdir $(SMOKE_DIR) $(RTL_SRCS) $(abspath tb_smoke.cpp)

smoke: $(SMOKE_EXE)
	$(SMOKE_EXE)

clean:
	rm -rf $(BUILD_ROOT)
