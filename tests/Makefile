VERILATOR ?= verilator
RTLDIR := $(abspath ..)
BUILD_ROOT ?= $(CURDIR)/build
FIR_BUILD_ROOT ?= $(BUILD_ROOT)/fir

.PHONY: test-fir test-nan clean

test-fir:
	mkdir -p $(FIR_BUILD_ROOT)
	$(VERILATOR) --binary --timing -Wno-fatal --top-module r4000_fir_tb \
		-I$(RTLDIR) --Mdir $(FIR_BUILD_ROOT)/async \
		$(CURDIR)/r4000_fir_tb.v $(RTLDIR)/r4000_fpu.v
	$(FIR_BUILD_ROOT)/async/Vr4000_fir_tb
	$(VERILATOR) --binary --timing -Wno-fatal -DR4000_FPU_SYNC_RESET \
		--top-module r4000_fir_tb -I$(RTLDIR) --Mdir $(FIR_BUILD_ROOT)/sync \
		$(CURDIR)/r4000_fir_tb.v $(RTLDIR)/r4000_fpu.v
	$(FIR_BUILD_ROOT)/sync/Vr4000_fir_tb

test-nan:
	mkdir -p $(BUILD_ROOT)/nan
	$(VERILATOR) --binary --timing -Wno-fatal --top-module r4000_nan_tb \
		-I$(RTLDIR) --Mdir $(BUILD_ROOT)/nan \
		$(CURDIR)/r4000_nan_tb.v $(RTLDIR)/r4000_fpu.v
	$(BUILD_ROOT)/nan/Vr4000_nan_tb

clean:
	rm -rf $(BUILD_ROOT)
