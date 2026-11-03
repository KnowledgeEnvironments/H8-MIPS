# HaterMIPS FPU verification against separately obtained Berkeley tools.

RTLDIR ?= $(abspath ../..)
THIRD_PARTY_ROOT ?= $(RTLDIR)/third_party
SOFTFLOAT_ROOT ?= $(THIRD_PARTY_ROOT)/berkeley-softfloat-3
TESTFLOAT_ROOT ?= $(THIRD_PARTY_ROOT)/berkeley-testfloat-3
BUILD_ROOT ?= $(CURDIR)/build
TOOL_BUILD_ROOT ?= $(BUILD_ROOT)/tools

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  PLATFORM = macOS-GCC
else
  PLATFORM = Linux-x86_64-GCC
endif

SOFTFLOAT_BUILD = $(TOOL_BUILD_ROOT)/softfloat
TESTFLOAT_BUILD = $(TOOL_BUILD_ROOT)/testfloat
SOFTFLOAT_STAGE = $(TOOL_BUILD_ROOT)/softfloat-source
SOFTFLOAT_MIPS_PATCH = $(CURDIR)/softfloat-mips-legacy.patch
TESTFLOAT_GEN = $(TESTFLOAT_BUILD)/testfloat_gen
VERILATOR ?= verilator
OBJ_DIR = $(BUILD_ROOT)/obj_dir
FPU_EXE = $(OBJ_DIR)/Vr4000_fpu
MADD_OBJ_DIR = $(BUILD_ROOT)/obj_dir_madd
MADD_EXE = $(MADD_OBJ_DIR)/Vr4000_fpu_madd

RTL_SRCS = $(RTLDIR)/r4000_fpu.v
RTL_INCS = $(RTLDIR)/r4000_defs.vh

VERILATOR_FLAGS = --cc --exe --build -Wall \
  -Wno-UNUSED -Wno-UNDRIVEN -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
  -Wno-LATCH -Wno-CASEINCOMPLETE -Wno-CASEX -Wno-UNOPTFLAT \
  --x-initial 0 --top-module r4000_fpu -DR4000_FPU_SYNC_RESET \
  $(EXTRA_VERILATOR_FLAGS) -I$(RTLDIR)

.PHONY: all verify-inputs prepare-softfloat build-softfloat build-testfloat build-fpu \
	test-fpu test-madd clean

all: test-fpu

verify-inputs:
	@test -f "$(SOFTFLOAT_ROOT)/source/include/softfloat.h" || \
		{ echo 'SOFTFLOAT_ROOT does not contain Berkeley SoftFloat'; exit 2; }
	@test -d "$(SOFTFLOAT_ROOT)/source/8086-SSE" || \
		{ echo 'SoftFloat 3e 8086-SSE specialization is required'; exit 2; }
	@test -d "$(TESTFLOAT_ROOT)/source" || \
		{ echo 'TESTFLOAT_ROOT does not contain Berkeley TestFloat'; exit 2; }
	@test -f "$(SOFTFLOAT_ROOT)/build/$(PLATFORM)/Makefile"
	@test -f "$(TESTFLOAT_ROOT)/build/$(PLATFORM)/Makefile"

$(SOFTFLOAT_STAGE)/.mips-legacy-ready: $(SOFTFLOAT_MIPS_PATCH) | verify-inputs
	rm -rf $(SOFTFLOAT_STAGE)
	mkdir -p $(SOFTFLOAT_STAGE)
	cp -R "$(SOFTFLOAT_ROOT)/source" $(SOFTFLOAT_STAGE)/source
	rm -rf $(SOFTFLOAT_STAGE)/source/MIPS-legacy
	cp -R $(SOFTFLOAT_STAGE)/source/8086-SSE \
		$(SOFTFLOAT_STAGE)/source/MIPS-legacy
	cd $(SOFTFLOAT_STAGE) && patch -p1 < $(SOFTFLOAT_MIPS_PATCH)
	touch $@

prepare-softfloat: $(SOFTFLOAT_STAGE)/.mips-legacy-ready

$(SOFTFLOAT_BUILD)/Makefile: | verify-inputs
	mkdir -p $(SOFTFLOAT_BUILD)
	cp $(SOFTFLOAT_ROOT)/build/$(PLATFORM)/Makefile \
	   $(SOFTFLOAT_ROOT)/build/$(PLATFORM)/platform.h $(SOFTFLOAT_BUILD)/

$(TESTFLOAT_BUILD)/Makefile: | verify-inputs
	mkdir -p $(TESTFLOAT_BUILD)
	cp $(TESTFLOAT_ROOT)/build/$(PLATFORM)/Makefile \
	   $(TESTFLOAT_ROOT)/build/$(PLATFORM)/platform.h $(TESTFLOAT_BUILD)/

build-softfloat: $(SOFTFLOAT_BUILD)/Makefile prepare-softfloat
	$(MAKE) -C $(SOFTFLOAT_BUILD) SOURCE_DIR=$(SOFTFLOAT_STAGE)/source \
		SPECIALIZE_TYPE=MIPS-legacy

build-testfloat: build-softfloat $(TESTFLOAT_BUILD)/Makefile
	$(MAKE) -C $(TESTFLOAT_BUILD) testfloat_gen \
		SOURCE_DIR=$(TESTFLOAT_ROOT)/source \
		SOFTFLOAT_DIR=$(SOFTFLOAT_STAGE) \
		SOFTFLOAT_LIB=$(SOFTFLOAT_BUILD)/softfloat.a \
		SPECIALIZE_TYPE=MIPS-legacy

build-fpu: $(FPU_EXE)

$(FPU_EXE): tb_fpu_testfloat.cpp $(RTL_SRCS) $(RTL_INCS)
	mkdir -p $(BUILD_ROOT)
	rm -rf $(OBJ_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --Mdir $(OBJ_DIR) \
		-o Vr4000_fpu $(RTL_SRCS) $(abspath tb_fpu_testfloat.cpp)

$(MADD_EXE): tb_fpu_madd.cpp $(RTL_SRCS) $(RTL_INCS)
	mkdir -p $(BUILD_ROOT)
	rm -rf $(MADD_OBJ_DIR)
	$(VERILATOR) $(VERILATOR_FLAGS) --Mdir $(MADD_OBJ_DIR) \
		-o Vr4000_fpu_madd $(RTL_SRCS) $(abspath tb_fpu_madd.cpp)

test-madd: $(MADD_EXE)
	@$(MADD_EXE)

test-fpu: build-testfloat $(FPU_EXE)
	@sh run_all.sh $(TESTFLOAT_GEN) $(FPU_EXE)

clean:
	rm -rf $(BUILD_ROOT)
