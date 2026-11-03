.PHONY: help sim test fir nan cheritest testfloat clean

help:
	@printf '%s\n' \
	  'HaterMIPS targets:' \
	  '  make test       self-contained public regression gate' \
	  '  make cheritest  external CHERI/BERI MIPS adapter' \
	  '  make testfloat  external Berkeley TestFloat adapter' \
	  '  make clean      remove local build products'

sim:
	$(MAKE) -C sim smoke

fir:
	$(MAKE) -C tests test-fir

nan:
	$(MAKE) -C tests test-nan

test: sim fir nan

cheritest:
	@test -n "$(CHERITEST_ROOT)" || { echo 'CHERITEST_ROOT is required'; exit 2; }
	@test -n "$(R4000_SIM_BIN)" || { echo 'R4000_SIM_BIN is required'; exit 2; }
	CHERITEST_ROOT="$(CHERITEST_ROOT)" R4000_SIM_BIN="$(R4000_SIM_BIN)" \
		python3 tests/run_cheritest.py --all-raw --all-framework

testfloat:
	@test -n "$(SOFTFLOAT_ROOT)" || { echo 'SOFTFLOAT_ROOT is required'; exit 2; }
	@test -n "$(TESTFLOAT_ROOT)" || { echo 'TESTFLOAT_ROOT is required'; exit 2; }
	$(MAKE) -C tests/testfloat test-fpu \
		SOFTFLOAT_ROOT="$(SOFTFLOAT_ROOT)" TESTFLOAT_ROOT="$(TESTFLOAT_ROOT)"

clean:
	$(MAKE) -C sim clean
	$(MAKE) -C tests clean
	$(MAKE) -C tests/testfloat clean
