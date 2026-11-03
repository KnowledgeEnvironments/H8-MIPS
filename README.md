# HaterMIPS

<p align="center">
  <img src="assets/hmips.png" alt="HaterMIPS project logo" width="627">
</p>

HaterMIPS is a synthesizable, big-endian R4000-class CPU implemented in
Verilog. It includes an eight-stage integer pipeline, CP0/TLB support, an
IEEE-754 floating-point unit, primary instruction and data caches, and a
secondary-cache controller.

HaterMIPS is proven in complete systems across multiple FPGA vendors. It runs
at 100 MHz on Xilinx Kintex UltraScale and at 50 MHz on Altera Cyclone IV GX,
booting and running IRIX on both.

The name is a nod to the Unix-Haters tradition.
HaterMIPS is an independent open-source implementation and is not affiliated
with or endorsed by MIPS Tech, LLC.

The integration top is `r4000_top`. The RTL has no firmware or operating
system dependency.

## Verification and demonstrated systems

HaterMIPS is a deployed workstation CPU, not only an ISA demonstration. Its
integer pipeline, CP0, TLB, FPU, primary caches, secondary cache, exception
paths, and DMA-coherence interfaces have been exercised independently and as
part of complete systems:

- The recorded CHERI/BERI-derived MIPS regression passes all 483 applicable
  architectural tests with zero failures or errors out of 522 discovered
  tests. The remainder are explicitly classified as CHERI, BERI-platform, or
  otherwise inapplicable to an R4000.
- Berkeley SoftFloat/TestFloat comparison passes all 92 applicable
  operation/rounding-mode groups with zero failures. The other 36 generated
  groups are redundant comparison operations for rounding modes that cannot
  affect their result. Separate fixtures check R4000 legacy NaN encoding,
  signs, payloads, operand precedence, quieting, exception flags, multiply-add
  behavior, and synchronous/asynchronous FPU reset.
- On physical CYAN hardware, stock SGI IP20 diagnostics pass the complete TLB
  data/probe/translation/valid/modified/PID/global/cached sequence, the UTLB
  miss exception test, the non-parity instruction-cache test, and all 14 FPU
  subtests through the infinite-series test. Cache parity and secondary-cache
  ECC diagnostics are excluded because the system does not yet include parity logic.
- The core boots and runs IRIX 5.3 in the complete
  [CYAN](../CYAN) workstation, including
  multiuser services, networking, storage and DMA activity, and the X11
  desktop and applications.
- On the Xilinx KCU105's Kintex UltraScale FPGA, the routed CPU/cache island
  meets a 10 ns constraint with positive setup and hold slack and runs at
  100 MHz in complete CYAN images. The same RTL has also been implemented on
  the Altera Cyclone IV GX EP4CGX150: a timing-clean 25 MHz system and a
  demonstrated 50 MHz IRIX system with positive CPU-clock slack.

These are complementary checks: architectural vectors and floating-point
oracles catch local semantic errors, vendor diagnostics exercise authentic
R4000/cache lifecycles, and IRIX operation stresses the integrated CPU under
real interrupts, exceptions, DMA, virtual memory, and application workloads.

## Design intent

HaterMIPS deliberately retains an R4000-style primary instruction/data cache
and secondary-cache hierarchy, precise exception behavior, a full TLB and
CP0, and coherent external-DMA interactions. A smaller FPGA-oriented cache,
shallower memory system, or less faithful pipeline would probably use fewer
resources and reach a higher clock frequency. That was not the design goal.
The project prioritizes architectural and system fidelity—enough to run the
original diagnostics and operating-system workloads—while still achieving
practical FPGA performance.

## Requirements

- Verilator 5 or newer
- A C++ compiler
- GNU Make
- Python 3 for optional external test adapters

## Quick start

```sh
make test
```

The default gate elaborates and runs a reset/fetch smoke test and checks the
FPU FIR/reset interface in asynchronous- and synchronous-reset configurations.

## RTL

The synthesis sources are:

- `r4000_top.v`
- `r4000_core.v`
- `r4000_alu.v`
- `r4000_muldiv.v`
- `r4000_fpu.v`
- `r4000_tlb.v`
- `r4000_icache.v`
- `r4000_dcache.v`
- `r4000_scache.v`
- `r4000_defs.vh`

`r4000_top` exposes physical instruction and data interfaces, hardware
interrupts, DMA coherence ports, and optional machine-friendly retirement
signals. Integrators select the secondary-cache geometry with the
`SCACHE_LINES` parameter.

## External verification adapters

The repository does not vendor third-party test suites.

To run the CHERI/BERI MIPS tests, obtain the upstream suite separately and
provide both its source root and a compatible HaterMIPS simulator executable:

```sh
CHERITEST_ROOT=/path/to/cheritest/tests \
R4000_SIM_BIN=/path/to/Vr4000_top \
make cheritest
```

To run Berkeley TestFloat against the FPU, obtain Berkeley SoftFloat and
TestFloat separately:

```sh
SOFTFLOAT_ROOT=/path/to/berkeley-softfloat-3 \
TESTFLOAT_ROOT=/path/to/berkeley-testfloat-3 \
make testfloat
```

The adapter is pinned to SoftFloat commit
`a0c6494cdc11865811dec815d5c0049fba9d82a8` and TestFloat commit
`a9c849f1b0eb0264b626d9686ffae167d996e3be`. It stages the supplied SoftFloat
tree in the local build directory and applies the included BSD-licensed
binary32/binary64 MIPS-legacy NaN patch; the upstream checkout is not changed.
NaN results, payloads, signs, precedence, and exception flags are compared
exactly.
See [THIRD_PARTY.md](THIRD_PARTY.md) for dependency and licensing notes.

## Integration

HaterMIPS is maintained independently from CYAN. CYAN pins an exact HaterMIPS
commit at `rtl/cpu`; the CPU repository does not depend on CYAN board or system
files.

## License

HaterMIPS is released under the MIT License. See [LICENSE](LICENSE).

Copyright (c) 2026 I. Finder <src@lispm.net>
