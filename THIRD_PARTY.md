# Third-party inputs

HaterMIPS does not redistribute the following optional verification projects:

- CHERI/BERI MIPS tests
- Berkeley SoftFloat
- Berkeley TestFloat

Users who enable those gates must obtain the projects from their upstream
sources and comply with the licenses distributed with them. The files under
`tests/` are HaterMIPS-side adapters and harnesses; they are not copies of the
upstream suites.

`tests/testfloat/softfloat-mips-legacy.patch` adapts Berkeley SoftFloat 3e
commit `a0c6494cdc11865811dec815d5c0049fba9d82a8` to the R4000 binary32 and
binary64 NaN convention. The adjacent provenance document carries Berkeley's
BSD license notice. Berkeley TestFloat is pinned to commit
`a9c849f1b0eb0264b626d9686ffae167d996e3be`.

No firmware, operating-system image, vendor manual, or generated FPGA IP is
included in this repository.
