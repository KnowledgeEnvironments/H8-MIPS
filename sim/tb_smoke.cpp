#include "Vr4000_top.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>

static void tick(Vr4000_top &top) {
    top.clk = 0;
    top.eval();
    top.clk = 1;
    top.eval();
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vr4000_top top;

    top.rst_n = 0;
    top.imem_rdata = 0;
    top.dmem_rdata = 0;
    top.dmem_ready = 1;
    top.hw_int = 0;
    top.sim_idle_boost = 0;
    top.force_uncached = 0;
    top.snoop_valid = 0;
    top.snoop_addr = 0;
    top.snoop_wdata = 0;
    top.snoop_wmask = 0;
    top.external_hold = 0;
    top.snoop_rd_valid = 0;
    top.snoop_rd_addr = 0;
    top.debug_tlb_idx = 0;

    for (int i = 0; i < 8; ++i)
        tick(top);
    top.rst_n = 1;

    bool fetched = false;
    for (int i = 0; i < 256; ++i) {
        tick(top);
        fetched = fetched || top.imem_ren;
    }

    top.final();
    if (!fetched) {
        std::fprintf(stderr, "FAIL: no instruction fetch after reset\n");
        return 1;
    }

    std::puts("PASS: reset/fetch smoke");
    return 0;
}
