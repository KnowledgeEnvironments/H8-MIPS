// tb_fpu_madd.cpp - focused direct tests for COP1X MADD/MSUB paths.

#include <cstdio>
#include <cstdint>
#include <cstdlib>

#include "Vr4000_fpu.h"
#include "verilated.h"

static uint64_t failures = 0;

static void tick(Vr4000_fpu *fpu) {
    fpu->clk = 0;
    fpu->eval();
    Verilated::timeInc(1);
    fpu->clk = 1;
    fpu->eval();
    Verilated::timeInc(1);
    fpu->clk = 0;
    fpu->eval();
    Verilated::timeInc(1);
}

static void reset(Vr4000_fpu *fpu) {
    fpu->clk = 0;
    fpu->rst_n = 0;
    fpu->fr_mode = 1;
    fpu->fs = 0;
    fpu->ft = 0;
    fpu->fd = 0;
    fpu->fr = 0;
    fpu->gpr_wdata = 0;
    fpu->fd_wr = 0;
    fpu->fpr_wdata = 0;
    fpu->fpr_wdata_word = 0;
    fpu->fcsr_wdata = 0;
    fpu->fmt = 0;
    fpu->funct = 0;
    fpu->gpr_rt = 0;
    fpu->gpr_rt_zero = 1;
    fpu->wen = 0;
    fpu->fcsr_wen = 0;
    fpu->wen_early = 0;
    fpu->fd_wr_early = 0;
    fpu->fpr_wdata_early = 0;
    fpu->fpr_wdata_early_word = 0;
    fpu->flags_wen_early = 0;
    fpu->fcsr_wen_early = 0;
    fpu->fcsr_wdata_early = 0;
    fpu->exc_trap_early = 0;
    fpu->fr_mode_wr = 1;
    fpu->do_fpop = 0;
    fpu->madd_op = 0;
    fpu->fpu_cancel = 0;
    fpu->eval();
    tick(fpu);
    fpu->rst_n = 1;
    fpu->eval();
}

static void set_fcsr(Vr4000_fpu *fpu, uint32_t fcsr) {
    fpu->fcsr_wen_early = 1;
    fpu->fcsr_wdata_early = fcsr;
    tick(fpu);
    fpu->fcsr_wen_early = 0;
    fpu->eval();
}

static void write_fpr(Vr4000_fpu *fpu, uint8_t reg, uint64_t value, bool word) {
    fpu->do_fpop = 0;
    fpu->madd_op = 0;
    fpu->wen_early = 1;
    fpu->fd_wr_early = reg & 31;
    fpu->fpr_wdata_early = word ? (uint64_t)(uint32_t)value : value;
    fpu->fpr_wdata_early_word = word ? 1 : 0;
    tick(fpu);
    fpu->wen_early = 0;
    fpu->fpr_wdata_early_word = 0;
    fpu->eval();
}

static void expect(bool cond, const char *name) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", name);
        failures++;
    }
}

static void expect_eq64(uint64_t got, uint64_t expected, const char *name) {
    if (got != expected) {
        std::fprintf(stderr, "FAIL: %s got=%016llx expected=%016llx\n", name,
                     (unsigned long long)got, (unsigned long long)expected);
        failures++;
    }
}

static uint64_t run_madd(Vr4000_fpu *fpu, bool is_double, bool is_msub,
                         uint64_t fs, uint64_t ft, uint64_t fr,
                         uint8_t *flags_out, int *latency_out) {
    write_fpr(fpu, 0, fs, !is_double);
    write_fpr(fpu, 1, ft, !is_double);
    write_fpr(fpu, 3, fr, !is_double);
    fpu->fs = 0;
    fpu->ft = 1;
    fpu->fr = 3;
    fpu->eval();
    if (is_double) {
        expect_eq64(fpu->fs_val, fs, "MADD.D fs operand load");
        expect_eq64(fpu->ft_val, ft, "MADD.D ft operand load");
        expect_eq64(fpu->fr_val, fr, "MADD.D fr operand load");
    } else {
        expect_eq64(fpu->fs_val & UINT64_C(0xffffffff), fs & UINT64_C(0xffffffff), "MADD.S fs operand load");
        expect_eq64(fpu->ft_val & UINT64_C(0xffffffff), ft & UINT64_C(0xffffffff), "MADD.S ft operand load");
        expect_eq64(fpu->fr_val & UINT64_C(0xffffffff), fr & UINT64_C(0xffffffff), "MADD.S fr operand load");
    }
    fpu->fs = 0;
    fpu->ft = 1;
    fpu->fd = 2;
    fpu->fr = 3;
    fpu->fmt = is_double ? 0x11 : 0x10;
    fpu->funct = is_double ? 1 : 0;
    fpu->do_fpop = 0;
    fpu->madd_op = is_msub ? 2 : 1;
    fpu->gpr_rt = 0;
    fpu->gpr_rt_zero = 1;
    fpu->eval();

    int cycles = 0;
    while (!fpu->fpu_result_valid && cycles < 16) {
        tick(fpu);
        cycles++;
    }

    expect(fpu->fpu_result_valid, "MADD result became valid");
    if (latency_out) *latency_out = cycles;
    if (flags_out) *flags_out = fpu->fp_exc_flags;
    uint64_t result = fpu->fp_result;
    fpu->madd_op = 0;
    fpu->eval();
    tick(fpu);
    return result;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    auto fpu = new Vr4000_fpu;
    reset(fpu);

    uint8_t flags = 0;
    int latency = 0;

    expect_eq64(run_madd(fpu, false, false, 0x40000000u, 0x40400000u,
                         0x40800000u, &flags, &latency) & 0xffffffffu,
                0x41200000u, "MADD.S 2*3+4 == 10");
    expect(flags == 0, "MADD.S exact flags");
    expect(latency > 1, "MADD.S is internally multicycle");

    expect_eq64(run_madd(fpu, false, true, 0x40000000u, 0x40400000u,
                         0x40800000u, &flags, nullptr) & 0xffffffffu,
                0x40000000u, "MSUB.S 2*3-4 == 2");
    expect(flags == 0, "MSUB.S exact flags");

    expect_eq64(run_madd(fpu, true, false, UINT64_C(0x4000000000000000),
                         UINT64_C(0x4008000000000000), UINT64_C(0x4010000000000000),
                         &flags, nullptr),
                UINT64_C(0x4024000000000000), "MADD.D 2*3+4 == 10");
    expect(flags == 0, "MADD.D exact flags");

    expect_eq64(run_madd(fpu, true, true, UINT64_C(0x4000000000000000),
                         UINT64_C(0x4008000000000000), UINT64_C(0x4010000000000000),
                         &flags, nullptr),
                UINT64_C(0x4000000000000000), "MSUB.D 2*3-4 == 2");
    expect(flags == 0, "MSUB.D exact flags");

    expect((run_madd(fpu, false, false, 0x3f800000u, 0x3f800000u,
                     0xbf800000u, &flags, nullptr) & 0x7fffffffu) == 0,
           "MADD.S cancellation to zero");
    expect(flags == 0, "MADD.S cancellation flags");

    uint64_t inf_cancel = run_madd(fpu, false, false, 0x7f800000u, 0x40000000u,
                                   0xff800000u, &flags, nullptr);
    expect(((uint32_t)inf_cancel & 0x7f800000u) == 0x7f800000u &&
           ((uint32_t)inf_cancel & 0x007fffffu) != 0,
           "MADD.S inf + -inf returns NaN");
    expect((flags & 0x10) != 0, "MADD.S inf + -inf invalid flag");

    fpu->fs = 0;
    fpu->ft = 1;
    fpu->fr = 3;
    fpu->funct = 0;
    fpu->madd_op = 1;
    fpu->eval();
    tick(fpu);
    fpu->fpu_cancel = 1;
    tick(fpu);
    fpu->fpu_cancel = 0;
    fpu->madd_op = 0;
    fpu->eval();
    expect(!fpu->fpu_busy && !fpu->fpu_result_valid, "MADD cancel clears busy/valid");

    uint64_t back_to_back_a = run_madd(fpu, false, false, 0x40000000u, 0x40000000u,
                                       0x3f800000u, &flags, nullptr);
    uint64_t back_to_back_b = run_madd(fpu, false, false, 0x40400000u, 0x40000000u,
                                       0x3f800000u, &flags, nullptr);
    expect_eq64(back_to_back_a & 0xffffffffu, 0x40a00000u, "back-to-back MADD.S first");
    expect_eq64(back_to_back_b & 0xffffffffu, 0x40e00000u, "back-to-back MADD.S second");

    delete fpu;
    if (failures) {
        std::fprintf(stderr, "%llu MADD focused test(s) failed\n",
                     (unsigned long long)failures);
        return 1;
    }
    std::printf("MADD focused tests passed\n");
    return 0;
}
