// tb_fpu_testfloat.cpp — Verilator testbench for r4000_fpu TestFloat verification
// Reads TestFloat vectors from stdin, drives FPU, compares results.
//
// Usage: testfloat_gen <op> [-rnear_even|-rminMag|-rmin|-rmax] [-level 1|2] |
//        ./Vr4000_fpu_tb --op=<op> [--rmode=<0-3>] [--max-fails=N]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "Vr4000_fpu.h"
#include "verilated.h"

static int max_fails = 10;
static int fail_count = 0;
static uint64_t vector_count = 0;
static uint64_t unimpl_count = 0;

enum OpType {
    OP_F32_ADD, OP_F32_SUB, OP_F32_MUL, OP_F32_DIV, OP_F32_SQRT,
    OP_F64_ADD, OP_F64_SUB, OP_F64_MUL, OP_F64_DIV, OP_F64_SQRT,
    OP_F32_TO_F64, OP_F64_TO_F32,
    OP_F32_TO_I32, OP_F32_TO_I64, OP_F64_TO_I32, OP_F64_TO_I64,
    OP_I32_TO_F32, OP_I32_TO_F64, OP_I64_TO_F32, OP_I64_TO_F64,
    OP_F32_TO_UI32, OP_F32_TO_UI64, OP_F64_TO_UI32, OP_F64_TO_UI64,
    OP_UI32_TO_F32, OP_UI32_TO_F64, OP_UI64_TO_F32, OP_UI64_TO_F64,
    OP_F32_EQ, OP_F32_LE, OP_F32_LT,
    OP_F64_EQ, OP_F64_LE, OP_F64_LT,
    OP_F32_EQ_SIGNALING, OP_F32_LE_QUIET, OP_F32_LT_QUIET,
    OP_F64_EQ_SIGNALING, OP_F64_LE_QUIET, OP_F64_LT_QUIET,
    OP_UNKNOWN
};

struct OpInfo {
    const char *name;
    OpType type;
    int n_operands;  // 1 or 2
    int is_f64;      // operand format: 0=f32, 1=f64
    int result_f64;  // result format: 0=f32, 1=f64, 2=i32, 3=i64, 4=bool
    uint8_t fmt;     // MIPS COP1 fmt field (0x10=S, 0x11=D, 0x14=W, 0x15=L)
    uint8_t funct;   // MIPS COP1 funct field
};

// MIPS COP1 function codes
#define FPU_ADD  0x00
#define FPU_SUB  0x01
#define FPU_MUL  0x02
#define FPU_DIV  0x03
#define FPU_SQRT 0x04
#define FPU_ROUND_L 0x08
#define FPU_TRUNC_L 0x09
#define FPU_CEIL_L  0x0A
#define FPU_FLOOR_L 0x0B
#define FPU_ROUND_W 0x0C
#define FPU_TRUNC_W 0x0D
#define FPU_CEIL_W  0x0E
#define FPU_FLOOR_W 0x0F
#define FPU_CVT_S 0x20
#define FPU_CVT_D 0x21
#define FPU_CVT_W 0x24
#define FPU_CVT_L 0x25
// MIPS cond[3:0] = {signaling, less, equal, unordered}
// TestFloat signaling = raises invalid on ANY NaN → MIPS cond[3]=1
// TestFloat quiet     = raises invalid only on sNaN → MIPS cond[3]=0, ordered (no U bit)
// eq       → C.EQ  (0x32): quiet, equal only
// le       → C.LE  (0x3E): signaling, less|equal
// lt       → C.LT  (0x3C): signaling, less
// eq_sig   → C.SEQ (0x3A): signaling, equal
// le_quiet → C.OLE (0x36): quiet, less|equal (ordered)
// lt_quiet → C.OLT (0x34): quiet, less (ordered)

#define COP1_FMT_S 0x10
#define COP1_FMT_D 0x11
#define COP1_FMT_W 0x14
#define COP1_FMT_L 0x15

static const OpInfo ops[] = {
    // Arithmetic f32
    {"f32_add",  OP_F32_ADD,  2, 0, 0, COP1_FMT_S, FPU_ADD},
    {"f32_sub",  OP_F32_SUB,  2, 0, 0, COP1_FMT_S, FPU_SUB},
    {"f32_mul",  OP_F32_MUL,  2, 0, 0, COP1_FMT_S, FPU_MUL},
    {"f32_div",  OP_F32_DIV,  2, 0, 0, COP1_FMT_S, FPU_DIV},
    {"f32_sqrt", OP_F32_SQRT, 1, 0, 0, COP1_FMT_S, FPU_SQRT},
    // Arithmetic f64
    {"f64_add",  OP_F64_ADD,  2, 1, 1, COP1_FMT_D, FPU_ADD},
    {"f64_sub",  OP_F64_SUB,  2, 1, 1, COP1_FMT_D, FPU_SUB},
    {"f64_mul",  OP_F64_MUL,  2, 1, 1, COP1_FMT_D, FPU_MUL},
    {"f64_div",  OP_F64_DIV,  2, 1, 1, COP1_FMT_D, FPU_DIV},
    {"f64_sqrt", OP_F64_SQRT, 1, 1, 1, COP1_FMT_D, FPU_SQRT},
    // Conversions float<->float
    {"f32_to_f64", OP_F32_TO_F64, 1, 0, 1, COP1_FMT_S, FPU_CVT_D},
    {"f64_to_f32", OP_F64_TO_F32, 1, 1, 0, COP1_FMT_D, FPU_CVT_S},
    // Conversions float->int (signed)
    {"f32_to_i32", OP_F32_TO_I32, 1, 0, 2, COP1_FMT_S, FPU_CVT_W},
    {"f32_to_i64", OP_F32_TO_I64, 1, 0, 3, COP1_FMT_S, FPU_CVT_L},
    {"f64_to_i32", OP_F64_TO_I32, 1, 1, 2, COP1_FMT_D, FPU_CVT_W},
    {"f64_to_i64", OP_F64_TO_I64, 1, 1, 3, COP1_FMT_D, FPU_CVT_L},
    // Conversions int->float (signed)
    {"i32_to_f32", OP_I32_TO_F32, 1, 0, 0, COP1_FMT_W, FPU_CVT_S},
    {"i32_to_f64", OP_I32_TO_F64, 1, 0, 1, COP1_FMT_W, FPU_CVT_D},
    {"i64_to_f32", OP_I64_TO_F32, 1, 1, 0, COP1_FMT_L, FPU_CVT_S},
    {"i64_to_f64", OP_I64_TO_F64, 1, 1, 1, COP1_FMT_L, FPU_CVT_D},
    // Comparisons f32 (signaling: eq_signaling raises invalid on any NaN)
    {"f32_eq",   OP_F32_EQ,  2, 0, 4, COP1_FMT_S, 0x32},  // C.EQ (quiet)
    {"f32_le",   OP_F32_LE,  2, 0, 4, COP1_FMT_S, 0x3E},  // C.LE (signaling)
    {"f32_lt",   OP_F32_LT,  2, 0, 4, COP1_FMT_S, 0x3C},  // C.LT (signaling)
    {"f32_eq_signaling", OP_F32_EQ_SIGNALING, 2, 0, 4, COP1_FMT_S, 0x3A},  // C.SEQ
    {"f32_le_quiet",     OP_F32_LE_QUIET,     2, 0, 4, COP1_FMT_S, 0x36},  // C.OLE (quiet)
    {"f32_lt_quiet",     OP_F32_LT_QUIET,     2, 0, 4, COP1_FMT_S, 0x34},  // C.OLT (quiet)
    // Comparisons f64
    {"f64_eq",   OP_F64_EQ,  2, 1, 4, COP1_FMT_D, 0x32},  // C.EQ (quiet)
    {"f64_le",   OP_F64_LE,  2, 1, 4, COP1_FMT_D, 0x3E},  // C.LE (signaling)
    {"f64_lt",   OP_F64_LT,  2, 1, 4, COP1_FMT_D, 0x3C},  // C.LT (signaling)
    {"f64_eq_signaling", OP_F64_EQ_SIGNALING, 2, 1, 4, COP1_FMT_D, 0x3A},  // C.SEQ
    {"f64_le_quiet",     OP_F64_LE_QUIET,     2, 1, 4, COP1_FMT_D, 0x36},  // C.OLE (quiet)
    {"f64_lt_quiet",     OP_F64_LT_QUIET,     2, 1, 4, COP1_FMT_D, 0x34},  // C.OLT (quiet)
    {nullptr, OP_UNKNOWN, 0, 0, 0, 0, 0}
};

static const OpInfo *find_op(const char *name) {
    for (const OpInfo *p = ops; p->name; p++)
        if (strcmp(p->name, name) == 0) return p;
    return nullptr;
}

static void init_inputs(Vr4000_fpu *fpu) {
    fpu->clk = 0;
    fpu->rst_n = 0;
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
    fpu->fr_mode = 1;
    fpu->fr_mode_wr = 1;
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
    fpu->do_fpop = 0;
    fpu->gpr_rt = 0;
    fpu->gpr_rt_zero = 1;
    fpu->madd_op = 0;
    fpu->fpu_cancel = 0;
}

static bool is_multicycle_funct(uint8_t funct) {
    return funct == FPU_ADD || funct == FPU_SUB || funct == FPU_MUL ||
           funct == FPU_DIV || funct == FPU_SQRT ||
           funct == FPU_ROUND_L || funct == FPU_TRUNC_L ||
           funct == FPU_CEIL_L  || funct == FPU_FLOOR_L ||
           funct == FPU_ROUND_W || funct == FPU_TRUNC_W ||
           funct == FPU_CEIL_W  || funct == FPU_FLOOR_W ||
           funct == FPU_CVT_S || funct == FPU_CVT_D ||
           funct == FPU_CVT_W || funct == FPU_CVT_L;
}

// Both TestFloat and MIPS FPU use the same flag bit positions:
// bit 4=invalid(V), bit 3=divzero(Z), bit 2=overflow(O), bit 1=underflow(U), bit 0=inexact(I)
static uint8_t testfloat_flags_to_fpu(uint8_t tf) {
    return tf & 0x1f;
}

static bool is_f32_denorm(uint64_t value) {
    const uint32_t word = static_cast<uint32_t>(value);
    return (word & 0x7f800000u) == 0 && (word & 0x007fffffu) != 0;
}

static bool is_f64_denorm(uint64_t value) {
    return (value & UINT64_C(0x7ff0000000000000)) == 0 &&
           (value & UINT64_C(0x000fffffffffffff)) != 0;
}

static const char *flags_str(uint8_t fpu_flags, int slot = 0) {
    static char bufs[2][6];
    char *buf = bufs[slot & 1];
    buf[0] = (fpu_flags & 0x10) ? 'V' : '.';
    buf[1] = (fpu_flags & 0x08) ? 'Z' : '.';
    buf[2] = (fpu_flags & 0x04) ? 'O' : '.';
    buf[3] = (fpu_flags & 0x02) ? 'U' : '.';
    buf[4] = (fpu_flags & 0x01) ? 'I' : '.';
    buf[5] = 0;
    return buf;
}

int main(int argc, char **argv) {
    const char *op_name = nullptr;
    int rmode = 0;  // 0=RNE, 1=RTZ, 2=RDN(RM), 3=RUP(RP)

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--op=", 5) == 0) op_name = argv[i] + 5;
        else if (strncmp(argv[i], "--rmode=", 8) == 0) rmode = atoi(argv[i] + 8);
        else if (strncmp(argv[i], "--max-fails=", 12) == 0) max_fails = atoi(argv[i] + 12);
        else { fprintf(stderr, "Unknown arg: %s\n", argv[i]); return 1; }
    }
    if (!op_name) { fprintf(stderr, "Usage: %s --op=<opname> [--rmode=0-3] [--max-fails=N]\n", argv[0]); return 1; }

    const OpInfo *op = find_op(op_name);
    if (!op) { fprintf(stderr, "Unknown op: %s\n", op_name); return 1; }

    Verilated::commandArgs(argc, argv);
    auto fpu = new Vr4000_fpu;

    // Reset
    init_inputs(fpu);
    fpu->eval();
    fpu->clk = 1; fpu->eval(); Verilated::timeInc(1);
    fpu->clk = 0; fpu->eval(); Verilated::timeInc(1);
    fpu->rst_n = 1;

    // Initialize FPU state: FR=1 (64-bit FPR mode), FCSR rounding mode
    fpu->fr_mode = 1;
    fpu->fr_mode_wr = 1;

    // Set up FCSR with rounding mode via early write
    uint32_t fcsr_val = (uint32_t)rmode;  // bits [1:0] = rounding mode
    fpu->fcsr_wen_early = 1;
    fpu->fcsr_wdata_early = fcsr_val;
    fpu->wen_early = 0;
    fpu->flags_wen_early = 0;
    fpu->wen = 0;
    fpu->fcsr_wen = 0;
    fpu->do_fpop = 0;
    fpu->madd_op = 0;
    fpu->exc_trap_early = 0;
    fpu->fpr_wdata_early_word = 0;
    fpu->fpu_cancel = 0;
    fpu->clk = 1; fpu->eval(); Verilated::timeInc(1);
    fpu->clk = 0; fpu->eval(); Verilated::timeInc(1);
    fpu->fcsr_wen_early = 0;

    // Read vectors from stdin
    char line[256];
    while (fgets(line, sizeof(line), stdin)) {
        uint64_t op_a = 0, op_b = 0, expected_result = 0;
        uint32_t expected_flags_tf = 0;

        // Parse TestFloat vector
        int n_parsed;
        if (op->n_operands == 2) {
            if (op->is_f64) {
                n_parsed = sscanf(line, "%llx %llx %llx %x",
                    (unsigned long long*)&op_a, (unsigned long long*)&op_b,
                    (unsigned long long*)&expected_result, &expected_flags_tf);
                if (n_parsed < 4) continue;
            } else {
                uint32_t a32, b32, r32;
                n_parsed = sscanf(line, "%x %x %x %x", &a32, &b32, &r32, &expected_flags_tf);
                if (n_parsed < 4) continue;
                op_a = a32; op_b = b32; expected_result = r32;
            }
        } else {
            // Unary ops
            if (op->is_f64 || op->type == OP_I64_TO_F32 || op->type == OP_I64_TO_F64) {
                if (op->result_f64 || op->result_f64 == 3) {
                    n_parsed = sscanf(line, "%llx %llx %x",
                        (unsigned long long*)&op_a, (unsigned long long*)&expected_result, &expected_flags_tf);
                } else {
                    uint32_t r32;
                    n_parsed = sscanf(line, "%llx %x %x",
                        (unsigned long long*)&op_a, &r32, &expected_flags_tf);
                    expected_result = r32;
                }
                if (n_parsed < 3) continue;
            } else {
                if (op->result_f64 == 1 || op->result_f64 == 3) {
                    uint32_t a32;
                    n_parsed = sscanf(line, "%x %llx %x",
                        &a32, (unsigned long long*)&expected_result, &expected_flags_tf);
                    op_a = a32;
                } else {
                    uint32_t a32, r32;
                    n_parsed = sscanf(line, "%x %x %x", &a32, &r32, &expected_flags_tf);
                    op_a = a32; expected_result = r32;
                }
                if (n_parsed < 3) continue;
            }
        }

        uint8_t expected_fpu_flags = testfloat_flags_to_fpu(expected_flags_tf);
        // TestFloat describes IEEE results.  The R4000 instead requires
        // software emulation for denormal operands to computational/convert
        // operations.  Comparisons are the architectural exception; integer
        // source formats cannot encode an FP denormal.
        const bool computational = op->result_f64 != 4;
        const bool expected_r4000_unimpl = computational &&
            (((op->fmt == COP1_FMT_S) &&
              (is_f32_denorm(op_a) ||
               (op->n_operands == 2 && is_f32_denorm(op_b)))) ||
             ((op->fmt == COP1_FMT_D) &&
              (is_f64_denorm(op_a) ||
               (op->n_operands == 2 && is_f64_denorm(op_b)))));

        // This legacy MIPS conversion mode uses 0x7FFFFFFF (i32) /
        // 0x7FFFFFFFFFFFFFFF (i64) as
        // overflow sentinel, not IEEE 754-2008's 0x80000000 / 0x8000000000000000
        if (op->result_f64 == 2 && expected_result == 0x80000000u && (expected_fpu_flags & 0x10))
            expected_result = 0x7FFFFFFFu;
        if (op->result_f64 == 3 && expected_result == UINT64_C(0x8000000000000000) && (expected_fpu_flags & 0x10))
            expected_result = UINT64_C(0x7FFFFFFFFFFFFFFF);

        // Load operands into FPR via early write.
        // fs = reg 0, ft = reg 1
        fpu->wen_early = 1;
        fpu->fd_wr_early = 0;
        fpu->fpr_wdata_early_word = !op->is_f64;
        if (op->is_f64 || op->type == OP_I64_TO_F32 || op->type == OP_I64_TO_F64)
            fpu->fpr_wdata_early = op_a;
        else
            fpu->fpr_wdata_early = (uint64_t)(uint32_t)op_a;
        fpu->clk = 1; fpu->eval(); Verilated::timeInc(1);
        fpu->clk = 0; fpu->eval(); Verilated::timeInc(1);

        if (op->n_operands == 2) {
            fpu->fd_wr_early = 1;
            if (op->is_f64)
                fpu->fpr_wdata_early = op_b;
            else
                fpu->fpr_wdata_early = (uint64_t)(uint32_t)op_b;
            fpu->clk = 1; fpu->eval(); Verilated::timeInc(1);
            fpu->clk = 0; fpu->eval(); Verilated::timeInc(1);
        }
        fpu->wen_early = 0;
        fpu->flags_wen_early = 0;

        // Drive the operation
        fpu->fs = 0;
        fpu->ft = 1;
        fpu->fd = 2;
        fpu->fr = 3;
        fpu->fmt = op->fmt;
        fpu->funct = op->funct;
        fpu->do_fpop = 1;
        fpu->madd_op = 0;
        fpu->gpr_rt = 0;
        fpu->gpr_rt_zero = 1;

        fpu->fpu_cancel = 0;
        fpu->eval();

        // Multicycle ops: clock FSM until result is valid
        if (is_multicycle_funct(op->funct)) {
            fpu->clk = 1; fpu->eval(); Verilated::timeInc(1);
            fpu->clk = 0; fpu->eval(); Verilated::timeInc(1);
            int watchdog = 200;
            while (!fpu->fpu_result_valid && watchdog > 0) {
                fpu->clk = 1; fpu->eval(); Verilated::timeInc(1);
                fpu->clk = 0; fpu->eval(); Verilated::timeInc(1);
                watchdog--;
            }
            if (watchdog == 0) {
                fprintf(stderr, "ERROR: multicycle op timed out\n");
                break;
            }
        }

        // Read results
        uint64_t actual_result;
        if (op->result_f64 == 4) {
            // Comparison: result is 1 bit in fp_result[0]
            actual_result = fpu->fp_result & 1;
        } else if (op->result_f64 == 1 || op->result_f64 == 3) {
            actual_result = fpu->fp_result;
        } else {
            actual_result = (uint32_t)(fpu->fp_result & 0xFFFFFFFF);
        }
        uint8_t actual_flags = fpu->fp_exc_flags;
        const bool actual_unimpl = fpu->fp_exc_unimpl;

        fpu->do_fpop = 0;

        // Advance clock to reset FPU state for next vector
        fpu->clk = 1; fpu->eval(); Verilated::timeInc(1);
        fpu->clk = 0; fpu->eval(); Verilated::timeInc(1);

        vector_count++;

        // Compare
        bool result_match = expected_r4000_unimpl ||
                            (actual_result == expected_result);
        // For float→int (CVT.W/CVT.L): MIPS sets inexact per 1985 spec,
        // but TestFloat generates 2008-style vectors (no inexact).
        // Mask inexact (bit 0) from flag comparison for these ops.
        uint8_t flags_mask = 0x1f;
        if (op->result_f64 == 2 || op->result_f64 == 3)
            flags_mask = 0x1e;
        bool flags_match = expected_r4000_unimpl ? (actual_flags == 0) :
            ((actual_flags & flags_mask) == (expected_fpu_flags & flags_mask));
        const bool unimpl_match = actual_unimpl == expected_r4000_unimpl;
        if (expected_r4000_unimpl && actual_unimpl)
            unimpl_count++;

        if (!result_match || !flags_match || !unimpl_match) {
            fail_count++;
            // Format inputs based on is_f64, results based on result_f64
            char inp_a[20], inp_b[20], got[20], exp[20];
            if (op->is_f64)
                snprintf(inp_a, sizeof(inp_a), "%016llx", (unsigned long long)op_a);
            else
                snprintf(inp_a, sizeof(inp_a), "%08x", (uint32_t)op_a);
            if (op->is_f64)
                snprintf(inp_b, sizeof(inp_b), "%016llx", (unsigned long long)op_b);
            else
                snprintf(inp_b, sizeof(inp_b), "%08x", (uint32_t)op_b);
            if (op->result_f64 == 1 || op->result_f64 == 3) {
                snprintf(got, sizeof(got), "%016llx", (unsigned long long)actual_result);
                snprintf(exp, sizeof(exp), "%016llx", (unsigned long long)expected_result);
            } else {
                snprintf(got, sizeof(got), "%08x", (uint32_t)actual_result);
                snprintf(exp, sizeof(exp), "%08x", (uint32_t)expected_result);
            }
            if (op->n_operands == 2)
                fprintf(stderr, "FAIL #%d vec=%llu: %s %s -> got %s [%s]%s, exp %s [%s]%s%s%s\n",
                    fail_count, (unsigned long long)vector_count,
                    inp_a, inp_b,
                    got, flags_str(actual_flags, 0),
                    actual_unimpl ? " E" : "",
                    exp, flags_str(expected_fpu_flags, 1),
                    result_match ? "" : " RESULT",
                    flags_match ? "" : " FLAGS",
                    unimpl_match ? "" : " UNIMPL");
            else
                fprintf(stderr, "FAIL #%d vec=%llu: %s -> got %s [%s]%s, exp %s [%s]%s%s%s\n",
                    fail_count, (unsigned long long)vector_count,
                    inp_a,
                    got, flags_str(actual_flags, 0),
                    actual_unimpl ? " E" : "",
                    exp, flags_str(expected_fpu_flags, 1),
                        result_match ? "" : " RESULT",
                        flags_match ? "" : " FLAGS",
                        unimpl_match ? "" : " UNIMPL");
            if (fail_count >= max_fails) {
                fprintf(stderr, "Stopped after %d failures (--max-fails=%d)\n", fail_count, max_fails);
                break;
            }
        }
    }

    printf("%s rmode=%d: %llu vectors, %llu R4000 E, %d failures\n",
        op_name, rmode, (unsigned long long)vector_count,
        (unsigned long long)unimpl_count, fail_count);

    delete fpu;
    return fail_count > 0 ? 1 : 0;
}
