// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_fpu.v — R4000SC Floating Point Unit (CP1)
// Implements MIPS III FPU: 32x64-bit register file, FCSR, data transfer,
// arithmetic (ADD/SUB/MUL/DIV/SQRT), comparisons (C.cond), conversions,
// ABS/NEG/MOV/MOVZ/MOVN.
//
// The core drives this module from the RF/EX stage. Simple operations return
// combinationally; arithmetic multicycle operations assert fpu_result_valid
// when the registered result and flags are ready.

`default_nettype none
`include "r4000_defs.vh"

/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off LATCH */

`ifdef R4000_FPU_SYNC_RESET
`define R4000_FPU_SEQ always @(posedge clk)
`else
`define R4000_FPU_SEQ always @(posedge clk or negedge rst_n)
`endif

module r4000_fpu (
    input  wire        clk,
    input  wire        rst_n,

    // Control — late write (WB stage, for loads)
    input  wire        wen,           // Write result to FPR (at commit)
    input  wire        fcsr_wen,      // Write FCSR (CTC1/CMP)

    // Control — early write (RF stage, speculative)
    input  wire        wen_early,     // Write FPR early (MTC1, arithmetic)
    input  wire [ 4:0] fd_wr_early,   // Early write destination
    input  wire [63:0] fpr_wdata_early, // Early write data
    input  wire        fpr_wdata_early_word, // Early write is word-sized
    input  wire        flags_wen_early, // Early arithmetic FCSR flag update
    input  wire        fcsr_wen_early, // Early FCSR write (CTC1, CMP)
    input  wire [31:0] fcsr_wdata_early, // Early FCSR data
    input  wire        exc_trap_early, // FPU exception trapping — write cause only

    // FR mode (Status[26]): 1=32x64-bit FPRs, 0=16x64-bit pairs (D32 mode)
    input  wire        fr_mode,
    input  wire        fr_mode_wr,

    // Register file access
    input  wire [ 4:0] fs,            // Source register 1 index
    input  wire [ 4:0] ft,            // Source register 2 index
    input  wire [ 4:0] fd,            // Destination register index
    input  wire [ 4:0] fr,            // Source register 3 index (MADD/MSUB)
    output wire [63:0] fs_val,        // Source 1 value
    output wire [63:0] ft_val,        // Source 2 value
    output wire [63:0] fr_val,        // Source 3 value (MADD/MSUB)

    // Data transfer (MFC1/MTC1/DMFC1/DMTC1/CFC1/CTC1)
    input  wire [63:0] gpr_wdata,     // Data from GPR (for MTC1/DMTC1/CTC1)
    input  wire [ 4:0] fd_wr,         // FPR destination for writes (may differ from fd)
    input  wire [63:0] fpr_wdata,     // Data to write to FPR (from op result or GPR)
    input  wire        fpr_wdata_word,// Write only lower 32 bits (MTC1, LWC1)

    // FCSR access
    input  wire [31:0] fcsr_wdata,    // For CTC1
    output wire [31:0] fcsr_rdata,    // For CFC1
    output wire        fcc0,          // FP condition code 0 (for BC1T/BC1F)
    output wire [7:0]  fcc_bits_out,  // All 8 FCC bits (for MOVCI)

    // FIR (read-only)
    output wire [31:0] fir_rdata,

    // FP operation interface
    input  wire [ 4:0] fmt,           // Format (S/D/W/L)
    input  wire [ 5:0] funct,         // Function code
    input  wire        do_fpop,       // Perform FP operation
    input  wire [ 4:0] gpr_rt,        // GPR rt value for MOVZ/MOVN condition
    input  wire        gpr_rt_zero,   // GPR[rt] == 0
    input  wire [ 2:0] madd_op,       // MADD operation: 0=none, 4=MADD, 5=MSUB, 6=NMADD, 7=NMSUB

    // Results
    output reg  [63:0] fp_result,     // Operation result
    output reg         fp_exc,        // FP exception raised
    output reg         fp_exc_unimpl, // Unimplemented operation
    output reg  [4:0]  fp_exc_flags,  // {V,Z,O,U,I} exception flags

    // Multicycle operation interface
    output wire        fpu_busy,         // multicycle op in progress
    output wire        fpu_result_valid, // result ready on fp_result/fp_exc_flags
    input  wire        fpu_cancel        // kill in-flight op (pipeline flush)
);

`ifdef FPGA_FPU_DISABLE_ADDSUB
    localparam FPU_HAS_ADDSUB = 1'b0;
`else
    localparam FPU_HAS_ADDSUB = 1'b1;
`endif

`ifdef FPGA_FPU_DISABLE_MUL
    localparam FPU_HAS_MUL = 1'b0;
`else
    localparam FPU_HAS_MUL = 1'b1;
`endif

`ifdef FPGA_FPU_DISABLE_DIV
    localparam FPU_HAS_DIV = 1'b0;
`else
    localparam FPU_HAS_DIV = 1'b1;
`endif

`ifdef FPGA_FPU_DISABLE_SQRT
    localparam FPU_HAS_SQRT = 1'b0;
`else
    localparam FPU_HAS_SQRT = 1'b1;
`endif

`ifdef FPGA_FPU_DISABLE_CVT
    localparam FPU_HAS_CVT = 1'b0;
`else
    localparam FPU_HAS_CVT = 1'b1;
`endif

`ifdef FPGA_FPU_DISABLE_CMP
    localparam FPU_HAS_CMP = 1'b0;
`else
    localparam FPU_HAS_CMP = 1'b1;
`endif

    // ---------------------------------------------------------------
    // FP Register File — 32 x 64-bit
    // ---------------------------------------------------------------
    reg [63:0] fpr [0:31];

    // FR=0 (D32 mode): odd-numbered registers access the upper 32 bits of
    // the paired even register. Swap halves so bits [31:0] contain the value
    // that single-precision and MFC1 operations expect.
    assign fs_val = (!fr_mode && fs[0]) ?
        {fpr[{fs[4:1], 1'b0}][31:0], fpr[{fs[4:1], 1'b0}][63:32]} : fpr[fs];
    assign ft_val = (!fr_mode && ft[0]) ?
        {fpr[{ft[4:1], 1'b0}][31:0], fpr[{ft[4:1], 1'b0}][63:32]} : fpr[ft];
    assign fr_val = (!fr_mode && fr[0]) ?
        {fpr[{fr[4:1], 1'b0}][31:0], fpr[{fr[4:1], 1'b0}][63:32]} : fpr[fr];

    function [63:0] fpr_merge_write;
        input [63:0] old_val;
        input [4:0]  fd_in;
        input [63:0] data_in;
        input        word_in;
        input        fr_mode_in;
        begin
            if (word_in && !fr_mode_in) begin
                if (fd_in[0])
                    fpr_merge_write = {data_in[31:0], old_val[31:0]};
                else
                    fpr_merge_write = {old_val[63:32], data_in[31:0]};
            end else if (word_in) begin
                fpr_merge_write = {{32{data_in[31]}}, data_in[31:0]};
            end else if (!fr_mode_in && fd_in[0]) begin
                fpr_merge_write = {data_in[31:0], old_val[31:0]};
            end else begin
                fpr_merge_write = data_in;
            end
        end
    endfunction

    wire [4:0] fpr_early_wr_idx = (!fr_mode && fd_wr_early[0]) ?
        {fd_wr_early[4:1], 1'b0} : fd_wr_early;
    wire [4:0] fpr_late_wr_idx = (!fr_mode_wr && fd_wr[0]) ?
        {fd_wr[4:1], 1'b0} : fd_wr;
    wire       fpr_early_wen = wen_early && !exc_trap_early;
    // ---------------------------------------------------------------
    // FCSR — FP Control/Status Register (CP1 register 31)
    // ---------------------------------------------------------------
    reg [31:0] fcsr;
    assign fcsr_rdata = fcsr;
    assign fcc0 = fcsr[`FCSR_FCC0];

    // FCCR ($25): condition codes — bits [7:0] = {FCC7..FCC0}
    // FCCR is a view into FCSR: bit0 = FCSR[23], bits7:1 = FCSR[31:25]
    wire [7:0] fcc_bits = {fcsr[31:25], fcsr[23]};
    assign fcc_bits_out = fcc_bits;

    // FIR — FP Implementation Register (CP1 register 0, read-only)
    assign fir_rdata = `FIR_R4000;

    // ---------------------------------------------------------------
    // Rounding mode
    // ---------------------------------------------------------------
    wire [1:0] rm = fcsr[`FCSR_RM];
    wire flush_denorms = fcsr[`FCSR_FS];

    function fp32_is_denorm;
        input [31:0] value;
        begin
            fp32_is_denorm = (value[30:23] == 8'd0) &&
                             (value[22:0] != 23'd0);
        end
    endfunction

    function fp64_is_denorm;
        input [63:0] value;
        begin
            fp64_is_denorm = (value[62:52] == 11'd0) &&
                             (value[51:0] != 52'd0);
        end
    endfunction

    // Effective rounding mode: ROUND/CEIL/FLOOR/TRUNC override FCSR
    reg [1:0] eff_rm;
    always @(*) begin
        case (funct)
            `FPU_ROUND_W, `FPU_ROUND_L: eff_rm = `RM_RN;
            `FPU_TRUNC_W, `FPU_TRUNC_L: eff_rm = `RM_RZ;
            `FPU_CEIL_W,  `FPU_CEIL_L:  eff_rm = `RM_RP;
            `FPU_FLOOR_W, `FPU_FLOOR_L: eff_rm = `RM_RM;
            default:                     eff_rm = rm;
        endcase
    end

    // Shift-right with guard/sticky accumulation for underflow normalization.
    // This replaces long one-bit-at-a-time combinational loops in the rounders.
    function [25:0] shrjam24;
        input [23:0] man_in;
        input        guard_in;
        input        sticky_in;
        input [5:0]  shift;
        reg   [23:0] guard_mask;
        reg   [23:0] lower_mask;
        reg   [23:0] man_out;
        reg          guard_out;
        reg          sticky_out;
        begin
            guard_mask = 24'd0;
            lower_mask = 24'd0;
            if (shift == 6'd0) begin
                man_out = man_in;
                guard_out = guard_in;
                sticky_out = sticky_in;
            end else if (shift <= 6'd24) begin
                guard_mask = 24'd1 << (shift - 6'd1);
                lower_mask = guard_mask - 24'd1;
                man_out = man_in >> shift;
                guard_out = |(man_in & guard_mask);
                sticky_out = sticky_in | guard_in | (|(man_in & lower_mask));
            end else begin
                man_out = 24'd0;
                guard_out = 1'b0;
                sticky_out = sticky_in | guard_in | (|man_in);
            end
            shrjam24 = {man_out, guard_out, sticky_out};
        end
    endfunction

    function [54:0] shrjam53;
        input [52:0] man_in;
        input        guard_in;
        input        sticky_in;
        input [6:0]  shift;
        reg   [52:0] guard_mask;
        reg   [52:0] lower_mask;
        reg   [52:0] man_out;
        reg          guard_out;
        reg          sticky_out;
        begin
            guard_mask = 53'd0;
            lower_mask = 53'd0;
            if (shift == 7'd0) begin
                man_out = man_in;
                guard_out = guard_in;
                sticky_out = sticky_in;
            end else if (shift <= 7'd53) begin
                guard_mask = 53'd1 << (shift - 7'd1);
                lower_mask = guard_mask - 53'd1;
                man_out = man_in >> shift;
                guard_out = |(man_in & guard_mask);
                sticky_out = sticky_in | guard_in | (|(man_in & lower_mask));
            end else begin
                man_out = 53'd0;
                guard_out = 1'b0;
                sticky_out = sticky_in | guard_in | (|man_in);
            end
            shrjam53 = {man_out, guard_out, sticky_out};
        end
    endfunction

    function [0:0] lzc2;
        input [1:0] value;
        begin
            lzc2 = value[1] ? 1'd0 : 1'd1;
        end
    endfunction

    function [1:0] lzc3;
        input [2:0] value;
        begin
            if (value[2])
                lzc3 = 2'd0;
            else if (value[1])
                lzc3 = 2'd1;
            else
                lzc3 = 2'd2;
        end
    endfunction

    function [1:0] lzc4;
        input [3:0] value;
        begin
            if (|value[3:2])
                lzc4 = {1'b0, lzc2(value[3:2])};
            else
                lzc4 = {1'b1, lzc2(value[1:0])};
        end
    endfunction

    function [2:0] lzc5;
        input [4:0] value;
        begin
            if (value[4])
                lzc5 = 3'd0;
            else
                lzc5 = 3'd1 + {1'b0, lzc4(value[3:0])};
        end
    endfunction

    function [2:0] lzc7;
        input [6:0] value;
        begin
            if (|value[6:3])
                lzc7 = {1'b0, lzc4(value[6:3])};
            else
                lzc7 = 3'd4 + {1'b0, lzc3(value[2:0])};
        end
    endfunction

    function [2:0] lzc8;
        input [7:0] value;
        begin
            if (|value[7:4])
                lzc8 = {1'b0, lzc4(value[7:4])};
            else
                lzc8 = {1'b1, lzc4(value[3:0])};
        end
    endfunction

    function [3:0] lzc16;
        input [15:0] value;
        begin
            if (|value[15:8])
                lzc16 = {1'b0, lzc8(value[15:8])};
            else
                lzc16 = {1'b1, lzc8(value[7:0])};
        end
    endfunction

    function [4:0] lzc24;
        input [23:0] value;
        begin
            if (|value[23:16])
                lzc24 = {2'b00, lzc8(value[23:16])};
            else if (|value[15:8])
                lzc24 = 5'd8 + {2'b00, lzc8(value[15:8])};
            else
                lzc24 = 5'd16 + {2'b00, lzc8(value[7:0])};
        end
    endfunction

    function [4:0] lzc26;
        input [25:0] value;
        begin
            if (|value[25:10])
                lzc26 = {1'b0, lzc16(value[25:10])};
            else if (|value[9:2])
                lzc26 = 5'd16 + {2'b00, lzc8(value[9:2])};
            else if (value[1])
                lzc26 = 5'd24;
            else
                lzc26 = 5'd25;
        end
    endfunction

    function [4:0] lzc32;
        input [31:0] value;
        begin
            if (|value[31:16])
                lzc32 = {1'b0, lzc16(value[31:16])};
            else
                lzc32 = {1'b1, lzc16(value[15:0])};
        end
    endfunction

    function [5:0] lzc53;
        input [52:0] value;
        begin
            if (|value[52:37])
                lzc53 = {2'b00, lzc16(value[52:37])};
            else if (|value[36:21])
                lzc53 = 6'd16 + {2'b00, lzc16(value[36:21])};
            else if (|value[20:5])
                lzc53 = 6'd32 + {2'b00, lzc16(value[20:5])};
            else if (|value[4:0])
                lzc53 = 6'd48 + {3'b000, lzc5(value[4:0])};
            else
                lzc53 = 6'd52;
        end
    endfunction

    function [5:0] lzc55;
        input [54:0] value;
        begin
            if (|value[54:39])
                lzc55 = {2'b00, lzc16(value[54:39])};
            else if (|value[38:23])
                lzc55 = 6'd16 + {2'b00, lzc16(value[38:23])};
            else if (|value[22:7])
                lzc55 = 6'd32 + {2'b00, lzc16(value[22:7])};
            else if (|value[6:0])
                lzc55 = 6'd48 + {3'b000, lzc7(value[6:0])};
            else
                lzc55 = 6'd54;
        end
    endfunction

    function [5:0] lzc64;
        input [63:0] value;
        begin
            if (|value[63:32])
                lzc64 = {1'b0, lzc32(value[63:32])};
            else
                lzc64 = {1'b1, lzc32(value[31:0])};
        end
    endfunction

    // ---------------------------------------------------------------
    // IEEE 754 field extraction helpers
    // ---------------------------------------------------------------
    // Single precision: 1 sign + 8 exp + 23 mantissa
    wire        s_sign_a = fs_val[31];
    wire [ 7:0] s_exp_a  = fs_val[30:23];
    wire [22:0] s_man_a  = fs_val[22:0];
    wire        s_sign_b = ft_val[31];
    wire [ 7:0] s_exp_b  = ft_val[30:23];
    wire [22:0] s_man_b  = ft_val[22:0];

    // Double precision: 1 sign + 11 exp + 52 mantissa
    wire        d_sign_a = fs_val[63];
    wire [10:0] d_exp_a  = fs_val[62:52];
    wire [51:0] d_man_a  = fs_val[51:0];
    wire        d_sign_b = ft_val[63];
    wire [10:0] d_exp_b  = ft_val[62:52];
    wire [51:0] d_man_b  = ft_val[51:0];

    // Classification
    wire s_is_nan_a  = (s_exp_a == 8'hFF) && (s_man_a != 0);
    wire s_is_nan_b  = (s_exp_b == 8'hFF) && (s_man_b != 0);
    wire s_is_inf_a  = (s_exp_a == 8'hFF) && (s_man_a == 0);
    wire s_is_inf_b  = (s_exp_b == 8'hFF) && (s_man_b == 0);
    wire s_is_denorm_a = (s_exp_a == 0) && (s_man_a != 0);
    wire s_is_denorm_b = (s_exp_b == 0) && (s_man_b != 0);
    // When flush_denorms (FS=1), treat denorms as zero
    wire s_is_zero_a = (s_exp_a == 0) && (s_man_a == 0 || flush_denorms);
    wire s_is_zero_b = (s_exp_b == 0) && (s_man_b == 0 || flush_denorms);
    // R4000 legacy NaN convention: MSB of mantissa = 1 → SNaN, MSB = 0 → QNaN
    wire s_is_snan_a = s_is_nan_a && s_man_a[22];
    wire s_is_snan_b = s_is_nan_b && s_man_b[22];
    wire s_is_qnan_a = s_is_nan_a && !s_man_a[22];

    wire d_is_nan_a  = (d_exp_a == 11'h7FF) && (d_man_a != 0);
    wire d_is_nan_b  = (d_exp_b == 11'h7FF) && (d_man_b != 0);
    wire d_is_inf_a  = (d_exp_a == 11'h7FF) && (d_man_a == 0);
    wire d_is_inf_b  = (d_exp_b == 11'h7FF) && (d_man_b == 0);
    wire d_is_denorm_a = (d_exp_a == 0) && (d_man_a != 0);
    wire d_is_denorm_b = (d_exp_b == 0) && (d_man_b != 0);
    wire d_is_zero_a = (d_exp_a == 0) && (d_man_a == 0 || flush_denorms);
    wire d_is_zero_b = (d_exp_b == 0) && (d_man_b == 0 || flush_denorms);

    // R4000 legacy NaN convention: MSB of mantissa = 1 → SNaN, MSB = 0 → QNaN
    wire d_is_snan_a = d_is_nan_a && d_man_a[51];
    wire d_is_snan_b = d_is_nan_b && d_man_b[51];
    wire d_is_qnan_a = d_is_nan_a && !d_man_a[51];

    // Default QNaN values (R4000 legacy: MSB=0 is quiet)
    wire [31:0] s_default_qnan = 32'h7FBFFFFF;  // MIPS default QNaN
    wire [63:0] d_default_qnan = 64'h7FF7FFFFFFFFFFFF;

    // Legacy MIPS uses the opposite quiet/signaling discriminator from
    // IEEE-754-2008: a clear top fraction bit is quiet.  Merely clearing the
    // bit of a minimal signaling NaN would create infinity, so canonicalize
    // that one degenerate payload to the architectural default quiet NaN.
    function [31:0] quiet_legacy_nan32;
        input [31:0] value;
        reg [31:0] quieted;
        begin
            quieted = {value[31], 8'hFF, 1'b0, value[21:0]};
            quiet_legacy_nan32 = (quieted[21:0] == 22'd0) ?
                32'h7FBFFFFF : quieted;
        end
    endfunction

    function [63:0] quiet_legacy_nan64;
        input [63:0] value;
        reg [63:0] quieted;
        begin
            quieted = {value[63], 11'h7FF, 1'b0, value[50:0]};
            quiet_legacy_nan64 = (quieted[50:0] == 51'd0) ?
                64'h7FF7FFFFFFFFFFFF : quieted;
        end
    endfunction

    // NaN propagation: return first NaN operand, quieted
    // R4000 legacy: quiet = MSB cleared to 0. QNaN (MSB=0) propagated unchanged.
    wire [31:0] s_prop_nan = quiet_legacy_nan32(
        s_is_nan_a ? fs_val[31:0] : ft_val[31:0]);
    wire [63:0] d_prop_nan = quiet_legacy_nan64(
        d_is_nan_a ? fs_val : ft_val);

    // Registered ADD/SUB operands. 50 MHz builds cannot afford a direct RF
    // instruction/FPR-register path into the align/add stage.
    wire        add_s_sign_a = add_fs_val_r[31];
    wire [ 7:0] add_s_exp_a  = add_fs_val_r[30:23];
    wire [22:0] add_s_man_a  = add_fs_val_r[22:0];
    wire        add_s_sign_b = add_ft_val_r[31];
    wire [ 7:0] add_s_exp_b  = add_ft_val_r[30:23];
    wire [22:0] add_s_man_b  = add_ft_val_r[22:0];
    wire        add_d_sign_a = add_fs_val_r[63];
    wire [10:0] add_d_exp_a  = add_fs_val_r[62:52];
    wire [51:0] add_d_man_a  = add_fs_val_r[51:0];
    wire        add_d_sign_b = add_ft_val_r[63];
    wire [10:0] add_d_exp_b  = add_ft_val_r[62:52];
    wire [51:0] add_d_man_b  = add_ft_val_r[51:0];

    wire add_s_is_nan_a = (add_s_exp_a == 8'hFF) && (add_s_man_a != 0);
    wire add_s_is_nan_b = (add_s_exp_b == 8'hFF) && (add_s_man_b != 0);
    wire add_s_is_inf_a = (add_s_exp_a == 8'hFF) && (add_s_man_a == 0);
    wire add_s_is_inf_b = (add_s_exp_b == 8'hFF) && (add_s_man_b == 0);
    wire add_s_is_snan_a = add_s_is_nan_a && add_s_man_a[22];
    wire add_s_is_snan_b = add_s_is_nan_b && add_s_man_b[22];
    wire add_d_is_nan_a = (add_d_exp_a == 11'h7FF) && (add_d_man_a != 0);
    wire add_d_is_nan_b = (add_d_exp_b == 11'h7FF) && (add_d_man_b != 0);
    wire add_d_is_inf_a = (add_d_exp_a == 11'h7FF) && (add_d_man_a == 0);
    wire add_d_is_inf_b = (add_d_exp_b == 11'h7FF) && (add_d_man_b == 0);
    wire add_d_is_snan_a = add_d_is_nan_a && add_d_man_a[51];
    wire add_d_is_snan_b = add_d_is_nan_b && add_d_man_b[51];

    wire add_s_eff_sign_b = add_s_sign_b ^ add_funct_r[0];
    wire add_d_eff_sign_b = add_d_sign_b ^ add_funct_r[0];
    wire [31:0] add_s_prop_nan = quiet_legacy_nan32(
        add_s_is_nan_a ? add_fs_val_r[31:0] : add_ft_val_r[31:0]);
    wire [63:0] add_d_prop_nan = quiet_legacy_nan64(
        add_d_is_nan_a ? add_fs_val_r : add_ft_val_r);

    // ---------------------------------------------------------------
    // Single/double-precision ADD/SUB results (driven by FSM + rounding logic)
    // ---------------------------------------------------------------
    reg [31:0] s_addnorm_result;
    reg        s_addnorm_roundable;
    reg        s_addnorm_sign;
    reg [8:0]  s_addnorm_exp;
    reg [25:0] s_addnorm_man;
    reg        s_addnorm_sticky;
    reg        s_addnorm_exc_invalid;
    reg        s_addnorm_exc_overflow;
    reg        s_addnorm_exc_underflow;
    reg        s_addnorm_exc_inexact;

    reg [63:0] d_addnorm_result;
    reg        d_addnorm_roundable;
    reg        d_addnorm_sign;
    reg [11:0] d_addnorm_exp;
    reg [54:0] d_addnorm_man;
    reg        d_addnorm_sticky;
    reg        d_addnorm_exc_invalid;
    reg        d_addnorm_exc_overflow;
    reg        d_addnorm_exc_underflow;
    reg        d_addnorm_exc_inexact;

    reg [31:0] s_addsub_result;
    reg        s_addsub_exc_invalid;
    reg        s_addsub_exc_overflow;
    reg        s_addsub_exc_underflow;
    reg        s_addsub_exc_inexact;

    reg [63:0] d_addsub_result;
    reg        d_addsub_exc_invalid;
    reg        d_addsub_exc_overflow;
    reg        d_addsub_exc_underflow;
    reg        d_addsub_exc_inexact;

    wire s_eff_sign_b = s_sign_b ^ funct[0];
    wire d_eff_sign_b = d_sign_b ^ funct[0];

    // ---------------------------------------------------------------
    // ADD/SUB FSM — pipeline plus final result register
    // Cycle 1 (IDLE): capture op/control/operands from RF/FPR outputs
    // Cycle 2 (START): classify, swap, align, add/sub -> register man_sum
    // Cycle 3 (DONE): combinational LZC + normalize, latch pre-round state
    // Cycle 4 (ROUND): combinational round, latch result
    // Cycle 5 (RESULT): result/flags valid
    // ---------------------------------------------------------------
    localparam [2:0] ADD_IDLE   = 3'd0,
                     ADD_START  = 3'd1,
                     ADD_DONE   = 3'd2,
                     ADD_ROUND  = 3'd3,
                     ADD_RESULT = 3'd4;

    reg [2:0]          add_state;
    reg [ 4:0]         add_fmt_r;
    reg [ 5:0]         add_funct_r;
    reg [63:0]         add_fs_val_r;
    reg [63:0]         add_ft_val_r;
    reg                add_is_double;
    reg                add_sign_r;
    reg [11:0]         add_exp_r;
    reg [55:0]         add_man_sum;
    reg                add_do_sub;
    reg                add_sticky;
    reg [1:0]          add_rm_r;
    reg                add_flush_denorms_r;
    reg                add_special;
    reg [63:0]         add_special_result;
    reg [4:0]          add_special_exc;
    reg                add_norm_roundable_r;
    reg                add_norm_sign_r;
    reg [11:0]         add_norm_exp_r;
    reg [54:0]         add_norm_man_r;
    reg                add_norm_sticky_r;
    reg [63:0]         add_norm_result_r;
    reg                add_norm_exc_invalid_r;
    reg                add_norm_exc_overflow_r;
    reg                add_norm_exc_underflow_r;
    reg                add_norm_exc_inexact_r;
    reg [63:0]         add_result_r;
    reg                add_result_exc_invalid;
    reg                add_result_exc_overflow;
    reg                add_result_exc_underflow;
    reg                add_result_exc_inexact;

    `R4000_FPU_SEQ begin
        if (!rst_n) begin
            add_state          <= ADD_IDLE;
            add_fmt_r          <= 5'd0;
            add_funct_r        <= 6'd0;
            add_fs_val_r       <= 64'd0;
            add_ft_val_r       <= 64'd0;
            add_is_double      <= 1'b0;
            add_sign_r         <= 1'b0;
            add_exp_r          <= 12'd0;
            add_man_sum        <= 56'd0;
            add_do_sub         <= 1'b0;
            add_sticky         <= 1'b0;
            add_rm_r           <= 2'd0;
            add_flush_denorms_r <= 1'b0;
            add_special        <= 1'b0;
            add_special_result <= 64'd0;
            add_special_exc    <= 5'd0;
            add_norm_roundable_r     <= 1'b0;
            add_norm_sign_r          <= 1'b0;
            add_norm_exp_r           <= 12'd0;
            add_norm_man_r           <= 55'd0;
            add_norm_sticky_r        <= 1'b0;
            add_norm_result_r        <= 64'd0;
            add_norm_exc_invalid_r   <= 1'b0;
            add_norm_exc_overflow_r  <= 1'b0;
            add_norm_exc_underflow_r <= 1'b0;
            add_norm_exc_inexact_r   <= 1'b0;
            add_result_r             <= 64'd0;
            add_result_exc_invalid   <= 1'b0;
            add_result_exc_overflow  <= 1'b0;
            add_result_exc_underflow <= 1'b0;
            add_result_exc_inexact   <= 1'b0;
        end else if (fpu_cancel) begin
            add_state <= ADD_IDLE;
        end else begin
            case (add_state)
                ADD_IDLE: begin
                    if (FPU_HAS_ADDSUB && do_fpop && (funct == `FPU_ADD || funct == `FPU_SUB)) begin
                        add_fmt_r <= fmt;
                        add_funct_r <= funct;
                        add_fs_val_r <= fs_val;
                        add_ft_val_r <= ft_val;
                        add_rm_r <= rm;
                        add_flush_denorms_r <= flush_denorms;
                        add_state <= ADD_START;
                    end
                end

                ADD_START: begin
                    if (add_fmt_r == `COP1_FMT_D) begin : add_start_d
                        reg        sa, sb, sr;
                        reg [11:0] ea, eb, er, ediff;
                        reg [54:0] ma, mb;
                        reg [55:0] msum;
                        reg        dsub, sw, stk;

                        add_is_double <= 1'b1;
                        sa = add_d_sign_a;
                        sb = add_d_eff_sign_b;

                        if (add_d_is_nan_a || add_d_is_nan_b) begin
                            add_special        <= 1'b1;
                            add_special_result <= add_d_prop_nan;
                            add_special_exc    <= {(add_d_is_snan_a || add_d_is_snan_b), 4'd0};
                            add_state          <= ADD_DONE;
                        end else if (add_d_is_inf_a && add_d_is_inf_b) begin
                            add_special <= 1'b1;
                            if (sa == sb) begin
                                add_special_result <= {sa, 11'h7FF, 52'd0};
                                add_special_exc    <= 5'd0;
                            end else begin
                                add_special_result <= d_default_qnan;
                                add_special_exc    <= 5'b10000;
                            end
                            add_state <= ADD_DONE;
                        end else if (add_d_is_inf_a) begin
                            add_special        <= 1'b1;
                            add_special_result <= {sa, 11'h7FF, 52'd0};
                            add_special_exc    <= 5'd0;
                            add_state          <= ADD_DONE;
                        end else if (add_d_is_inf_b) begin
                            add_special        <= 1'b1;
                            add_special_result <= {sb, 11'h7FF, 52'd0};
                            add_special_exc    <= 5'd0;
                            add_state          <= ADD_DONE;
                        end else begin
                            ma = (add_d_exp_a == 0) ? {1'b0, add_d_man_a, 2'b00} : {1'b1, add_d_man_a, 2'b00};
                            mb = (add_d_exp_b == 0) ? {1'b0, add_d_man_b, 2'b00} : {1'b1, add_d_man_b, 2'b00};
                            ea = (add_d_exp_a == 0) ? 12'd1 : {1'b0, add_d_exp_a};
                            eb = (add_d_exp_b == 0) ? 12'd1 : {1'b0, add_d_exp_b};
                            sw = (eb > ea || (eb == ea && mb > ma));
                            stk = 1'b0;
                            if (sw) begin
                                ediff = eb - ea;
                                if (ediff >= 55) begin stk = |ma; ma = 55'd0; end
                                else if (ediff > 0) begin
                                    stk = |(ma & ((55'd1 << ediff[5:0]) - 55'd1));
                                    ma = ma >> ediff[5:0];
                                end
                                er = eb; sr = sb;
                                dsub = (sa != sb);
                                msum = dsub ? ({1'b0, mb} - {1'b0, ma}) : ({1'b0, mb} + {1'b0, ma});
                            end else begin
                                ediff = ea - eb;
                                if (ediff >= 55) begin stk = |mb; mb = 55'd0; end
                                else if (ediff > 0) begin
                                    stk = |(mb & ((55'd1 << ediff[5:0]) - 55'd1));
                                    mb = mb >> ediff[5:0];
                                end
                                er = ea; sr = sa;
                                dsub = (sa != sb);
                                msum = dsub ? ({1'b0, ma} - {1'b0, mb}) : ({1'b0, ma} + {1'b0, mb});
                            end
                            add_sign_r  <= sr;
                            add_exp_r   <= er;
                            add_man_sum <= msum;
                            add_do_sub  <= dsub;
                            add_sticky  <= stk;
                            add_special <= 1'b0;
                            add_state   <= ADD_DONE;
                        end
                    end else begin : add_start_s
                        reg        sa, sb, sr;
                        reg [8:0]  ea, eb, er, ediff;
                        reg [25:0] ma, mb;
                        reg [26:0] msum;
                        reg        dsub, sw, stk;

                        add_is_double <= 1'b0;
                        sa = add_s_sign_a;
                        sb = add_s_eff_sign_b;

                        if (add_s_is_nan_a || add_s_is_nan_b) begin
                            add_special        <= 1'b1;
                            add_special_result <= {32'd0, add_s_prop_nan};
                            add_special_exc    <= {(add_s_is_snan_a || add_s_is_snan_b), 4'd0};
                            add_state          <= ADD_DONE;
                        end else if (add_s_is_inf_a && add_s_is_inf_b) begin
                            add_special <= 1'b1;
                            if (sa == sb) begin
                                add_special_result <= {32'd0, sa, 8'hFF, 23'd0};
                                add_special_exc    <= 5'd0;
                            end else begin
                                add_special_result <= {32'd0, s_default_qnan};
                                add_special_exc    <= 5'b10000;
                            end
                            add_state <= ADD_DONE;
                        end else if (add_s_is_inf_a) begin
                            add_special        <= 1'b1;
                            add_special_result <= {32'd0, sa, 8'hFF, 23'd0};
                            add_special_exc    <= 5'd0;
                            add_state          <= ADD_DONE;
                        end else if (add_s_is_inf_b) begin
                            add_special        <= 1'b1;
                            add_special_result <= {32'd0, sb, 8'hFF, 23'd0};
                            add_special_exc    <= 5'd0;
                            add_state          <= ADD_DONE;
                        end else begin
                            ma = (add_s_exp_a == 0) ? {1'b0, add_s_man_a, 2'b00} : {1'b1, add_s_man_a, 2'b00};
                            mb = (add_s_exp_b == 0) ? {1'b0, add_s_man_b, 2'b00} : {1'b1, add_s_man_b, 2'b00};
                            ea = (add_s_exp_a == 0) ? 9'd1 : {1'b0, add_s_exp_a};
                            eb = (add_s_exp_b == 0) ? 9'd1 : {1'b0, add_s_exp_b};
                            sw = (eb > ea || (eb == ea && mb > ma));
                            stk = 1'b0;
                            if (sw) begin
                                ediff = eb - ea;
                                if (ediff >= 26) begin stk = |ma; ma = 26'd0; end
                                else if (ediff > 0) begin
                                    stk = |(ma & ((26'd1 << ediff[4:0]) - 26'd1));
                                    ma = ma >> ediff[4:0];
                                end
                                er = eb; sr = sb;
                                dsub = (sa != sb);
                                msum = dsub ? ({1'b0, mb} - {1'b0, ma}) : ({1'b0, mb} + {1'b0, ma});
                            end else begin
                                ediff = ea - eb;
                                if (ediff >= 26) begin stk = |mb; mb = 26'd0; end
                                else if (ediff > 0) begin
                                    stk = |(mb & ((26'd1 << ediff[4:0]) - 26'd1));
                                    mb = mb >> ediff[4:0];
                                end
                                er = ea; sr = sa;
                                dsub = (sa != sb);
                                msum = dsub ? ({1'b0, ma} - {1'b0, mb}) : ({1'b0, ma} + {1'b0, mb});
                            end
                            add_sign_r  <= sr;
                            add_exp_r   <= {3'd0, er};
                            add_man_sum <= {29'd0, msum};
                            add_do_sub  <= dsub;
                            add_sticky  <= stk;
                            add_special <= 1'b0;
                            add_state   <= ADD_DONE;
                        end
                    end
                end

                ADD_DONE: begin
                    if (!add_is_double) begin
                        add_norm_roundable_r     <= s_addnorm_roundable;
                        add_norm_sign_r          <= s_addnorm_sign;
                        add_norm_exp_r           <= {3'd0, s_addnorm_exp};
                        add_norm_man_r           <= {29'd0, s_addnorm_man};
                        add_norm_sticky_r        <= s_addnorm_sticky;
                        add_norm_result_r        <= {32'd0, s_addnorm_result};
                        add_norm_exc_invalid_r   <= s_addnorm_exc_invalid;
                        add_norm_exc_overflow_r  <= s_addnorm_exc_overflow;
                        add_norm_exc_underflow_r <= s_addnorm_exc_underflow;
                        add_norm_exc_inexact_r   <= s_addnorm_exc_inexact;
                    end else begin
                        add_norm_roundable_r     <= d_addnorm_roundable;
                        add_norm_sign_r          <= d_addnorm_sign;
                        add_norm_exp_r           <= d_addnorm_exp;
                        add_norm_man_r           <= d_addnorm_man;
                        add_norm_sticky_r        <= d_addnorm_sticky;
                        add_norm_result_r        <= d_addnorm_result;
                        add_norm_exc_invalid_r   <= d_addnorm_exc_invalid;
                        add_norm_exc_overflow_r  <= d_addnorm_exc_overflow;
                        add_norm_exc_underflow_r <= d_addnorm_exc_underflow;
                        add_norm_exc_inexact_r   <= d_addnorm_exc_inexact;
                    end
                    add_state <= ADD_ROUND;
                end

                ADD_ROUND: begin
                    if (!add_is_double) begin
                        add_result_r             <= {32'd0, s_addsub_result};
                        add_result_exc_invalid   <= s_addsub_exc_invalid;
                        add_result_exc_overflow  <= s_addsub_exc_overflow;
                        add_result_exc_underflow <= s_addsub_exc_underflow;
                        add_result_exc_inexact   <= s_addsub_exc_inexact;
                    end else begin
                        add_result_r             <= d_addsub_result;
                        add_result_exc_invalid   <= d_addsub_exc_invalid;
                        add_result_exc_overflow  <= d_addsub_exc_overflow;
                        add_result_exc_underflow <= d_addsub_exc_underflow;
                        add_result_exc_inexact   <= d_addsub_exc_inexact;
                    end
                    add_state <= ADD_RESULT;
                end

                ADD_RESULT: begin
                    add_state <= ADD_IDLE;
                end

                default: add_state <= ADD_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // ADD/SUB normalize logic — combinational, reads add_man_sum state
    // and produces the registered pre-round stage.
    // ---------------------------------------------------------------
    always @(*) begin : add_norm_s
        reg [8:0]  exp_r;
        reg [26:0] msum;
        reg [25:0] man_norm;
        reg [4:0]  lzc;
        reg        sticky;

        s_addnorm_result        = 32'd0;
        s_addnorm_roundable     = 1'b0;
        s_addnorm_sign          = add_sign_r;
        s_addnorm_exp           = 9'd0;
        s_addnorm_man           = 26'd0;
        s_addnorm_sticky        = 1'b0;
        s_addnorm_exc_invalid   = 1'b0;
        s_addnorm_exc_overflow  = 1'b0;
        s_addnorm_exc_underflow = 1'b0;
        s_addnorm_exc_inexact   = 1'b0;

        if (!add_is_double && add_special) begin
            s_addnorm_result = add_special_result[31:0];
            s_addnorm_exc_invalid = add_special_exc[4];
        end else if (!add_is_double) begin
            exp_r = add_exp_r[8:0];
            msum = add_man_sum[26:0];
            sticky = add_sticky;

            if (msum == 0) begin
                if (add_do_sub)
                    s_addnorm_result = {(add_rm_r == `RM_RM) ? 1'b1 : 1'b0, 31'd0};
                else
                    s_addnorm_result = {add_sign_r, 31'd0};
            end else begin
                if (msum[26]) begin
                    sticky = sticky | msum[0];
                    msum = msum >> 1;
                    exp_r = exp_r + 9'd1;
                end

                lzc = lzc26(msum[25:0]);

                if (lzc > 0) begin
                    if ({4'd0, lzc} >= exp_r) begin
                        msum = msum << (exp_r - 1);
                        exp_r = 0;
                    end else begin
                        msum = msum << lzc;
                        exp_r = exp_r - {4'd0, lzc};
                    end
                end

                man_norm = msum[25:0];
                if (add_do_sub && sticky) begin
                    man_norm = man_norm - 26'd1;
                    if (!man_norm[25]) begin
                        man_norm = {man_norm[24:0], 1'b1};
                        exp_r = exp_r - 9'd1;
                    end
                end
                if (man_norm[1:0] != 2'b00 || sticky)
                    s_addnorm_exc_inexact = 1'b1;

                s_addnorm_roundable = 1'b1;
                s_addnorm_sign = add_sign_r;
                s_addnorm_exp = exp_r;
                s_addnorm_man = man_norm;
                s_addnorm_sticky = sticky;
            end
        end
    end

    always @(*) begin : add_norm_d
        reg [11:0] exp_r;
        reg [55:0] msum;
        reg [54:0] man_norm;
        reg [5:0]  lzc;
        reg        sticky;

        d_addnorm_result        = 64'd0;
        d_addnorm_roundable     = 1'b0;
        d_addnorm_sign          = add_sign_r;
        d_addnorm_exp           = 12'd0;
        d_addnorm_man           = 55'd0;
        d_addnorm_sticky        = 1'b0;
        d_addnorm_exc_invalid   = 1'b0;
        d_addnorm_exc_overflow  = 1'b0;
        d_addnorm_exc_underflow = 1'b0;
        d_addnorm_exc_inexact   = 1'b0;

        if (add_is_double && add_special) begin
            d_addnorm_result = add_special_result;
            d_addnorm_exc_invalid = add_special_exc[4];
        end else if (add_is_double) begin
            exp_r = add_exp_r;
            msum = add_man_sum;
            sticky = add_sticky;

            if (msum == 0) begin
                if (add_do_sub)
                    d_addnorm_result = {(add_rm_r == `RM_RM) ? 1'b1 : 1'b0, 63'd0};
                else
                    d_addnorm_result = {add_sign_r, 63'd0};
            end else begin
                if (msum[55]) begin
                    sticky = sticky | msum[0];
                    msum = msum >> 1;
                    exp_r = exp_r + 12'd1;
                end

                lzc = lzc55(msum[54:0]);

                if (lzc > 0) begin
                    if ({6'd0, lzc} >= exp_r) begin
                        msum = msum << (exp_r - 1);
                        exp_r = 0;
                    end else begin
                        msum = msum << lzc;
                        exp_r = exp_r - {6'd0, lzc};
                    end
                end

                man_norm = msum[54:0];
                if (add_do_sub && sticky) begin
                    man_norm = man_norm - 55'd1;
                    if (!man_norm[54]) begin
                        man_norm = {man_norm[53:0], 1'b1};
                        exp_r = exp_r - 12'd1;
                    end
                end
                if (man_norm[1:0] != 2'b00 || sticky)
                    d_addnorm_exc_inexact = 1'b1;

                d_addnorm_roundable = 1'b1;
                d_addnorm_sign = add_sign_r;
                d_addnorm_exp = exp_r;
                d_addnorm_man = man_norm;
                d_addnorm_sticky = sticky;
            end
        end
    end

    // ---------------------------------------------------------------
    // ADD/SUB rounding logic — combinational, reads registered
    // normalized state when add_state == ADD_ROUND.
    // ---------------------------------------------------------------
    always @(*) begin : add_round_s
        reg [8:0]  exp_r;
        reg [25:0] man_norm;
        reg        sticky, round_up;

        s_addsub_result        = 32'd0;
        s_addsub_exc_invalid   = 1'b0;
        s_addsub_exc_overflow  = 1'b0;
        s_addsub_exc_underflow = 1'b0;
        s_addsub_exc_inexact   = 1'b0;

        if (!add_is_double) begin
            s_addsub_result        = add_norm_result_r[31:0];
            s_addsub_exc_invalid   = add_norm_exc_invalid_r;
            s_addsub_exc_overflow  = add_norm_exc_overflow_r;
            s_addsub_exc_underflow = add_norm_exc_underflow_r;
            s_addsub_exc_inexact   = add_norm_exc_inexact_r;

            if (add_norm_roundable_r) begin
                exp_r = add_norm_exp_r[8:0];
                man_norm = add_norm_man_r[25:0];
                sticky = add_norm_sticky_r;
                round_up = 1'b0;
                case (add_rm_r)
                    `RM_RN: round_up = man_norm[1] && (man_norm[0] || sticky || man_norm[2]);
                    `RM_RP: round_up = !add_norm_sign_r && (man_norm[1] || man_norm[0] || sticky);
                    `RM_RM: round_up = add_norm_sign_r && (man_norm[1] || man_norm[0] || sticky);
                    default: ;
                endcase

                if (exp_r >= 9'd255) begin
                    s_addsub_exc_overflow = 1'b1;
                    s_addsub_exc_inexact = 1'b1;
                    case (add_rm_r)
                        `RM_RN: s_addsub_result = {add_norm_sign_r, 8'hFF, 23'd0};
                        `RM_RZ: s_addsub_result = {add_norm_sign_r, 8'hFE, 23'h7FFFFF};
                        `RM_RP: s_addsub_result = add_norm_sign_r ? {1'b1, 8'hFE, 23'h7FFFFF} : {1'b0, 8'hFF, 23'd0};
                        `RM_RM: s_addsub_result = add_norm_sign_r ? {1'b1, 8'hFF, 23'd0} : {1'b0, 8'hFE, 23'h7FFFFF};
                    endcase
                end else begin
                    s_addsub_result = {add_norm_sign_r, exp_r[7:0], man_norm[24:2]};
                    if (round_up)
                        s_addsub_result = s_addsub_result + 32'd1;
                    if (s_addsub_result[30:23] == 8'h00 && s_addsub_result[22:0] != 23'd0 && s_addsub_exc_inexact)
                        s_addsub_exc_underflow = 1'b1;
                    if (s_addsub_result[30:23] == 8'hFF) begin
                        s_addsub_exc_overflow = 1'b1;
                        s_addsub_exc_inexact = 1'b1;
                        case (add_rm_r)
                            `RM_RN: s_addsub_result = {add_norm_sign_r, 8'hFF, 23'd0};
                            `RM_RZ: s_addsub_result = {add_norm_sign_r, 8'hFE, 23'h7FFFFF};
                            `RM_RP: s_addsub_result = add_norm_sign_r ? {1'b1, 8'hFE, 23'h7FFFFF} : {1'b0, 8'hFF, 23'd0};
                            `RM_RM: s_addsub_result = add_norm_sign_r ? {1'b1, 8'hFF, 23'd0} : {1'b0, 8'hFE, 23'h7FFFFF};
                        endcase
                    end else if (add_flush_denorms_r && s_addsub_result[30:23] == 8'h00) begin
                        s_addsub_result = {add_norm_sign_r, 31'd0};
                    end
                end
            end
        end
    end

    always @(*) begin : add_round_d
        reg [11:0] exp_r;
        reg [54:0] man_norm;
        reg        sticky, round_up;

        d_addsub_result        = 64'd0;
        d_addsub_exc_invalid   = 1'b0;
        d_addsub_exc_overflow  = 1'b0;
        d_addsub_exc_underflow = 1'b0;
        d_addsub_exc_inexact   = 1'b0;

        if (add_is_double) begin
            d_addsub_result        = add_norm_result_r;
            d_addsub_exc_invalid   = add_norm_exc_invalid_r;
            d_addsub_exc_overflow  = add_norm_exc_overflow_r;
            d_addsub_exc_underflow = add_norm_exc_underflow_r;
            d_addsub_exc_inexact   = add_norm_exc_inexact_r;

            if (add_norm_roundable_r) begin
                exp_r = add_norm_exp_r;
                man_norm = add_norm_man_r;
                sticky = add_norm_sticky_r;
                round_up = 1'b0;
                case (add_rm_r)
                    `RM_RN: round_up = man_norm[1] && (man_norm[0] || sticky || man_norm[2]);
                    `RM_RP: round_up = !add_norm_sign_r && (man_norm[1] || man_norm[0] || sticky);
                    `RM_RM: round_up = add_norm_sign_r && (man_norm[1] || man_norm[0] || sticky);
                    default: ;
                endcase

                if (exp_r >= 12'd2047) begin
                    d_addsub_exc_overflow = 1'b1;
                    d_addsub_exc_inexact = 1'b1;
                    case (add_rm_r)
                        `RM_RN: d_addsub_result = {add_norm_sign_r, 11'h7FF, 52'd0};
                        `RM_RZ: d_addsub_result = {add_norm_sign_r, 11'h7FE, 52'hFFFFFFFFFFFFF};
                        `RM_RP: d_addsub_result = add_norm_sign_r ? {1'b1, 11'h7FE, 52'hFFFFFFFFFFFFF} : {1'b0, 11'h7FF, 52'd0};
                        `RM_RM: d_addsub_result = add_norm_sign_r ? {1'b1, 11'h7FF, 52'd0} : {1'b0, 11'h7FE, 52'hFFFFFFFFFFFFF};
                    endcase
                end else begin
                    d_addsub_result = {add_norm_sign_r, exp_r[10:0], man_norm[53:2]};
                    if (round_up)
                        d_addsub_result = d_addsub_result + 64'd1;
                    if (d_addsub_result[62:52] == 11'h000 && d_addsub_result[51:0] != 52'd0 && d_addsub_exc_inexact)
                        d_addsub_exc_underflow = 1'b1;
                    if (d_addsub_result[62:52] == 11'h7FF) begin
                        d_addsub_exc_overflow = 1'b1;
                        d_addsub_exc_inexact = 1'b1;
                        case (add_rm_r)
                            `RM_RN: d_addsub_result = {add_norm_sign_r, 11'h7FF, 52'd0};
                            `RM_RZ: d_addsub_result = {add_norm_sign_r, 11'h7FE, 52'hFFFFFFFFFFFFF};
                            `RM_RP: d_addsub_result = add_norm_sign_r ? {1'b1, 11'h7FE, 52'hFFFFFFFFFFFFF} : {1'b0, 11'h7FF, 52'd0};
                            `RM_RM: d_addsub_result = add_norm_sign_r ? {1'b1, 11'h7FF, 52'd0} : {1'b0, 11'h7FE, 52'hFFFFFFFFFFFFF};
                        endcase
                    end else if (add_flush_denorms_r && d_addsub_result[62:52] == 11'h000) begin
                        d_addsub_exc_underflow = 1'b1;
                        d_addsub_result = {add_norm_sign_r, 63'd0};
                    end
                end
            end
        end
    end

    // ---------------------------------------------------------------
    // Single-precision MUL
    // ---------------------------------------------------------------
    // ---------------------------------------------------------------
    // Single/double-precision MUL results (driven by FSM + rounding logic)
    // ---------------------------------------------------------------
    reg [31:0] s_mul_result;
    reg        s_mul_exc_invalid;
    reg        s_mul_exc_overflow;
    reg        s_mul_exc_underflow;
    reg        s_mul_exc_inexact;

    reg [63:0] d_mul_result;
    reg        d_mul_exc_invalid;
    reg        d_mul_exc_overflow;
    reg        d_mul_exc_underflow;
    reg        d_mul_exc_inexact;

    // ---------------------------------------------------------------
    // MUL FSM — pipelined multiply plus final rounding cut
    // Cycle 1 (IDLE→START): capture op/control/operands from RF/FPR outputs
    // Cycle 2 (START→CALC): classify, normalize denorms, latch multiply inputs
    // Cycle 3 (CALC→DONE): product available, normalize/pre-round
    // Cycle 4 (DONE→ROUND): latch pre-round state
    // Cycle 5 (ROUND→RESULT/ALIGN): final rounding and result/flags latch
    // ---------------------------------------------------------------
    localparam [3:0] MUL_IDLE = 4'd0,
                     MUL_START = 4'd1,
                     MUL_CALC = 4'd2,
                     MUL_DONE = 4'd3,
                     MUL_MADD_ALIGN = 4'd4,
                     MUL_MADD = 4'd5,
                     MUL_RESULT = 4'd6,
                     MUL_MADD_RESULT = 4'd7,
                     MUL_ROUND = 4'd8,
                     MUL_MADD_ROUND = 4'd9;

    reg [3:0]          mul_state;
    reg [ 4:0]         mul_fmt_r;
    reg [ 5:0]         mul_funct_r;
    reg [63:0]         mul_fs_val_r;
    reg [63:0]         mul_ft_val_r;
    reg                mul_is_double;
    reg                mul_sign;
    reg signed [14:0]  mul_exp;
    reg [52:0]         mul_man_a;
    reg [52:0]         mul_man_b;
    reg [105:0]        mul_prod;
    reg                mul_special;
    reg [63:0]         mul_special_result;
    reg [4:0]          mul_special_exc;
    reg                mul_is_madd;
    reg [2:0]          mul_madd_op;
    reg [63:0]         mul_addend;
    reg [1:0]          mul_rm;
    reg                mul_flush_denorms;
    reg [63:0]         mul_result_r;
    reg                mul_result_exc_invalid;
    reg                mul_result_exc_overflow;
    reg                mul_result_exc_underflow;
    reg                mul_result_exc_inexact;

    reg                mul_round_is_double;
    reg                mul_round_sign;
    reg [1:0]          mul_round_rm;
    reg                mul_round_flush_denorms;
    reg signed [14:0]  mul_round_exp;
    reg [52:0]         mul_round_man;
    reg                mul_round_guard;
    reg                mul_round_sticky;
    reg                mul_round_is_tiny;
    reg                mul_round_exc_underflow;
    reg                mul_round_exc_inexact;

    reg signed [14:0]  s_mul_pre_exp;
    reg [23:0]         s_mul_pre_man;
    reg                s_mul_pre_guard;
    reg                s_mul_pre_sticky;
    reg                s_mul_pre_is_tiny;
    reg                s_mul_pre_exc_underflow;
    reg                s_mul_pre_exc_inexact;

    reg signed [14:0]  d_mul_pre_exp;
    reg [52:0]         d_mul_pre_man;
    reg                d_mul_pre_guard;
    reg                d_mul_pre_sticky;
    reg                d_mul_pre_is_tiny;
    reg                d_mul_pre_exc_underflow;
    reg                d_mul_pre_exc_inexact;

    // MADD add step registers (populated in MUL_MADD_ALIGN -> MUL_MADD)
    reg [63:0]         madd_mul_result;
    reg                madd_mul_exc_invalid;
    reg                madd_mul_exc_overflow;
    reg                madd_mul_exc_underflow;
    reg                madd_mul_exc_inexact;
    reg                madd_sign_r;
    reg [11:0]         madd_exp_r;
    reg [55:0]         madd_man_sum;
    reg                madd_sticky;
    reg                madd_add_special;
    reg [63:0]         madd_add_special_result;
    reg [4:0]          madd_add_special_exc;
    reg                madd_norm_roundable_r;
    reg                madd_norm_sign_r;
    reg [11:0]         madd_norm_exp_r;
    reg [54:0]         madd_norm_man_r;
    reg [63:0]         madd_norm_result_r;
    reg                madd_norm_exc_invalid_r;
    reg                madd_norm_exc_overflow_r;
    reg                madd_norm_exc_underflow_r;
    reg                madd_norm_exc_inexact_r;

    wire mul_trigger = FPU_HAS_MUL && ((do_fpop && funct == `FPU_MUL) || (madd_op != 3'd0));

    // Registered MUL/MADD operands. Like ADD/SUB, the 50 MHz profile cannot
    // afford RF instruction decode plus FPR classify/denorm/exponent work in
    // the same cycle that writes the multiply operands.
    wire        mul_s_sign_a = mul_fs_val_r[31];
    wire [ 7:0] mul_s_exp_a  = mul_fs_val_r[30:23];
    wire [22:0] mul_s_man_a  = mul_fs_val_r[22:0];
    wire        mul_s_sign_b = mul_ft_val_r[31];
    wire [ 7:0] mul_s_exp_b  = mul_ft_val_r[30:23];
    wire [22:0] mul_s_man_b  = mul_ft_val_r[22:0];
    wire        mul_d_sign_a = mul_fs_val_r[63];
    wire [10:0] mul_d_exp_a  = mul_fs_val_r[62:52];
    wire [51:0] mul_d_man_a  = mul_fs_val_r[51:0];
    wire        mul_d_sign_b = mul_ft_val_r[63];
    wire [10:0] mul_d_exp_b  = mul_ft_val_r[62:52];
    wire [51:0] mul_d_man_b  = mul_ft_val_r[51:0];

    wire mul_s_is_nan_a = (mul_s_exp_a == 8'hFF) && (mul_s_man_a != 0);
    wire mul_s_is_nan_b = (mul_s_exp_b == 8'hFF) && (mul_s_man_b != 0);
    wire mul_s_is_inf_a = (mul_s_exp_a == 8'hFF) && (mul_s_man_a == 0);
    wire mul_s_is_inf_b = (mul_s_exp_b == 8'hFF) && (mul_s_man_b == 0);
    wire mul_s_is_zero_a = (mul_s_exp_a == 0) && (mul_s_man_a == 0 || mul_flush_denorms);
    wire mul_s_is_zero_b = (mul_s_exp_b == 0) && (mul_s_man_b == 0 || mul_flush_denorms);
    wire mul_s_is_snan_a = mul_s_is_nan_a && mul_s_man_a[22];
    wire mul_s_is_snan_b = mul_s_is_nan_b && mul_s_man_b[22];
    wire mul_d_is_nan_a = (mul_d_exp_a == 11'h7FF) && (mul_d_man_a != 0);
    wire mul_d_is_nan_b = (mul_d_exp_b == 11'h7FF) && (mul_d_man_b != 0);
    wire mul_d_is_inf_a = (mul_d_exp_a == 11'h7FF) && (mul_d_man_a == 0);
    wire mul_d_is_inf_b = (mul_d_exp_b == 11'h7FF) && (mul_d_man_b == 0);
    wire mul_d_is_zero_a = (mul_d_exp_a == 0) && (mul_d_man_a == 0 || mul_flush_denorms);
    wire mul_d_is_zero_b = (mul_d_exp_b == 0) && (mul_d_man_b == 0 || mul_flush_denorms);
    wire mul_d_is_snan_a = mul_d_is_nan_a && mul_d_man_a[51];
    wire mul_d_is_snan_b = mul_d_is_nan_b && mul_d_man_b[51];
    wire [31:0] mul_s_prop_nan = quiet_legacy_nan32(
        mul_s_is_nan_a ? mul_fs_val_r[31:0] : mul_ft_val_r[31:0]);
    wire [63:0] mul_d_prop_nan = quiet_legacy_nan64(
        mul_d_is_nan_a ? mul_fs_val_r : mul_ft_val_r);

    `R4000_FPU_SEQ begin
        if (!rst_n) begin
            mul_state          <= MUL_IDLE;
            mul_fmt_r          <= 5'd0;
            mul_funct_r        <= 6'd0;
            mul_fs_val_r       <= 64'd0;
            mul_ft_val_r       <= 64'd0;
            mul_is_double      <= 1'b0;
            mul_sign           <= 1'b0;
            mul_exp            <= 15'sd0;
            mul_man_a          <= 53'd0;
            mul_man_b          <= 53'd0;
            mul_prod           <= 106'd0;
            mul_special        <= 1'b0;
            mul_special_result <= 64'd0;
            mul_special_exc    <= 5'd0;
            mul_is_madd        <= 1'b0;
            mul_madd_op        <= 3'd0;
            mul_addend         <= 64'd0;
            mul_rm             <= `RM_RN;
            mul_flush_denorms  <= 1'b0;
            mul_result_r             <= 64'd0;
            mul_result_exc_invalid   <= 1'b0;
            mul_result_exc_overflow  <= 1'b0;
            mul_result_exc_underflow <= 1'b0;
            mul_result_exc_inexact   <= 1'b0;
            mul_round_is_double      <= 1'b0;
            mul_round_sign           <= 1'b0;
            mul_round_rm             <= `RM_RN;
            mul_round_flush_denorms  <= 1'b0;
            mul_round_exp            <= 15'sd0;
            mul_round_man            <= 53'd0;
            mul_round_guard          <= 1'b0;
            mul_round_sticky         <= 1'b0;
            mul_round_is_tiny        <= 1'b0;
            mul_round_exc_underflow  <= 1'b0;
            mul_round_exc_inexact    <= 1'b0;
            madd_mul_result        <= 64'd0;
            madd_mul_exc_invalid   <= 1'b0;
            madd_mul_exc_overflow  <= 1'b0;
            madd_mul_exc_underflow <= 1'b0;
            madd_mul_exc_inexact   <= 1'b0;
            madd_sign_r        <= 1'b0;
            madd_exp_r         <= 12'd0;
            madd_man_sum       <= 56'd0;
            madd_sticky        <= 1'b0;
            madd_add_special        <= 1'b0;
            madd_add_special_result <= 64'd0;
            madd_add_special_exc    <= 5'd0;
            madd_norm_roundable_r     <= 1'b0;
            madd_norm_sign_r          <= 1'b0;
            madd_norm_exp_r           <= 12'd0;
            madd_norm_man_r           <= 55'd0;
            madd_norm_result_r        <= 64'd0;
            madd_norm_exc_invalid_r   <= 1'b0;
            madd_norm_exc_overflow_r  <= 1'b0;
            madd_norm_exc_underflow_r <= 1'b0;
            madd_norm_exc_inexact_r   <= 1'b0;
        end else if (fpu_cancel) begin
            mul_state <= MUL_IDLE;
        end else begin
            case (mul_state)
                MUL_IDLE: begin
                    if (mul_trigger) begin
                        mul_is_madd <= (madd_op != 3'd0);
                        mul_madd_op <= madd_op;
                        mul_fmt_r   <= fmt;
                        mul_funct_r <= funct;
                        mul_fs_val_r <= fs_val;
                        mul_ft_val_r <= ft_val;
                        mul_addend  <= fr_val;
                        mul_rm      <= eff_rm;
                        mul_flush_denorms <= flush_denorms;
                        mul_state   <= MUL_START;
                    end
                end

                MUL_START: begin
                    if ((mul_fmt_r == `COP1_FMT_D) || (mul_is_madd && mul_funct_r[0])) begin : mul_start_d
                            reg [52:0] ma, mb;
                            reg [5:0] lza, lzb;
                            reg signed [14:0] expr;

                            mul_is_double <= 1'b1;
                            mul_sign      <= mul_d_sign_a ^ mul_d_sign_b;

                            if (mul_d_is_nan_a || mul_d_is_nan_b) begin
                                mul_special        <= 1'b1;
                                mul_special_result <= mul_d_prop_nan;
                                mul_special_exc    <= {(mul_d_is_snan_a || mul_d_is_snan_b), 4'd0};
                                mul_state          <= MUL_DONE;
                            end else if (mul_d_is_inf_a || mul_d_is_inf_b) begin
                                if (mul_d_is_zero_a || mul_d_is_zero_b) begin
                                    mul_special        <= 1'b1;
                                    mul_special_result <= d_default_qnan;
                                    mul_special_exc    <= 5'b10000;
                                    mul_state          <= MUL_DONE;
                                end else begin
                                    mul_special        <= 1'b1;
                                    mul_special_result <= {mul_d_sign_a ^ mul_d_sign_b, 11'h7FF, 52'd0};
                                    mul_special_exc    <= 5'd0;
                                    mul_state          <= MUL_DONE;
                                end
                            end else if (mul_d_is_zero_a || mul_d_is_zero_b) begin
                                mul_special        <= 1'b1;
                                mul_special_result <= {mul_d_sign_a ^ mul_d_sign_b, 63'd0};
                                mul_special_exc    <= 5'd0;
                                mul_state          <= MUL_DONE;
                            end else begin
                                ma = (mul_d_exp_a == 0) ? {1'b0, mul_d_man_a} : {1'b1, mul_d_man_a};
                                mb = (mul_d_exp_b == 0) ? {1'b0, mul_d_man_b} : {1'b1, mul_d_man_b};
                                lza = 6'd0;
                                if (mul_d_exp_a == 0) begin
                                    lza = lzc53(ma);
                                    ma = ma << lza;
                                end
                                lzb = 6'd0;
                                if (mul_d_exp_b == 0) begin
                                    lzb = lzc53(mb);
                                    mb = mb << lzb;
                                end
                                expr = $signed({4'd0, mul_d_exp_a}) + $signed({4'd0, mul_d_exp_b}) - 15'sd1023;
                                if (mul_d_exp_a == 0) expr = expr + 15'sd1 - $signed({9'd0, lza});
                                if (mul_d_exp_b == 0) expr = expr + 15'sd1 - $signed({9'd0, lzb});
                                mul_exp     <= expr;
                                mul_man_a   <= ma;
                                mul_man_b   <= mb;
                                mul_special <= 1'b0;
                                mul_state   <= MUL_CALC;
                            end
                        end else begin : mul_start_s
                            reg [23:0] ma, mb;
                            reg [4:0] lza, lzb;
                            reg signed [14:0] expr;

                            mul_is_double <= 1'b0;
                            mul_sign      <= mul_s_sign_a ^ mul_s_sign_b;

                            if (mul_s_is_nan_a || mul_s_is_nan_b) begin
                                mul_special        <= 1'b1;
                                mul_special_result <= {32'd0, mul_s_prop_nan};
                                mul_special_exc    <= {(mul_s_is_snan_a || mul_s_is_snan_b), 4'd0};
                                mul_state          <= MUL_DONE;
                            end else if (mul_s_is_inf_a || mul_s_is_inf_b) begin
                                if (mul_s_is_zero_a || mul_s_is_zero_b) begin
                                    mul_special        <= 1'b1;
                                    mul_special_result <= {32'd0, s_default_qnan};
                                    mul_special_exc    <= 5'b10000;
                                    mul_state          <= MUL_DONE;
                                end else begin
                                    mul_special        <= 1'b1;
                                    mul_special_result <= {32'd0, mul_s_sign_a ^ mul_s_sign_b, 8'hFF, 23'd0};
                                    mul_special_exc    <= 5'd0;
                                    mul_state          <= MUL_DONE;
                                end
                            end else if (mul_s_is_zero_a || mul_s_is_zero_b) begin
                                mul_special        <= 1'b1;
                                mul_special_result <= {32'd0, mul_s_sign_a ^ mul_s_sign_b, 31'd0};
                                mul_special_exc    <= 5'd0;
                                mul_state          <= MUL_DONE;
                            end else begin
                                ma = (mul_s_exp_a == 0) ? {1'b0, mul_s_man_a} : {1'b1, mul_s_man_a};
                                mb = (mul_s_exp_b == 0) ? {1'b0, mul_s_man_b} : {1'b1, mul_s_man_b};
                                lza = 5'd0;
                                if (mul_s_exp_a == 0) begin
                                    lza = lzc24(ma);
                                    ma = ma << lza;
                                end
                                lzb = 5'd0;
                                if (mul_s_exp_b == 0) begin
                                    lzb = lzc24(mb);
                                    mb = mb << lzb;
                                end
                                expr = $signed({7'd0, mul_s_exp_a}) + $signed({7'd0, mul_s_exp_b}) - 15'sd127;
                                if (mul_s_exp_a == 0) expr = expr + 15'sd1 - $signed({10'd0, lza});
                                if (mul_s_exp_b == 0) expr = expr + 15'sd1 - $signed({10'd0, lzb});
                                mul_exp     <= expr;
                                mul_man_a   <= {29'd0, ma};
                                mul_man_b   <= {29'd0, mb};
                                mul_special <= 1'b0;
                                mul_state   <= MUL_CALC;
                            end
                        end
                end

                MUL_CALC: begin
                    mul_prod  <= mul_man_a * mul_man_b;
                    mul_state <= MUL_DONE;
                end

                MUL_DONE: begin
                    if (mul_special) begin
                        if (mul_is_madd) begin
                            madd_mul_result        <= mul_is_double ? mul_special_result :
                                                      {32'd0, mul_special_result[31:0]};
                            madd_mul_exc_invalid   <= mul_special_exc[4];
                            madd_mul_exc_overflow  <= mul_special_exc[2];
                            madd_mul_exc_underflow <= mul_special_exc[1];
                            madd_mul_exc_inexact   <= mul_special_exc[0];
                            mul_state              <= MUL_MADD_ALIGN;
                        end else begin
                            mul_result_r             <= mul_is_double ? mul_special_result :
                                                        {32'd0, mul_special_result[31:0]};
                            mul_result_exc_invalid   <= mul_special_exc[4];
                            mul_result_exc_overflow  <= mul_special_exc[2];
                            mul_result_exc_underflow <= mul_special_exc[1];
                            mul_result_exc_inexact   <= mul_special_exc[0];
                            mul_state                <= MUL_RESULT;
                        end
                    end else if (!mul_is_double) begin
                        mul_round_is_double     <= 1'b0;
                        mul_round_sign          <= mul_sign;
                        mul_round_rm            <= mul_rm;
                        mul_round_flush_denorms <= mul_flush_denorms;
                        mul_round_exp           <= s_mul_pre_exp;
                        mul_round_man           <= {29'd0, s_mul_pre_man};
                        mul_round_guard         <= s_mul_pre_guard;
                        mul_round_sticky        <= s_mul_pre_sticky;
                        mul_round_is_tiny       <= s_mul_pre_is_tiny;
                        mul_round_exc_underflow <= s_mul_pre_exc_underflow;
                        mul_round_exc_inexact   <= s_mul_pre_exc_inexact;
                        mul_state               <= MUL_ROUND;
                    end else begin
                        mul_round_is_double     <= 1'b1;
                        mul_round_sign          <= mul_sign;
                        mul_round_rm            <= mul_rm;
                        mul_round_flush_denorms <= mul_flush_denorms;
                        mul_round_exp           <= d_mul_pre_exp;
                        mul_round_man           <= d_mul_pre_man;
                        mul_round_guard         <= d_mul_pre_guard;
                        mul_round_sticky        <= d_mul_pre_sticky;
                        mul_round_is_tiny       <= d_mul_pre_is_tiny;
                        mul_round_exc_underflow <= d_mul_pre_exc_underflow;
                        mul_round_exc_inexact   <= d_mul_pre_exc_inexact;
                        mul_state               <= MUL_ROUND;
                    end
                end

                MUL_ROUND: begin
                    if (mul_is_madd) begin
                        if (!mul_is_double) begin
                            madd_mul_result        <= {32'd0, s_mul_result};
                            madd_mul_exc_invalid   <= s_mul_exc_invalid;
                            madd_mul_exc_overflow  <= s_mul_exc_overflow;
                            madd_mul_exc_underflow <= s_mul_exc_underflow;
                            madd_mul_exc_inexact   <= s_mul_exc_inexact;
                        end else begin
                            madd_mul_result        <= d_mul_result;
                            madd_mul_exc_invalid   <= d_mul_exc_invalid;
                            madd_mul_exc_overflow  <= d_mul_exc_overflow;
                            madd_mul_exc_underflow <= d_mul_exc_underflow;
                            madd_mul_exc_inexact   <= d_mul_exc_inexact;
                        end
                        mul_state <= MUL_MADD_ALIGN;
                    end else begin
                        if (!mul_is_double) begin
                            mul_result_r             <= {32'd0, s_mul_result};
                            mul_result_exc_invalid   <= s_mul_exc_invalid;
                            mul_result_exc_overflow  <= s_mul_exc_overflow;
                            mul_result_exc_underflow <= s_mul_exc_underflow;
                            mul_result_exc_inexact   <= s_mul_exc_inexact;
                        end else begin
                            mul_result_r             <= d_mul_result;
                            mul_result_exc_invalid   <= d_mul_exc_invalid;
                            mul_result_exc_overflow  <= d_mul_exc_overflow;
                            mul_result_exc_underflow <= d_mul_exc_underflow;
                            mul_result_exc_inexact   <= d_mul_exc_inexact;
                        end
                        mul_state <= MUL_RESULT;
                    end
                end

                MUL_MADD_ALIGN: begin
                    if (mul_is_madd) begin
                        if (!mul_is_double) begin : madd_align_s
                            reg        sign_a, sign_b, sign_r;
                            reg [ 7:0] ea_raw, eb_raw;
                            reg [22:0] ma_raw, mb_raw;
                            reg [ 8:0] exp_a_ext, exp_b_ext, er, exp_diff;
                            reg [25:0] man_a_ext, man_b_ext;
                            reg [26:0] msum;
                            reg        sw, stk;
                            reg        a_nan, a_inf, b_nan, b_inf;

                            sign_a = madd_mul_result[31];
                            ea_raw = madd_mul_result[30:23];
                            ma_raw = madd_mul_result[22:0];
                            a_nan  = (ea_raw == 8'hFF) && (ma_raw != 0);
                            a_inf  = (ea_raw == 8'hFF) && (ma_raw == 0);

                            sign_b = mul_addend[31];
                            eb_raw = mul_addend[30:23];
                            mb_raw = mul_addend[22:0];
                            b_nan  = (eb_raw == 8'hFF) && (mb_raw != 0);
                            b_inf  = (eb_raw == 8'hFF) && (mb_raw == 0);

                            if (mul_madd_op == 3'd2) sign_b = ~sign_b;

                            if (a_nan || b_nan) begin
                                madd_add_special <= 1'b1;
                                madd_add_special_exc <= {(a_nan && ma_raw[22]) || (b_nan && mb_raw[22]), 4'd0};
                                if (a_nan)
                                    madd_add_special_result <= {32'd0, quiet_legacy_nan32(madd_mul_result[31:0])};
                                else
                                    madd_add_special_result <= {32'd0, quiet_legacy_nan32(mul_addend[31:0])};
                                mul_state <= MUL_MADD;
                            end else if (a_inf && b_inf) begin
                                madd_add_special <= 1'b1;
                                if (sign_a == sign_b) begin
                                    madd_add_special_result <= {32'd0, sign_a, 8'hFF, 23'd0};
                                    madd_add_special_exc <= 5'd0;
                                end else begin
                                    madd_add_special_result <= {32'd0, s_default_qnan};
                                    madd_add_special_exc <= 5'b10000;
                                end
                                mul_state <= MUL_MADD;
                            end else if (a_inf) begin
                                madd_add_special <= 1'b1;
                                madd_add_special_result <= {32'd0, sign_a, 8'hFF, 23'd0};
                                madd_add_special_exc <= 5'd0;
                                mul_state <= MUL_MADD;
                            end else if (b_inf) begin
                                madd_add_special <= 1'b1;
                                madd_add_special_result <= {32'd0, sign_b, 8'hFF, 23'd0};
                                madd_add_special_exc <= 5'd0;
                                mul_state <= MUL_MADD;
                            end else begin
                                if (ea_raw == 0)
                                    man_a_ext = {1'b0, ma_raw, 2'b00};
                                else
                                    man_a_ext = {1'b1, ma_raw, 2'b00};
                                if (eb_raw == 0)
                                    man_b_ext = {1'b0, mb_raw, 2'b00};
                                else
                                    man_b_ext = {1'b1, mb_raw, 2'b00};
                                exp_a_ext = (ea_raw == 0) ? 9'd1 : {1'b0, ea_raw};
                                exp_b_ext = (eb_raw == 0) ? 9'd1 : {1'b0, eb_raw};
                                sw = (exp_b_ext > exp_a_ext || (exp_b_ext == exp_a_ext && man_b_ext > man_a_ext));
                                stk = 1'b0;
                                if (sw) begin
                                    exp_diff = exp_b_ext - exp_a_ext;
                                    if (exp_diff >= 26) stk = |man_a_ext;
                                    else if (exp_diff > 0)
                                        stk = |(man_a_ext & ((26'd1 << exp_diff[4:0]) - 26'd1));
                                    man_a_ext = man_a_ext >> (exp_diff > 25 ? 5'd25 : exp_diff[4:0]);
                                    er = exp_b_ext;
                                    sign_r = sign_b;
                                    if (sign_a != sign_b)
                                        msum = {1'b0, man_b_ext} - {1'b0, man_a_ext};
                                    else
                                        msum = {1'b0, man_b_ext} + {1'b0, man_a_ext};
                                end else begin
                                    exp_diff = exp_a_ext - exp_b_ext;
                                    if (exp_diff >= 26) stk = |man_b_ext;
                                    else if (exp_diff > 0)
                                        stk = |(man_b_ext & ((26'd1 << exp_diff[4:0]) - 26'd1));
                                    man_b_ext = man_b_ext >> (exp_diff > 25 ? 5'd25 : exp_diff[4:0]);
                                    er = exp_a_ext;
                                    sign_r = sign_a;
                                    if (sign_a != sign_b)
                                        msum = {1'b0, man_a_ext} - {1'b0, man_b_ext};
                                    else
                                        msum = {1'b0, man_a_ext} + {1'b0, man_b_ext};
                                end
                                madd_sign_r  <= sign_r;
                                madd_exp_r   <= {3'd0, er};
                                madd_man_sum <= {29'd0, msum};
                                madd_sticky  <= stk;
                                madd_add_special <= 1'b0;
                                mul_state <= MUL_MADD;
                            end
                        end else begin : madd_align_d
                            reg         sign_a, sign_b, sign_r;
                            reg [10:0]  ea_raw, eb_raw;
                            reg [51:0]  ma_raw, mb_raw;
                            reg [11:0]  exp_a_ext, exp_b_ext, er, exp_diff;
                            reg [54:0]  man_a_ext, man_b_ext;
                            reg [55:0]  msum;
                            reg         sw, stk;
                            reg         a_nan, a_inf, b_nan, b_inf;

                            sign_a = madd_mul_result[63];
                            ea_raw = madd_mul_result[62:52];
                            ma_raw = madd_mul_result[51:0];
                            a_nan  = (ea_raw == 11'h7FF) && (ma_raw != 0);
                            a_inf  = (ea_raw == 11'h7FF) && (ma_raw == 0);

                            sign_b = mul_addend[63];
                            eb_raw = mul_addend[62:52];
                            mb_raw = mul_addend[51:0];
                            b_nan  = (eb_raw == 11'h7FF) && (mb_raw != 0);
                            b_inf  = (eb_raw == 11'h7FF) && (mb_raw == 0);

                            if (mul_madd_op == 3'd2) sign_b = ~sign_b;

                            if (a_nan || b_nan) begin
                                madd_add_special <= 1'b1;
                                madd_add_special_exc <= {(a_nan && ma_raw[51]) || (b_nan && mb_raw[51]), 4'd0};
                                if (a_nan)
                                    madd_add_special_result <= quiet_legacy_nan64(madd_mul_result);
                                else
                                    madd_add_special_result <= quiet_legacy_nan64(mul_addend);
                                mul_state <= MUL_MADD;
                            end else if (a_inf && b_inf) begin
                                madd_add_special <= 1'b1;
                                if (sign_a == sign_b) begin
                                    madd_add_special_result <= {sign_a, 11'h7FF, 52'd0};
                                    madd_add_special_exc <= 5'd0;
                                end else begin
                                    madd_add_special_result <= d_default_qnan;
                                    madd_add_special_exc <= 5'b10000;
                                end
                                mul_state <= MUL_MADD;
                            end else if (a_inf) begin
                                madd_add_special <= 1'b1;
                                madd_add_special_result <= {sign_a, 11'h7FF, 52'd0};
                                madd_add_special_exc <= 5'd0;
                                mul_state <= MUL_MADD;
                            end else if (b_inf) begin
                                madd_add_special <= 1'b1;
                                madd_add_special_result <= {sign_b, 11'h7FF, 52'd0};
                                madd_add_special_exc <= 5'd0;
                                mul_state <= MUL_MADD;
                            end else begin
                                if (ea_raw == 0)
                                    man_a_ext = {1'b0, ma_raw, 2'b00};
                                else
                                    man_a_ext = {1'b1, ma_raw, 2'b00};
                                if (eb_raw == 0)
                                    man_b_ext = {1'b0, mb_raw, 2'b00};
                                else
                                    man_b_ext = {1'b1, mb_raw, 2'b00};
                                exp_a_ext = (ea_raw == 0) ? 12'd1 : {1'b0, ea_raw};
                                exp_b_ext = (eb_raw == 0) ? 12'd1 : {1'b0, eb_raw};
                                sw = (exp_b_ext > exp_a_ext || (exp_b_ext == exp_a_ext && man_b_ext > man_a_ext));
                                stk = 1'b0;
                                if (sw) begin
                                    exp_diff = exp_b_ext - exp_a_ext;
                                    if (exp_diff >= 55) stk = |man_a_ext;
                                    else if (exp_diff > 0)
                                        stk = |(man_a_ext & ((55'd1 << exp_diff[5:0]) - 55'd1));
                                    man_a_ext = man_a_ext >> (exp_diff > 54 ? 6'd54 : exp_diff[5:0]);
                                    er = exp_b_ext;
                                    sign_r = sign_b;
                                    if (sign_a != sign_b)
                                        msum = {1'b0, man_b_ext} - {1'b0, man_a_ext};
                                    else
                                        msum = {1'b0, man_b_ext} + {1'b0, man_a_ext};
                                end else begin
                                    exp_diff = exp_a_ext - exp_b_ext;
                                    if (exp_diff >= 55) stk = |man_b_ext;
                                    else if (exp_diff > 0)
                                        stk = |(man_b_ext & ((55'd1 << exp_diff[5:0]) - 55'd1));
                                    man_b_ext = man_b_ext >> (exp_diff > 54 ? 6'd54 : exp_diff[5:0]);
                                    er = exp_a_ext;
                                    sign_r = sign_a;
                                    if (sign_a != sign_b)
                                        msum = {1'b0, man_a_ext} - {1'b0, man_b_ext};
                                    else
                                        msum = {1'b0, man_a_ext} + {1'b0, man_b_ext};
                                end
                                madd_sign_r  <= sign_r;
                                madd_exp_r   <= er;
                                madd_man_sum <= msum;
                                madd_sticky  <= stk;
                                madd_add_special <= 1'b0;
                                mul_state <= MUL_MADD;
                            end
                        end
                    end else begin
                        mul_state <= MUL_IDLE;
                    end
                end

                MUL_MADD: begin
                    if (mul_is_madd) begin
                        if (!mul_is_double) begin
                            madd_norm_roundable_r     <= s_maddnorm_roundable;
                            madd_norm_sign_r          <= s_maddnorm_sign;
                            madd_norm_exp_r           <= {3'd0, s_maddnorm_exp};
                            madd_norm_man_r           <= {29'd0, s_maddnorm_man};
                            madd_norm_result_r        <= {32'd0, s_maddnorm_result};
                            madd_norm_exc_invalid_r   <= s_maddnorm_exc_invalid;
                            madd_norm_exc_overflow_r  <= s_maddnorm_exc_overflow;
                            madd_norm_exc_underflow_r <= s_maddnorm_exc_underflow;
                            madd_norm_exc_inexact_r   <= s_maddnorm_exc_inexact;
                        end else begin
                            madd_norm_roundable_r     <= d_maddnorm_roundable;
                            madd_norm_sign_r          <= d_maddnorm_sign;
                            madd_norm_exp_r           <= d_maddnorm_exp;
                            madd_norm_man_r           <= d_maddnorm_man;
                            madd_norm_result_r        <= d_maddnorm_result;
                            madd_norm_exc_invalid_r   <= d_maddnorm_exc_invalid;
                            madd_norm_exc_overflow_r  <= d_maddnorm_exc_overflow;
                            madd_norm_exc_underflow_r <= d_maddnorm_exc_underflow;
                            madd_norm_exc_inexact_r   <= d_maddnorm_exc_inexact;
                        end
                        mul_state <= MUL_MADD_ROUND;
                    end else begin
                        mul_state <= MUL_IDLE;
                    end
                end

                MUL_MADD_ROUND: begin
                    if (mul_is_madd) begin
                        if (!mul_is_double) begin
                            mul_result_r             <= {32'd0, s_madd_result};
                            mul_result_exc_invalid   <= madd_mul_exc_invalid | s_madd_exc_invalid;
                            mul_result_exc_overflow  <= madd_mul_exc_overflow | s_madd_exc_overflow;
                            mul_result_exc_underflow <= madd_mul_exc_underflow | s_madd_exc_underflow;
                            mul_result_exc_inexact   <= madd_mul_exc_inexact | s_madd_exc_inexact;
                        end else begin
                            mul_result_r             <= d_madd_result;
                            mul_result_exc_invalid   <= madd_mul_exc_invalid | d_madd_exc_invalid;
                            mul_result_exc_overflow  <= madd_mul_exc_overflow | d_madd_exc_overflow;
                            mul_result_exc_underflow <= madd_mul_exc_underflow | d_madd_exc_underflow;
                            mul_result_exc_inexact   <= madd_mul_exc_inexact | d_madd_exc_inexact;
                        end
                        mul_state <= MUL_MADD_RESULT;
                    end else begin
                        mul_state <= MUL_IDLE;
                    end
                end

                MUL_RESULT: begin
                    mul_state <= MUL_IDLE;
                end

                MUL_MADD_RESULT: begin
                    mul_state <= MUL_IDLE;
                end

                default: mul_state <= MUL_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // MUL normalize/pre-round logic — combinational, reads FSM registers
    // and is latched by MUL_DONE before final rounding.
    // ---------------------------------------------------------------
    always @(*) begin : mul_preround_s
        reg signed [14:0] exp_r;
        reg [23:0] man_norm;
        reg guard, sticky, is_tiny;
        reg [5:0] denorm_shift;
        reg [25:0] jammed;

        s_mul_pre_exp           = 15'sd0;
        s_mul_pre_man           = 24'd0;
        s_mul_pre_guard         = 1'b0;
        s_mul_pre_sticky        = 1'b0;
        s_mul_pre_is_tiny       = 1'b0;
        s_mul_pre_exc_underflow = 1'b0;
        s_mul_pre_exc_inexact   = 1'b0;

        if (!mul_is_double && !mul_special) begin
            exp_r = mul_exp;

            if (mul_prod[47]) begin
                exp_r = exp_r + 15'sd1;
                man_norm = mul_prod[47:24];
                guard = mul_prod[23];
                sticky = |mul_prod[22:0];
            end else begin
                man_norm = mul_prod[46:23];
                guard = mul_prod[22];
                sticky = |mul_prod[21:0];
            end

            is_tiny = (exp_r < 15'sd1);
            if (is_tiny) begin
                if (exp_r == 15'sd0 && man_norm == 24'hFFFFFF) begin
                    case (mul_rm)
                        `RM_RN: if (guard) is_tiny = 1'b0;
                        `RM_RP: if (!mul_sign && (guard || sticky)) is_tiny = 1'b0;
                        `RM_RM: if (mul_sign && (guard || sticky)) is_tiny = 1'b0;
                        default: ;
                    endcase
                end
            end
            if (exp_r < 15'sd1) begin
                denorm_shift = (exp_r < -15'sd24) ? 6'd25 : (6'd1 - exp_r[5:0]);
                jammed = shrjam24(man_norm, guard, sticky, denorm_shift);
                {man_norm, guard, sticky} = jammed;
                exp_r = 15'sd0;
            end

            s_mul_pre_exp           = exp_r;
            s_mul_pre_man           = man_norm;
            s_mul_pre_guard         = guard;
            s_mul_pre_sticky        = sticky;
            s_mul_pre_is_tiny       = is_tiny;
            s_mul_pre_exc_inexact   = guard || sticky;
            s_mul_pre_exc_underflow = is_tiny && (guard || sticky);
        end
    end

    always @(*) begin : mul_preround_d
        reg signed [14:0] exp_r;
        reg [52:0] man_norm;
        reg guard, sticky, is_tiny;
        reg [6:0] denorm_shift;
        reg [54:0] jammed;

        d_mul_pre_exp           = 15'sd0;
        d_mul_pre_man           = 53'd0;
        d_mul_pre_guard         = 1'b0;
        d_mul_pre_sticky        = 1'b0;
        d_mul_pre_is_tiny       = 1'b0;
        d_mul_pre_exc_underflow = 1'b0;
        d_mul_pre_exc_inexact   = 1'b0;

        if (mul_is_double && !mul_special) begin
            exp_r = mul_exp;

            if (mul_prod[105]) begin
                exp_r = exp_r + 15'sd1;
                man_norm = mul_prod[105:53];
                guard = mul_prod[52];
                sticky = |mul_prod[51:0];
            end else begin
                man_norm = mul_prod[104:52];
                guard = mul_prod[51];
                sticky = |mul_prod[50:0];
            end

            is_tiny = (exp_r < 15'sd1);
            if (is_tiny) begin
                if (exp_r == 15'sd0 && man_norm == 53'h1FFFFFFFFFFFFF) begin
                    case (mul_rm)
                        `RM_RN: if (guard) is_tiny = 1'b0;
                        `RM_RP: if (!mul_sign && (guard || sticky)) is_tiny = 1'b0;
                        `RM_RM: if (mul_sign && (guard || sticky)) is_tiny = 1'b0;
                        default: ;
                    endcase
                end
            end
            if (exp_r < 15'sd1) begin
                denorm_shift = (exp_r < -15'sd53) ? 7'd54 : (7'd1 - exp_r[6:0]);
                jammed = shrjam53(man_norm, guard, sticky, denorm_shift);
                {man_norm, guard, sticky} = jammed;
                exp_r = 15'sd0;
            end

            d_mul_pre_exp           = exp_r;
            d_mul_pre_man           = man_norm;
            d_mul_pre_guard         = guard;
            d_mul_pre_sticky        = sticky;
            d_mul_pre_is_tiny       = is_tiny;
            d_mul_pre_exc_inexact   = guard || sticky;
            d_mul_pre_exc_underflow = is_tiny && (guard || sticky);
        end
    end

    always @(*) begin : mul_round_s
        reg signed [14:0] exp_r;
        reg [23:0] man_norm;
        reg guard, sticky, round_up, lsb;

        s_mul_result        = 32'd0;
        s_mul_exc_invalid   = 1'b0;
        s_mul_exc_overflow  = 1'b0;
        s_mul_exc_underflow = 1'b0;
        s_mul_exc_inexact   = 1'b0;

        if (!mul_round_is_double) begin
            exp_r = mul_round_exp;
            man_norm = mul_round_man[23:0];
            guard = mul_round_guard;
            sticky = mul_round_sticky;
            s_mul_exc_underflow = mul_round_exc_underflow;
            s_mul_exc_inexact = mul_round_exc_inexact;

            lsb = man_norm[0];
            round_up = 1'b0;
            case (mul_round_rm)
                `RM_RN: round_up = guard && (sticky || lsb);
                `RM_RP: round_up = !mul_round_sign && (guard || sticky);
                `RM_RM: round_up = mul_round_sign && (guard || sticky);
                default: ;
            endcase

            s_mul_result = {mul_round_sign, exp_r[7:0], man_norm[22:0]};
            if (round_up)
                s_mul_result = s_mul_result + 32'd1;

            if (!mul_round_is_tiny && (exp_r >= 15'sd255 || s_mul_result[30:23] == 8'hFF)) begin
                s_mul_exc_overflow = 1'b1;
                s_mul_exc_inexact = 1'b1;
                case (mul_round_rm)
                    `RM_RN: s_mul_result = {mul_round_sign, 8'hFF, 23'd0};
                    `RM_RZ: s_mul_result = {mul_round_sign, 8'hFE, 23'h7FFFFF};
                    `RM_RP: s_mul_result = mul_round_sign ?
                        {1'b1, 8'hFE, 23'h7FFFFF} : {1'b0, 8'hFF, 23'd0};
                    `RM_RM: s_mul_result = mul_round_sign ?
                        {1'b1, 8'hFF, 23'd0} : {1'b0, 8'hFE, 23'h7FFFFF};
                endcase
            end

            if (mul_round_flush_denorms &&
                s_mul_result[30:23] == 8'h00 && s_mul_result[22:0] != 23'd0) begin
                s_mul_exc_underflow = 1'b1;
                s_mul_result = {mul_round_sign, 31'd0};
            end
        end
    end

    always @(*) begin : mul_round_d
        reg signed [14:0] exp_r;
        reg [52:0] man_norm;
        reg guard, sticky, round_up, lsb;

        d_mul_result        = 64'd0;
        d_mul_exc_invalid   = 1'b0;
        d_mul_exc_overflow  = 1'b0;
        d_mul_exc_underflow = 1'b0;
        d_mul_exc_inexact   = 1'b0;

        if (mul_round_is_double) begin
            exp_r = mul_round_exp;
            man_norm = mul_round_man;
            guard = mul_round_guard;
            sticky = mul_round_sticky;
            d_mul_exc_underflow = mul_round_exc_underflow;
            d_mul_exc_inexact = mul_round_exc_inexact;

            lsb = man_norm[0];
            round_up = 1'b0;
            case (mul_round_rm)
                `RM_RN: round_up = guard && (sticky || lsb);
                `RM_RP: round_up = !mul_round_sign && (guard || sticky);
                `RM_RM: round_up = mul_round_sign && (guard || sticky);
                default: ;
            endcase

            d_mul_result = {mul_round_sign, exp_r[10:0], man_norm[51:0]};
            if (round_up)
                d_mul_result = d_mul_result + 64'd1;

            if (!mul_round_is_tiny && (exp_r >= 15'sd2047 || d_mul_result[62:52] == 11'h7FF)) begin
                d_mul_exc_overflow = 1'b1;
                d_mul_exc_inexact = 1'b1;
                case (mul_round_rm)
                    `RM_RN: d_mul_result = {mul_round_sign, 11'h7FF, 52'd0};
                    `RM_RZ: d_mul_result = {mul_round_sign, 11'h7FE, 52'hFFFFFFFFFFFFF};
                    `RM_RP: d_mul_result = mul_round_sign ?
                        {1'b1, 11'h7FE, 52'hFFFFFFFFFFFFF} : {1'b0, 11'h7FF, 52'd0};
                    `RM_RM: d_mul_result = mul_round_sign ?
                        {1'b1, 11'h7FF, 52'd0} : {1'b0, 11'h7FE, 52'hFFFFFFFFFFFFF};
                endcase
            end

            if (mul_round_flush_denorms &&
                d_mul_result[62:52] == 11'h000 && d_mul_result[51:0] != 52'd0) begin
                d_mul_exc_underflow = 1'b1;
                d_mul_result = {mul_round_sign, 63'd0};
            end
        end
    end

    // ---------------------------------------------------------------
    // MADD/MSUB normalize logic: reads madd_* add result state and produces
    // the registered pre-round stage used by MUL_MADD_ROUND.
    // ---------------------------------------------------------------
    reg [31:0] s_maddnorm_result;
    reg        s_maddnorm_roundable;
    reg        s_maddnorm_sign;
    reg [8:0]  s_maddnorm_exp;
    reg [25:0] s_maddnorm_man;
    reg        s_maddnorm_exc_invalid;
    reg        s_maddnorm_exc_overflow;
    reg        s_maddnorm_exc_underflow;
    reg        s_maddnorm_exc_inexact;

    reg [63:0] d_maddnorm_result;
    reg        d_maddnorm_roundable;
    reg        d_maddnorm_sign;
    reg [11:0] d_maddnorm_exp;
    reg [54:0] d_maddnorm_man;
    reg        d_maddnorm_exc_invalid;
    reg        d_maddnorm_exc_overflow;
    reg        d_maddnorm_exc_underflow;
    reg        d_maddnorm_exc_inexact;

    always @(*) begin : madd_norm_s
        reg [ 8:0] exp_r;
        reg [26:0] msum;
        reg [25:0] man_norm;
        reg [ 4:0] lzc;
        reg        sticky;

        s_maddnorm_result        = 32'd0;
        s_maddnorm_roundable     = 1'b0;
        s_maddnorm_sign          = madd_sign_r;
        s_maddnorm_exp           = 9'd0;
        s_maddnorm_man           = 26'd0;
        s_maddnorm_exc_invalid   = 1'b0;
        s_maddnorm_exc_overflow  = 1'b0;
        s_maddnorm_exc_underflow = 1'b0;
        s_maddnorm_exc_inexact   = 1'b0;

        if (!mul_is_double && madd_add_special) begin
            s_maddnorm_result = madd_add_special_result[31:0];
            {s_maddnorm_exc_invalid, s_maddnorm_exc_overflow, s_maddnorm_exc_underflow,
             s_maddnorm_exc_inexact} = {madd_add_special_exc[4], madd_add_special_exc[2:0]};
        end else if (!mul_is_double) begin
            exp_r = madd_exp_r[8:0];
            msum = madd_man_sum[26:0];
            sticky = madd_sticky;

            if (msum == 0) begin
                s_maddnorm_result = {(mul_rm == `RM_RM) ? 1'b1 : 1'b0, 31'd0};
            end else begin
                if (msum[26]) begin
                    sticky = sticky | msum[0];
                    msum = msum >> 1;
                    exp_r = exp_r + 9'd1;
                end

                lzc = lzc26(msum[25:0]);

                if (lzc > 0) begin
                    if ({4'd0, lzc} >= exp_r) begin
                        msum = msum << (exp_r - 1);
                        exp_r = 0;
                    end else begin
                        msum = msum << lzc;
                        exp_r = exp_r - {4'd0, lzc};
                    end
                end

                man_norm = msum[25:0];
                if (man_norm[1:0] != 2'b00 || sticky)
                    s_maddnorm_exc_inexact = 1'b1;

                s_maddnorm_roundable = 1'b1;
                s_maddnorm_sign = madd_sign_r;
                s_maddnorm_exp = exp_r;
                s_maddnorm_man = man_norm;
            end
        end
    end

    always @(*) begin : madd_norm_d
        reg [11:0] exp_r;
        reg [55:0] msum;
        reg [54:0] man_norm;
        reg [ 5:0] lzc;
        reg        sticky;

        d_maddnorm_result        = 64'd0;
        d_maddnorm_roundable     = 1'b0;
        d_maddnorm_sign          = madd_sign_r;
        d_maddnorm_exp           = 12'd0;
        d_maddnorm_man           = 55'd0;
        d_maddnorm_exc_invalid   = 1'b0;
        d_maddnorm_exc_overflow  = 1'b0;
        d_maddnorm_exc_underflow = 1'b0;
        d_maddnorm_exc_inexact   = 1'b0;

        if (mul_is_double && madd_add_special) begin
            d_maddnorm_result = madd_add_special_result;
            {d_maddnorm_exc_invalid, d_maddnorm_exc_overflow, d_maddnorm_exc_underflow,
             d_maddnorm_exc_inexact} = {madd_add_special_exc[4], madd_add_special_exc[2:0]};
        end else if (mul_is_double) begin
            exp_r = madd_exp_r;
            msum = madd_man_sum;
            sticky = madd_sticky;

            if (msum == 0) begin
                d_maddnorm_result = {(mul_rm == `RM_RM) ? 1'b1 : 1'b0, 63'd0};
            end else begin
                if (msum[55]) begin
                    sticky = sticky | msum[0];
                    msum = msum >> 1;
                    exp_r = exp_r + 12'd1;
                end

                lzc = lzc55(msum[54:0]);

                if (lzc > 0) begin
                    if ({6'd0, lzc} >= exp_r) begin
                        msum = msum << (exp_r - 1);
                        exp_r = 0;
                    end else begin
                        msum = msum << lzc;
                        exp_r = exp_r - {6'd0, lzc};
                    end
                end

                man_norm = msum[54:0];
                if (man_norm[1:0] != 2'b00 || sticky)
                    d_maddnorm_exc_inexact = 1'b1;

                d_maddnorm_roundable = 1'b1;
                d_maddnorm_sign = madd_sign_r;
                d_maddnorm_exp = exp_r;
                d_maddnorm_man = man_norm;
            end
        end
    end

    // ---------------------------------------------------------------
    // MADD/MSUB single-precision rounding (reads registered norm state)
    // ---------------------------------------------------------------
    reg [31:0] s_madd_result;
    reg        s_madd_exc_invalid;
    reg        s_madd_exc_overflow;
    reg        s_madd_exc_underflow;
    reg        s_madd_exc_inexact;

    always @(*) begin : madd_round_s
        reg [ 8:0] exp_r;
        reg [25:0] man_norm;

        s_madd_result = 32'd0;
        s_madd_exc_invalid = 1'b0;
        s_madd_exc_overflow = 1'b0;
        s_madd_exc_underflow = 1'b0;
        s_madd_exc_inexact = 1'b0;

        if (!mul_is_double) begin
            s_madd_result        = madd_norm_result_r[31:0];
            s_madd_exc_invalid   = madd_norm_exc_invalid_r;
            s_madd_exc_overflow  = madd_norm_exc_overflow_r;
            s_madd_exc_underflow = madd_norm_exc_underflow_r;
            s_madd_exc_inexact   = madd_norm_exc_inexact_r;

            if (madd_norm_roundable_r) begin
                exp_r = madd_norm_exp_r[8:0];
                man_norm = madd_norm_man_r[25:0];

                if (man_norm[1:0] != 2'b00) begin
                    case (mul_rm)
                        `RM_RN: begin
                            if (man_norm[1] && (man_norm[0] || man_norm[2])) begin
                                man_norm = man_norm + 26'd4;
                                if (man_norm[25] && !madd_norm_man_r[25])
                                    exp_r = exp_r + 9'd1;
                            end
                        end
                        `RM_RP: begin
                            if (!madd_norm_sign_r) begin
                                man_norm = man_norm + 26'd3;
                                if (man_norm[25] && !madd_norm_man_r[25])
                                    exp_r = exp_r + 9'd1;
                            end
                        end
                        `RM_RM: begin
                            if (madd_norm_sign_r) begin
                                man_norm = man_norm + 26'd3;
                                if (man_norm[25] && !madd_norm_man_r[25])
                                    exp_r = exp_r + 9'd1;
                            end
                        end
                        default: ;
                    endcase
                end

                if (exp_r >= 9'd255) begin
                    s_madd_exc_overflow = 1'b1;
                    s_madd_exc_inexact = 1'b1;
                    s_madd_result = {madd_norm_sign_r, 8'hFF, 23'd0};
                end else if (mul_flush_denorms && exp_r == 0) begin
                    s_madd_result = {madd_norm_sign_r, 31'd0};
                end else begin
                    s_madd_result = {madd_norm_sign_r, exp_r[7:0], man_norm[24:2]};
                end
            end
        end
    end

    // ---------------------------------------------------------------
    // MADD/MSUB double-precision rounding (reads registered norm state)
    // ---------------------------------------------------------------
    reg [63:0] d_madd_result;
    reg        d_madd_exc_invalid;
    reg        d_madd_exc_overflow;
    reg        d_madd_exc_underflow;
    reg        d_madd_exc_inexact;

    always @(*) begin : madd_round_d
        reg [11:0] exp_r;
        reg [54:0] man_norm;

        d_madd_result = 64'd0;
        d_madd_exc_invalid = 1'b0;
        d_madd_exc_overflow = 1'b0;
        d_madd_exc_underflow = 1'b0;
        d_madd_exc_inexact = 1'b0;

        if (mul_is_double) begin
            d_madd_result        = madd_norm_result_r;
            d_madd_exc_invalid   = madd_norm_exc_invalid_r;
            d_madd_exc_overflow  = madd_norm_exc_overflow_r;
            d_madd_exc_underflow = madd_norm_exc_underflow_r;
            d_madd_exc_inexact   = madd_norm_exc_inexact_r;

            if (madd_norm_roundable_r) begin
                exp_r = madd_norm_exp_r;
                man_norm = madd_norm_man_r;

                if (man_norm[1:0] != 2'b00) begin
                    case (mul_rm)
                        `RM_RN: begin
                            if (man_norm[1] && (man_norm[0] || man_norm[2]))
                                man_norm = man_norm + 55'd4;
                        end
                        `RM_RP: if (!madd_norm_sign_r) man_norm = man_norm + 55'd3;
                        `RM_RM: if (madd_norm_sign_r)  man_norm = man_norm + 55'd3;
                        default: ;
                    endcase
                end

                if (exp_r >= 12'd2047) begin
                    d_madd_exc_overflow = 1'b1;
                    d_madd_exc_inexact = 1'b1;
                    d_madd_result = {madd_norm_sign_r, 11'h7FF, 52'd0};
                end else if (mul_flush_denorms && exp_r == 0) begin
                    d_madd_exc_underflow = 1'b1;
                    d_madd_result = {madd_norm_sign_r, 63'd0};
                end else begin
                    d_madd_result = {madd_norm_sign_r, exp_r[10:0], man_norm[53:2]};
                end
            end
        end
    end

    // ---------------------------------------------------------------
    // Single/double-precision DIV results (driven by FSM + rounding logic)
    // ---------------------------------------------------------------
    reg [31:0] s_div_result;
    reg        s_div_exc_invalid;
    reg        s_div_exc_divzero;
    reg        s_div_exc_overflow;
    reg        s_div_exc_underflow;
    reg        s_div_exc_inexact;

    reg [63:0] d_div_result;
    reg        d_div_exc_invalid;
    reg        d_div_exc_divzero;
    reg        d_div_exc_overflow;
    reg        d_div_exc_underflow;
    reg        d_div_exc_inexact;

    // ---------------------------------------------------------------
    // DIV FSM — sequential restoring division
    // ---------------------------------------------------------------
    localparam [2:0] DIV_IDLE   = 3'd0,
                     DIV_START  = 3'd1,
                     DIV_ITER   = 3'd2,
                     DIV_DONE   = 3'd3,
                     DIV_ROUND  = 3'd4,
                     DIV_RESULT = 3'd5;

    reg [2:0]         div_state;
    reg [4:0]         div_fmt_r;
    reg [63:0]        div_fs_val_r;
    reg [63:0]        div_ft_val_r;
    reg [1:0]         div_rm;
    reg               div_flush_denorms;
    reg               div_is_double;
    reg               div_sign;
    reg signed [14:0] div_exp;
    reg [52:0]        div_man_b;
    reg [53:0]        div_rem;
    reg [53:0]        div_quot;
    reg [5:0]         div_count;
    reg               div_special;
    reg [63:0]        div_special_result;
    reg [4:0]         div_special_exc;
    reg [63:0]        div_result_r;
    reg               div_result_exc_invalid;
    reg               div_result_exc_divzero;
    reg               div_result_exc_overflow;
    reg               div_result_exc_underflow;
    reg               div_result_exc_inexact;

    reg               div_round_is_double;
    reg               div_round_sign;
    reg [1:0]         div_round_rm;
    reg               div_round_flush_denorms;
    reg signed [14:0] div_round_exp;
    reg [52:0]        div_round_man;
    reg               div_round_guard;
    reg               div_round_sticky;
    reg               div_round_is_tiny;
    reg               div_round_exc_underflow;
    reg               div_round_exc_inexact;

    reg signed [14:0] s_div_pre_exp;
    reg [23:0]        s_div_pre_man;
    reg               s_div_pre_guard;
    reg               s_div_pre_sticky;
    reg               s_div_pre_is_tiny;
    reg               s_div_pre_exc_underflow;
    reg               s_div_pre_exc_inexact;

    reg signed [14:0] d_div_pre_exp;
    reg [52:0]        d_div_pre_man;
    reg               d_div_pre_guard;
    reg               d_div_pre_sticky;
    reg               d_div_pre_is_tiny;
    reg               d_div_pre_exc_underflow;
    reg               d_div_pre_exc_inexact;

    // Registered DIV operands. Keep RF instruction decode and live FPR
    // classification out of the cycle that seeds the iterative divider.
    wire        div_s_sign_a = div_fs_val_r[31];
    wire [ 7:0] div_s_exp_a  = div_fs_val_r[30:23];
    wire [22:0] div_s_man_a  = div_fs_val_r[22:0];
    wire        div_s_sign_b = div_ft_val_r[31];
    wire [ 7:0] div_s_exp_b  = div_ft_val_r[30:23];
    wire [22:0] div_s_man_b  = div_ft_val_r[22:0];
    wire        div_d_sign_a = div_fs_val_r[63];
    wire [10:0] div_d_exp_a  = div_fs_val_r[62:52];
    wire [51:0] div_d_man_a  = div_fs_val_r[51:0];
    wire        div_d_sign_b = div_ft_val_r[63];
    wire [10:0] div_d_exp_b  = div_ft_val_r[62:52];
    wire [51:0] div_d_man_b  = div_ft_val_r[51:0];

    wire div_s_is_nan_a = (div_s_exp_a == 8'hFF) && (div_s_man_a != 0);
    wire div_s_is_nan_b = (div_s_exp_b == 8'hFF) && (div_s_man_b != 0);
    wire div_s_is_inf_a = (div_s_exp_a == 8'hFF) && (div_s_man_a == 0);
    wire div_s_is_inf_b = (div_s_exp_b == 8'hFF) && (div_s_man_b == 0);
    wire div_s_is_zero_a = (div_s_exp_a == 0) && (div_s_man_a == 0 || div_flush_denorms);
    wire div_s_is_zero_b = (div_s_exp_b == 0) && (div_s_man_b == 0 || div_flush_denorms);
    wire div_s_is_snan_a = div_s_is_nan_a && div_s_man_a[22];
    wire div_s_is_snan_b = div_s_is_nan_b && div_s_man_b[22];
    wire div_d_is_nan_a = (div_d_exp_a == 11'h7FF) && (div_d_man_a != 0);
    wire div_d_is_nan_b = (div_d_exp_b == 11'h7FF) && (div_d_man_b != 0);
    wire div_d_is_inf_a = (div_d_exp_a == 11'h7FF) && (div_d_man_a == 0);
    wire div_d_is_inf_b = (div_d_exp_b == 11'h7FF) && (div_d_man_b == 0);
    wire div_d_is_zero_a = (div_d_exp_a == 0) && (div_d_man_a == 0 || div_flush_denorms);
    wire div_d_is_zero_b = (div_d_exp_b == 0) && (div_d_man_b == 0 || div_flush_denorms);
    wire div_d_is_snan_a = div_d_is_nan_a && div_d_man_a[51];
    wire div_d_is_snan_b = div_d_is_nan_b && div_d_man_b[51];
    wire [31:0] div_s_prop_nan = quiet_legacy_nan32(
        div_s_is_nan_a ? div_fs_val_r[31:0] : div_ft_val_r[31:0]);
    wire [63:0] div_d_prop_nan = quiet_legacy_nan64(
        div_d_is_nan_a ? div_fs_val_r : div_ft_val_r);

    localparam [2:0] CVT_IDLE          = 3'd0,
                     CVT_CALC          = 3'd1,
                     CVT_RESULT        = 3'd2,
                     CVT_S_D_ROUND     = 3'd3,
                     CVT_L_FLOAT_ROUND = 3'd4;

    reg [2:0]  cvt_state;
    reg [63:0] cvt_fs_val_r;
    reg [ 4:0] cvt_fmt_r;
    reg [ 5:0] cvt_funct_r;
    reg [ 1:0] cvt_rm_r;
    reg        cvt_flush_denorms_r;
    reg [63:0] cvt_result_r;
    reg        cvt_exc_r;
    reg        cvt_exc_unimpl_r;
    reg [ 4:0] cvt_exc_flags_r;

    reg        cvt_s_d_round_special;
    reg [31:0] cvt_s_d_round_special_result;
    reg        cvt_s_d_round_special_exc_invalid;
    reg        cvt_s_d_round_sign;
    reg [ 1:0] cvt_s_d_round_rm;
    reg        cvt_s_d_round_flush_denorms;
    reg signed [11:0] cvt_s_d_round_exp;
    reg [23:0] cvt_s_d_round_man;
    reg        cvt_s_d_round_guard;
    reg        cvt_s_d_round_sticky;
    reg        cvt_s_d_round_is_tiny;
    reg        cvt_s_d_round_exc_underflow;
    reg        cvt_s_d_round_exc_inexact;
    reg        cvt_l_float_sign;
    reg [63:0] cvt_l_float_abs;

    wire fpu_cvt_op = FPU_HAS_CVT && do_fpop &&
        (((fmt == `COP1_FMT_S) &&
          (funct == `FPU_CVT_D || funct == `FPU_CVT_W || funct == `FPU_CVT_L ||
           funct == `FPU_ROUND_W || funct == `FPU_TRUNC_W ||
           funct == `FPU_CEIL_W  || funct == `FPU_FLOOR_W ||
           funct == `FPU_ROUND_L || funct == `FPU_TRUNC_L ||
           funct == `FPU_CEIL_L  || funct == `FPU_FLOOR_L)) ||
         ((fmt == `COP1_FMT_D) &&
          (funct == `FPU_CVT_S || funct == `FPU_CVT_W || funct == `FPU_CVT_L ||
           funct == `FPU_ROUND_W || funct == `FPU_TRUNC_W ||
           funct == `FPU_CEIL_W  || funct == `FPU_FLOOR_W ||
           funct == `FPU_ROUND_L || funct == `FPU_TRUNC_L ||
           funct == `FPU_CEIL_L  || funct == `FPU_FLOOR_L)) ||
         ((fmt == `COP1_FMT_W) &&
          (funct == `FPU_CVT_S || funct == `FPU_CVT_D)) ||
         ((fmt == `COP1_FMT_L) &&
          (funct == `FPU_CVT_S || funct == `FPU_CVT_D)));

    assign fpu_busy         = (div_state != DIV_IDLE) || (sqrt_state != SQRT_IDLE) ||
                              (mul_state != MUL_IDLE) || (add_state != ADD_IDLE) ||
                              (cvt_state != CVT_IDLE);
    assign fpu_result_valid = (div_state == DIV_RESULT) || (sqrt_state == SQRT_RESULT) ||
                              (mul_state == MUL_RESULT) || (mul_state == MUL_MADD_RESULT) ||
                              (add_state == ADD_RESULT) || (cvt_state == CVT_RESULT);

    wire [54:0] fdiv_trial = {1'b0, div_rem} - {2'b00, div_man_b};

    `R4000_FPU_SEQ begin
        if (!rst_n) begin
            div_state          <= DIV_IDLE;
            div_fmt_r          <= 5'd0;
            div_fs_val_r       <= 64'd0;
            div_ft_val_r       <= 64'd0;
            div_rm             <= `RM_RN;
            div_flush_denorms  <= 1'b0;
            div_is_double      <= 1'b0;
            div_sign           <= 1'b0;
            div_exp            <= 15'sd0;
            div_man_b          <= 53'd0;
            div_rem            <= 54'd0;
            div_quot           <= 54'd0;
            div_count          <= 6'd0;
            div_special        <= 1'b0;
            div_special_result <= 64'd0;
            div_special_exc    <= 5'd0;
            div_result_r             <= 64'd0;
            div_result_exc_invalid   <= 1'b0;
            div_result_exc_divzero   <= 1'b0;
            div_result_exc_overflow  <= 1'b0;
            div_result_exc_underflow <= 1'b0;
            div_result_exc_inexact   <= 1'b0;
            div_round_is_double      <= 1'b0;
            div_round_sign           <= 1'b0;
            div_round_rm             <= 2'd0;
            div_round_flush_denorms  <= 1'b0;
            div_round_exp            <= 15'sd0;
            div_round_man            <= 53'd0;
            div_round_guard          <= 1'b0;
            div_round_sticky         <= 1'b0;
            div_round_is_tiny        <= 1'b0;
            div_round_exc_underflow  <= 1'b0;
            div_round_exc_inexact    <= 1'b0;
        end else if (fpu_cancel) begin
            div_state <= DIV_IDLE;
        end else begin
            case (div_state)
                DIV_IDLE: begin
                    if (FPU_HAS_DIV && do_fpop && funct == `FPU_DIV) begin
                        div_fmt_r <= fmt;
                        div_fs_val_r <= fs_val;
                        div_ft_val_r <= ft_val;
                        div_rm <= rm;
                        div_flush_denorms <= flush_denorms;
                        div_state <= DIV_START;
                    end
                end

                DIV_START: begin
                        if (div_fmt_r == `COP1_FMT_D) begin : div_start_d
                            reg [52:0] ma, mb;
                            reg [5:0] lza, lzb;
                            reg signed [14:0] expr;

                            div_is_double <= 1'b1;
                            div_sign      <= div_d_sign_a ^ div_d_sign_b;

                            if (div_d_is_nan_a || div_d_is_nan_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= div_d_prop_nan;
                                div_special_exc    <= {(div_d_is_snan_a || div_d_is_snan_b), 4'd0};
                                div_state          <= DIV_DONE;
                            end else if (div_d_is_inf_a && div_d_is_inf_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= d_default_qnan;
                                div_special_exc    <= 5'b10000;
                                div_state          <= DIV_DONE;
                            end else if (div_d_is_inf_a) begin
                                div_special        <= 1'b1;
                                div_special_result <= {div_d_sign_a ^ div_d_sign_b, 11'h7FF, 52'd0};
                                div_special_exc    <= 5'd0;
                                div_state          <= DIV_DONE;
                            end else if (div_d_is_zero_a && div_d_is_zero_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= d_default_qnan;
                                div_special_exc    <= 5'b10000;
                                div_state          <= DIV_DONE;
                            end else if (div_d_is_zero_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= {div_d_sign_a ^ div_d_sign_b, 11'h7FF, 52'd0};
                                div_special_exc    <= 5'b01000;
                                div_state          <= DIV_DONE;
                            end else if (div_d_is_zero_a || div_d_is_inf_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= {div_d_sign_a ^ div_d_sign_b, 63'd0};
                                div_special_exc    <= 5'd0;
                                div_state          <= DIV_DONE;
                            end else begin
                                ma = (div_d_exp_a == 0) ? {1'b0, div_d_man_a} : {1'b1, div_d_man_a};
                                mb = (div_d_exp_b == 0) ? {1'b0, div_d_man_b} : {1'b1, div_d_man_b};
                                lza = 6'd0;
                                if (div_d_exp_a == 0) begin
                                    lza = lzc53(ma);
                                    ma = ma << lza;
                                end
                                lzb = 6'd0;
                                if (div_d_exp_b == 0) begin
                                    lzb = lzc53(mb);
                                    mb = mb << lzb;
                                end
                                expr = $signed({4'd0, div_d_exp_a}) - $signed({4'd0, div_d_exp_b}) + 15'sd1023;
                                if (div_d_exp_a == 0) expr = expr + 15'sd1 - $signed({9'd0, lza});
                                if (div_d_exp_b == 0) expr = expr - 15'sd1 + $signed({9'd0, lzb});
                                div_exp     <= expr;
                                div_rem     <= {1'b0, ma};
                                div_man_b   <= mb;
                                div_quot    <= 54'd0;
                                div_count   <= 6'd54;
                                div_special <= 1'b0;
                                div_state   <= DIV_ITER;
                            end
                        end else begin : div_start_s
                            reg [23:0] ma, mb;
                            reg [4:0] lza, lzb;
                            reg signed [14:0] expr;

                            div_is_double <= 1'b0;
                            div_sign      <= div_s_sign_a ^ div_s_sign_b;

                            if (div_s_is_nan_a || div_s_is_nan_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= {32'd0, div_s_prop_nan};
                                div_special_exc    <= {(div_s_is_snan_a || div_s_is_snan_b), 4'd0};
                                div_state          <= DIV_DONE;
                            end else if (div_s_is_inf_a && div_s_is_inf_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= {32'd0, s_default_qnan};
                                div_special_exc    <= 5'b10000;
                                div_state          <= DIV_DONE;
                            end else if (div_s_is_inf_a) begin
                                div_special        <= 1'b1;
                                div_special_result <= {32'd0, div_s_sign_a ^ div_s_sign_b, 8'hFF, 23'd0};
                                div_special_exc    <= 5'd0;
                                div_state          <= DIV_DONE;
                            end else if (div_s_is_zero_a && div_s_is_zero_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= {32'd0, s_default_qnan};
                                div_special_exc    <= 5'b10000;
                                div_state          <= DIV_DONE;
                            end else if (div_s_is_zero_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= {32'd0, div_s_sign_a ^ div_s_sign_b, 8'hFF, 23'd0};
                                div_special_exc    <= 5'b01000;
                                div_state          <= DIV_DONE;
                            end else if (div_s_is_zero_a || div_s_is_inf_b) begin
                                div_special        <= 1'b1;
                                div_special_result <= {32'd0, div_s_sign_a ^ div_s_sign_b, 31'd0};
                                div_special_exc    <= 5'd0;
                                div_state          <= DIV_DONE;
                            end else begin
                                ma = (div_s_exp_a == 0) ? {1'b0, div_s_man_a} : {1'b1, div_s_man_a};
                                mb = (div_s_exp_b == 0) ? {1'b0, div_s_man_b} : {1'b1, div_s_man_b};
                                lza = 5'd0;
                                if (div_s_exp_a == 0) begin
                                    lza = lzc24(ma);
                                    ma = ma << lza;
                                end
                                lzb = 5'd0;
                                if (div_s_exp_b == 0) begin
                                    lzb = lzc24(mb);
                                    mb = mb << lzb;
                                end
                                expr = $signed({7'd0, div_s_exp_a}) - $signed({7'd0, div_s_exp_b}) + 15'sd127;
                                if (div_s_exp_a == 0) expr = expr + 15'sd1 - $signed({10'd0, lza});
                                if (div_s_exp_b == 0) expr = expr - 15'sd1 + $signed({10'd0, lzb});
                                div_exp     <= expr;
                                div_rem     <= {29'd0, 1'b0, ma};
                                div_man_b   <= {29'd0, mb};
                                div_quot    <= 54'd0;
                                div_count   <= 6'd25;
                                div_special <= 1'b0;
                                div_state   <= DIV_ITER;
                            end
                        end
                end

                DIV_ITER: begin
                    if (!fdiv_trial[54]) begin
                        div_rem  <= {fdiv_trial[52:0], 1'b0};
                        div_quot <= {div_quot[52:0], 1'b1};
                    end else begin
                        div_rem  <= {div_rem[52:0], 1'b0};
                        div_quot <= {div_quot[52:0], 1'b0};
                    end
                    div_count <= div_count - 6'd1;
                    if (div_count == 6'd1)
                        div_state <= DIV_DONE;
                end

                DIV_DONE: begin
                    if (div_special) begin
                        div_result_r             <= div_is_double ? div_special_result :
                                                    {32'd0, div_special_result[31:0]};
                        div_result_exc_invalid   <= div_special_exc[4];
                        div_result_exc_divzero   <= div_special_exc[3];
                        div_result_exc_overflow  <= div_special_exc[2];
                        div_result_exc_underflow <= div_special_exc[1];
                        div_result_exc_inexact   <= div_special_exc[0];
                        div_state                <= DIV_RESULT;
                    end else if (!div_is_double) begin
                        div_round_is_double     <= 1'b0;
                        div_round_sign          <= div_sign;
                        div_round_rm            <= div_rm;
                        div_round_flush_denorms <= div_flush_denorms;
                        div_round_exp           <= s_div_pre_exp;
                        div_round_man           <= {29'd0, s_div_pre_man};
                        div_round_guard         <= s_div_pre_guard;
                        div_round_sticky        <= s_div_pre_sticky;
                        div_round_is_tiny       <= s_div_pre_is_tiny;
                        div_round_exc_underflow <= s_div_pre_exc_underflow;
                        div_round_exc_inexact   <= s_div_pre_exc_inexact;
                        div_state               <= DIV_ROUND;
                    end else begin
                        div_round_is_double     <= 1'b1;
                        div_round_sign          <= div_sign;
                        div_round_rm            <= div_rm;
                        div_round_flush_denorms <= div_flush_denorms;
                        div_round_exp           <= d_div_pre_exp;
                        div_round_man           <= d_div_pre_man;
                        div_round_guard         <= d_div_pre_guard;
                        div_round_sticky        <= d_div_pre_sticky;
                        div_round_is_tiny       <= d_div_pre_is_tiny;
                        div_round_exc_underflow <= d_div_pre_exc_underflow;
                        div_round_exc_inexact   <= d_div_pre_exc_inexact;
                        div_state               <= DIV_ROUND;
                    end
                end

                DIV_ROUND: begin
                    if (!div_round_is_double) begin
                        div_result_r             <= {32'd0, s_div_result};
                        div_result_exc_invalid   <= s_div_exc_invalid;
                        div_result_exc_divzero   <= s_div_exc_divzero;
                        div_result_exc_overflow  <= s_div_exc_overflow;
                        div_result_exc_underflow <= s_div_exc_underflow;
                        div_result_exc_inexact   <= s_div_exc_inexact;
                    end else begin
                        div_result_r             <= d_div_result;
                        div_result_exc_invalid   <= d_div_exc_invalid;
                        div_result_exc_divzero   <= d_div_exc_divzero;
                        div_result_exc_overflow  <= d_div_exc_overflow;
                        div_result_exc_underflow <= d_div_exc_underflow;
                        div_result_exc_inexact   <= d_div_exc_inexact;
                    end
                    div_state <= DIV_RESULT;
                end

                DIV_RESULT: begin
                    div_state <= DIV_IDLE;
                end

                default: div_state <= DIV_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // DIV normalize/pre-round logic — combinational, reads FSM registers
    // and is latched by DIV_DONE before final rounding.
    // ---------------------------------------------------------------
    always @(*) begin : div_preround_s
        reg signed [14:0] exp_r;
        reg [23:0] man_norm;
        reg guard, sticky, is_tiny;
        reg [5:0] denorm_shift;
        reg [25:0] jammed;

        s_div_pre_exp           = 15'sd0;
        s_div_pre_man           = 24'd0;
        s_div_pre_guard         = 1'b0;
        s_div_pre_sticky        = 1'b0;
        s_div_pre_is_tiny       = 1'b0;
        s_div_pre_exc_underflow = 1'b0;
        s_div_pre_exc_inexact   = 1'b0;

        if (!div_is_double && !div_special) begin
            exp_r = div_exp;

            if (div_quot[24]) begin
                man_norm = div_quot[24:1];
                guard = div_quot[0];
                sticky = (div_rem != 54'd0);
            end else begin
                exp_r = exp_r - 15'sd1;
                man_norm = div_quot[23:0];
                if ({1'b0, div_rem} >= {2'b00, div_man_b}) begin
                    guard = 1'b1;
                    sticky = (({1'b0, div_rem} - {2'b00, div_man_b}) != 55'd0);
                end else begin
                    guard = 1'b0;
                    sticky = (div_rem != 54'd0);
                end
            end

            is_tiny = (exp_r < 15'sd1);
            if (is_tiny) begin
                if (exp_r == 15'sd0 && man_norm == 24'hFFFFFF) begin
                    case (div_rm)
                        `RM_RN: if (guard) is_tiny = 1'b0;
                        `RM_RP: if (!div_sign && (guard || sticky)) is_tiny = 1'b0;
                        `RM_RM: if (div_sign && (guard || sticky)) is_tiny = 1'b0;
                        default: ;
                    endcase
                end
            end
            if (exp_r < 15'sd1) begin
                denorm_shift = (exp_r < -15'sd24) ? 6'd25 : (6'd1 - exp_r[5:0]);
                jammed = shrjam24(man_norm, guard, sticky, denorm_shift);
                {man_norm, guard, sticky} = jammed;
                exp_r = 15'sd0;
            end

            s_div_pre_exp           = exp_r;
            s_div_pre_man           = man_norm;
            s_div_pre_guard         = guard;
            s_div_pre_sticky        = sticky;
            s_div_pre_is_tiny       = is_tiny;
            s_div_pre_exc_inexact   = guard || sticky;
            s_div_pre_exc_underflow = is_tiny && (guard || sticky);
        end
    end

    always @(*) begin : div_preround_d
        reg signed [14:0] exp_r;
        reg [52:0] man_norm;
        reg guard, sticky, is_tiny;
        reg [6:0] denorm_shift;
        reg [54:0] jammed;

        d_div_pre_exp           = 15'sd0;
        d_div_pre_man           = 53'd0;
        d_div_pre_guard         = 1'b0;
        d_div_pre_sticky        = 1'b0;
        d_div_pre_is_tiny       = 1'b0;
        d_div_pre_exc_underflow = 1'b0;
        d_div_pre_exc_inexact   = 1'b0;

        if (div_is_double && !div_special) begin
            exp_r = div_exp;

            if (div_quot[53]) begin
                man_norm = div_quot[53:1];
                guard = div_quot[0];
                sticky = (div_rem != 54'd0);
            end else begin
                exp_r = exp_r - 15'sd1;
                man_norm = div_quot[52:0];
                if ({1'b0, div_rem} >= {2'b00, div_man_b}) begin
                    guard = 1'b1;
                    sticky = (({1'b0, div_rem} - {2'b00, div_man_b}) != 55'd0);
                end else begin
                    guard = 1'b0;
                    sticky = (div_rem != 54'd0);
                end
            end

            is_tiny = (exp_r < 15'sd1);
            if (is_tiny) begin
                if (exp_r == 15'sd0 && man_norm == 53'h1FFFFFFFFFFFFF) begin
                    case (div_rm)
                        `RM_RN: if (guard) is_tiny = 1'b0;
                        `RM_RP: if (!div_sign && (guard || sticky)) is_tiny = 1'b0;
                        `RM_RM: if (div_sign && (guard || sticky)) is_tiny = 1'b0;
                        default: ;
                    endcase
                end
            end
            if (exp_r < 15'sd1) begin
                denorm_shift = (exp_r < -15'sd53) ? 7'd54 : (7'd1 - exp_r[6:0]);
                jammed = shrjam53(man_norm, guard, sticky, denorm_shift);
                {man_norm, guard, sticky} = jammed;
                exp_r = 15'sd0;
            end

            d_div_pre_exp           = exp_r;
            d_div_pre_man           = man_norm;
            d_div_pre_guard         = guard;
            d_div_pre_sticky        = sticky;
            d_div_pre_is_tiny       = is_tiny;
            d_div_pre_exc_inexact   = guard || sticky;
            d_div_pre_exc_underflow = is_tiny && (guard || sticky);
        end
    end

    always @(*) begin : div_round_s
        reg signed [14:0] exp_r;
        reg [23:0] man_norm;
        reg guard, sticky, round_up, lsb;

        s_div_result        = 32'd0;
        s_div_exc_invalid   = 1'b0;
        s_div_exc_divzero   = 1'b0;
        s_div_exc_overflow  = 1'b0;
        s_div_exc_underflow = 1'b0;
        s_div_exc_inexact   = 1'b0;

        if (!div_round_is_double) begin
            exp_r = div_round_exp;
            man_norm = div_round_man[23:0];
            guard = div_round_guard;
            sticky = div_round_sticky;
            s_div_exc_underflow = div_round_exc_underflow;
            s_div_exc_inexact = div_round_exc_inexact;

            lsb = man_norm[0];
            round_up = 1'b0;
            case (div_round_rm)
                `RM_RN: round_up = guard && (sticky || lsb);
                `RM_RP: round_up = !div_round_sign && (guard || sticky);
                `RM_RM: round_up = div_round_sign && (guard || sticky);
                default: ;
            endcase

            s_div_result = {div_round_sign, exp_r[7:0], man_norm[22:0]};
            if (round_up)
                s_div_result = s_div_result + 32'd1;

            if (!div_round_is_tiny && (exp_r >= 15'sd255 || s_div_result[30:23] == 8'hFF)) begin
                s_div_exc_overflow = 1'b1;
                s_div_exc_inexact = 1'b1;
                case (div_round_rm)
                    `RM_RN: s_div_result = {div_round_sign, 8'hFF, 23'd0};
                    `RM_RZ: s_div_result = {div_round_sign, 8'hFE, 23'h7FFFFF};
                    `RM_RP: s_div_result = div_round_sign ?
                        {1'b1, 8'hFE, 23'h7FFFFF} : {1'b0, 8'hFF, 23'd0};
                    `RM_RM: s_div_result = div_round_sign ?
                        {1'b1, 8'hFF, 23'd0} : {1'b0, 8'hFE, 23'h7FFFFF};
                endcase
            end

            if (div_round_flush_denorms &&
                s_div_result[30:23] == 8'h00 && s_div_result[22:0] != 23'd0) begin
                s_div_exc_underflow = 1'b1;
                s_div_result = {div_round_sign, 31'd0};
            end
        end
    end

    always @(*) begin : div_round_d
        reg signed [14:0] exp_r;
        reg [52:0] man_norm;
        reg guard, sticky, round_up, lsb;

        d_div_result        = 64'd0;
        d_div_exc_invalid   = 1'b0;
        d_div_exc_divzero   = 1'b0;
        d_div_exc_overflow  = 1'b0;
        d_div_exc_underflow = 1'b0;
        d_div_exc_inexact   = 1'b0;

        if (div_round_is_double) begin
            exp_r = div_round_exp;
            man_norm = div_round_man;
            guard = div_round_guard;
            sticky = div_round_sticky;
            d_div_exc_underflow = div_round_exc_underflow;
            d_div_exc_inexact = div_round_exc_inexact;

            lsb = man_norm[0];
            round_up = 1'b0;
            case (div_round_rm)
                `RM_RN: round_up = guard && (sticky || lsb);
                `RM_RP: round_up = !div_round_sign && (guard || sticky);
                `RM_RM: round_up = div_round_sign && (guard || sticky);
                default: ;
            endcase

            d_div_result = {div_round_sign, exp_r[10:0], man_norm[51:0]};
            if (round_up)
                d_div_result = d_div_result + 64'd1;

            if (!div_round_is_tiny && (exp_r >= 15'sd2047 || d_div_result[62:52] == 11'h7FF)) begin
                d_div_exc_overflow = 1'b1;
                d_div_exc_inexact = 1'b1;
                case (div_round_rm)
                    `RM_RN: d_div_result = {div_round_sign, 11'h7FF, 52'd0};
                    `RM_RZ: d_div_result = {div_round_sign, 11'h7FE, 52'hFFFFFFFFFFFFF};
                    `RM_RP: d_div_result = div_round_sign ?
                        {1'b1, 11'h7FE, 52'hFFFFFFFFFFFFF} : {1'b0, 11'h7FF, 52'd0};
                    `RM_RM: d_div_result = div_round_sign ?
                        {1'b1, 11'h7FF, 52'd0} : {1'b0, 11'h7FE, 52'hFFFFFFFFFFFFF};
                endcase
            end

            if (div_round_flush_denorms &&
                d_div_result[62:52] == 11'h000 && d_div_result[51:0] != 52'd0) begin
                d_div_exc_underflow = 1'b1;
                d_div_result = {div_round_sign, 63'd0};
            end
        end
    end

    // ---------------------------------------------------------------
    // Comparisons (C.cond) — single and double
    // ---------------------------------------------------------------
    reg        s_cmp_result;
    reg        s_cmp_exc_invalid;
    reg        d_cmp_result;
    reg        d_cmp_exc_invalid;

    always @(*) begin : s_cmp_block
        reg [3:0] cond;
        reg       less, equal, unordered;

        cond = funct[3:0];
        s_cmp_result = 1'b0;
        s_cmp_exc_invalid = 1'b0;

        unordered = s_is_nan_a || s_is_nan_b;
        if (unordered) begin
            less = 1'b0;
            equal = 1'b0;
            // Signal invalid for signaling conditions (cond[3] set)
            if (cond[3] || s_is_snan_a || s_is_snan_b)
                s_cmp_exc_invalid = 1'b1;
        end else begin
            if (s_is_zero_a && s_is_zero_b)
                equal = 1'b1;
            else
                equal = (fs_val[31:0] == ft_val[31:0]);

            if (equal)
                less = 1'b0;
            else if (s_sign_a != s_sign_b)
                less = s_sign_a;
            else if (s_sign_a)
                less = (fs_val[30:0] > ft_val[30:0]);
            else
                less = (fs_val[30:0] < ft_val[30:0]);
        end

        // cond encoding: {FC, FL, FE, FU}
        s_cmp_result = (cond[0] & unordered) | (cond[1] & equal) | (cond[2] & less);
    end

    always @(*) begin : d_cmp_block
        reg [3:0] cond;
        reg       less, equal, unordered;

        cond = funct[3:0];
        d_cmp_result = 1'b0;
        d_cmp_exc_invalid = 1'b0;

        unordered = d_is_nan_a || d_is_nan_b;
        if (unordered) begin
            less = 1'b0;
            equal = 1'b0;
            if (cond[3] || d_is_snan_a || d_is_snan_b)
                d_cmp_exc_invalid = 1'b1;
        end else begin
            if (d_is_zero_a && d_is_zero_b)
                equal = 1'b1;
            else
                equal = (fs_val == ft_val);

            if (equal)
                less = 1'b0;
            else if (d_sign_a != d_sign_b)
                less = d_sign_a;
            else if (d_sign_a)
                less = (fs_val[62:0] > ft_val[62:0]);
            else
                less = (fs_val[62:0] < ft_val[62:0]);
        end

        d_cmp_result = (cond[0] & unordered) | (cond[1] & equal) | (cond[2] & less);
    end

    // ---------------------------------------------------------------
    // Single/double-precision SQRT results (driven by FSM + rounding logic)
    // ---------------------------------------------------------------
    reg [31:0] s_sqrt_result;
    reg        s_sqrt_exc_invalid;
    reg        s_sqrt_exc_inexact;

    reg [63:0] d_sqrt_result;
    reg        d_sqrt_exc_invalid;
    reg        d_sqrt_exc_inexact;

    // ---------------------------------------------------------------
    // SQRT FSM — sequential digit-recurrence
    // ---------------------------------------------------------------
    localparam [2:0] SQRT_IDLE = 3'd0,
                     SQRT_START = 3'd1,
                     SQRT_ITER = 3'd2,
                     SQRT_DONE = 3'd3,
                     SQRT_RESULT = 3'd4;

    reg [2:0]          sqrt_state;
    reg                sqrt_is_double;
    reg [63:0]         sqrt_fs_val_r;
    reg [1:0]          sqrt_rm_r;
    reg                sqrt_flush_denorms_r;
    reg [14:0]         sqrt_exp;
    reg [57:0]         sqrt_rem;
    reg [55:0]         sqrt_root;
    reg [111:0]        sqrt_radicand;
    reg [5:0]          sqrt_count;
    reg                sqrt_special;
    reg [63:0]         sqrt_special_result;
    reg [4:0]          sqrt_special_exc;
    reg [63:0]         sqrt_result_r;
    reg                sqrt_result_exc_invalid;
    reg                sqrt_result_exc_inexact;

    wire [57:0] fsqrt_trial = {sqrt_root, 2'b01};
    wire [63:0] sqrt_fs_val = sqrt_fs_val_r;
    wire [1:0]  sqrt_rm = sqrt_rm_r;
    wire        sqrt_flush_denorms = sqrt_flush_denorms_r;

    wire        sqrt_s_sign_a = sqrt_fs_val[31];
    wire [ 7:0] sqrt_s_exp_a  = sqrt_fs_val[30:23];
    wire [22:0] sqrt_s_man_a  = sqrt_fs_val[22:0];
    wire        sqrt_d_sign_a = sqrt_fs_val[63];
    wire [10:0] sqrt_d_exp_a  = sqrt_fs_val[62:52];
    wire [51:0] sqrt_d_man_a  = sqrt_fs_val[51:0];

    wire sqrt_s_is_nan_a  = (sqrt_s_exp_a == 8'hFF) && (sqrt_s_man_a != 0);
    wire sqrt_s_is_inf_a  = (sqrt_s_exp_a == 8'hFF) && (sqrt_s_man_a == 0);
    wire sqrt_s_is_zero_a = (sqrt_s_exp_a == 0) && (sqrt_s_man_a == 0 || sqrt_flush_denorms);
    wire sqrt_s_is_snan_a = sqrt_s_is_nan_a && sqrt_s_man_a[22];
    wire sqrt_d_is_nan_a  = (sqrt_d_exp_a == 11'h7FF) && (sqrt_d_man_a != 0);
    wire sqrt_d_is_inf_a  = (sqrt_d_exp_a == 11'h7FF) && (sqrt_d_man_a == 0);
    wire sqrt_d_is_zero_a = (sqrt_d_exp_a == 0) && (sqrt_d_man_a == 0 || sqrt_flush_denorms);
    wire sqrt_d_is_snan_a = sqrt_d_is_nan_a && sqrt_d_man_a[51];

    wire [31:0] sqrt_s_prop_nan = quiet_legacy_nan32(sqrt_fs_val[31:0]);
    wire [63:0] sqrt_d_prop_nan = quiet_legacy_nan64(sqrt_fs_val);

    `R4000_FPU_SEQ begin
        if (!rst_n) begin
            sqrt_state          <= SQRT_IDLE;
            sqrt_is_double      <= 1'b0;
            sqrt_fs_val_r       <= 64'd0;
            sqrt_rm_r           <= 2'd0;
            sqrt_flush_denorms_r <= 1'b0;
            sqrt_exp            <= 15'd0;
            sqrt_rem            <= 58'd0;
            sqrt_root           <= 56'd0;
            sqrt_radicand       <= 112'd0;
            sqrt_count          <= 6'd0;
            sqrt_special        <= 1'b0;
            sqrt_special_result <= 64'd0;
            sqrt_special_exc    <= 5'd0;
            sqrt_result_r           <= 64'd0;
            sqrt_result_exc_invalid <= 1'b0;
            sqrt_result_exc_inexact <= 1'b0;
        end else if (fpu_cancel) begin
            sqrt_state <= SQRT_IDLE;
        end else begin
            case (sqrt_state)
                SQRT_IDLE: begin
                    if (FPU_HAS_SQRT && do_fpop && funct == `FPU_SQRT) begin
                        sqrt_is_double       <= (fmt == `COP1_FMT_D);
                        sqrt_fs_val_r        <= fs_val;
                        sqrt_rm_r            <= rm;
                        sqrt_flush_denorms_r <= flush_denorms;
                        sqrt_state           <= SQRT_START;
                    end
                end

                SQRT_START: begin
                    if (sqrt_is_double) begin : sqrt_start_d
                            reg [52:0] ma;
                            reg [5:0] lza;
                            reg signed [14:0] eff_exp;
                            reg [14:0] expr;
                            reg [111:0] rad;

                            if (sqrt_d_is_nan_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= sqrt_d_prop_nan;
                                sqrt_special_exc    <= {sqrt_d_is_snan_a, 4'd0};
                                sqrt_state          <= SQRT_DONE;
                            end else if (sqrt_d_is_inf_a && !sqrt_d_sign_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= sqrt_fs_val;
                                sqrt_special_exc    <= 5'd0;
                                sqrt_state          <= SQRT_DONE;
                            end else if (sqrt_d_is_inf_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= d_default_qnan;
                                sqrt_special_exc    <= 5'b10000;
                                sqrt_state          <= SQRT_DONE;
                            end else if (sqrt_d_is_zero_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= {sqrt_d_sign_a, 63'd0};
                                sqrt_special_exc    <= 5'd0;
                                sqrt_state          <= SQRT_DONE;
                            end else if (sqrt_d_sign_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= d_default_qnan;
                                sqrt_special_exc    <= 5'b10000;
                                sqrt_state          <= SQRT_DONE;
                            end else begin
                                ma = (sqrt_d_exp_a == 0) ? {1'b0, sqrt_d_man_a} : {1'b1, sqrt_d_man_a};
                                lza = 6'd0;
                                if (sqrt_d_exp_a == 0) begin
                                    lza = lzc53(ma);
                                    ma = ma << lza;
                                end

                                if (sqrt_d_exp_a == 0) eff_exp = 15'sd1 - $signed({9'd0, lza});
                                else                    eff_exp = $signed({4'd0, sqrt_d_exp_a});

                                expr = ((eff_exp - 15'sd1023) >>> 1) + 15'sd1023;

                                if (eff_exp[0])
                                    rad = {1'b0, ma, 58'd0};
                                else
                                    rad = {ma, 59'd0};

                                sqrt_exp       <= expr;
                                sqrt_rem       <= 58'd0;
                                sqrt_root      <= 56'd0;
                                sqrt_radicand  <= rad;
                                sqrt_count     <= 6'd56;
                                sqrt_special   <= 1'b0;
                                sqrt_state     <= SQRT_ITER;
                            end
                    end else begin : sqrt_start_s
                            reg [23:0] ma;
                            reg [4:0] lza;
                            reg signed [14:0] eff_exp;
                            reg [14:0] expr;
                            reg [111:0] rad;

                            if (sqrt_s_is_nan_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= {32'd0, sqrt_s_prop_nan};
                                sqrt_special_exc    <= {sqrt_s_is_snan_a, 4'd0};
                                sqrt_state          <= SQRT_DONE;
                            end else if (sqrt_s_is_inf_a && !sqrt_s_sign_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= {32'd0, sqrt_fs_val[31:0]};
                                sqrt_special_exc    <= 5'd0;
                                sqrt_state          <= SQRT_DONE;
                            end else if (sqrt_s_is_inf_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= {32'd0, s_default_qnan};
                                sqrt_special_exc    <= 5'b10000;
                                sqrt_state          <= SQRT_DONE;
                            end else if (sqrt_s_is_zero_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= {32'd0, sqrt_s_sign_a, 31'd0};
                                sqrt_special_exc    <= 5'd0;
                                sqrt_state          <= SQRT_DONE;
                            end else if (sqrt_s_sign_a) begin
                                sqrt_special        <= 1'b1;
                                sqrt_special_result <= {32'd0, s_default_qnan};
                                sqrt_special_exc    <= 5'b10000;
                                sqrt_state          <= SQRT_DONE;
                            end else begin
                                ma = (sqrt_s_exp_a == 0) ? {1'b0, sqrt_s_man_a} : {1'b1, sqrt_s_man_a};
                                lza = 5'd0;
                                if (sqrt_s_exp_a == 0) begin
                                    lza = lzc24(ma);
                                    ma = ma << lza;
                                end

                                if (sqrt_s_exp_a == 0) eff_exp = 15'sd1 - $signed({10'd0, lza});
                                else                    eff_exp = $signed({7'd0, sqrt_s_exp_a});

                                expr = ((eff_exp - 15'sd127) >>> 1) + 15'sd127;

                                if (eff_exp[0])
                                    rad = {1'b0, ma, 27'd0, 60'd0};
                                else
                                    rad = {ma, 28'd0, 60'd0};

                                sqrt_exp       <= expr;
                                sqrt_rem       <= 58'd0;
                                sqrt_root      <= 56'd0;
                                sqrt_radicand  <= rad;
                                sqrt_count     <= 6'd26;
                                sqrt_special   <= 1'b0;
                                sqrt_state     <= SQRT_ITER;
                            end
                    end
                end

                SQRT_ITER: begin : sqrt_iterate
                    reg [57:0] new_rem;
                    new_rem = {sqrt_rem[55:0], sqrt_radicand[111], sqrt_radicand[110]};
                    if (new_rem >= fsqrt_trial) begin
                        sqrt_rem  <= new_rem - fsqrt_trial;
                        sqrt_root <= {sqrt_root[54:0], 1'b1};
                    end else begin
                        sqrt_rem  <= new_rem;
                        sqrt_root <= {sqrt_root[54:0], 1'b0};
                    end
                    sqrt_radicand <= {sqrt_radicand[109:0], 2'b00};
                    sqrt_count    <= sqrt_count - 6'd1;
                    if (sqrt_count == 6'd1)
                        sqrt_state <= SQRT_DONE;
                end

                SQRT_DONE: begin
                    if (!sqrt_is_double) begin
                        sqrt_result_r           <= {32'd0, s_sqrt_result};
                        sqrt_result_exc_invalid <= s_sqrt_exc_invalid;
                        sqrt_result_exc_inexact <= s_sqrt_exc_inexact;
                    end else begin
                        sqrt_result_r           <= d_sqrt_result;
                        sqrt_result_exc_invalid <= d_sqrt_exc_invalid;
                        sqrt_result_exc_inexact <= d_sqrt_exc_inexact;
                    end
                    sqrt_state <= SQRT_RESULT;
                end

                SQRT_RESULT: begin
                    sqrt_state <= SQRT_IDLE;
                end

                default: sqrt_state <= SQRT_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // SQRT rounding logic — combinational, reads FSM registers
    // Produces s_sqrt_result / d_sqrt_result when sqrt_state == SQRT_DONE
    // ---------------------------------------------------------------
    always @(*) begin : sqrt_round_s
        reg guard, sticky, round_up, lsb;

        s_sqrt_result      = 32'd0;
        s_sqrt_exc_invalid = 1'b0;
        s_sqrt_exc_inexact = 1'b0;

        if (!sqrt_is_double && sqrt_special) begin
            s_sqrt_result      = sqrt_special_result[31:0];
            s_sqrt_exc_invalid = sqrt_special_exc[4];
        end else if (!sqrt_is_double) begin
            guard = sqrt_root[1];
            sticky = sqrt_root[0] || (sqrt_rem != 58'd0);
            lsb = sqrt_root[2];
            if (guard || sticky)
                s_sqrt_exc_inexact = 1'b1;
            round_up = 1'b0;
            case (sqrt_rm)
                `RM_RN: round_up = guard && (sticky || lsb);
                `RM_RP: round_up = guard || sticky;
                `RM_RM: round_up = 1'b0;
                default: ;
            endcase
            s_sqrt_result = {1'b0, sqrt_exp[7:0], sqrt_root[24:2]};
            if (round_up)
                s_sqrt_result = s_sqrt_result + 32'd1;
        end
    end

    always @(*) begin : sqrt_round_d
        reg guard, sticky, round_up, lsb;

        d_sqrt_result      = 64'd0;
        d_sqrt_exc_invalid = 1'b0;
        d_sqrt_exc_inexact = 1'b0;

        if (sqrt_is_double && sqrt_special) begin
            d_sqrt_result      = sqrt_special_result;
            d_sqrt_exc_invalid = sqrt_special_exc[4];
        end else if (sqrt_is_double) begin
            guard = sqrt_root[2];
            sticky = sqrt_root[1] || sqrt_root[0] || (sqrt_rem != 58'd0);
            lsb = sqrt_root[3];
            if (guard || sticky)
                d_sqrt_exc_inexact = 1'b1;
            round_up = 1'b0;
            case (sqrt_rm)
                `RM_RN: round_up = guard && (sticky || lsb);
                `RM_RP: round_up = guard || sticky;
                `RM_RM: round_up = 1'b0;
                default: ;
            endcase
            d_sqrt_result = {1'b0, sqrt_exp[10:0], sqrt_root[54:3]};
            if (round_up)
                d_sqrt_result = d_sqrt_result + 64'd1;
        end
    end

    // ---------------------------------------------------------------
    // Conversions
    // ---------------------------------------------------------------
    wire [63:0] cvt_fs_val = cvt_fs_val_r;
    wire [ 5:0] cvt_funct = cvt_funct_r;
    wire [ 1:0] cvt_rm = cvt_rm_r;
    wire        cvt_flush_denorms = cvt_flush_denorms_r;

    reg [1:0] cvt_eff_rm;
    always @(*) begin
        case (cvt_funct)
            `FPU_ROUND_W, `FPU_ROUND_L: cvt_eff_rm = `RM_RN;
            `FPU_TRUNC_W, `FPU_TRUNC_L: cvt_eff_rm = `RM_RZ;
            `FPU_CEIL_W,  `FPU_CEIL_L:  cvt_eff_rm = `RM_RP;
            `FPU_FLOOR_W, `FPU_FLOOR_L: cvt_eff_rm = `RM_RM;
            default:                     cvt_eff_rm = cvt_rm;
        endcase
    end

    wire        cvt_s_sign_a = cvt_fs_val[31];
    wire [ 7:0] cvt_s_exp_a  = cvt_fs_val[30:23];
    wire [22:0] cvt_s_man_a  = cvt_fs_val[22:0];
    wire        cvt_d_sign_a = cvt_fs_val[63];
    wire [10:0] cvt_d_exp_a  = cvt_fs_val[62:52];
    wire [51:0] cvt_d_man_a  = cvt_fs_val[51:0];

    wire cvt_s_is_nan_a    = (cvt_s_exp_a == 8'hFF) && (cvt_s_man_a != 0);
    wire cvt_s_is_inf_a    = (cvt_s_exp_a == 8'hFF) && (cvt_s_man_a == 0);
    wire cvt_s_is_zero_a   = (cvt_s_exp_a == 0) && (cvt_s_man_a == 0 || cvt_flush_denorms);
    wire cvt_s_is_snan_a   = cvt_s_is_nan_a && cvt_s_man_a[22];
    wire cvt_d_is_nan_a    = (cvt_d_exp_a == 11'h7FF) && (cvt_d_man_a != 0);
    wire cvt_d_is_inf_a    = (cvt_d_exp_a == 11'h7FF) && (cvt_d_man_a == 0);
    wire cvt_d_is_zero_a   = (cvt_d_exp_a == 0) && (cvt_d_man_a == 0 || cvt_flush_denorms);
    wire cvt_d_is_snan_a   = cvt_d_is_nan_a && cvt_d_man_a[51];

    // CVT.S.D: double -> single. Split pre-round normalization from final
    // rounding so the denorm jam path does not also drive cvt_result_r.
    reg        cvt_s_d_pre_special;
    reg [31:0] cvt_s_d_pre_special_result;
    reg        cvt_s_d_pre_special_exc_invalid;
    reg        cvt_s_d_pre_sign;
    reg signed [11:0] cvt_s_d_pre_exp;
    reg [23:0] cvt_s_d_pre_man;
    reg        cvt_s_d_pre_guard;
    reg        cvt_s_d_pre_sticky;
    reg        cvt_s_d_pre_is_tiny;
    reg        cvt_s_d_pre_exc_underflow;
    reg        cvt_s_d_pre_exc_inexact;
    always @(*) begin : cvt_s_d_pre_block
        reg        sign;
        reg [10:0] exp_d;
        reg [51:0] man_d;
        reg signed [11:0] exp_s;
        reg [23:0] man_norm;
        reg        guard, sticky, is_tiny;
        reg [5:0]  denorm_shift;
        reg [25:0] jammed;

        sign = cvt_fs_val[63];
        exp_d = cvt_fs_val[62:52];
        man_d = cvt_fs_val[51:0];
        exp_s = 12'sd0;
        man_norm = 24'd0;
        guard = 1'b0;
        sticky = 1'b0;
        is_tiny = 1'b0;

        cvt_s_d_pre_special = 1'b0;
        cvt_s_d_pre_special_result = 32'd0;
        cvt_s_d_pre_special_exc_invalid = 1'b0;
        cvt_s_d_pre_sign = sign;
        cvt_s_d_pre_exp = 12'sd0;
        cvt_s_d_pre_man = 24'd0;
        cvt_s_d_pre_guard = 1'b0;
        cvt_s_d_pre_sticky = 1'b0;
        cvt_s_d_pre_is_tiny = 1'b0;
        cvt_s_d_pre_exc_underflow = 1'b0;
        cvt_s_d_pre_exc_inexact = 1'b0;

        if (cvt_d_is_nan_a) begin
            cvt_s_d_pre_special = 1'b1;
            cvt_s_d_pre_special_exc_invalid = cvt_d_is_snan_a;
            cvt_s_d_pre_special_result = quiet_legacy_nan32(
                {cvt_fs_val[63], 8'hFF, 1'b0, cvt_fs_val[50:29]});
        end else if (cvt_d_is_inf_a) begin
            cvt_s_d_pre_special = 1'b1;
            cvt_s_d_pre_special_result = {sign, 8'hFF, 23'd0};
        end else if (cvt_d_is_zero_a) begin
            cvt_s_d_pre_special = 1'b1;
            cvt_s_d_pre_special_result = {sign, 31'd0};
        end else begin
            if (exp_d == 0) begin
                exp_s = 12'sd0;
                man_norm = 24'd0;
                guard = 1'b0;
                sticky = |man_d;
            end else begin
                exp_s = $signed({1'b0, exp_d}) - 12'sd896;
                man_norm = {1'b1, man_d[51:29]};
                guard = man_d[28];
                sticky = |man_d[27:0];
            end

            is_tiny = (exp_s < 12'sd1);
            if (is_tiny) begin
                if (exp_s == 12'sd0 && man_norm == 24'hFFFFFF) begin
                    case (cvt_rm)
                        `RM_RN: if (guard) is_tiny = 1'b0;
                        `RM_RP: if (!sign && (guard || sticky)) is_tiny = 1'b0;
                        `RM_RM: if (sign && (guard || sticky)) is_tiny = 1'b0;
                        default: ;
                    endcase
                end
            end
            if (exp_s < 12'sd1) begin
                denorm_shift = (exp_s < -12'sd24) ? 6'd25 : (6'd1 - exp_s[5:0]);
                jammed = shrjam24(man_norm, guard, sticky, denorm_shift);
                {man_norm, guard, sticky} = jammed;
                exp_s = 12'sd0;
            end

            cvt_s_d_pre_sign = sign;
            cvt_s_d_pre_exp = exp_s;
            cvt_s_d_pre_man = man_norm;
            cvt_s_d_pre_guard = guard;
            cvt_s_d_pre_sticky = sticky;
            cvt_s_d_pre_is_tiny = is_tiny;
            cvt_s_d_pre_exc_inexact = guard || sticky;
            cvt_s_d_pre_exc_underflow = is_tiny && (guard || sticky);
        end
    end

    reg [31:0] cvt_s_d_final_result;
    reg        cvt_s_d_final_exc_inexact;
    reg        cvt_s_d_final_exc_overflow;
    reg        cvt_s_d_final_exc_underflow;
    reg        cvt_s_d_final_exc_invalid;
    always @(*) begin : cvt_s_d_round_block
        reg round_up, lsb;

        cvt_s_d_final_result = 32'd0;
        cvt_s_d_final_exc_invalid = 1'b0;
        cvt_s_d_final_exc_overflow = 1'b0;
        cvt_s_d_final_exc_underflow = 1'b0;
        cvt_s_d_final_exc_inexact = 1'b0;

        if (cvt_s_d_round_special) begin
            cvt_s_d_final_result = cvt_s_d_round_special_result;
            cvt_s_d_final_exc_invalid = cvt_s_d_round_special_exc_invalid;
        end else begin
            cvt_s_d_final_exc_underflow = cvt_s_d_round_exc_underflow;
            cvt_s_d_final_exc_inexact = cvt_s_d_round_exc_inexact;

            lsb = cvt_s_d_round_man[0];
            round_up = 1'b0;
            case (cvt_s_d_round_rm)
                `RM_RN: round_up = cvt_s_d_round_guard && (cvt_s_d_round_sticky || lsb);
                `RM_RP: round_up = !cvt_s_d_round_sign &&
                                   (cvt_s_d_round_guard || cvt_s_d_round_sticky);
                `RM_RM: round_up = cvt_s_d_round_sign &&
                                   (cvt_s_d_round_guard || cvt_s_d_round_sticky);
                default: ;
            endcase

            cvt_s_d_final_result = {cvt_s_d_round_sign, cvt_s_d_round_exp[7:0],
                                    cvt_s_d_round_man[22:0]};
            if (round_up)
                cvt_s_d_final_result = cvt_s_d_final_result + 32'd1;

            if (!cvt_s_d_round_is_tiny &&
                (cvt_s_d_round_exp >= 12'sd255 || cvt_s_d_final_result[30:23] == 8'hFF)) begin
                cvt_s_d_final_exc_overflow = 1'b1;
                cvt_s_d_final_exc_inexact = 1'b1;
                case (cvt_s_d_round_rm)
                    `RM_RN: cvt_s_d_final_result = {cvt_s_d_round_sign, 8'hFF, 23'd0};
                    `RM_RZ: cvt_s_d_final_result = {cvt_s_d_round_sign, 8'hFE, 23'h7FFFFF};
                    `RM_RP: cvt_s_d_final_result = cvt_s_d_round_sign ?
                        {1'b1, 8'hFE, 23'h7FFFFF} : {1'b0, 8'hFF, 23'd0};
                    `RM_RM: cvt_s_d_final_result = cvt_s_d_round_sign ?
                        {1'b1, 8'hFF, 23'd0} : {1'b0, 8'hFE, 23'h7FFFFF};
                endcase
            end

            if (cvt_s_d_round_flush_denorms &&
                cvt_s_d_final_result[30:23] == 8'h00 &&
                cvt_s_d_final_result[22:0] != 23'd0) begin
                cvt_s_d_final_exc_underflow = 1'b1;
                cvt_s_d_final_result = {cvt_s_d_round_sign, 31'd0};
            end
        end
    end

    // CVT.D.S: single → double
    reg [63:0] cvt_d_s_result;
    reg        cvt_d_s_exc_invalid;
    always @(*) begin : cvt_d_s_block
        reg        sign;
        reg [ 7:0] exp_s;
        reg [22:0] man_s;
        reg [10:0] exp_d;
        reg [22:0] m;
        reg [ 4:0] lzc;

        sign = cvt_fs_val[31];
        exp_s = cvt_fs_val[30:23];
        man_s = cvt_fs_val[22:0];
        cvt_d_s_exc_invalid = 1'b0;

        if (cvt_s_is_nan_a) begin
            cvt_d_s_exc_invalid = cvt_s_is_snan_a;
            cvt_d_s_result = quiet_legacy_nan64(
                {sign, 11'h7FF, 1'b0, man_s[21:0], 29'd0});
        end
        else if (cvt_s_is_inf_a)
            cvt_d_s_result = {sign, 11'h7FF, 52'd0};
        else if (cvt_s_is_zero_a)
            cvt_d_s_result = {sign, 63'd0};
        else if (exp_s == 0) begin
            m = man_s;
            lzc = lzc24({man_s, 1'b0});
            m = m << lzc;
            exp_d = 11'd896 - {6'd0, lzc};
            cvt_d_s_result = {sign, exp_d, m[21:0], 30'd0};
        end else begin
            exp_d = {3'd0, exp_s} + 11'd896;
            cvt_d_s_result = {sign, exp_d, man_s, 29'd0};
        end
    end

    // CVT.S.W: word integer → single float
    reg [31:0] cvt_s_w_result;
    reg        cvt_s_w_exc_inexact;
    always @(*) begin : cvt_s_w_block
        reg        sign;
        reg [31:0] abs_val;
        reg [ 4:0] lzc;
        reg [31:0] shifted;
        reg [ 7:0] exp_r;
        reg        guard, sticky, round_up;

        sign = cvt_fs_val[31];
        abs_val = sign ? (~cvt_fs_val[31:0] + 32'd1) : cvt_fs_val[31:0];

        cvt_s_w_exc_inexact = 1'b0;
        if (abs_val == 0) begin
            cvt_s_w_result = 32'd0;
        end else begin
            lzc = lzc32(abs_val);
            shifted = abs_val << lzc;
            exp_r = 8'd158 - {3'd0, lzc};  // 127 + 31 - lzc
            guard = shifted[7];
            sticky = |shifted[6:0];
            if (guard || sticky)
                cvt_s_w_exc_inexact = 1'b1;
            round_up = 1'b0;
            case (cvt_rm)
                `RM_RN: round_up = guard && (sticky || shifted[8]);
                `RM_RZ: round_up = 1'b0;
                `RM_RM: round_up = sign && (guard || sticky);
                `RM_RP: round_up = !sign && (guard || sticky);
            endcase
            if (round_up)
                cvt_s_w_result = {sign, {exp_r, shifted[30:8]} + 31'd1};
            else
                cvt_s_w_result = {sign, exp_r, shifted[30:8]};
        end
    end

    // CVT.D.W: word integer → double float
    reg [63:0] cvt_d_w_result;
    always @(*) begin : cvt_d_w_block
        reg        sign;
        reg [31:0] abs_val;
        reg [ 4:0] lzc;
        reg [31:0] shifted;
        reg [10:0] exp_r;

        sign = cvt_fs_val[31];
        abs_val = sign ? (~cvt_fs_val[31:0] + 32'd1) : cvt_fs_val[31:0];

        if (abs_val == 0) begin
            cvt_d_w_result = 64'd0;
        end else begin
            lzc = lzc32(abs_val);
            shifted = abs_val << lzc;
            exp_r = 11'd1054 - {6'd0, lzc};  // 1023 + 31 - lzc
            cvt_d_w_result = {sign, exp_r, shifted[30:0], 21'd0};
        end
    end

    // CVT.W.S: single float → word integer (with rounding via eff_rm)
    reg [31:0] cvt_w_s_result;
    reg        cvt_w_s_exc_invalid;
    reg        cvt_w_s_exc_inexact;
    always @(*) begin : cvt_w_s_block
        reg        sign;
        reg [ 7:0] exp_s;
        reg [23:0] man_full;
        reg [31:0] int_val;
        reg        round_up;
        reg        has_frac;
        reg [23:0] frac_bits;
        reg        guard;
        /* verilator lint_off WIDTHEXPAND */

        cvt_w_s_exc_invalid = 1'b0;
        cvt_w_s_exc_inexact = 1'b0;
        sign = cvt_fs_val[31];
        exp_s = cvt_fs_val[30:23];
        man_full = {1'b1, cvt_fs_val[22:0]};

        if (cvt_s_is_nan_a || cvt_s_is_inf_a || exp_s > 8'd158 ||
            (exp_s == 8'd158 && (!sign || cvt_fs_val[22:0] != 23'd0))) begin
            cvt_w_s_exc_invalid = 1'b1;
            cvt_w_s_result = 32'h7FFFFFFF;
        end else if (exp_s < 8'd103) begin
            // |x| < 2^-24 → rounds to 0 for all modes except possibly RP/RM
            has_frac = (exp_s != 8'd0 || cvt_fs_val[22:0] != 23'd0);
            cvt_w_s_exc_inexact = has_frac;
            round_up = 1'b0;
            if (has_frac) begin
                case (cvt_eff_rm)
                    `RM_RP: round_up = !sign;
                    `RM_RM: round_up = sign;
                    default: round_up = 1'b0;
                endcase
            end
            cvt_w_s_result = round_up ? (sign ? 32'hFFFFFFFF : 32'd1) : 32'd0;
        end else if (exp_s < 8'd127) begin
            // 0 < |x| < 1.0 — fractional value, may round to ±1
            cvt_w_s_exc_inexact = 1'b1;
            round_up = 1'b0;
            case (cvt_eff_rm)
                `RM_RN: begin
                    // Round to nearest: if |x| > 0.5, round away; if == 0.5, round to even (0)
                    if (exp_s == 8'd126) begin
                        // |x| >= 0.5; check if > 0.5 (any mantissa bits set)
                        if (cvt_fs_val[22:0] != 23'd0)
                            round_up = 1'b1;
                        // else exactly 0.5 → ties to even = 0
                    end
                    // else |x| < 0.5 → rounds to 0
                end
                `RM_RZ: round_up = 1'b0;
                `RM_RP: round_up = !sign;
                `RM_RM: round_up = sign;
            endcase
            cvt_w_s_result = round_up ? (sign ? 32'hFFFFFFFF : 32'd1) : 32'd0;
        end else begin
            // exp_s >= 127: integer part exists
            // shift = number of fractional mantissa bits = 150 - exp_s
            if (exp_s >= 8'd150) begin
                // No fractional bits — exact integer
                int_val = {8'd0, man_full} << (exp_s - 8'd150);
                cvt_w_s_result = sign ? (~int_val + 32'd1) : int_val;
            end else begin
                // Has fractional bits
                int_val = {8'd0, man_full} >> (8'd150 - exp_s);
                // Check fractional remainder for rounding
                frac_bits = man_full << (exp_s - 8'd126);
                has_frac = (frac_bits != 24'd0);
                cvt_w_s_exc_inexact = has_frac;
                guard = frac_bits[23]; // highest fractional bit
                round_up = 1'b0;
                if (has_frac) begin
                    case (cvt_eff_rm)
                        `RM_RN: begin
                            if (guard) begin
                                if (frac_bits[22:0] != 23'd0)
                                    round_up = 1'b1;
                                else
                                    round_up = int_val[0]; // ties to even
                            end
                        end
                        `RM_RZ: round_up = 1'b0;
                        `RM_RP: round_up = !sign;
                        `RM_RM: round_up = sign;
                    endcase
                end
                if (round_up) int_val = int_val + 32'd1;
                cvt_w_s_result = sign ? (~int_val + 32'd1) : int_val;
            end
        end
        /* verilator lint_on WIDTHEXPAND */
    end

    // CVT.W.D: double float → word integer (with rounding via eff_rm)
    reg [31:0] cvt_w_d_result;
    reg        cvt_w_d_exc_invalid;
    reg        cvt_w_d_exc_inexact;
    always @(*) begin : cvt_w_d_block
        reg         sign;
        reg [10:0]  exp_d;
        reg [52:0]  man_full;
        reg [31:0]  int_val;
        reg [52:0]  shifted_int;
        reg         round_up;
        reg         has_frac;
        reg [52:0]  frac_bits;
        reg         guard;
        /* verilator lint_off WIDTHEXPAND */
        /* verilator lint_off WIDTHTRUNC */

        cvt_w_d_exc_invalid = 1'b0;
        cvt_w_d_exc_inexact = 1'b0;
        sign = cvt_fs_val[63];
        exp_d = cvt_fs_val[62:52];
        man_full = {1'b1, cvt_fs_val[51:0]};
        shifted_int = 53'd0;

        if (cvt_d_is_nan_a || cvt_d_is_inf_a || exp_d > 11'd1054) begin
            cvt_w_d_exc_invalid = 1'b1;
            cvt_w_d_result = 32'h7FFFFFFF;
        end else if (exp_d < 11'd971) begin
            // |x| < 2^-52: effectively zero, but may round
            has_frac = (exp_d != 11'd0 || cvt_fs_val[51:0] != 52'd0);
            cvt_w_d_exc_inexact = has_frac;
            round_up = 1'b0;
            if (has_frac) begin
                case (cvt_eff_rm)
                    `RM_RP: round_up = !sign;
                    `RM_RM: round_up = sign;
                    default: round_up = 1'b0;
                endcase
            end
            cvt_w_d_result = round_up ? (sign ? 32'hFFFFFFFF : 32'd1) : 32'd0;
        end else if (exp_d < 11'd1023) begin
            // 0 < |x| < 1.0 — fractional
            cvt_w_d_exc_inexact = 1'b1;
            round_up = 1'b0;
            case (cvt_eff_rm)
                `RM_RN: begin
                    if (exp_d == 11'd1022) begin
                        if (cvt_fs_val[51:0] != 52'd0)
                            round_up = 1'b1; // > 0.5
                    end
                end
                `RM_RZ: round_up = 1'b0;
                `RM_RP: round_up = !sign;
                `RM_RM: round_up = sign;
            endcase
            cvt_w_d_result = round_up ? (sign ? 32'hFFFFFFFF : 32'd1) : 32'd0;
        end else begin
            if (exp_d >= 11'd1075) begin
                // No fractional bits
                int_val = man_full[52:21] << (exp_d - 11'd1075);
                cvt_w_d_result = sign ? (~int_val + 32'd1) : int_val;
            end else begin
                shifted_int = man_full >> (11'd1075 - exp_d);
                int_val = shifted_int[31:0];
                frac_bits = man_full << (exp_d - 11'd1022);
                has_frac = (frac_bits != 53'd0);
                cvt_w_d_exc_inexact = has_frac;
                guard = frac_bits[52];
                round_up = 1'b0;
                if (has_frac) begin
                    case (cvt_eff_rm)
                        `RM_RN: begin
                            if (guard) begin
                                if (frac_bits[51:0] != 52'd0)
                                    round_up = 1'b1;
                                else
                                    round_up = int_val[0]; // ties to even
                            end
                        end
                        `RM_RZ: round_up = 1'b0;
                        `RM_RP: round_up = !sign;
                        `RM_RM: round_up = sign;
                    endcase
                end
                if (round_up && int_val == 32'hFFFFFFFF) begin
                    cvt_w_d_exc_invalid = 1'b1;
                    cvt_w_d_exc_inexact = 1'b0;
                    cvt_w_d_result = 32'h7FFFFFFF;
                end else begin
                    if (round_up) int_val = int_val + 32'd1;
                    if (!sign && int_val[31]) begin
                        cvt_w_d_exc_invalid = 1'b1;
                        cvt_w_d_exc_inexact = 1'b0;
                        cvt_w_d_result = 32'h7FFFFFFF;
                    end else if (sign && int_val > 32'h80000000) begin
                        cvt_w_d_exc_invalid = 1'b1;
                        cvt_w_d_exc_inexact = 1'b0;
                        cvt_w_d_result = 32'h7FFFFFFF;
                    end else
                        cvt_w_d_result = sign ? (~int_val + 32'd1) : int_val;
                end
            end
        end
        /* verilator lint_on WIDTHTRUNC */
        /* verilator lint_on WIDTHEXPAND */
    end

    // CVT.S.L / CVT.D.L: 64-bit integer -> float
    reg [31:0] cvt_s_l_result;
    reg        cvt_s_l_exc_inexact;
    reg [63:0] cvt_d_l_result;
    reg        cvt_d_l_exc_inexact;

    always @(*) begin : cvt_s_l_block
        reg        sign;
        reg [63:0] abs_val;
        reg [ 5:0] lzc;
        reg [63:0] shifted;
        reg [ 7:0] exp_r;
        reg        guard, sticky, round_up;

        sign = cvt_l_float_sign;
        abs_val = cvt_l_float_abs;

        cvt_s_l_exc_inexact = 1'b0;
        if (abs_val == 64'd0) begin
            cvt_s_l_result = 32'd0;
        end else begin
            lzc = lzc64(abs_val);
            shifted = abs_val << lzc;
            exp_r = 8'd190 - {2'd0, lzc};  // 127 + 63 - lzc
            guard = shifted[39];
            sticky = |shifted[38:0];
            if (guard || sticky)
                cvt_s_l_exc_inexact = 1'b1;
            round_up = 1'b0;
            case (cvt_rm)
                `RM_RN: round_up = guard && (sticky || shifted[40]);
                `RM_RZ: round_up = 1'b0;
                `RM_RM: round_up = sign && (guard || sticky);
                `RM_RP: round_up = !sign && (guard || sticky);
            endcase
            if (round_up)
                cvt_s_l_result = {sign, {exp_r, shifted[62:40]} + 31'd1};
            else
                cvt_s_l_result = {sign, exp_r, shifted[62:40]};
        end
    end

    always @(*) begin : cvt_d_l_block
        reg        sign;
        reg [63:0] abs_val;
        reg [ 5:0] lzc;
        reg [63:0] shifted;
        reg [10:0] exp_r;
        reg        guard, sticky, round_up;

        sign = cvt_l_float_sign;
        abs_val = cvt_l_float_abs;

        cvt_d_l_exc_inexact = 1'b0;
        if (abs_val == 64'd0) begin
            cvt_d_l_result = 64'd0;
        end else begin
            lzc = lzc64(abs_val);
            shifted = abs_val << lzc;
            exp_r = 11'd1086 - {5'd0, lzc};  // 1023 + 63 - lzc
            guard = shifted[10];
            sticky = |shifted[9:0];
            if (guard || sticky)
                cvt_d_l_exc_inexact = 1'b1;
            round_up = 1'b0;
            case (cvt_rm)
                `RM_RN: round_up = guard && (sticky || shifted[11]);
                `RM_RZ: round_up = 1'b0;
                `RM_RM: round_up = sign && (guard || sticky);
                `RM_RP: round_up = !sign && (guard || sticky);
            endcase
            if (round_up)
                cvt_d_l_result = {sign, {exp_r, shifted[62:11]} + 63'd1};
            else
                cvt_d_l_result = {sign, exp_r, shifted[62:11]};
        end
    end

    // CVT.L.S: single float → 64-bit integer (with rounding via eff_rm)
    reg [63:0] cvt_l_s_result;
    reg        cvt_l_s_exc_invalid;
    reg        cvt_l_s_exc_inexact;
    always @(*) begin : cvt_l_s_block
        reg        sign;
        reg [ 7:0] exp_s;
        reg [23:0] man_full;
        reg [63:0] int_val;
        reg        round_up, has_frac, guard;
        reg [23:0] frac_bits;
        /* verilator lint_off WIDTHEXPAND */

        cvt_l_s_exc_invalid = 1'b0;
        cvt_l_s_exc_inexact = 1'b0;
        sign = cvt_fs_val[31];
        exp_s = cvt_fs_val[30:23];
        man_full = {1'b1, cvt_fs_val[22:0]};

        if (cvt_s_is_nan_a || cvt_s_is_inf_a || exp_s > 8'd190 ||
            (exp_s == 8'd190 && (!sign || cvt_fs_val[22:0] != 23'd0))) begin
            cvt_l_s_exc_invalid = 1'b1;
            cvt_l_s_result = 64'h7FFFFFFFFFFFFFFF;
        end else if (exp_s < 8'd127) begin
            // |x| < 1.0
            round_up = 1'b0;
            has_frac = (exp_s != 8'd0 || cvt_fs_val[22:0] != 23'd0);
            cvt_l_s_exc_inexact = has_frac;
            if (has_frac) begin
                case (cvt_eff_rm)
                    `RM_RN: begin
                        if (exp_s == 8'd126 && cvt_fs_val[22:0] != 23'd0)
                            round_up = 1'b1;
                    end
                    `RM_RP: round_up = !sign;
                    `RM_RM: round_up = sign;
                    default: round_up = 1'b0;
                endcase
            end
            cvt_l_s_result = round_up ? (sign ? 64'hFFFFFFFFFFFFFFFF : 64'd1) : 64'd0;
        end else begin
            if (exp_s >= 8'd150) begin
                int_val = {40'd0, man_full} << (exp_s - 8'd150);
                cvt_l_s_result = sign ? (~int_val + 64'd1) : int_val;
            end else begin
                int_val = {40'd0, man_full} >> (8'd150 - exp_s);
                frac_bits = man_full << (exp_s - 8'd126);
                has_frac = (frac_bits != 24'd0);
                cvt_l_s_exc_inexact = has_frac;
                guard = frac_bits[23];
                round_up = 1'b0;
                if (has_frac) begin
                    case (cvt_eff_rm)
                        `RM_RN: begin
                            if (guard) begin
                                if (frac_bits[22:0] != 23'd0)
                                    round_up = 1'b1;
                                else
                                    round_up = int_val[0];
                            end
                        end
                        `RM_RZ: round_up = 1'b0;
                        `RM_RP: round_up = !sign;
                        `RM_RM: round_up = sign;
                    endcase
                end
                if (round_up) int_val = int_val + 64'd1;
                cvt_l_s_result = sign ? (~int_val + 64'd1) : int_val;
            end
        end
        /* verilator lint_on WIDTHEXPAND */
    end

    // CVT.L.D: double float → 64-bit integer (with rounding via eff_rm)
    reg [63:0] cvt_l_d_result;
    reg        cvt_l_d_exc_invalid;
    reg        cvt_l_d_exc_inexact;
    always @(*) begin : cvt_l_d_block
        reg         sign;
        reg [10:0]  exp_d;
        reg [52:0]  man_full;
        reg [63:0]  int_val;
        reg         round_up, has_frac, guard;
        reg [52:0]  frac_bits;
        /* verilator lint_off WIDTHEXPAND */

        cvt_l_d_exc_invalid = 1'b0;
        cvt_l_d_exc_inexact = 1'b0;
        sign = cvt_fs_val[63];
        exp_d = cvt_fs_val[62:52];
        man_full = {1'b1, cvt_fs_val[51:0]};

        if (cvt_d_is_nan_a || cvt_d_is_inf_a || exp_d > 11'd1086 ||
            (exp_d == 11'd1086 && (!sign || cvt_fs_val[51:0] != 52'd0))) begin
            cvt_l_d_exc_invalid = 1'b1;
            cvt_l_d_result = 64'h7FFFFFFFFFFFFFFF;
        end else if (exp_d < 11'd1023) begin
            // |x| < 1.0
            round_up = 1'b0;
            has_frac = (exp_d != 11'd0 || cvt_fs_val[51:0] != 52'd0);
            cvt_l_d_exc_inexact = has_frac;
            if (has_frac) begin
                case (cvt_eff_rm)
                    `RM_RN: begin
                        if (exp_d == 11'd1022 && cvt_fs_val[51:0] != 52'd0)
                            round_up = 1'b1;
                    end
                    `RM_RP: round_up = !sign;
                    `RM_RM: round_up = sign;
                    default: round_up = 1'b0;
                endcase
            end
            cvt_l_d_result = round_up ? (sign ? 64'hFFFFFFFFFFFFFFFF : 64'd1) : 64'd0;
        end else begin
            if (exp_d >= 11'd1075) begin
                int_val = {11'd0, man_full} << (exp_d - 11'd1075);
                cvt_l_d_result = sign ? (~int_val + 64'd1) : int_val;
            end else begin
                int_val = {11'd0, man_full} >> (11'd1075 - exp_d);
                frac_bits = man_full << (exp_d - 11'd1022);
                has_frac = (frac_bits != 53'd0);
                cvt_l_d_exc_inexact = has_frac;
                guard = frac_bits[52];
                round_up = 1'b0;
                if (has_frac) begin
                    case (cvt_eff_rm)
                        `RM_RN: begin
                            if (guard) begin
                                if (frac_bits[51:0] != 52'd0)
                                    round_up = 1'b1;
                                else
                                    round_up = int_val[0];
                            end
                        end
                        `RM_RZ: round_up = 1'b0;
                        `RM_RP: round_up = !sign;
                        `RM_RM: round_up = sign;
                    endcase
                end
                if (round_up) int_val = int_val + 64'd1;
                cvt_l_d_result = sign ? (~int_val + 64'd1) : int_val;
            end
        end
        /* verilator lint_on WIDTHEXPAND */
    end

    // TRUNC/ROUND/CEIL/FLOOR.W.S / .W.D / .L.S / .L.D
    // These are similar to CVT.W/CVT.L but with explicit rounding mode override
    // For now, TRUNC = CVT with RM_RZ, ROUND = CVT with RM_RN, etc.
    // We'll use the CVT results with appropriate rounding applied later.
    // For the initial implementation, TRUNC/ROUND/CEIL/FLOOR all go through CVT
    // with the rounding mode embedded in the operation.

    reg [63:0] cvt_calc_result;
    reg        cvt_calc_exc;
    reg        cvt_calc_exc_unimpl;
    reg [4:0]  cvt_calc_exc_flags;

    always @(*) begin
        cvt_calc_result = 64'd0;
        cvt_calc_exc = 1'b0;
        cvt_calc_exc_unimpl = 1'b0;
        cvt_calc_exc_flags = 5'd0;

        case (cvt_fmt_r)
            `COP1_FMT_S: begin
                case (cvt_funct_r)
                    `FPU_CVT_D: begin
                        cvt_calc_result = cvt_d_s_result;
                        cvt_calc_exc_flags = {cvt_d_s_exc_invalid, 4'd0};
                    end
                    `FPU_CVT_W: begin
                        cvt_calc_result = {32'd0, cvt_w_s_result};
                        cvt_calc_exc = cvt_w_s_exc_invalid;
                        cvt_calc_exc_flags = {cvt_w_s_exc_invalid, 3'd0, cvt_w_s_exc_inexact};
                    end
                    `FPU_CVT_L: begin
                        cvt_calc_result = cvt_l_s_result;
                        cvt_calc_exc = cvt_l_s_exc_invalid;
                        cvt_calc_exc_flags = {cvt_l_s_exc_invalid, 3'd0, cvt_l_s_exc_inexact};
                    end
                    `FPU_ROUND_W, `FPU_TRUNC_W, `FPU_CEIL_W, `FPU_FLOOR_W: begin
                        cvt_calc_result = {32'd0, cvt_w_s_result};
                        cvt_calc_exc = cvt_w_s_exc_invalid;
                        cvt_calc_exc_flags = {cvt_w_s_exc_invalid, 3'd0, cvt_w_s_exc_inexact};
                    end
                    `FPU_ROUND_L, `FPU_TRUNC_L, `FPU_CEIL_L, `FPU_FLOOR_L: begin
                        cvt_calc_result = cvt_l_s_result;
                        cvt_calc_exc = cvt_l_s_exc_invalid;
                        cvt_calc_exc_flags = {cvt_l_s_exc_invalid, 3'd0, cvt_l_s_exc_inexact};
                    end
                    default: cvt_calc_exc_unimpl = 1'b1;
                endcase
            end

	            `COP1_FMT_D: begin
	                case (cvt_funct_r)
	                    `FPU_CVT_S: begin
	                        // CVT.S.D is rounded in CVT_S_D_ROUND after
	                        // CVT_CALC latches its pre-round state.
	                        cvt_calc_result = 64'd0;
	                    end
                    `FPU_CVT_W: begin
                        cvt_calc_result = {32'd0, cvt_w_d_result};
                        cvt_calc_exc = cvt_w_d_exc_invalid;
                        cvt_calc_exc_flags = {cvt_w_d_exc_invalid, 3'd0, cvt_w_d_exc_inexact};
                    end
                    `FPU_CVT_L: begin
                        cvt_calc_result = cvt_l_d_result;
                        cvt_calc_exc = cvt_l_d_exc_invalid;
                        cvt_calc_exc_flags = {cvt_l_d_exc_invalid, 3'd0, cvt_l_d_exc_inexact};
                    end
                    `FPU_ROUND_W, `FPU_TRUNC_W, `FPU_CEIL_W, `FPU_FLOOR_W: begin
                        cvt_calc_result = {32'd0, cvt_w_d_result};
                        cvt_calc_exc = cvt_w_d_exc_invalid;
                        cvt_calc_exc_flags = {cvt_w_d_exc_invalid, 3'd0, cvt_w_d_exc_inexact};
                    end
                    `FPU_ROUND_L, `FPU_TRUNC_L, `FPU_CEIL_L, `FPU_FLOOR_L: begin
                        cvt_calc_result = cvt_l_d_result;
                        cvt_calc_exc = cvt_l_d_exc_invalid;
                        cvt_calc_exc_flags = {cvt_l_d_exc_invalid, 3'd0, cvt_l_d_exc_inexact};
                    end
                    default: cvt_calc_exc_unimpl = 1'b1;
                endcase
            end

            `COP1_FMT_W: begin
                case (cvt_funct_r)
                    `FPU_CVT_S: begin
                        cvt_calc_result = {32'd0, cvt_s_w_result};
                        cvt_calc_exc_flags = {4'd0, cvt_s_w_exc_inexact};
                    end
                    `FPU_CVT_D: begin
                        cvt_calc_result = cvt_d_w_result;
                    end
                    default: cvt_calc_exc_unimpl = 1'b1;
                endcase
            end

            `COP1_FMT_L: begin
                case (cvt_funct_r)
                    `FPU_CVT_S: begin
                        // Handled by CVT_L_FLOAT_ROUND after CVT_CALC
                        // latches sign and absolute value.
                        cvt_calc_result = 64'd0;
                    end
                    `FPU_CVT_D: begin
                        cvt_calc_result = 64'd0;
                    end
                    default: cvt_calc_exc_unimpl = 1'b1;
                endcase
            end

            default: cvt_calc_exc_unimpl = 1'b1;
        endcase
    end

    `R4000_FPU_SEQ begin
        if (!rst_n) begin
            cvt_state <= CVT_IDLE;
            cvt_fs_val_r <= 64'd0;
            cvt_fmt_r <= 5'd0;
            cvt_funct_r <= 6'd0;
            cvt_rm_r <= 2'd0;
            cvt_flush_denorms_r <= 1'b0;
	            cvt_result_r <= 64'd0;
	            cvt_exc_r <= 1'b0;
	            cvt_exc_unimpl_r <= 1'b0;
	            cvt_exc_flags_r <= 5'd0;
	            cvt_s_d_round_special <= 1'b0;
	            cvt_s_d_round_special_result <= 32'd0;
	            cvt_s_d_round_special_exc_invalid <= 1'b0;
	            cvt_s_d_round_sign <= 1'b0;
	            cvt_s_d_round_rm <= 2'd0;
	            cvt_s_d_round_flush_denorms <= 1'b0;
	            cvt_s_d_round_exp <= 12'sd0;
	            cvt_s_d_round_man <= 24'd0;
	            cvt_s_d_round_guard <= 1'b0;
	            cvt_s_d_round_sticky <= 1'b0;
	            cvt_s_d_round_is_tiny <= 1'b0;
	            cvt_s_d_round_exc_underflow <= 1'b0;
	            cvt_s_d_round_exc_inexact <= 1'b0;
	            cvt_l_float_sign <= 1'b0;
	            cvt_l_float_abs <= 64'd0;
        end else if (fpu_cancel) begin
            cvt_state <= CVT_IDLE;
        end else begin
            case (cvt_state)
                CVT_IDLE: begin
                    if (fpu_cvt_op) begin
                        cvt_fs_val_r <= fs_val;
                        cvt_fmt_r <= fmt;
                        cvt_funct_r <= funct;
                        cvt_rm_r <= rm;
                        cvt_flush_denorms_r <= flush_denorms;
                        cvt_state <= CVT_CALC;
                    end
                end

	                CVT_CALC: begin
	                    if (cvt_fmt_r == `COP1_FMT_D && cvt_funct_r == `FPU_CVT_S) begin
	                        cvt_s_d_round_special <= cvt_s_d_pre_special;
	                        cvt_s_d_round_special_result <= cvt_s_d_pre_special_result;
	                        cvt_s_d_round_special_exc_invalid <= cvt_s_d_pre_special_exc_invalid;
	                        cvt_s_d_round_sign <= cvt_s_d_pre_sign;
	                        cvt_s_d_round_rm <= cvt_rm_r;
	                        cvt_s_d_round_flush_denorms <= cvt_flush_denorms_r;
	                        cvt_s_d_round_exp <= cvt_s_d_pre_exp;
	                        cvt_s_d_round_man <= cvt_s_d_pre_man;
	                        cvt_s_d_round_guard <= cvt_s_d_pre_guard;
	                        cvt_s_d_round_sticky <= cvt_s_d_pre_sticky;
	                        cvt_s_d_round_is_tiny <= cvt_s_d_pre_is_tiny;
	                        cvt_s_d_round_exc_underflow <= cvt_s_d_pre_exc_underflow;
	                        cvt_s_d_round_exc_inexact <= cvt_s_d_pre_exc_inexact;
	                        cvt_state <= CVT_S_D_ROUND;
	                    end else if (cvt_fmt_r == `COP1_FMT_L &&
	                                 (cvt_funct_r == `FPU_CVT_S ||
	                                  cvt_funct_r == `FPU_CVT_D)) begin
	                        cvt_l_float_sign <= cvt_fs_val_r[63];
	                        cvt_l_float_abs <= cvt_fs_val_r[63] ?
	                            (~cvt_fs_val_r + 64'd1) : cvt_fs_val_r;
	                        cvt_state <= CVT_L_FLOAT_ROUND;
	                    end else begin
	                        cvt_result_r <= cvt_calc_result;
	                        cvt_exc_r <= cvt_calc_exc;
	                        cvt_exc_unimpl_r <= cvt_calc_exc_unimpl;
	                        cvt_exc_flags_r <= cvt_calc_exc_flags;
	                        cvt_state <= CVT_RESULT;
	                    end
	                end

	                CVT_S_D_ROUND: begin
	                    cvt_result_r <= {32'd0, cvt_s_d_final_result};
	                    cvt_exc_r <= 1'b0;
	                    cvt_exc_unimpl_r <= 1'b0;
	                    cvt_exc_flags_r <= {cvt_s_d_final_exc_invalid, 1'b0, cvt_s_d_final_exc_overflow,
	                                        cvt_s_d_final_exc_underflow, cvt_s_d_final_exc_inexact};
	                    cvt_state <= CVT_RESULT;
	                end

	                CVT_L_FLOAT_ROUND: begin
	                    if (cvt_funct_r == `FPU_CVT_S) begin
	                        cvt_result_r <= {32'd0, cvt_s_l_result};
	                        cvt_exc_flags_r <= {4'd0, cvt_s_l_exc_inexact};
	                    end else begin
	                        cvt_result_r <= cvt_d_l_result;
	                        cvt_exc_flags_r <= {4'd0, cvt_d_l_exc_inexact};
	                    end
	                    cvt_exc_r <= 1'b0;
	                    cvt_exc_unimpl_r <= 1'b0;
	                    cvt_state <= CVT_RESULT;
	                end

                CVT_RESULT: begin
                    cvt_state <= CVT_IDLE;
                end

                default: cvt_state <= CVT_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // Main operation dispatch
    // ---------------------------------------------------------------
    // R4000 computational and convert operations require software emulation
    // for a denormalized operand.  Multicycle results must classify the
    // operands captured with that launched operation, not the live RF/FPR
    // ports present when its RESULT state is later observed.
    wire add_denorm_operand_unimpl = (add_state == ADD_RESULT) &&
        (((add_fmt_r == `COP1_FMT_S) &&
          (fp32_is_denorm(add_fs_val_r[31:0]) ||
           fp32_is_denorm(add_ft_val_r[31:0]))) ||
         ((add_fmt_r == `COP1_FMT_D) &&
          (fp64_is_denorm(add_fs_val_r) || fp64_is_denorm(add_ft_val_r))));
    wire mul_denorm_operand_unimpl = (mul_state == MUL_RESULT) &&
        (((mul_fmt_r == `COP1_FMT_S) &&
          (fp32_is_denorm(mul_fs_val_r[31:0]) ||
           fp32_is_denorm(mul_ft_val_r[31:0]))) ||
         ((mul_fmt_r == `COP1_FMT_D) &&
          (fp64_is_denorm(mul_fs_val_r) || fp64_is_denorm(mul_ft_val_r))));
    wire div_denorm_operand_unimpl = (div_state == DIV_RESULT) &&
        (((div_fmt_r == `COP1_FMT_S) &&
          (fp32_is_denorm(div_fs_val_r[31:0]) ||
           fp32_is_denorm(div_ft_val_r[31:0]))) ||
         ((div_fmt_r == `COP1_FMT_D) &&
          (fp64_is_denorm(div_fs_val_r) || fp64_is_denorm(div_ft_val_r))));
    wire sqrt_denorm_operand_unimpl = (sqrt_state == SQRT_RESULT) &&
        ((!sqrt_is_double && fp32_is_denorm(sqrt_fs_val_r[31:0])) ||
         (sqrt_is_double && fp64_is_denorm(sqrt_fs_val_r)));
    wire cvt_denorm_operand_unimpl = (cvt_state == CVT_RESULT) &&
        (((cvt_fmt_r == `COP1_FMT_S) &&
          fp32_is_denorm(cvt_fs_val_r[31:0])) ||
         ((cvt_fmt_r == `COP1_FMT_D) && fp64_is_denorm(cvt_fs_val_r)));

    // ABS/NEG are the only supported computational operations whose result
    // is combinational.  Gate their live-port classification against every
    // multicycle engine so live decode cannot contaminate an older result.
    wire live_denorm_operand_unimpl = do_fpop && !fpu_busy &&
        ((funct == `FPU_ABS) || (funct == `FPU_NEG)) &&
        (((fmt == `COP1_FMT_S) && s_is_denorm_a) ||
         ((fmt == `COP1_FMT_D) && d_is_denorm_a));
    wire fpu_denorm_operand_unimpl = live_denorm_operand_unimpl ||
        add_denorm_operand_unimpl || mul_denorm_operand_unimpl ||
        div_denorm_operand_unimpl || sqrt_denorm_operand_unimpl ||
        cvt_denorm_operand_unimpl;

    always @(*) begin
        fp_result = 64'd0;
        fp_exc = 1'b0;
        fp_exc_unimpl = 1'b0;
        fp_exc_flags = 5'd0;

        if (cvt_state == CVT_RESULT) begin
            fp_result = cvt_result_r;
            fp_exc = cvt_exc_r;
            fp_exc_unimpl = cvt_exc_unimpl_r;
            fp_exc_flags = cvt_exc_flags_r;
        end else if (fpu_cvt_op) begin
            fp_result = 64'd0;
            fp_exc = 1'b0;
            fp_exc_unimpl = 1'b0;
            fp_exc_flags = 5'd0;
        end else if (do_fpop) begin
            case (fmt)
                `COP1_FMT_S: begin
                    case (1'b1)
                        (FPU_HAS_ADDSUB && (funct == `FPU_ADD || funct == `FPU_SUB)): begin
                            if (add_state == ADD_RESULT) begin
                                fp_result = {32'd0, add_result_r[31:0]};
                                fp_exc = add_result_exc_invalid;
                                fp_exc_flags = {add_result_exc_invalid, 1'b0, add_result_exc_overflow,
                                               add_result_exc_underflow, add_result_exc_inexact};
                            end
                        end
                        (FPU_HAS_MUL && funct == `FPU_MUL): begin
                            if (mul_state == MUL_RESULT) begin
                                fp_result = {32'd0, mul_result_r[31:0]};
                                fp_exc = mul_result_exc_invalid;
                                fp_exc_flags = {mul_result_exc_invalid, 1'b0, mul_result_exc_overflow,
                                               mul_result_exc_underflow, mul_result_exc_inexact};
                            end
                        end
                        (FPU_HAS_DIV && funct == `FPU_DIV): begin
                            if (div_state == DIV_RESULT) begin
                                fp_result = {32'd0, div_result_r[31:0]};
                                fp_exc = div_result_exc_invalid || div_result_exc_divzero;
                                fp_exc_flags = {div_result_exc_invalid, div_result_exc_divzero,
                                               div_result_exc_overflow, div_result_exc_underflow,
                                               div_result_exc_inexact};
                            end
                        end
                        (FPU_HAS_SQRT && funct == `FPU_SQRT): begin
                            if (sqrt_state == SQRT_RESULT) begin
                                fp_result = {32'd0, sqrt_result_r[31:0]};
                                fp_exc = sqrt_result_exc_invalid;
                                fp_exc_flags = {sqrt_result_exc_invalid, 3'd0, sqrt_result_exc_inexact};
                            end
                        end
                        (funct == `FPU_ABS): begin
                            if (s_is_nan_a) begin
                                fp_result = {32'd0, s_default_qnan};
                                fp_exc = 1'b1;
                                fp_exc_flags = 5'b10000;
                            end else if (flush_denorms && s_is_denorm_a)
                                fp_result = 64'd0;
                            else
                                fp_result = {32'd0, 1'b0, fs_val[30:0]};
                        end
                        (funct == `FPU_NEG): begin
                            if (s_is_nan_a) begin
                                fp_result = {32'd0, s_default_qnan};
                                fp_exc = 1'b1;
                                fp_exc_flags = 5'b10000;
                            end else if (flush_denorms && s_is_denorm_a)
                                fp_result = {32'd0, ~fs_val[31], 31'd0};
                            else
                                fp_result = {32'd0, ~fs_val[31], fs_val[30:0]};
                        end
                        (funct == `FPU_MOV): begin
                            fp_result = {{32{fs_val[31]}}, fs_val[31:0]};
                        end
                        (funct == `FPU_MOVZ || funct == `FPU_MOVN): begin
                            // The core suppresses the FPR write when the
                            // condition is false, so the result data need not
                            // depend on the GPR condition path.
                            fp_result = {{32{fs_val[31]}}, fs_val[31:0]};
                        end
                        (funct == `FPU_MOVCF): begin
                            // MOVF.S/MOVT.S: cc=ft[4:2], tf=ft[0]
                            if (fcc_bits[ft[4:2]] == ft[0])
                                fp_result = {{32{fs_val[31]}}, fs_val[31:0]};
                        end
                        (FPU_HAS_CMP && funct >= 6'h30): begin  // C.cond.S
                            fp_result = {63'd0, s_cmp_result};
                            fp_exc = s_cmp_exc_invalid;
                            fp_exc_flags = {s_cmp_exc_invalid, 4'd0};
                        end
                        default: begin
                            fp_exc_unimpl = 1'b1;
                        end
                    endcase
                end

                `COP1_FMT_D: begin
                    case (1'b1)
                        (FPU_HAS_ADDSUB && (funct == `FPU_ADD || funct == `FPU_SUB)): begin
                            if (add_state == ADD_RESULT) begin
                                fp_result = add_result_r;
                                fp_exc = add_result_exc_invalid;
                                fp_exc_flags = {add_result_exc_invalid, 1'b0, add_result_exc_overflow,
                                               add_result_exc_underflow, add_result_exc_inexact};
                            end
                        end
                        (FPU_HAS_MUL && funct == `FPU_MUL): begin
                            if (mul_state == MUL_RESULT) begin
                                fp_result = mul_result_r;
                                fp_exc = mul_result_exc_invalid;
                                fp_exc_flags = {mul_result_exc_invalid, 1'b0, mul_result_exc_overflow,
                                               mul_result_exc_underflow, mul_result_exc_inexact};
                            end
                        end
                        (FPU_HAS_DIV && funct == `FPU_DIV): begin
                            if (div_state == DIV_RESULT) begin
                                fp_result = div_result_r;
                                fp_exc = div_result_exc_invalid || div_result_exc_divzero;
                                fp_exc_flags = {div_result_exc_invalid, div_result_exc_divzero,
                                               div_result_exc_overflow, div_result_exc_underflow,
                                               div_result_exc_inexact};
                            end
                        end
                        (FPU_HAS_SQRT && funct == `FPU_SQRT): begin
                            if (sqrt_state == SQRT_RESULT) begin
                                fp_result = sqrt_result_r;
                                fp_exc = sqrt_result_exc_invalid;
                                fp_exc_flags = {sqrt_result_exc_invalid, 3'd0, sqrt_result_exc_inexact};
                            end
                        end
                        (funct == `FPU_ABS): begin
                            if (d_is_nan_a) begin
                                fp_result = d_default_qnan;
                                fp_exc = 1'b1;
                                fp_exc_flags = 5'b10000;
                            end else if (flush_denorms && d_is_denorm_a)
                                fp_result = 64'd0;
                            else
                                fp_result = {1'b0, fs_val[62:0]};
                        end
                        (funct == `FPU_NEG): begin
                            if (d_is_nan_a) begin
                                fp_result = d_default_qnan;
                                fp_exc = 1'b1;
                                fp_exc_flags = 5'b10000;
                            end else if (flush_denorms && d_is_denorm_a)
                                fp_result = {~fs_val[63], 63'd0};
                            else
                                fp_result = {~fs_val[63], fs_val[62:0]};
                        end
                        (funct == `FPU_MOV): begin
                            fp_result = fs_val;
                        end
                        (funct == `FPU_MOVZ || funct == `FPU_MOVN): begin
                            // The core suppresses the FPR write when the
                            // condition is false, so the result data need not
                            // depend on the GPR condition path.
                            fp_result = fs_val;
                        end
                        (funct == `FPU_MOVCF): begin
                            // MOVF.D/MOVT.D: cc=ft[4:2], tf=ft[0]
                            if (fcc_bits[ft[4:2]] == ft[0])
                                fp_result = fs_val;
                        end
                        (FPU_HAS_CMP && funct >= 6'h30): begin  // C.cond.D
                            fp_result = {63'd0, d_cmp_result};
                            fp_exc = d_cmp_exc_invalid;
                            fp_exc_flags = {d_cmp_exc_invalid, 4'd0};
                        end
                        default: begin
                            fp_exc_unimpl = 1'b1;
                        end
                    endcase
                end

                `COP1_FMT_W: begin
                    case (funct)
                        default: fp_exc_unimpl = 1'b1;
                    endcase
                end

                `COP1_FMT_L: begin
                    case (funct)
                        default: fp_exc_unimpl = 1'b1;
                    endcase
                end

                default: fp_exc_unimpl = 1'b1;
            endcase
        end

        // MADD/MSUB dispatch — valid result is selected from the latched
        // multicycle operation state, not the live RF decode bus.
        if (FPU_HAS_MUL && mul_is_madd && mul_state == MUL_MADD_RESULT) begin
            if (!mul_is_double) begin  // Single precision (MADD.S / MSUB.S)
                fp_result = {32'd0, mul_result_r[31:0]};
            end else begin  // Double precision (MADD.D / MSUB.D)
                fp_result = mul_result_r;
            end
            fp_exc = mul_result_exc_invalid;
            fp_exc_flags = {mul_result_exc_invalid, 1'b0, mul_result_exc_overflow,
                           mul_result_exc_underflow, mul_result_exc_inexact};
        end

        // Operand emulation takes precedence over any result the arithmetic
        // datapath happened to compute.  E has no ordinary V/Z/O/U/I flags
        // and always traps, so the architectural destination remains intact.
        if (fpu_denorm_operand_unimpl) begin
            fp_result = 64'd0;
            fp_exc = 1'b0;
            fp_exc_unimpl = 1'b1;
            fp_exc_flags = 5'd0;
        end
    end

    // ---------------------------------------------------------------
    // Register file write
    // ---------------------------------------------------------------
    genvar fpr_g;
    generate
        for (fpr_g = 0; fpr_g < 32; fpr_g = fpr_g + 1) begin : fpr_write_regs
            localparam [4:0] FPR_IDX = fpr_g[4:0];
            `R4000_FPU_SEQ begin
                if (!rst_n) begin
                    fpr[fpr_g] <= 64'd0;
                end else begin
                    if (fpr_early_wen && fpr_early_wr_idx == FPR_IDX)
                        fpr[fpr_g] <= fpr_merge_write(fpr[fpr_g], fd_wr_early,
                                                       fpr_wdata_early,
                                                       fpr_wdata_early_word,
                                                       fr_mode);

                    // Late WB write keeps the existing priority if both paths
                    // target the same FPR in one cycle.
                    if (wen && fpr_late_wr_idx == FPR_IDX)
                        fpr[fpr_g] <= fpr_merge_write(fpr[fpr_g], fd_wr,
                                                       fpr_wdata,
                                                       fpr_wdata_word,
                                                       fr_mode_wr);
                end
            end
        end
    endgenerate

    `R4000_FPU_SEQ begin
        if (!rst_n) begin : fpr_reset_loop
            fcsr <= 32'd0;
        end else begin
            // Early write (RF stage — speculative, for MTC1/DMTC1/arithmetic/CMP)
            if (exc_trap_early) begin
                // Trapping FPU exception: write cause bits only, NOT flags or FPR
                fcsr[16:12] <= fp_exc_flags;
            end else if (flags_wen_early) begin
                // Update FCSR exception flags for arithmetic ops
                if (fp_exc_flags != 5'd0) begin
                    fcsr[6:2] <= fcsr[6:2] | fp_exc_flags;   // Flags (sticky)
                    fcsr[16:12] <= fp_exc_flags;              // Cause
                end
            end

            if (fcsr_wen_early) begin
                fcsr <= fcsr_wdata_early;
            end

            // Late write (WB stage — committed, for LWC1/LDC1)
            // Late write takes priority if both fire on same cycle (shouldn't happen)
            if (fcsr_wen) begin
                fcsr <= fcsr_wdata;
            end
        end
    end

endmodule
`undef R4000_FPU_SEQ
