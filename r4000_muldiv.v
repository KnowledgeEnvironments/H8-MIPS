// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_muldiv.v — Synthesizable Multiply/Divide unit for R4000SC
//
// Multiplies use iterative partial-product accumulation with a single
// 32x32 multiply that synthesis tools can infer as DSP blocks.
// Divides use a restoring algorithm at 1 bit per cycle.
//
// R4000-accurate latencies: MULT/MULTU=10, DMULT/DMULTU=20,
// DIV/DIVU=35, DDIV/DDIVU=69.
//
// Interface:
//   - start=1 latches operands and begins computation
//   - busy=1 while computing
//   - done=1 for exactly 1 cycle when result is ready
//   - hi/lo valid only when done=1, otherwise 0
//   - cancel=1 aborts in-flight operation

`default_nettype none

module r4000_muldiv (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [63:0] a,           // rs value
    input  wire [63:0] b,           // rt value
    input  wire [63:0] hi_in,       // current HI (for MADD/MSUB)
    input  wire [63:0] lo_in,       // current LO (for MADD/MSUB)
    input  wire [ 3:0] op,         // operation select
    input  wire        start,       // begin operation
    input  wire        cancel,      // cancel in-flight operation (pipeline flush)
    output reg  [63:0] hi,          // HI result
    output reg  [63:0] lo,          // LO result
    output wire        done,        // result ready (pulses 1 cycle)
    output wire        busy         // unit is busy
);

    // Operation encoding
    localparam MD_MULT   = 4'd0;
    localparam MD_MULTU  = 4'd1;
    localparam MD_DIV    = 4'd2;
    localparam MD_DIVU   = 4'd3;
    localparam MD_DMULT  = 4'd4;
    localparam MD_DMULTU = 4'd5;
    localparam MD_DDIV   = 4'd6;
    localparam MD_DDIVU  = 4'd7;
    localparam MD_MADD   = 4'd8;
    localparam MD_MADDU  = 4'd9;
    localparam MD_MUL    = 4'd10;
    localparam MD_MSUB   = 4'd11;
    localparam MD_MSUBU  = 4'd12;

    // R4000 latencies (cycles from start to done, inclusive)
    localparam [6:0] LAT_MULT   = 7'd10;
    localparam [6:0] LAT_MULTU  = 7'd10;
    localparam [6:0] LAT_DIV    = 7'd35;
    localparam [6:0] LAT_DIVU   = 7'd35;
    localparam [6:0] LAT_DMULT  = 7'd20;
    localparam [6:0] LAT_DMULTU = 7'd20;
    localparam [6:0] LAT_DDIV   = 7'd69;
    localparam [6:0] LAT_DDIVU  = 7'd69;
    localparam [6:0] LAT_MADD   = 7'd10;
    localparam [6:0] LAT_MUL    = 7'd10;

    // FSM states
    localparam S_IDLE     = 3'd0;
    localparam S_MUL_32   = 3'd1;  // 32-bit multiply (single 32x32)
    localparam S_MUL_64   = 3'd2;  // 64-bit multiply (four 32x32 partial products)
    localparam S_DIV      = 3'd3;  // division (restoring, 1 bit/cycle)
    localparam S_DIV_FIN  = 3'd4;  // division finalization
    localparam S_PAD      = 3'd5;  // padding cycles to match R4000 latency

    reg [2:0]   state;
    reg [6:0]   count;          // cycles remaining to done
    reg [3:0]   saved_op;       // latched operation
    reg         negate_result;  // signed mul: negate product

    // Multiply accumulator: 128-bit product built from 32x32 partial products
    reg [127:0] prod;           // accumulating product
    reg [63:0]  mul_a;          // latched multiplicand (absolute value)
    reg [63:0]  mul_b;          // latched multiplier (absolute value)
    reg [2:0]   mul_step;       // which partial product step

    // Divide registers — restoring algorithm
    // div_quot holds the dividend (shifted out from MSB) and quotient (shifted in at LSB)
    reg [64:0]  div_rem;        // remainder accumulator (65 bits for trial subtraction sign)
    reg [63:0]  div_quot;       // dividend bits shift out, quotient bits shift in
    reg [63:0]  div_divisor;    // absolute value of divisor
    reg [6:0]   div_step;       // iterations remaining
    reg         div_neg_quot;   // negate quotient at end (signed division)
    reg         div_neg_rem;    // negate remainder at end (signed division)
    reg         div_is_32;      // 1 = 32-bit division, 0 = 64-bit

    // Result registers
    reg [63:0]  result_hi;
    reg [63:0]  result_lo;

    // MADD/MSUB accumulator input
    reg [63:0]  acc_hi;
    reg [63:0]  acc_lo;

    assign busy = (state != S_IDLE);
    assign done = (state != S_IDLE) && (count == 7'd1);

    // Output mux: valid on done only
    always @(*) begin
        if (done) begin
            hi = result_hi;
            lo = result_lo;
        end else begin
            hi = 64'd0;
            lo = 64'd0;
        end
    end

    // Latency lookup
    function [6:0] get_latency;
        input [3:0] op_in;
        begin
            case (op_in)
                MD_MULT:   get_latency = LAT_MULT;
                MD_MULTU:  get_latency = LAT_MULTU;
                MD_DIV:    get_latency = LAT_DIV;
                MD_DIVU:   get_latency = LAT_DIVU;
                MD_DMULT:  get_latency = LAT_DMULT;
                MD_DMULTU: get_latency = LAT_DMULTU;
                MD_DDIV:   get_latency = LAT_DDIV;
                MD_DDIVU:  get_latency = LAT_DDIVU;
                MD_MADD:   get_latency = LAT_MADD;
                MD_MADDU:  get_latency = LAT_MADD;
                MD_MUL:    get_latency = LAT_MUL;
                MD_MSUB:   get_latency = LAT_MADD;
                MD_MSUBU:  get_latency = LAT_MADD;
                default:   get_latency = 7'd1;
            endcase
        end
    endfunction

    // ---------------------------------------------------------------
    // 32x32 → 64-bit multiplier — the only multiply primitive.
    // Synthesis tools infer DSP blocks from this.
    // ---------------------------------------------------------------
    reg  [31:0] pp_a, pp_b;
    wire [63:0] pp_result = pp_a * pp_b;

    // ---------------------------------------------------------------
    // Restoring divider: trial subtraction
    // Shift dividend MSB into remainder, try subtract divisor.
    // If non-negative: keep result, quotient bit = 1
    // If negative: discard (restore), quotient bit = 0
    // ---------------------------------------------------------------
    wire [64:0] div_trial = {div_rem[63:0], div_quot[63]} - {1'b0, div_divisor};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            count       <= 7'd0;
            saved_op    <= 4'd0;
            result_hi   <= 64'd0;
            result_lo   <= 64'd0;
            prod        <= 128'd0;
            mul_a       <= 64'd0;
            mul_b       <= 64'd0;
            mul_step    <= 3'd0;
            div_rem     <= 65'd0;
            div_quot    <= 64'd0;
            div_divisor <= 64'd0;
            div_step    <= 7'd0;
            div_neg_quot <= 1'b0;
            div_neg_rem  <= 1'b0;
            div_is_32   <= 1'b0;
            negate_result <= 1'b0;
            pp_a        <= 32'd0;
            pp_b        <= 32'd0;
            acc_hi      <= 64'd0;
            acc_lo      <= 64'd0;
        end else if (cancel && state != S_IDLE) begin
            state <= S_IDLE;
        end else if (start) begin
            saved_op <= op;
            count    <= get_latency(op);

            case (op)
                // ---- 32-bit multiply ops ----
                MD_MULT, MD_MUL: begin
                    mul_a <= {32'd0, a[31] ? (~a[31:0] + 32'd1) : a[31:0]};
                    mul_b <= {32'd0, b[31] ? (~b[31:0] + 32'd1) : b[31:0]};
                    negate_result <= a[31] ^ b[31];
                    prod <= 128'd0;
                    mul_step <= 3'd0;
                    acc_hi <= 64'd0;
                    acc_lo <= 64'd0;
                    state <= S_MUL_32;
                    pp_a <= a[31] ? (~a[31:0] + 32'd1) : a[31:0];
                    pp_b <= b[31] ? (~b[31:0] + 32'd1) : b[31:0];
                end

                MD_MULTU: begin
                    mul_a <= {32'd0, a[31:0]};
                    mul_b <= {32'd0, b[31:0]};
                    negate_result <= 1'b0;
                    prod <= 128'd0;
                    mul_step <= 3'd0;
                    acc_hi <= 64'd0;
                    acc_lo <= 64'd0;
                    state <= S_MUL_32;
                    pp_a <= a[31:0];
                    pp_b <= b[31:0];
                end

                MD_MADD: begin
                    mul_a <= {32'd0, a[31] ? (~a[31:0] + 32'd1) : a[31:0]};
                    mul_b <= {32'd0, b[31] ? (~b[31:0] + 32'd1) : b[31:0]};
                    negate_result <= a[31] ^ b[31];
                    prod <= 128'd0;
                    mul_step <= 3'd0;
                    acc_hi <= hi_in;
                    acc_lo <= lo_in;
                    state <= S_MUL_32;
                    pp_a <= a[31] ? (~a[31:0] + 32'd1) : a[31:0];
                    pp_b <= b[31] ? (~b[31:0] + 32'd1) : b[31:0];
                end

                MD_MADDU: begin
                    mul_a <= {32'd0, a[31:0]};
                    mul_b <= {32'd0, b[31:0]};
                    negate_result <= 1'b0;
                    prod <= 128'd0;
                    mul_step <= 3'd0;
                    acc_hi <= hi_in;
                    acc_lo <= lo_in;
                    state <= S_MUL_32;
                    pp_a <= a[31:0];
                    pp_b <= b[31:0];
                end

                MD_MSUB: begin
                    mul_a <= {32'd0, a[31] ? (~a[31:0] + 32'd1) : a[31:0]};
                    mul_b <= {32'd0, b[31] ? (~b[31:0] + 32'd1) : b[31:0]};
                    negate_result <= a[31] ^ b[31];
                    prod <= 128'd0;
                    mul_step <= 3'd0;
                    acc_hi <= hi_in;
                    acc_lo <= lo_in;
                    state <= S_MUL_32;
                    pp_a <= a[31] ? (~a[31:0] + 32'd1) : a[31:0];
                    pp_b <= b[31] ? (~b[31:0] + 32'd1) : b[31:0];
                end

                MD_MSUBU: begin
                    mul_a <= {32'd0, a[31:0]};
                    mul_b <= {32'd0, b[31:0]};
                    negate_result <= 1'b0;
                    prod <= 128'd0;
                    mul_step <= 3'd0;
                    acc_hi <= hi_in;
                    acc_lo <= lo_in;
                    state <= S_MUL_32;
                    pp_a <= a[31:0];
                    pp_b <= b[31:0];
                end

                // ---- 64-bit multiply ops ----
                MD_DMULT: begin
                    mul_a <= a[63] ? (~a + 64'd1) : a;
                    mul_b <= b[63] ? (~b + 64'd1) : b;
                    negate_result <= a[63] ^ b[63];
                    prod <= 128'd0;
                    mul_step <= 3'd0;
                    acc_hi <= 64'd0;
                    acc_lo <= 64'd0;
                    state <= S_MUL_64;
                    // Set up first partial product: abs(a)[31:0] * abs(b)[31:0]
                    // Note: lower 32 bits of abs(x) = (~x[31:0] + 1) regardless of upper bits
                    pp_a <= a[63] ? (~a[31:0] + 32'd1) : a[31:0];
                    pp_b <= b[63] ? (~b[31:0] + 32'd1) : b[31:0];
                end

                MD_DMULTU: begin
                    mul_a <= a;
                    mul_b <= b;
                    negate_result <= 1'b0;
                    prod <= 128'd0;
                    mul_step <= 3'd0;
                    acc_hi <= 64'd0;
                    acc_lo <= 64'd0;
                    state <= S_MUL_64;
                    pp_a <= a[31:0];
                    pp_b <= b[31:0];
                end

                // ---- 32-bit divide ops ----
                MD_DIV: begin
                    if (b[31:0] == 32'd0) begin
                        result_hi <= 64'd0;
                        result_lo <= 64'd0;
                        state <= S_PAD;
                    end else if (a[31:0] == 32'h80000000 && b[31:0] == 32'hFFFFFFFF) begin
                        result_lo <= 64'hFFFFFFFF_80000000;
                        result_hi <= 64'd0;
                        state <= S_PAD;
                    end else begin
                        div_rem      <= 65'd0;
                        // Place dividend in upper 32 bits so bit 63 shifts out first
                        div_quot     <= {(a[31] ? (~a[31:0] + 32'd1) : a[31:0]), 32'd0};
                        div_divisor  <= {32'd0, (b[31] ? (~b[31:0] + 32'd1) : b[31:0])};
                        div_neg_quot <= a[31] ^ b[31];
                        div_neg_rem  <= a[31];
                        div_step     <= 7'd32;
                        div_is_32    <= 1'b1;
                        state        <= S_DIV;
                    end
                end

                MD_DIVU: begin
                    if (b[31:0] == 32'd0) begin
                        result_hi <= 64'd0;
                        result_lo <= 64'd0;
                        state <= S_PAD;
                    end else begin
                        div_rem      <= 65'd0;
                        div_quot     <= {a[31:0], 32'd0};
                        div_divisor  <= {32'd0, b[31:0]};
                        div_neg_quot <= 1'b0;
                        div_neg_rem  <= 1'b0;
                        div_step     <= 7'd32;
                        div_is_32    <= 1'b1;
                        state        <= S_DIV;
                    end
                end

                // ---- 64-bit divide ops ----
                MD_DDIV: begin
                    if (b == 64'd0) begin
                        result_hi <= 64'd0;
                        result_lo <= 64'd0;
                        state <= S_PAD;
                    end else if (a == 64'h8000000000000000 && b == 64'hFFFFFFFFFFFFFFFF) begin
                        result_lo <= 64'h8000000000000000;
                        result_hi <= 64'd0;
                        state <= S_PAD;
                    end else begin
                        div_rem      <= 65'd0;
                        div_quot     <= a[63] ? (~a + 64'd1) : a;
                        div_divisor  <= b[63] ? (~b + 64'd1) : b;
                        div_neg_quot <= a[63] ^ b[63];
                        div_neg_rem  <= a[63];
                        div_step     <= 7'd64;
                        div_is_32    <= 1'b0;
                        state        <= S_DIV;
                    end
                end

                MD_DDIVU: begin
                    if (b == 64'd0) begin
                        result_hi <= 64'd0;
                        result_lo <= 64'd0;
                        state <= S_PAD;
                    end else begin
                        div_rem      <= 65'd0;
                        div_quot     <= a;
                        div_divisor  <= b;
                        div_neg_quot <= 1'b0;
                        div_neg_rem  <= 1'b0;
                        div_step     <= 7'd64;
                        div_is_32    <= 1'b0;
                        state        <= S_DIV;
                    end
                end

                default: begin
                    result_hi <= 64'd0;
                    result_lo <= 64'd0;
                    state <= S_PAD;
                end
            endcase

        end else begin
            // Running: advance computation
            case (state)

                // -------------------------------------------------------
                // 32-bit multiply: single 32x32 partial product
                // -------------------------------------------------------
                S_MUL_32: begin
                    if (mul_step == 3'd0) begin
                        // pp_result has our 32x32 unsigned product
                        prod[63:0] <= pp_result;
                        mul_step <= 3'd1;
                    end else if (mul_step == 3'd1) begin
                        // Finalize: apply sign, accumulate for MADD/MSUB
                        begin
                            reg [63:0] signed_prod;
                            reg [63:0] final_val;

                            // Negate product if signed multiply with differing signs
                            if (negate_result)
                                signed_prod = ~prod[63:0] + 64'd1;
                            else
                                signed_prod = prod[63:0];

                            // MADD/MSUB: add/subtract from HI:LO accumulator
                            case (saved_op)
                                MD_MADD: begin
                                    final_val = {acc_hi[31:0], acc_lo[31:0]} + signed_prod;
                                end
                                MD_MADDU: begin
                                    final_val = {acc_hi[31:0], acc_lo[31:0]} + signed_prod;
                                end
                                MD_MSUB: begin
                                    final_val = {acc_hi[31:0], acc_lo[31:0]} - signed_prod;
                                end
                                MD_MSUBU: begin
                                    final_val = {acc_hi[31:0], acc_lo[31:0]} - prod[63:0];
                                end
                                default: begin
                                    final_val = signed_prod;
                                end
                            endcase

                            result_lo <= {{32{final_val[31]}}, final_val[31:0]};
                            result_hi <= {{32{final_val[63]}}, final_val[63:32]};
                        end
                        state <= S_PAD;
                    end
                    count <= count - 7'd1;
                end

                // -------------------------------------------------------
                // 64-bit multiply: four 32x32 partial products
                // P = (AH*2^32 + AL) * (BH*2^32 + BL)
                //   = AH*BH*2^64 + (AH*BL + AL*BH)*2^32 + AL*BL
                // -------------------------------------------------------
                S_MUL_64: begin
                    case (mul_step)
                        3'd0: begin
                            // AL*BL ready from pp_result
                            prod <= {64'd0, pp_result};
                            pp_a <= mul_a[63:32];
                            pp_b <= mul_b[31:0];
                            mul_step <= 3'd1;
                        end
                        3'd1: begin
                            // AH*BL ready, add at offset 32
                            prod <= prod + ({64'd0, pp_result} << 32);
                            pp_a <= mul_a[31:0];
                            pp_b <= mul_b[63:32];
                            mul_step <= 3'd2;
                        end
                        3'd2: begin
                            // AL*BH ready, add at offset 32
                            prod <= prod + ({64'd0, pp_result} << 32);
                            pp_a <= mul_a[63:32];
                            pp_b <= mul_b[63:32];
                            mul_step <= 3'd3;
                        end
                        3'd3: begin
                            // AH*BH ready, add at offset 64
                            prod <= prod + ({64'd0, pp_result} << 64);
                            mul_step <= 3'd4;
                        end
                        3'd4: begin
                            // Finalize: negate if needed
                            if (negate_result) begin
                                begin
                                    reg [127:0] neg_prod;
                                    neg_prod = ~prod + 128'd1;
                                    result_lo <= neg_prod[63:0];
                                    result_hi <= neg_prod[127:64];
                                end
                            end else begin
                                result_lo <= prod[63:0];
                                result_hi <= prod[127:64];
                            end
                            state <= S_PAD;
                        end
                        default: ;
                    endcase
                    count <= count - 7'd1;
                end

                // -------------------------------------------------------
                // Restoring division: 1 bit per cycle
                // Shift dividend MSB (bit 63 of div_quot) into remainder,
                // trial-subtract divisor. If non-negative, keep and set
                // quotient bit = 1. Otherwise restore and set bit = 0.
                // -------------------------------------------------------
                S_DIV: begin
                    if (div_step != 7'd0) begin
                        // div_trial = {div_rem[63:0], div_quot[63]} - {0, div_divisor}
                        if (!div_trial[64]) begin
                            // Trial succeeded (non-negative): accept
                            div_rem  <= div_trial;
                            div_quot <= {div_quot[62:0], 1'b1};
                        end else begin
                            // Trial failed (negative): restore (just shift, don't subtract)
                            div_rem  <= {div_rem[63:0], div_quot[63]};
                            div_quot <= {div_quot[62:0], 1'b0};
                        end
                        div_step <= div_step - 7'd1;
                    end else begin
                        state <= S_DIV_FIN;
                    end
                    count <= count - 7'd1;
                end

                // -------------------------------------------------------
                // Division finalization: apply signs, format result
                // -------------------------------------------------------
                S_DIV_FIN: begin
                    begin
                        reg [63:0] raw_quot;
                        reg [63:0] raw_rem;
                        reg [63:0] final_quot;
                        reg [63:0] final_rem;

                        raw_rem = div_rem[63:0];

                        if (div_is_32) begin
                            // 32-bit: quotient in lower 32 bits of div_quot
                            raw_quot = {32'd0, div_quot[31:0]};

                            final_quot = div_neg_quot ? (~raw_quot + 64'd1) : raw_quot;
                            final_rem  = div_neg_rem  ? (~raw_rem  + 64'd1) : raw_rem;

                            // Sign-extend 32-bit results to 64-bit
                            result_lo <= {{32{final_quot[31]}}, final_quot[31:0]};
                            result_hi <= {{32{final_rem[31]}},  final_rem[31:0]};
                        end else begin
                            // 64-bit: quotient fills all of div_quot
                            raw_quot = div_quot;

                            final_quot = div_neg_quot ? (~raw_quot + 64'd1) : raw_quot;
                            final_rem  = div_neg_rem  ? (~raw_rem  + 64'd1) : raw_rem;

                            result_lo <= final_quot;
                            result_hi <= final_rem;
                        end
                    end
                    state <= S_PAD;
                    count <= count - 7'd1;
                end

                // -------------------------------------------------------
                // Padding to match R4000 latency
                // -------------------------------------------------------
                S_PAD: begin
                    if (count == 7'd1)
                        state <= S_IDLE;
                    count <= count - 7'd1;
                end

                default: ;
            endcase
        end
    end

endmodule
