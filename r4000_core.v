// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_core.v — R4000SC pipelined core (Phase 2, 8-stage)
// 8-stage pipeline: IF → IS → RF → EX → DF → DS → TC → WB
// Full forwarding, load-use stall, branch delay slots, multi-cycle muldiv.

`default_nettype none
`include "r4000_defs.vh"
`ifdef SYNTHESIS
`include "r4000_tlb.v"
`default_nettype none
`endif

module r4000_core (
    input  wire        clk,
    input  wire        rst_n,

    // Instruction memory (physical address → combinational response)
    output wire [63:0] imem_addr,
    output wire        imem_valid,
    input  wire [31:0] imem_rdata,
    // Registered-L1 lookahead.  imem_addr identifies the instruction in IS;
    // imem_fetch_addr identifies the IF request that will enter IS at the
    // next advancing edge.  A synchronous cache may read the latter while
    // returning the former without adding a pipeline stage.
    output wire [63:0] imem_fetch_addr,
    output wire        imem_fetch_advance,

    // Data memory interface
    output reg         dmem_ren,
    output reg         dmem_wen,
    output reg  [63:0] dmem_addr,
    // DF-stage physical address for registered Primary-D tag/data lookup.
    // dmem_addr remains the identity of the DS-stage request.  A synchronous
    // cache may read this address while DF advances into DS, then fall back to
    // dmem_addr whenever the memory pipeline is held.
    output wire [63:0] dmem_lookup_addr,
    output wire        dmem_lookup_advance,
    output reg  [ 7:0] dmem_wmask,
    output reg  [63:0] dmem_wdata,
    input  wire [63:0] dmem_rdata,

    // Cache control
    input  wire        cache_stall,     // freeze pipeline (I$ or D$ miss)
    input  wire        uncached_started,  // pulse: uncached D op buffered for external issue
    input  wire        uncached_accepted, // pulse: uncached D operation accepted by device
    input  wire        cached_store_visible, // pulse: cached store mutated D$ in DS
    output wire        imem_uncached,   // instruction fetch is uncached
    output wire        dmem_uncached,   // data access is uncached
    input  wire        force_uncached,  // diagnostic: force all D accesses to uncached
                                        //            (Gate A fs_lock cache hypothesis)

    // Hardware interrupts (active-high, directly mapped to Cause IP[7:2])
    input  wire [5:0]  hw_int,

    // Simulation: skip Count forward to trigger timer interrupt sooner
    input  wire        sim_idle_boost,

    // DMA snoop invalidation (from cache snoop logic) — CC-2: clears llbit when
    // the LL'd cache line is invalidated by external bus activity (128 B line).
    input  wire        snoop_valid,
    input  wire [31:0] snoop_addr,

    // S-cache line eviction (inclusion maintenance signal) — CC-4: clears llbit
    // when the LL'd L2 line is replaced due to conflict miss or CACHE op. Prevents
    // "lladdr aliasing in L2 with current PC → infinite SC loop" (direct-mapped
    // 1 MB L2 with I-fetch or D-access at 1 MB stride collides with the LL'd line).
    input  wire        evict_valid,
    input  wire [31:0] evict_addr,

    // CACHE instruction (from WB commit)
    output wire        cache_cop_valid,
    output wire [ 2:0] cache_cop_op,
    output wire [ 1:0] cache_cop_type,  // 0=I$, 1=D$, 2/3=S$
    output wire [31:0] cache_cop_paddr,
    output wire [31:0] cache_cop_taglo_out, // TagLo for Index Store Tag
    input  wire [31:0] cache_cop_taglo_in,  // Tag readback for Index Load Tag
    input  wire        cache_cop_taglo_update_valid,
    input  wire [31:0] cache_cop_taglo_update_data,
    output wire        retirement_tail_empty,

    // MFI — MIPS Formal Interface (co-simulation)
    output reg         mfi_valid,
    output reg  [63:0] mfi_order,
    output reg  [63:0] mfi_pc,
    output reg  [31:0] mfi_insn,
    output reg         mfi_trap,
    output reg  [ 4:0] mfi_rd_addr,
    output reg  [63:0] mfi_rd_wdata,
    output reg  [ 1:0] mfi_hilo_wen,
    output reg  [63:0] mfi_hi_wdata,
    output reg  [63:0] mfi_lo_wdata,
    output reg  [63:0] mfi_mem_addr,
    output reg  [ 7:0] mfi_mem_rmask,
    output reg  [ 7:0] mfi_mem_wmask,
    output reg  [63:0] mfi_mem_rdata,
    output reg  [63:0] mfi_mem_wdata,

    output wire [31:0] debug_pc,
    output wire [31:0] debug_is_pc,
    output wire [31:0] debug_rf_pc,
    output wire [31:0] debug_rf_insn,
    output wire [31:0] debug_wb_pc,
    output wire [31:0] debug_wb_insn,
    output wire [31:0] debug_epc,
    output wire [31:0] debug_cause,
    output wire [31:0] debug_badvaddr,
    output wire [31:0] debug_status,
    output wire [31:0] debug_context,
    output wire [31:0] debug_entryhi,
    output wire [31:0] debug_flags,
    output wire [31:0] debug_gpr_v0,
    output wire [31:0] debug_gpr_v1,
    output wire [31:0] debug_gpr_at,
    output wire [31:0] debug_gpr_t0,
    output wire [31:0] debug_gpr_t1,
    output wire [31:0] debug_gpr_a0,
    output wire [31:0] debug_gpr_a1,
    output wire [31:0] debug_gpr_a2,
    output wire [31:0] debug_gpr_t8,
    output wire [31:0] debug_gpr_s4,
    output wire [31:0] debug_gpr_s5,
    output wire [31:0] debug_gpr_s6,
    output wire [31:0] debug_gpr_t2,
    output wire [31:0] debug_gpr_t3,
    output wire [31:0] debug_gpr_a3,
    output wire [31:0] debug_gpr_s0,
    output wire [31:0] debug_gpr_s1,
    output wire [31:0] debug_gpr_s2,
    output wire [31:0] debug_gpr_s3,
    output wire [31:0] debug_gpr_t4,
    output wire [31:0] debug_gpr_t6,
    output wire [31:0] debug_gpr_t7,
    output wire [31:0] debug_gpr_t9,
    output wire [31:0] debug_gpr_sp,
    output wire [31:0] debug_gpr_ra,
    output wire [31:0] debug_last_cf_pc,
    output wire [31:0] debug_last_cf_target,
    output wire [31:0] debug_wb_eff_addr,
    output wire [31:0] debug_wb_load_data,
    output wire [31:0] debug_dmem_pc,
    output wire [31:0] debug_dmem_insn,
    output wire [31:0] debug_dmem_vaddr,
    output wire [31:0] debug_dtlb_state,
    output wire [31:0] debug_dtlb_paddr,
    output wire [31:0] debug_dtlb_lo,
    output wire [31:0] debug_dtlb_mask,
    input  wire [5:0]  debug_tlb_idx,
    output wire [31:0] debug_tlb_hi,
    output wire [31:0] debug_tlb_lo0,
    output wire [31:0] debug_tlb_lo1,
    output wire [31:0] debug_tlb_mask,
    output wire [31:0] debug_tlbwr_meta,
    output wire [31:0] debug_tlbwr_hi,
    output wire [31:0] debug_tlbwr_lo0,
    output wire [31:0] debug_tlbwr_lo1,
    output wire [31:0] debug_tlbwr_mask
);

    // ---------------------------------------------------------------
    // Architectural state
    // ---------------------------------------------------------------
    reg [63:0] pc;
    reg [63:0] gpr [1:31];
    reg [63:0] hi_reg, lo_reg;
    reg        llbit;
    reg [63:0] ll_addr;
    reg [31:0] ll_paddr;  // CC-2: physical address of locked line (for DMA snoop match)
    reg [31:0] last_cf_pc;
    reg [31:0] last_cf_target;
`ifndef FPGA_NO_MFI
    reg [63:0] insn_count;
`endif

    // CP0 registers
    reg [63:0] cp0_index, cp0_random, cp0_entrylo0, cp0_entrylo1;
    reg [63:0] cp0_context, cp0_pagemask, cp0_wired, cp0_badvaddr;
    reg        cp0_pagemask_valid;
    reg [63:0] cp0_count, cp0_entryhi, cp0_compare, cp0_status;
    reg        cp0_count_phase;
    reg [63:0] cp0_cause, cp0_epc, cp0_prid, cp0_config;
    reg [63:0] cp0_lladdr, cp0_watchlo, cp0_watchhi, cp0_xcontext;
    reg [63:0] cp0_ecc, cp0_cacheerr, cp0_taglo, cp0_taghi, cp0_errorepc;

    localparam [63:0] CP0_PRID_VAL = {32'd0, `PRID_R4000SC};
    localparam [63:0] CP0_CONFIG_RESET = 64'h0000_0000_83C0_8253;

    // Privilege level derived from Status register
    // Kernel mode when KSU=0 OR EXL=1 OR ERL=1
    wire kernel_mode = (cp0_status[4:3] == 2'b00) || cp0_status[1] || cp0_status[2];
    // CP0 accessible when in kernel mode OR CU0 bit set
    wire cp0_usable = kernel_mode || cp0_status[28];
    // CP1 (FPU) usable when CU1 bit set — forward from in-flight MTC0 to Status
    wire inflight_mtc0_status_ex = ex_valid && (ex_is_mtc0 || ex_is_dmtc0) && (ex_rd_idx == `CP0_STATUS);
    wire inflight_mtc0_status_df = df_valid && (df_is_mtc0 || df_is_dmtc0) && (df_rd_idx == `CP0_STATUS);
    wire inflight_mtc0_status_ds = ds_valid && (ds_is_mtc0 || ds_is_dmtc0) && (ds_rd_idx == `CP0_STATUS);
    wire inflight_mtc0_status_tc = tc_valid && (tc_is_mtc0 || tc_is_dmtc0) && (tc_rd_idx == `CP0_STATUS);
    wire inflight_mtc0_status_wb = wb_valid && (wb_is_mtc0 || wb_is_dmtc0) && (wb_rd_idx == `CP0_STATUS);
    wire inflight_mtc0_status_any = inflight_mtc0_status_ex ||
                                    inflight_mtc0_status_df ||
                                    inflight_mtc0_status_ds ||
                                    inflight_mtc0_status_tc ||
                                    inflight_mtc0_status_wb;
    // FR/CU1 are interlocked below for CP1 users; use the committed Status
    // bits here so in-flight MTC0 Status decode does not sit on every FPU path.
`ifdef FPGA_NO_FPU
    wire cp1_fr = 1'b0;
    wire cp1_usable = 1'b0;
`else
    wire cp1_fr = cp0_status[26];
    wire cp1_usable = cp0_status[29];
`endif

    // ---------------------------------------------------------------
    // FPU (CP1) — instantiation
    // ---------------------------------------------------------------
    reg         fpu_wen;
    reg         fpu_fcsr_wen;
    reg  [31:0] fpu_fcsr_wdata;
    wire [ 4:0] fpu_fs = rf_insn[15:11];   // fs field = rd position in COP1
    wire [ 4:0] fpu_ft = rf_rt_idx;         // ft field = rt position
    wire [ 4:0] fpu_fd = rf_insn[10:6];     // fd field = sa position in COP1
    wire [63:0] fpu_fs_val, fpu_ft_val, fpu_fr_val;
    reg  [ 4:0] fpu_fd_wr;
    reg  [63:0] fpu_fpr_wdata;
    reg         fpu_fpr_wdata_word;
    wire [31:0] fpu_fcsr_rdata;
    wire        fpu_fcc0;
    wire [7:0]  fpu_fcc_bits;
    wire [31:0] fpu_fir_rdata;
    wire [63:0] fpu_fp_result;
    wire        fpu_fp_exc;
    wire        fpu_fp_exc_unimpl;
    wire [4:0]  fpu_fp_exc_flags;
    wire        fpu_busy;
    wire        fpu_result_valid;

    // FPU operation control — set during RF decode
    reg         rf_do_fpop;
    reg  [ 4:0] rf_fpu_fmt;
    reg  [ 4:0] rf_fpu_fd;
    reg         rf_fpu_wr;           // Write result to FPR
    reg         rf_fpu_wr_word;      // Word-sized FPR write
    reg         rf_fpu_wr_gpr;       // Write GPR (MFC1/DMFC1/CFC1)
    reg         rf_fpu_cmp;          // Comparison (writes FCSR FCC)
    reg  [ 2:0] rf_fpu_cmp_cc;       // CC field for C.cond (bits [10:8])
    reg  [ 2:0] rf_madd_op;          // MADD op: 0=none, 1=MADD, 2=MSUB
    reg         rf_fpu_mtc1;         // MTC1/DMTC1 (GPR → FPR)
    reg         rf_fpu_ctc1;         // CTC1 (GPR → FCSR)
    reg         rf_fpu_lwc1;         // LWC1
    reg         rf_fpu_swc1;         // SWC1
    reg         rf_fpu_ldc1;         // LDC1
    reg         rf_fpu_sdc1;         // SDC1
    reg         rf_fpu_result_acked;
    reg  [63:0] rf_fpu_result_ack_data;
    reg  [ 4:0] rf_fpu_exc_flags_ack;
    reg         rf_fpu_exc_trap_ack;
    reg         rf_fpu_exc_unimpl_ack;

    // FPU exception detection: if cause & enable != 0, or unimplemented op, raise EXC_FPE
    wire rf_fpu_exc_trap = rf_fpu_result_op &&
                           (|(fpu_fp_exc_flags & fpu_fcsr_rdata[11:7]) ||
                            fpu_fp_exc_unimpl);

    // Early FPR/FCSR writes at RF→EX transition (non-load FPU ops).
    // FPGA timing: FPR writes commit at WB. The old speculative RF-stage FPR
    // write put forwarded Status.FR and older-exception decode on a 32x64 FPR
    // write-enable/data path. WB commit writes are simpler and are protected by
    // the FPR dependency stall below.
    wire rf_fpu_early_wr = 1'b0;
    wire rf_fpu_early_flags_wr = 1'b0;
    wire rf_fpu_early_fcsr_wr = 1'b0;
    wire [4:0] rf_fpu_early_fd = rf_fpu_mtc1 ? fpu_fs : fpu_fd;
    wire [63:0] rf_fpu_early_data = rf_fpu_mtc1 ? rf_rt_val : fpu_fp_result;
    wire rf_fpu_early_word = rf_fpu_mtc1 && (rf_rs_idx == `COP1_MTC1);
    // Map cc index to FCSR bit position for FCC writes
    // FCC0 = bit 23, FCC1 = bit 25, FCC2 = bit 26, ..., FCC7 = bit 31
    function [31:0] fcsr_set_fcc;
        input [31:0] fcsr_in;
        input [2:0]  cc;
        input        val;
        reg [31:0] result;
        begin
            result = fcsr_in;
            case (cc)
                3'd0: result[23] = val;
                3'd1: result[25] = val;
                3'd2: result[26] = val;
                3'd3: result[27] = val;
                3'd4: result[28] = val;
                3'd5: result[29] = val;
                3'd6: result[30] = val;
                3'd7: result[31] = val;
            endcase
            fcsr_set_fcc = result;
        end
    endfunction

`ifdef FPGA_NO_FPU
    assign fpu_fs_val = 64'd0;
    assign fpu_ft_val = 64'd0;
    assign fpu_fr_val = 64'd0;
    assign fpu_fcsr_rdata = 32'd0;
    assign fpu_fcc0 = 1'b0;
    assign fpu_fcc_bits = 8'd0;
    assign fpu_fir_rdata = 32'd0;
    assign fpu_fp_result = 64'd0;
    assign fpu_fp_exc = 1'b0;
    assign fpu_fp_exc_unimpl = 1'b0;
    assign fpu_fp_exc_flags = 5'd0;
    assign fpu_busy = 1'b0;
    assign fpu_result_valid = 1'b1;
`else
    r4000_fpu u_fpu (
        .clk           (clk),
        .rst_n         (rst_n),
        .wen           (fpu_wen),
        .fcsr_wen      (fpu_fcsr_wen),
        .fs            (fpu_fs),
        .ft            (fpu_ft),
        .fd            (fpu_fd),
        .fr            (rf_rs_idx),   // fr field = bits[25:21] (same as rs)
        .fs_val        (fpu_fs_val),
        .ft_val        (fpu_ft_val),
        .fr_val        (fpu_fr_val),
        .gpr_wdata     (rf_rt_val),
        .fd_wr         (fpu_fd_wr),
        .fpr_wdata     (fpu_fpr_wdata),
        .fpr_wdata_word(fpu_fpr_wdata_word),
        .fcsr_wdata    (fpu_fcsr_wdata),
        .fcsr_rdata    (fpu_fcsr_rdata),
        .fcc0          (fpu_fcc0),
        .fcc_bits_out  (fpu_fcc_bits),
        .fir_rdata     (fpu_fir_rdata),
        .fmt           (rf_fpu_fmt),
        .funct         (rf_funct),
        .do_fpop       (rf_fpu_start && rf_do_fpop),
        .gpr_rt        (rf_rt_idx),
        .gpr_rt_zero   (rf_rt_val == 64'd0),
        .madd_op       (rf_fpu_start ? rf_madd_op : 3'd0),
        .fp_result     (fpu_fp_result),
        .fp_exc        (fpu_fp_exc),
        .fp_exc_unimpl (fpu_fp_exc_unimpl),
        .fp_exc_flags  (fpu_fp_exc_flags),
        // Early write port (RF stage)
        .wen_early           (rf_fpu_early_wr),
        .fd_wr_early         (rf_fpu_early_fd),
        .fpr_wdata_early     (rf_fpu_early_data),
        .fpr_wdata_early_word(rf_fpu_early_word),
        .flags_wen_early     (rf_fpu_early_flags_wr),
        .fcsr_wen_early      (rf_fpu_early_fcsr_wr),
        .fcsr_wdata_early    (32'd0),
        .exc_trap_early      (1'b0),
        .fr_mode             (cp1_fr),
        .fr_mode_wr          (wb_fpu_fr_mode),
        .fpu_busy            (fpu_busy),
        .fpu_result_valid    (fpu_result_valid),
        .fpu_cancel          (flush_all)
    );
`endif

    // TLB storage. Cosim uses these in-place arrays + the procedural-for-loop
    // lookup paths below — Verilator-friendly and exhaustively verified by
    // cheritest/testfloat. Synthesis (Quartus 18.1 SE Lite) chokes on those
    // 48-deep priority for-loops, so under `SYNTHESIS the storage moves into
    // r4000_tlb.v (instantiated below) and the lookup paths are swapped with
    // its parallel-match-vec + ternary-priority outputs.
`ifndef SYNTHESIS
    reg [63:0] tlb_hi   [0:47];
    reg [63:0] tlb_lo0  [0:47];
    reg [63:0] tlb_lo1  [0:47];
    reg [63:0] tlb_mask [0:47];
`endif

    wire [5:0] tlb_wired_idx =
        (cp0_wired[5:0] > 6'd47) ? 6'd47 : cp0_wired[5:0];
    wire [5:0] tlb_random_idx =
        ((cp0_random[5:0] > 6'd47) || (cp0_random[5:0] < tlb_wired_idx)) ?
        6'd47 : cp0_random[5:0];
    wire [5:0] tlb_random_next =
        (tlb_random_idx <= tlb_wired_idx) ? 6'd47 : (tlb_random_idx - 6'd1);

    wire        tlbw_commit_en =
        wb_valid && !wb_take_exception && !wb_do_eret &&
        wb_is_cop0_co && !wb_exc_mcheck &&
        (wb_cop0_funct == `C0FN_TLBWI || wb_cop0_funct == `C0FN_TLBWR);
    wire        tlbw_commit_random = (wb_cop0_funct == `C0FN_TLBWR);
    wire [5:0]  tlbw_commit_idx = tlbw_commit_random ? tlb_random_idx
                                                      : cp0_index[5:0];
    wire [63:0] tlbw_commit_hi = cp0_entryhi;
    wire [63:0] tlbw_commit_lo0 = {34'd0, cp0_entrylo0[29:1], cp0_entrylo0[0] & cp0_entrylo1[0]};
    wire [63:0] tlbw_commit_lo1 = {34'd0, cp0_entrylo1[29:1], cp0_entrylo0[0] & cp0_entrylo1[0]};
    wire [63:0] tlbw_commit_mask = cp0_pagemask;

    reg        debug_tlbwr_valid_q;
    reg        debug_tlbwr_random_q;
    reg [5:0]  debug_tlbwr_idx_q;
    reg [5:0]  debug_tlbwr_random_idx_q;
    reg [5:0]  debug_tlbwr_wired_q;
    reg [31:0] debug_tlbwr_hi_q;
    reg [31:0] debug_tlbwr_lo0_q;
    reg [31:0] debug_tlbwr_lo1_q;
    reg [31:0] debug_tlbwr_mask_q;

`ifndef SYNTHESIS
    // Deferred TLB write registers (works around simulator optimization
    // that drops array writes with variable index in deep nesting)
    reg        tlbw_pending;
    reg [5:0]  tlbw_idx;
    reg [63:0] tlbw_hi, tlbw_lo0, tlbw_lo1, tlbw_mask;
`endif

`ifdef SYNTHESIS
    // r4000_tlb instance (storage + lookup + writes + reset). Drives the
    // SYNTHESIS path's lookup wires below.
    wire        itlb_match_raw, itlb_v_raw;
    wire [31:0] itlb_paddr_raw;
    wire [63:0] itlb_lo_sel_raw;
    wire        dtlb_match_raw, dtlb_v_raw, dtlb_d_raw;
    wire [31:0] dtlb_paddr_raw;
    wire [63:0] dtlb_lo_sel_raw;
    wire [63:0] dtlb_mask_raw;
    wire        tlb_probe_match;
    wire [5:0]  tlb_probe_idx;
    wire [63:0] tlb_read_hi, tlb_read_lo0, tlb_read_lo1, tlb_read_mask;
    wire [63:0] tlb_debug_hi, tlb_debug_lo0, tlb_debug_lo1, tlb_debug_mask;
    wire [5:0]  dtlb_match_idx_raw;

    r4000_tlb u_tlb (
        .clk        (clk),
        .rst_n      (rst_n),
        .asid       (cp0_entryhi[7:0]),
        .i_vaddr    (pc),
        .i_match    (itlb_match_raw),
        .i_v        (itlb_v_raw),
        .i_paddr    (itlb_paddr_raw),
        .i_lo_sel   (itlb_lo_sel_raw),
        .d_vaddr    (df_eff_addr),
        .d_match    (dtlb_match_raw),
        .d_v        (dtlb_v_raw),
        .d_d        (dtlb_d_raw),
        .d_paddr    (dtlb_paddr_raw),
        .d_lo_sel   (dtlb_lo_sel_raw),
        .d_mask     (dtlb_mask_raw),
        .d_match_idx(dtlb_match_idx_raw),
        .probe_vaddr(cp0_entryhi),
        .probe_match(tlb_probe_match),
        .probe_idx  (tlb_probe_idx),
        .read_idx   (cp0_index[5:0]),
        .read_hi    (tlb_read_hi),
        .read_lo0   (tlb_read_lo0),
        .read_lo1   (tlb_read_lo1),
        .read_mask  (tlb_read_mask),
        .debug_read_idx(debug_tlb_idx),
        .debug_read_hi (tlb_debug_hi),
        .debug_read_lo0(tlb_debug_lo0),
        .debug_read_lo1(tlb_debug_lo1),
        .debug_read_mask(tlb_debug_mask),
        .write_en   (tlbw_commit_en),
        .write_idx  (tlbw_commit_idx),
        .write_hi   (tlbw_commit_hi),
        .write_lo0  (tlbw_commit_lo0),
        .write_lo1  (tlbw_commit_lo1),
        .write_mask (tlbw_commit_mask)
    );
`endif

    // ---------------------------------------------------------------
    // TLB lookup functions
    // ---------------------------------------------------------------
    function [2:0] addr_segment;
        input [63:0] vaddr;
        begin
            if (vaddr[63:62] == 2'b11) begin
                case (vaddr[31:29])
                    3'b100: addr_segment = 3'd0; // kseg0
                    3'b101: addr_segment = 3'd1; // kseg1
                    3'b110: addr_segment = 3'd2; // ksseg
                    3'b111: addr_segment = 3'd3; // kseg3
                    default: addr_segment = 3'd4; // kuseg
                endcase
            end else if (vaddr[63:62] == 2'b10) begin
                addr_segment = 3'd5; // xkphys
            end else begin
                addr_segment = 3'd4; // xkuseg/xksseg/xkseg
            end
        end
    endfunction

    function erl_xuseg_unmapped;
        input [63:0] vaddr;
        begin
            // R4000 error level maps xuseg/kuseg as an uncached physical
            // identity window so cache-error handlers can use low scratch RAM.
            erl_xuseg_unmapped = cp0_status[2] && (vaddr[63:62] == 2'b00);
        end
    endfunction

    function is_mapped;
        input [63:0] vaddr;
        begin
            is_mapped = !erl_xuseg_unmapped(vaddr) &&
                        (addr_segment(vaddr) >= 3'd2) && (addr_segment(vaddr) != 3'd5);
        end
    endfunction

    // Address validity check for 64-bit MIPS
    // Returns 1 if the virtual address is invalid for the current mode
    // xkuseg/xksseg/xkseg: bits [63:40] must be sign-extension of bit 39
    // xkphys: bits [58:36] must be 0 (36-bit physical address space)
    // compat32: bits [63:32] must be sign-extension of bit 31
    function addr_error_64;
        input [63:0] vaddr;
        begin
            case (vaddr[63:62])
                2'b00: addr_error_64 = |vaddr[61:40]; // xkuseg: bits 61:40 must be 0
                2'b01: addr_error_64 = |vaddr[61:40]; // xksseg: bits 61:40 must be 0
                2'b10: addr_error_64 = |vaddr[58:36]; // xkphys: phys addr bits 58:36 must be 0
                2'b11: begin
                    if (&vaddr[61:40])
                        addr_error_64 = 1'b0;  // compat32 or xkseg: all ones OK
                    else if (|vaddr[61:40])
                        addr_error_64 = 1'b1;  // mixed bits = invalid
                    else
                        addr_error_64 = 1'b0;  // xkseg with all zeros OK
                end
            endcase
        end
    endfunction

    function addr_privilege_error;
        input [63:0] vaddr;
        input [63:0] status;
        begin
            addr_privilege_error =
                !status[2] && !status[1] && (status[4:3] == 2'b10) &&
                (addr_segment(vaddr) != 3'd4);
        end
    endfunction

    function [31:0] unmapped_paddr;
        input [63:0] vaddr;
        begin
            if (erl_xuseg_unmapped(vaddr))
                unmapped_paddr = vaddr[31:0];
            else if (addr_segment(vaddr) == 3'd5)
                unmapped_paddr = vaddr[31:0];
            else
                unmapped_paddr = {3'b000, vaddr[28:0]};
        end
    endfunction

    function odd_page_sel;
        input [63:0] vaddr;
        input [63:0] pmask;
        begin
            if      (pmask[24]) odd_page_sel = vaddr[24];
            else if (pmask[22]) odd_page_sel = vaddr[22];
            else if (pmask[20]) odd_page_sel = vaddr[20];
            else if (pmask[18]) odd_page_sel = vaddr[18];
            else if (pmask[16]) odd_page_sel = vaddr[16];
            else if (pmask[14]) odd_page_sel = vaddr[14];
            else                odd_page_sel = vaddr[12];
        end
    endfunction

    function [31:0] tlb_phys_addr;
        input [63:0] vaddr;
        input [63:0] entry_lo;
        input [63:0] pmask;
        reg [31:0] pfn_shifted, offset_mask;
        begin
            pfn_shifted = {entry_lo[25:6], 12'b0};
            offset_mask = {8'b0, pmask[24:13], 12'hFFF};
            tlb_phys_addr = (pfn_shifted & ~offset_mask) | (vaddr[31:0] & offset_mask);
        end
    endfunction

    // ===================================================================
    // STAGE 1: IF — Instruction Fetch (ITLB lookup)
    // ===================================================================

    // Instruction TLB lookup
`ifdef SYNTHESIS
    // Synthesis path: u_tlb's parallel-match-vec drives the lookup. xkphys/
    // kseg0/kseg1 bypass with unmapped_paddr() (always-hit, always-valid).
    wire        tlb_match_i  = is_mapped(pc) ? itlb_match_raw  : 1'b1;
    wire        tlb_v_i      = is_mapped(pc) ? itlb_v_raw      : 1'b1;
    wire [31:0] tlb_paddr_i  = is_mapped(pc) ? itlb_paddr_raw  : unmapped_paddr(pc);
    wire [63:0] tlb_lo_sel_i = is_mapped(pc) ? itlb_lo_sel_raw : 64'd0;
`else
    // Cosim path: procedural for-loop over the in-place tlb_* arrays.
    reg        tlb_match_i;
    reg        tlb_v_i;
    reg [31:0] tlb_paddr_i;
    reg [63:0] tlb_lo_sel_i;
    integer tlb_k;
    always @(*) begin
        tlb_k       = 0;
        tlb_match_i = 1'b0;
        tlb_v_i     = 1'b0;
        tlb_paddr_i = 32'd0;
        tlb_lo_sel_i = 64'd0;
        if (is_mapped(pc)) begin
            for (tlb_k = 0; tlb_k < 48; tlb_k = tlb_k + 1) begin
                if (((pc[39:13] ^ tlb_hi[tlb_k][39:13]) & ~tlb_mask[tlb_k][39:13]) == 27'd0 &&
                    pc[63:62] == tlb_hi[tlb_k][63:62] &&
                    ((tlb_lo0[tlb_k][0] & tlb_lo1[tlb_k][0]) ||
                     (tlb_hi[tlb_k][7:0] == cp0_entryhi[7:0]))) begin
                    tlb_match_i = 1'b1;
                    if (odd_page_sel(pc, tlb_mask[tlb_k]))
                        tlb_lo_sel_i = tlb_lo1[tlb_k];
                    else
                        tlb_lo_sel_i = tlb_lo0[tlb_k];
                    tlb_v_i = tlb_lo_sel_i[1];
                    tlb_paddr_i = tlb_phys_addr(pc, tlb_lo_sel_i, tlb_mask[tlb_k]);
                end
            end
        end else begin
            tlb_match_i = 1'b1;
            tlb_v_i     = 1'b1;
            tlb_paddr_i = unmapped_paddr(pc);
        end
    end
`endif

    wire if_exc_iadel = |pc[1:0] || addr_error_64(pc) ||
                         addr_privilege_error(pc, cp0_status);
    wire if_itlb_miss    = !if_exc_iadel && is_mapped(pc) && !tlb_match_i;
    wire if_itlb_invalid = !if_exc_iadel && is_mapped(pc) && tlb_match_i && !tlb_v_i;
    wire if_exc_itlb     = if_itlb_miss || if_itlb_invalid;
    // Cacheability: kseg1 is always uncached. TLB C=2 is uncached.
    wire if_uncached = erl_xuseg_unmapped(pc) ||
                       (addr_segment(pc) == 3'd1) ||
                       (is_mapped(pc) && tlb_match_i && tlb_lo_sel_i[5:3] == 3'd2);

    // Watch instruction fetch exception
    // Suppress for 1 cycle after ERET: the return address may still be in
    // the same 8-byte watch block as the faulting instruction, and the
    // R4000 spec says watch exceptions are inhibited while EXL is set.
    // ERET clears EXL atomically with the PC redirect, so without this
    // suppression the very first fetch after ERET can re-trigger the watch.
    reg just_did_eret;
    wire if_watch_insn_match = (pc[63:3] == cp0_watchlo[63:3]);
    wire if_exc_watch_insn = cp0_watchlo[2] && if_watch_insn_match &&
                             !cp0_status[1] && !cp0_status[2] &&
                             !just_did_eret;

    assign imem_fetch_addr = {32'd0, tlb_paddr_i};


    // ===================================================================
    // IF/IS Pipeline Register
    // ===================================================================
    reg        is_valid;
    reg [63:0] is_pc;
    reg [31:0] is_paddr;
    reg        is_uncached;
    reg        is_in_delay_slot;
    reg        is_exc_iadel;
    reg        is_exc_itlb;
    reg        is_itlb_miss;
    reg        is_exc_watch_insn;

`ifdef FPGA_SYNC_IMEM
    reg        imem_q_valid;
    reg [63:0] imem_q_pc;
    reg [31:0] imem_q_paddr;
    reg        imem_q_uncached;
    reg        imem_q_in_delay_slot;
    reg        imem_q_exc_iadel;
    reg        imem_q_exc_itlb;
    reg        imem_q_itlb_miss;
    reg        imem_q_exc_watch_insn;
`endif

    // ===================================================================
    // STAGE 2: IS — Instruction Select (read instruction memory)
    // ===================================================================
`ifndef FPGA_SYNC_IMEM
    assign imem_addr = {32'd0, is_paddr};
    // A faulting fetch has no usable physical address or instruction data.
    // In particular, the TLB intentionally returns paddr zero on a miss; do
    // not turn that sentinel into a speculative cache fill while the fault
    // is still advancing to WB.
    assign imem_valid = is_valid && !is_exc_iadel && !is_exc_itlb;
    assign imem_uncached = is_uncached;
`endif
    wire [31:0] is_insn = (is_exc_iadel || is_exc_itlb) ? 32'd0 : imem_rdata;

    // ===================================================================
    // IS/RF Pipeline Register
    // ===================================================================
    reg        rf_valid;
    reg [63:0] rf_pc;
    reg [31:0] rf_insn;
    reg        rf_in_delay_slot;
    reg        rf_exc_iadel;
    reg        rf_exc_itlb;
    reg        rf_itlb_miss;
    reg        rf_exc_watch_insn;

    // ===================================================================
    // STAGE 3: RF — Register File Read + Decode
    // ===================================================================
    wire [5:0]  rf_opcode  = rf_insn[31:26];
    wire [4:0]  rf_rs_idx  = rf_insn[25:21];
    wire [4:0]  rf_rt_idx  = rf_insn[20:16];
    wire [4:0]  rf_rd_idx  = rf_insn[15:11];
    wire [4:0]  rf_sa      = rf_insn[10:6];
    wire [5:0]  rf_funct   = rf_insn[5:0];
    wire [15:0] rf_imm16   = rf_insn[15:0];
    wire [25:0] rf_jtarget = rf_insn[25:0];
    wire [63:0] rf_se_imm  = {{48{rf_imm16[15]}}, rf_imm16};
    wire [63:0] rf_ze_imm  = {48'd0, rf_imm16};

    // Register file read (forwarding is added below, except WB which interlocks)
    wire [63:0] rf_rs_raw = (rf_rs_idx == 5'd0) ? 64'd0 : gpr[rf_rs_idx];
    wire [63:0] rf_rt_raw = (rf_rt_idx == 5'd0) ? 64'd0 : gpr[rf_rt_idx];

    // Forwarded values (computed below after all stage outputs defined)
    wire [63:0] rf_rs_val;
    wire [63:0] rf_rt_val;

    // CP0 register read (architectural value)
    // Selector field for Config register bank (bits [2:0] of instruction)
    wire [2:0] rf_cp0_sel = rf_insn[2:0];

    // Config1: reports cache/TLB configuration for software discovery
    // MMUSize=47 (48 entries), IS=3 (128 sets), IL=3 (16B line), IA=0 (DM),
    // DS=3 (128 sets), DL=3 (16B line), DA=0 (DM), WR=1 (watch regs)
`ifdef FPGA_NO_FPU
    localparam [31:0] CP0_CONFIG1_VAL = 32'h5AD8_6C09; // FP=0 for integer-only FPGA bring-up
`else
    localparam [31:0] CP0_CONFIG1_VAL = 32'h5ED8_6C09; // FP=1 (FPU present)
`endif

    reg [63:0] rf_cp0_rdata_arch;
    always @(*) begin
        case (rf_rd_idx)
            `CP0_INDEX:    rf_cp0_rdata_arch = cp0_index;
            `CP0_RANDOM:   rf_cp0_rdata_arch = cp0_random;
            `CP0_ENTRYLO0: rf_cp0_rdata_arch = cp0_entrylo0;
            `CP0_ENTRYLO1: rf_cp0_rdata_arch = cp0_entrylo1;
            `CP0_CONTEXT:  rf_cp0_rdata_arch = cp0_context;
            `CP0_PAGEMASK: rf_cp0_rdata_arch = cp0_pagemask;
            `CP0_WIRED:    rf_cp0_rdata_arch = cp0_wired;
            `CP0_BADVADDR: rf_cp0_rdata_arch = cp0_badvaddr;
            `CP0_COUNT:    rf_cp0_rdata_arch = cp0_count;
            `CP0_ENTRYHI:  rf_cp0_rdata_arch = cp0_entryhi;
            `CP0_COMPARE:  rf_cp0_rdata_arch = cp0_compare;
            `CP0_STATUS:   rf_cp0_rdata_arch = cp0_status;
            `CP0_CAUSE:    rf_cp0_rdata_arch = cp0_cause;
            `CP0_EPC:      rf_cp0_rdata_arch = cp0_epc;
            `CP0_PRID:     rf_cp0_rdata_arch = cp0_prid;
            `CP0_CONFIG: begin
                case (rf_cp0_sel)
                    3'd0:    rf_cp0_rdata_arch = cp0_config;
                    3'd1:    rf_cp0_rdata_arch = {32'd0, CP0_CONFIG1_VAL};
                    default: rf_cp0_rdata_arch = 64'd0;
                endcase
            end
            `CP0_LLADDR:   rf_cp0_rdata_arch = cp0_lladdr;
            `CP0_WATCHLO:  rf_cp0_rdata_arch = cp0_watchlo;
            `CP0_WATCHHI:  rf_cp0_rdata_arch = cp0_watchhi;
            `CP0_XCONTEXT: rf_cp0_rdata_arch = cp0_xcontext;
            `CP0_ECC:      rf_cp0_rdata_arch = cp0_ecc;
            `CP0_CACHEERR: rf_cp0_rdata_arch = cp0_cacheerr;
            `CP0_TAGLO:    rf_cp0_rdata_arch = cp0_taglo;
            `CP0_TAGHI:    rf_cp0_rdata_arch = cp0_taghi;
            `CP0_ERROREPC: rf_cp0_rdata_arch = cp0_errorepc;
            default:       rf_cp0_rdata_arch = 64'd0;
        endcase
    end

    // CP0 forwarding: declared as wire, defined after all stage regs exist
    wire [63:0] rf_cp0_rdata;

    // Decode logic — produces control signals for EX stage
    reg [ 3:0] rf_alu_op;
    reg [63:0] rf_alu_a, rf_alu_b;
    reg        rf_alu_word;
    reg        rf_check_overflow;
    reg [ 4:0] rf_wb_reg;
    reg [ 2:0] rf_wb_sel;
    reg        rf_do_load, rf_do_store;
    reg [ 2:0] rf_mem_size;
    reg        rf_mem_unsigned;
    reg        rf_do_branch, rf_branch_taken, rf_branch_likely;
    reg        rf_do_jump;
    reg [63:0] rf_jump_target;
    reg        rf_do_link;
    reg [ 3:0] rf_md_op;
    reg        rf_md_start;
    reg        rf_wr_hi, rf_wr_lo;
    reg [63:0] rf_hi_wdata, rf_lo_wdata;
    reg        rf_exc_syscall, rf_exc_break, rf_exc_trap, rf_exc_ri, rf_exc_cpu;
    reg [1:0]  rf_exc_cpu_ce;
    reg        rf_do_eret;
    reg        rf_is_mtc0, rf_is_dmtc0;
    reg        rf_is_sync;     // CC-5: sync instruction — stall RF until pipe drains
    reg        rf_is_cop0_co;
    reg [5:0]  rf_cop0_funct;
    reg        rf_is_cache;
    reg [ 2:0] rf_cache_op;
    reg [ 1:0] rf_cache_type;

    // Does RF stage read rs? rt? (for hazard detection)
    reg rf_uses_rs, rf_uses_rt;

    wire [63:0] rf_branch_offset = {{46{rf_imm16[15]}}, rf_imm16, 2'b00};
    wire [63:0] rf_branch_target = rf_pc + 64'd4 + rf_branch_offset;
    wire [63:0] rf_link_addr     = rf_pc + 64'd8;
    wire [63:0] rf_rs_ctrl_val;
    wire [63:0] rf_rt_ctrl_val;


    always @(*) begin
        rf_alu_op         = `ALU_ADD;
        rf_alu_a          = rf_rs_val;
        rf_alu_b          = rf_rt_val;
        rf_alu_word       = 1'b0;
        rf_check_overflow = 1'b0;
        rf_wb_reg         = 5'd0;
        rf_wb_sel         = `WB_ALU;
        rf_do_load        = 1'b0;
        rf_do_store       = 1'b0;
        rf_mem_size       = `MEM_WORD;
        rf_mem_unsigned   = 1'b0;
        rf_do_branch      = 1'b0;
        rf_branch_taken   = 1'b0;
        rf_branch_likely  = 1'b0;
        rf_do_jump        = 1'b0;
        rf_jump_target    = 64'd0;
        rf_do_link        = 1'b0;
        rf_md_op          = 4'd0;
        rf_md_start       = 1'b0;
        rf_wr_hi          = 1'b0;
        rf_wr_lo          = 1'b0;
        rf_hi_wdata       = 64'd0;
        rf_lo_wdata       = 64'd0;
        rf_exc_syscall    = 1'b0;
        rf_exc_break      = 1'b0;
        rf_exc_trap       = 1'b0;
        rf_exc_ri         = 1'b0;
        rf_exc_cpu        = 1'b0;
        rf_exc_cpu_ce     = 2'd0;
        rf_do_eret        = 1'b0;
        rf_is_mtc0        = 1'b0;
        rf_is_dmtc0       = 1'b0;
        rf_is_sync        = 1'b0;
        rf_is_cop0_co     = 1'b0;
        rf_cop0_funct     = 6'd0;
        rf_is_cache       = 1'b0;
        rf_cache_op       = 3'd0;
        rf_cache_type     = 2'd0;
        rf_uses_rs        = 1'b0;
        rf_uses_rt        = 1'b0;
        rf_do_fpop        = 1'b0;
        rf_fpu_fmt        = 5'd0;
        rf_fpu_fd         = fpu_fd;
        rf_fpu_wr         = 1'b0;
        rf_fpu_wr_word    = 1'b0;
        rf_fpu_wr_gpr     = 1'b0;
        rf_fpu_cmp        = 1'b0;
        rf_fpu_cmp_cc     = 3'b0;
        rf_fpu_mtc1       = 1'b0;
        rf_fpu_ctc1       = 1'b0;
        rf_fpu_lwc1       = 1'b0;
        rf_fpu_swc1       = 1'b0;
        rf_fpu_ldc1       = 1'b0;
        rf_fpu_sdc1       = 1'b0;
        rf_madd_op        = 3'd0;

        case (rf_opcode)
            `OP_SPECIAL: begin
                case (rf_funct)
                    `FN_SLL: begin
                        rf_alu_op = `ALU_SLL; rf_alu_a = {59'd0, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_alu_word = 1'b1; rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_SRL: begin
                        rf_alu_op = `ALU_SRL; rf_alu_a = {59'd0, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_alu_word = 1'b1; rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_SRA: begin
                        rf_alu_op = `ALU_SRA; rf_alu_a = {59'd0, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_alu_word = 1'b1; rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_SLLV: begin
                        rf_alu_op = `ALU_SLL; rf_alu_word = 1'b1; rf_wb_reg = rf_rd_idx;
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_SRLV: begin
                        rf_alu_op = `ALU_SRL; rf_alu_word = 1'b1; rf_wb_reg = rf_rd_idx;
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_SRAV: begin
                        rf_alu_op = `ALU_SRA; rf_alu_word = 1'b1; rf_wb_reg = rf_rd_idx;
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_DSLL: begin
                        rf_alu_op = `ALU_SLL; rf_alu_a = {59'd0, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_DSRL: begin
                        rf_alu_op = `ALU_SRL; rf_alu_a = {59'd0, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_DSRA: begin
                        rf_alu_op = `ALU_SRA; rf_alu_a = {59'd0, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_DSLL32: begin
                        rf_alu_op = `ALU_SLL; rf_alu_a = {58'd0, 1'b1, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_DSRL32: begin
                        rf_alu_op = `ALU_SRL; rf_alu_a = {58'd0, 1'b1, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_DSRA32: begin
                        rf_alu_op = `ALU_SRA; rf_alu_a = {58'd0, 1'b1, rf_sa}; rf_alu_b = rf_rt_val;
                        rf_wb_reg = rf_rd_idx; rf_uses_rt = 1'b1;
                    end
                    `FN_DSLLV: begin
                        rf_alu_op = `ALU_SLL; rf_wb_reg = rf_rd_idx;
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_DSRLV: begin
                        rf_alu_op = `ALU_SRL; rf_wb_reg = rf_rd_idx;
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_DSRAV: begin
                        rf_alu_op = `ALU_SRA; rf_wb_reg = rf_rd_idx;
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_ADD: begin
                        rf_alu_op = `ALU_ADD; rf_alu_word = 1'b1; rf_check_overflow = 1'b1;
                        rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_ADDU: begin
                        rf_alu_op = `ALU_ADD; rf_alu_word = 1'b1;
                        rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_SUB: begin
                        rf_alu_op = `ALU_SUB; rf_alu_word = 1'b1; rf_check_overflow = 1'b1;
                        rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_SUBU: begin
                        rf_alu_op = `ALU_SUB; rf_alu_word = 1'b1;
                        rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_DADD: begin
                        rf_alu_op = `ALU_ADD; rf_check_overflow = 1'b1;
                        rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_DADDU: begin
                        rf_alu_op = `ALU_ADD;
                        rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_DSUB: begin
                        rf_alu_op = `ALU_SUB; rf_check_overflow = 1'b1;
                        rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_DSUBU: begin
                        rf_alu_op = `ALU_SUB;
                        rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_AND: begin rf_alu_op = `ALU_AND; rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_OR:  begin rf_alu_op = `ALU_OR;  rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_XOR: begin rf_alu_op = `ALU_XOR; rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_NOR: begin rf_alu_op = `ALU_NOR; rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_SLT:  begin rf_alu_op = `ALU_SLT;  rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_SLTU: begin rf_alu_op = `ALU_SLTU; rf_wb_reg = rf_rd_idx; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_JR: begin
                        rf_do_jump = 1'b1; rf_jump_target = rf_rs_ctrl_val; rf_uses_rs = 1'b1;
                    end
                    `FN_JALR: begin
                        rf_do_jump = 1'b1; rf_jump_target = rf_rs_ctrl_val; rf_do_link = 1'b1;
                        rf_wb_reg = rf_rd_idx; rf_wb_sel = `WB_LINK; rf_uses_rs = 1'b1;
                    end
                    `FN_MFHI: begin rf_wb_reg = rf_rd_idx; rf_wb_sel = `WB_HI; end
                    `FN_MFLO: begin rf_wb_reg = rf_rd_idx; rf_wb_sel = `WB_LO; end
                    `FN_MTHI: begin rf_wr_hi = 1'b1; rf_hi_wdata = rf_rs_val; rf_uses_rs = 1'b1; end
                    `FN_MTLO: begin rf_wr_lo = 1'b1; rf_lo_wdata = rf_rs_val; rf_uses_rs = 1'b1; end
                    `FN_MOVZ: begin
                        if (rf_rt_val == 64'd0) begin
                            rf_wb_reg = rf_rd_idx; rf_wb_sel = `WB_ALU;
                            rf_alu_op = `ALU_PASSB; rf_alu_b = rf_rs_val;
                        end
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_MOVN: begin
                        if (rf_rt_val != 64'd0) begin
                            rf_wb_reg = rf_rd_idx; rf_wb_sel = `WB_ALU;
                            rf_alu_op = `ALU_PASSB; rf_alu_b = rf_rs_val;
                        end
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN_MOVCI: begin
                        // MOVF/MOVT: move GPR[rs] to GPR[rd] if FCC[cc] matches tf
                        // cc = insn[20:18], tf = insn[16]
                        if (!cp1_usable) begin rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd1; end
                        else begin
                            if (fpu_fcc_bits[rf_insn[20:18]] == rf_insn[16]) begin
                                rf_wb_reg = rf_rd_idx; rf_wb_sel = `WB_ALU;
                                rf_alu_op = `ALU_PASSB; rf_alu_b = rf_rs_val;
                            end
                            rf_uses_rs = 1'b1;
                        end
                    end
                    `FN_MULT:   begin rf_md_start = 1'b1; rf_md_op = 4'd0; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_MULTU:  begin rf_md_start = 1'b1; rf_md_op = 4'd1; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_DIV:    begin rf_md_start = 1'b1; rf_md_op = 4'd2; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_DIVU:   begin rf_md_start = 1'b1; rf_md_op = 4'd3; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_DMULT:  begin rf_md_start = 1'b1; rf_md_op = 4'd4; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_DMULTU: begin rf_md_start = 1'b1; rf_md_op = 4'd5; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_DDIV:   begin rf_md_start = 1'b1; rf_md_op = 4'd6; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_DDIVU:  begin rf_md_start = 1'b1; rf_md_op = 4'd7; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
`ifdef FPGA_NO_TRAP_INSNS
                    `FN_TGE, `FN_TGEU, `FN_TLT, `FN_TLTU, `FN_TEQ, `FN_TNE: begin
                        rf_exc_ri = 1'b1;
                    end
`else
                    `FN_TGE:  begin rf_exc_trap = ($signed(rf_rs_val) >= $signed(rf_rt_val)); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_TGEU: begin rf_exc_trap = (rf_rs_val >= rf_rt_val); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_TLT:  begin rf_exc_trap = ($signed(rf_rs_val) < $signed(rf_rt_val)); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_TLTU: begin rf_exc_trap = (rf_rs_val < rf_rt_val); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_TEQ:  begin rf_exc_trap = (rf_rs_val == rf_rt_val); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN_TNE:  begin rf_exc_trap = (rf_rs_val != rf_rt_val); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
`endif
                    `FN_SYSCALL: begin rf_exc_syscall = 1'b1; end
                    `FN_BREAK:   begin rf_exc_break = 1'b1; end
                    `FN_SYNC:    begin rf_is_sync = 1'b1; end
                    default: begin rf_exc_ri = 1'b1; end
                endcase
            end

            `OP_REGIMM: begin
                rf_uses_rs = 1'b1;
                case (rf_rt_idx)
                    `RT_BLTZ:    begin rf_do_branch = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) < 0; end
                    `RT_BGEZ:    begin rf_do_branch = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) >= 0; end
                    `RT_BLTZL:   begin rf_do_branch = 1'b1; rf_branch_likely = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) < 0; end
                    `RT_BGEZL:   begin rf_do_branch = 1'b1; rf_branch_likely = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) >= 0; end
                    `RT_BLTZAL:  begin rf_do_branch = 1'b1; rf_do_link = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) < 0;
                                       rf_wb_reg = 5'd31; rf_wb_sel = `WB_LINK; end
                    `RT_BGEZAL:  begin rf_do_branch = 1'b1; rf_do_link = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) >= 0;
                                       rf_wb_reg = 5'd31; rf_wb_sel = `WB_LINK; end
                    `RT_BLTZALL: begin rf_do_branch = 1'b1; rf_do_link = 1'b1; rf_branch_likely = 1'b1;
                                       rf_branch_taken = $signed(rf_rs_ctrl_val) < 0;
                                       rf_wb_reg = 5'd31; rf_wb_sel = `WB_LINK; end
                    `RT_BGEZALL: begin rf_do_branch = 1'b1; rf_do_link = 1'b1; rf_branch_likely = 1'b1;
                                       rf_branch_taken = $signed(rf_rs_ctrl_val) >= 0;
                                       rf_wb_reg = 5'd31; rf_wb_sel = `WB_LINK; end
`ifdef FPGA_NO_TRAP_INSNS
                    `RT_TGEI, `RT_TGEIU, `RT_TLTI, `RT_TLTIU, `RT_TEQI, `RT_TNEI: begin
                        rf_exc_ri = 1'b1;
                    end
`else
                    `RT_TGEI:  begin rf_exc_trap = ($signed(rf_rs_val) >= $signed(rf_se_imm)); end
                    `RT_TGEIU: begin rf_exc_trap = (rf_rs_val >= rf_se_imm); end
                    `RT_TLTI:  begin rf_exc_trap = ($signed(rf_rs_val) < $signed(rf_se_imm)); end
                    `RT_TLTIU: begin rf_exc_trap = (rf_rs_val < rf_se_imm); end
                    `RT_TEQI:  begin rf_exc_trap = (rf_rs_val == rf_se_imm); end
                    `RT_TNEI:  begin rf_exc_trap = (rf_rs_val != rf_se_imm); end
`endif
                    default: begin rf_exc_ri = 1'b1; end
                endcase
            end

            `OP_J: begin rf_do_jump = 1'b1; rf_jump_target = {rf_pc[63:28], rf_jtarget, 2'b00}; end
            `OP_JAL: begin
                rf_do_jump = 1'b1; rf_jump_target = {rf_pc[63:28], rf_jtarget, 2'b00};
                rf_do_link = 1'b1; rf_wb_reg = 5'd31; rf_wb_sel = `WB_LINK;
            end

            `OP_BEQ:   begin rf_do_branch = 1'b1; rf_branch_taken = (rf_rs_ctrl_val == rf_rt_ctrl_val); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_BNE:   begin rf_do_branch = 1'b1; rf_branch_taken = (rf_rs_ctrl_val != rf_rt_ctrl_val); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_BLEZ:  begin rf_do_branch = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) <= 0; rf_uses_rs = 1'b1; end
            `OP_BGTZ:  begin rf_do_branch = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) > 0; rf_uses_rs = 1'b1; end
            `OP_BEQL:  begin rf_do_branch = 1'b1; rf_branch_likely = 1'b1; rf_branch_taken = (rf_rs_ctrl_val == rf_rt_ctrl_val); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_BNEL:  begin rf_do_branch = 1'b1; rf_branch_likely = 1'b1; rf_branch_taken = (rf_rs_ctrl_val != rf_rt_ctrl_val); rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_BLEZL: begin rf_do_branch = 1'b1; rf_branch_likely = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) <= 0; rf_uses_rs = 1'b1; end
            `OP_BGTZL: begin rf_do_branch = 1'b1; rf_branch_likely = 1'b1; rf_branch_taken = $signed(rf_rs_ctrl_val) > 0; rf_uses_rs = 1'b1; end

            `OP_ADDI:   begin rf_alu_op = `ALU_ADD; rf_alu_b = rf_se_imm; rf_alu_word = 1'b1; rf_check_overflow = 1'b1; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_ADDIU:  begin rf_alu_op = `ALU_ADD; rf_alu_b = rf_se_imm; rf_alu_word = 1'b1; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_DADDI:  begin rf_alu_op = `ALU_ADD; rf_alu_b = rf_se_imm; rf_check_overflow = 1'b1; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_DADDIU: begin rf_alu_op = `ALU_ADD; rf_alu_b = rf_se_imm; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_SLTI:   begin rf_alu_op = `ALU_SLT;  rf_alu_b = rf_se_imm; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_SLTIU:  begin rf_alu_op = `ALU_SLTU; rf_alu_b = rf_se_imm; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_ANDI:   begin rf_alu_op = `ALU_AND; rf_alu_b = rf_ze_imm; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_ORI:    begin rf_alu_op = `ALU_OR;  rf_alu_b = rf_ze_imm; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_XORI:   begin rf_alu_op = `ALU_XOR; rf_alu_b = rf_ze_imm; rf_wb_reg = rf_rt_idx; rf_uses_rs = 1'b1; end
            `OP_LUI: begin
                rf_alu_op = `ALU_PASSB;
                rf_alu_b = {{32{rf_imm16[15]}}, rf_imm16, 16'b0};
                rf_wb_reg = rf_rt_idx;
            end

            `OP_LB:  begin rf_do_load = 1'b1; rf_mem_size = `MEM_BYTE;   rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end
            `OP_LBU: begin rf_do_load = 1'b1; rf_mem_size = `MEM_BYTE;   rf_mem_unsigned = 1'b1; rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end
            `OP_LH:  begin rf_do_load = 1'b1; rf_mem_size = `MEM_HALF;   rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end
            `OP_LHU: begin rf_do_load = 1'b1; rf_mem_size = `MEM_HALF;   rf_mem_unsigned = 1'b1; rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end
            `OP_LW:  begin rf_do_load = 1'b1; rf_mem_size = `MEM_WORD;   rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end
            `OP_LWU: begin rf_do_load = 1'b1; rf_mem_size = `MEM_WORD;   rf_mem_unsigned = 1'b1; rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end
            `OP_LD:  begin rf_do_load = 1'b1; rf_mem_size = `MEM_DWORD;  rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end
            `OP_LWL: begin rf_do_load = 1'b1; rf_mem_size = `MEM_WORDL;  rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_LWR: begin rf_do_load = 1'b1; rf_mem_size = `MEM_WORDR;  rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_LDL: begin rf_do_load = 1'b1; rf_mem_size = `MEM_DWORDL; rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_LDR: begin rf_do_load = 1'b1; rf_mem_size = `MEM_DWORDR; rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_LL:  begin rf_do_load = 1'b1; rf_mem_size = `MEM_WORD;   rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end
            `OP_LLD: begin rf_do_load = 1'b1; rf_mem_size = `MEM_DWORD;  rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_MEM; rf_uses_rs = 1'b1; end

            `OP_SB:  begin rf_do_store = 1'b1; rf_mem_size = `MEM_BYTE;   rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_SH:  begin rf_do_store = 1'b1; rf_mem_size = `MEM_HALF;   rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_SW:  begin rf_do_store = 1'b1; rf_mem_size = `MEM_WORD;   rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_SD:  begin rf_do_store = 1'b1; rf_mem_size = `MEM_DWORD;  rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_SWL: begin rf_do_store = 1'b1; rf_mem_size = `MEM_WORDL;  rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_SWR: begin rf_do_store = 1'b1; rf_mem_size = `MEM_WORDR;  rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_SDL: begin rf_do_store = 1'b1; rf_mem_size = `MEM_DWORDL; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_SDR: begin rf_do_store = 1'b1; rf_mem_size = `MEM_DWORDR; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
            `OP_SC: begin
                rf_do_store = 1'b1; rf_mem_size = `MEM_WORD;
                rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_LLSC;
                rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
            end
            `OP_SCD: begin
                rf_do_store = 1'b1; rf_mem_size = `MEM_DWORD;
                rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_LLSC;
                rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
            end

            `OP_COP0: begin
                if (!cp0_usable) begin
                    rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd0;
                end else if (rf_rs_idx == `COP0_MFC0) begin
                    rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_ALU;
                    rf_alu_op = `ALU_PASSB;
                    rf_alu_b = {{32{rf_cp0_rdata[31]}}, rf_cp0_rdata[31:0]};
                end else if (rf_rs_idx == `COP0_DMFC0) begin
                    rf_wb_reg = rf_rt_idx; rf_wb_sel = `WB_CP0;
                end else if (rf_rs_idx == `COP0_MTC0) begin
                    rf_is_mtc0 = 1'b1; rf_uses_rt = 1'b1;
                end else if (rf_rs_idx == `COP0_DMTC0) begin
                    rf_is_dmtc0 = 1'b1; rf_uses_rt = 1'b1;
                end else if (rf_rs_idx[4]) begin
                    rf_is_cop0_co = 1'b1;
                    rf_cop0_funct = rf_funct;
                    case (rf_funct)
                        `C0FN_ERET: begin rf_do_eret = 1'b1; end
                        default: ;
                    endcase
                end
            end

            `OP_COP1: begin
                if (!cp1_usable) begin
                    rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd1;
                end else begin
                    rf_fpu_fmt = rf_rs_idx;
                    case (rf_rs_idx)
                        `COP1_MFC1: begin
                            // GPR[rt] ← sign_extend(FPR[fs][31:0])
                            rf_wb_reg = rf_rt_idx;
                            rf_wb_sel = `WB_ALU;
                            rf_alu_op = `ALU_PASSB;
                            rf_alu_b = {{32{fpu_fs_val[31]}}, fpu_fs_val[31:0]};
                            rf_fpu_wr_gpr = 1'b1;
                        end
                        `COP1_DMFC1: begin
                            // GPR[rt] ← FPR[fs]
                            rf_wb_reg = rf_rt_idx;
                            rf_wb_sel = `WB_ALU;
                            rf_alu_op = `ALU_PASSB;
                            rf_alu_b = fpu_fs_val;
                            rf_fpu_wr_gpr = 1'b1;
                        end
                        `COP1_CFC1: begin
                            // GPR[rt] ← FCR[fs]
                            rf_wb_reg = rf_rt_idx;
                            rf_wb_sel = `WB_ALU;
                            rf_alu_op = `ALU_PASSB;
                            // CFC1: fs=0 → FIR, fs=25 → FCCR, fs=26 → FEXR,
                            //       fs=28 → FENR, fs=31 → FCSR
                            case (fpu_fs)
                                5'd0:  rf_alu_b = {32'd0, fpu_fir_rdata};
                                5'd25: rf_alu_b = {32'd0, 24'd0, fpu_fcsr_rdata[31:25], fpu_fcsr_rdata[23]};
                                5'd26: begin
                                    // FEXR: bits[17:12]=Cause, bits[6:2]=Flags
                                    rf_alu_b = 64'd0;
                                    rf_alu_b[17:12] = fpu_fcsr_rdata[17:12];
                                    rf_alu_b[6:2] = fpu_fcsr_rdata[6:2];
                                end
                                5'd28: begin
                                    // FENR: bits[11:7]=Enables, bit[2]=FS, bits[1:0]=RM
                                    rf_alu_b = 64'd0;
                                    rf_alu_b[11:7] = fpu_fcsr_rdata[11:7];
                                    rf_alu_b[2] = fpu_fcsr_rdata[24];
                                    rf_alu_b[1:0] = fpu_fcsr_rdata[1:0];
                                end
                                5'd31: rf_alu_b = {{32{fpu_fcsr_rdata[31]}}, fpu_fcsr_rdata};
                                default: rf_alu_b = 64'd0;
                            endcase
                            rf_fpu_wr_gpr = 1'b1;
                        end
                        `COP1_MTC1: begin
                            // FPR[fs] ← GPR[rt][31:0] (word)
                            rf_fpu_mtc1 = 1'b1;
                            rf_uses_rt = 1'b1;
                        end
                        `COP1_DMTC1: begin
                            // FPR[fs] ← GPR[rt] (doubleword)
                            rf_fpu_mtc1 = 1'b1;
                            rf_uses_rt = 1'b1;
                        end
                        `COP1_CTC1: begin
                            // FCR[fs] ← GPR[rt]
                            rf_fpu_ctc1 = 1'b1;
                            rf_uses_rt = 1'b1;
                        end
                        `COP1_BC1: begin
                            // BC1F/BC1T/BC1FL/BC1TL with cc field
                            // rt[4:2] = cc, rt[1] = nd (likely), rt[0] = tf
                            rf_do_branch = 1'b1;
                            rf_branch_likely = rf_insn[17]; // nd bit
                            rf_branch_taken = fpu_fcc_bits[rf_insn[20:18]] == rf_insn[16]; // tf bit
                        end
                        `COP1_FMT_S, `COP1_FMT_D, `COP1_FMT_W, `COP1_FMT_L: begin
                            rf_do_fpop = 1'b1;
                            if (rf_funct >= 6'h30) begin
                                // C.cond — comparison, writes FCC[cc]
                                rf_fpu_cmp = 1'b1;
                                rf_fpu_cmp_cc = rf_insn[10:8];
                            end else if (rf_funct == `FPU_MOVZ) begin
                                // MOVZ: false condition leaves fd unchanged, so suppress the FPR write.
                                rf_fpu_wr = (rf_rt_val == 64'd0);
                                rf_uses_rt = 1'b1;
                            end else if (rf_funct == `FPU_MOVN) begin
                                // MOVN: false condition leaves fd unchanged, so suppress the FPR write.
                                rf_fpu_wr = (rf_rt_val != 64'd0);
                                rf_uses_rt = 1'b1;
                            end else if (rf_funct == `FPU_MOVCF) begin
                                // MOVF/MOVT: false condition leaves fd unchanged.
                                rf_fpu_wr = (fpu_fcc_bits[fpu_ft[4:2]] == fpu_ft[0]);
                            end else begin
                                // Arithmetic/conversion → writes FPR[fd]
                                rf_fpu_wr = 1'b1;
                            end
                        end
                        default: rf_exc_ri = 1'b1;
                    endcase
                end
            end

            `OP_LWC1: begin
                if (!cp1_usable) begin rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd1; end
                else begin
                    rf_do_load = 1'b1; rf_mem_size = `MEM_WORD;
                    rf_fpu_lwc1 = 1'b1; rf_uses_rs = 1'b1;
                    // Use existing load path, result goes to FPR[rt] at WB
                end
            end
            `OP_SWC1: begin
                if (!cp1_usable) begin rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd1; end
                else begin
                    rf_do_store = 1'b1; rf_mem_size = `MEM_WORD;
                    rf_fpu_swc1 = 1'b1; rf_uses_rs = 1'b1;
                    // Store FPR[ft][31:0] — handled at store formatting
                end
            end
            `OP_LDC1: begin
                if (!cp1_usable) begin rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd1; end
                else begin
                    rf_do_load = 1'b1; rf_mem_size = `MEM_DWORD;
                    rf_fpu_ldc1 = 1'b1; rf_uses_rs = 1'b1;
                end
            end
            `OP_SDC1: begin
                if (!cp1_usable) begin rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd1; end
                else begin
                    rf_do_store = 1'b1; rf_mem_size = `MEM_DWORD;
                    rf_fpu_sdc1 = 1'b1; rf_uses_rs = 1'b1;
                end
            end

            `OP_SPECIAL2: begin
                rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                case (rf_funct)
                    `FN2_MUL: begin
                        rf_md_start = 1'b1; rf_md_op = 4'd10;
                        rf_wb_reg = rf_rd_idx; rf_wb_sel = `WB_MDLO;
                        rf_uses_rs = 1'b1; rf_uses_rt = 1'b1;
                    end
                    `FN2_MADD:  begin rf_md_start = 1'b1; rf_md_op = 4'd8;  rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN2_MADDU: begin rf_md_start = 1'b1; rf_md_op = 4'd9;  rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN2_MSUB:  begin rf_md_start = 1'b1; rf_md_op = 4'd11; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    `FN2_MSUBU: begin rf_md_start = 1'b1; rf_md_op = 4'd12; rf_uses_rs = 1'b1; rf_uses_rt = 1'b1; end
                    default: begin rf_exc_ri = 1'b1; end
                endcase
            end

            `OP_COP1X: begin
                // COP1X: MADD/MSUB/LWXC1/SWXC1/LDXC1/SDXC1/PREFX
                if (!cp1_usable) begin rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd1; end
                else begin
                    case (rf_funct)
                        6'h20: begin rf_madd_op = 3'd1; rf_fpu_wr = 1'b1; end  // MADD.S
                        6'h21: begin rf_madd_op = 3'd1; rf_fpu_wr = 1'b1; end  // MADD.D
                        6'h28: begin rf_madd_op = 3'd2; rf_fpu_wr = 1'b1; end  // MSUB.S
                        6'h29: begin rf_madd_op = 3'd2; rf_fpu_wr = 1'b1; end  // MSUB.D
                        default: rf_exc_ri = 1'b1;
                    endcase
                end
            end

            `OP_CACHE: begin
                // CACHE requires CP0 privilege (kernel mode or CU0)
                if (!cp0_usable) begin rf_exc_cpu = 1'b1; rf_exc_cpu_ce = 2'd0; end
                // Compute effective address: base + sign_ext(offset)
                rf_alu_op     = `ALU_ADD;
                rf_alu_a      = rf_rs_val;
                rf_alu_b      = rf_se_imm;
                rf_alu_word   = 1'b0;
                rf_uses_rs    = 1'b1;
                rf_is_cache   = 1'b1;
                rf_cache_op   = rf_insn[20:18]; // operation code
                rf_cache_type = rf_insn[17:16]; // 0=I$, 1=D$, 2/3=S$
            end

            default: begin rf_exc_ri = 1'b1; end
        endcase
    end

    // ===================================================================
    // RF/EX Pipeline Register
    // ===================================================================
    reg        ex_valid;
    reg [63:0] ex_pc;
    reg [31:0] ex_insn;
    reg        ex_in_delay_slot;
    reg [ 3:0] ex_alu_op;
    reg [63:0] ex_alu_a, ex_alu_b;
    reg        ex_alu_word;
    reg        ex_check_overflow;
    reg [ 4:0] ex_wb_reg;
    reg [ 2:0] ex_wb_sel;
    reg        ex_do_load, ex_do_store;
    reg [ 2:0] ex_mem_size;
    reg        ex_mem_unsigned;
    reg [63:0] ex_rs_val, ex_rt_val;
    reg [ 4:0] ex_rs_idx, ex_rt_idx;
    reg [ 4:0] ex_rd_idx;
    reg [5:0]  ex_opcode;
    reg [63:0] ex_se_imm;
    reg        ex_md_start;
    reg [ 3:0] ex_md_op;
    reg        ex_wr_hi, ex_wr_lo;
    reg [63:0] ex_hi_wdata, ex_lo_wdata;
    reg        ex_exc_iadel, ex_exc_itlb;
    reg        ex_itlb_miss;
    reg        ex_exc_watch_insn;
    reg        ex_exc_syscall, ex_exc_break, ex_exc_trap, ex_exc_ri, ex_exc_cpu, ex_exc_fpe;
    reg [1:0]  ex_exc_cpu_ce;
    reg        ex_do_eret;
    reg        ex_is_mtc0, ex_is_dmtc0;
    reg        ex_is_cop0_co;
    reg [5:0]  ex_cop0_funct;
    reg        ex_is_cache;
    reg [ 2:0] ex_cache_op;
    reg [ 1:0] ex_cache_type;
    reg [63:0] ex_link_addr;
    reg [63:0] ex_cp0_rdata;
    // FPU pipeline signals
    reg        ex_fpu_wr;
    reg        ex_fpu_wr_word;
    reg [ 4:0] ex_fpu_fd;
    reg [63:0] ex_fpu_result;
    reg [ 4:0] ex_fpu_exc_flags;
    reg        ex_fpu_exc_unimpl;
    reg        ex_fpu_cmp;
    reg        ex_fpu_cmp_result;
    reg [ 2:0] ex_fpu_cmp_cc;
    reg        ex_fpu_mtc1;
    reg        ex_fpu_ctc1;
    reg [31:0] ex_fpu_ctc1_data;
    reg        ex_fpu_early_wr_done;  // Early FPR write fired at RF
    reg        ex_fpu_early_fcsr_done; // Early FCSR write fired at RF
    reg        ex_fpu_lwc1, ex_fpu_swc1;
    reg        ex_fpu_ldc1, ex_fpu_sdc1;
    reg        ex_fpu_fr_mode;

    // ===================================================================
    // STAGE 4: EX — Execute (ALU, multiply/divide start, effective address)
    // ===================================================================

    // ALU
    wire [63:0] ex_alu_result;
    wire        ex_alu_overflow;

    r4000_alu u_alu (
        .a        (ex_alu_a),
        .b        (ex_alu_b),
        .op       (ex_alu_op),
        .is_word  (ex_alu_word),
        .result   (ex_alu_result),
        .overflow (ex_alu_overflow)
    );

    // HI/LO forwarding: effective HI/LO considering pending pipeline MTHI/MTLO
    // With multi-cycle muldiv, MFHI/MFLO stalls until md_done fires, so hi_reg/lo_reg
    // are always up to date when MFHI/MFLO proceeds. Only check pipeline for MTHI/MTLO.
    wire [63:0] effective_hi, effective_lo;

    // Multiply/divide unit — connected to EX stage
    wire [63:0] md_hi, md_lo;
    wire        md_done, md_busy;
    wire        ex_has_early_exc = ex_exc_iadel || ex_exc_itlb || ex_exc_syscall ||
                                   ex_exc_break || ex_exc_ri || ex_exc_cpu ||
                                   ex_exc_watch_insn || ex_exc_trap || ex_exc_fpe;
    // All multiply/divide ops (including MUL) go through the muldiv unit
    wire        ex_md_is_mul = (ex_md_op == 4'd10);
    // Start muldiv only on the first cycle EX sees this instruction (not while stalled or on done)
    wire        md_start_wire = ex_valid && !ex_has_early_exc && ex_md_start &&
                                !md_busy && !md_done && !flush_all;

    // Cancel speculative muldiv: if a muldiv-starting instruction is still in the
    // pipeline (hasn't committed at WB) when flush_all fires, cancel the muldiv.
    // Include EX as well: MUL can remain there while an older instruction
    // reaches WB and takes an exception.  A squashed operation must never
    // update architectural HI/LO after the handler starts executing.
    wire md_in_pipeline = (ex_valid && ex_md_start) ||
                          (df_valid && df_md_start) ||
                          (ds_valid && ds_md_start) ||
                          (tc_valid && tc_md_start);
    wire md_cancel = flush_all && md_busy && md_in_pipeline;

    r4000_muldiv u_muldiv (
        .clk   (clk),
        .rst_n (rst_n),
        .a     (ex_rs_val),
        .b     (ex_rt_val),
        .hi_in (effective_hi),
        .lo_in (effective_lo),
        .op    (ex_md_op),
        .start (md_start_wire),
        .cancel(md_cancel),
        .hi    (md_hi),
        .lo    (md_lo),
        .done  (md_done),
        .busy  (md_busy)
    );

    // Effective address for loads/stores
    wire [63:0] ex_eff_addr = ex_rs_val + ex_se_imm;
    wire [63:0] ex_watchlo_cmp =
        (tc_valid && (tc_is_mtc0 || tc_is_dmtc0) && tc_rd_idx == `CP0_WATCHLO) ?
        tc_rt_val : cp0_watchlo;
    wire ex_watch_data_match = (ex_eff_addr[63:3] == ex_watchlo_cmp[63:3]);

    // EX exception detection (overflow, trap)
    wire ex_exc_overflow = ex_check_overflow && ex_alu_overflow;

    // MUL stalls in EX until muldiv completes (no inline multiply shortcut)
    // Must also stall on the cycle md_start_wire fires (md_busy goes high next cycle)
    wire stall_ex = ex_valid && ex_md_is_mul && ex_md_start && (md_busy || md_start_wire) && !md_done;

    // Data for forwarding from EX
    reg [63:0] ex_fwd_data;
    always @(*) begin
        case (ex_wb_sel)
            `WB_ALU:  ex_fwd_data = ex_alu_result;
            `WB_LINK: ex_fwd_data = ex_link_addr;
            `WB_CP0:  ex_fwd_data = ex_cp0_rdata;
            `WB_HI:   ex_fwd_data = effective_hi;
            `WB_LO:   ex_fwd_data = effective_lo;
            `WB_MDLO: ex_fwd_data = {{32{md_lo[31]}}, md_lo[31:0]};  // from muldiv unit on md_done
            default:  ex_fwd_data = ex_alu_result;
        endcase
    end
    wire ex_fwd_valid = ex_valid && !ex_has_early_exc && !ex_exc_overflow && (ex_wb_reg != 5'd0);
    // Can forward from EX if data is available (not load, not LLSC, not MUL still computing)
    wire ex_fwd_ready = ex_fwd_valid && (ex_wb_sel != `WB_MEM) && (ex_wb_sel != `WB_LLSC) &&
                        !(ex_wb_sel == `WB_MDLO && !md_done);

    // ===================================================================
    // EX/DF Pipeline Register
    // ===================================================================
    reg        df_valid;
    reg [63:0] df_pc;
    reg [31:0] df_insn;
    reg        df_in_delay_slot;
    reg [63:0] df_alu_result;
    reg [ 4:0] df_wb_reg;
    reg [ 2:0] df_wb_sel;
    reg        df_do_load, df_do_store;
    reg [ 2:0] df_mem_size;
    reg        df_mem_unsigned;
    reg [63:0] df_rs_val, df_rt_val;
    reg [63:0] df_eff_addr;
    reg [5:0]  df_opcode;
    reg [63:0] df_se_imm;
    reg        df_md_start;
    reg        df_wr_hi, df_wr_lo;
    reg [63:0] df_hi_wdata, df_lo_wdata;
    reg [63:0] df_link_addr;
    reg [63:0] df_cp0_rdata;
    reg [ 4:0] df_rd_idx;
    reg        df_exc_iadel, df_exc_itlb;
    reg        df_itlb_miss;
    reg        df_exc_watch_insn;
    reg        df_watch_data_match;
    reg        df_exc_syscall, df_exc_break, df_exc_trap, df_exc_ri, df_exc_cpu, df_exc_fpe;
    reg [1:0]  df_exc_cpu_ce;
    reg        df_exc_overflow;
    reg        df_do_eret;
    reg        df_is_mtc0, df_is_dmtc0;
    reg        df_is_cop0_co;
    reg [5:0]  df_cop0_funct;
    reg        df_is_cache;
    reg [ 2:0] df_cache_op;
    reg [ 1:0] df_cache_type;
    reg        df_fpu_wr, df_fpu_wr_word, df_fpu_cmp, df_fpu_cmp_result;
    reg [ 2:0] df_fpu_cmp_cc;
    reg [ 4:0] df_fpu_fd;
    reg [63:0] df_fpu_result;
    reg [ 4:0] df_fpu_exc_flags;
    reg        df_fpu_exc_unimpl;
    reg        df_fpu_mtc1, df_fpu_ctc1;
    reg [31:0] df_fpu_ctc1_data;
    reg        df_fpu_early_wr_done, df_fpu_early_fcsr_done;
    reg        df_fpu_lwc1, df_fpu_swc1, df_fpu_ldc1, df_fpu_sdc1;
    reg        df_fpu_fr_mode;

    // ===================================================================
    // STAGE 5: DF — Data Fetch (DTLB lookup, address errors, store formatting)
    // ===================================================================

    // Data TLB lookup
    wire [2:0]  df_addr_segment = addr_segment(df_eff_addr);
    wire        df_erl_xuseg_unmapped =
        cp0_status[2] && (df_eff_addr[63:62] == 2'b00);
    wire        df_is_mapped =
        !df_erl_xuseg_unmapped &&
        (df_addr_segment >= 3'd2) && (df_addr_segment != 3'd5);
    wire [31:0] df_unmapped_paddr =
        (df_erl_xuseg_unmapped || (df_addr_segment == 3'd5)) ?
        df_eff_addr[31:0] : {3'b000, df_eff_addr[28:0]};

`ifdef SYNTHESIS
    wire        tlb_match_d  = df_is_mapped ? dtlb_match_raw  : 1'b1;
    wire        tlb_v_d      = df_is_mapped ? dtlb_v_raw      : 1'b1;
    wire        tlb_d_d      = df_is_mapped ? dtlb_d_raw      : 1'b1;
    wire [31:0] tlb_paddr_d  = df_is_mapped ? dtlb_paddr_raw  : df_unmapped_paddr;
    wire [63:0] tlb_lo_sel_d = df_is_mapped ? dtlb_lo_sel_raw : 64'd0;
    wire [63:0] tlb_mask_d   = df_is_mapped ? dtlb_mask_raw   : 64'd0;
    wire [5:0]  tlb_match_d_idx = (df_is_mapped && dtlb_match_raw) ?
                                  dtlb_match_idx_raw : 6'd63;
`else
    reg        tlb_match_d;
    reg        tlb_v_d, tlb_d_d;
    reg [31:0] tlb_paddr_d;
    reg [63:0] tlb_lo_sel_d;
    reg [63:0] tlb_mask_d;
    reg [5:0]  tlb_match_d_idx;
    integer tlb_j;
    always @(*) begin
        tlb_j        = 0;
        tlb_match_d  = 1'b0;
        tlb_v_d      = 1'b0;
        tlb_d_d      = 1'b0;
        tlb_paddr_d  = 32'd0;
        tlb_lo_sel_d = 64'd0;
        tlb_mask_d   = 64'd0;
        tlb_match_d_idx = 6'd63;
        if (is_mapped(df_eff_addr)) begin
            for (tlb_j = 0; tlb_j < 48; tlb_j = tlb_j + 1) begin
                if (((df_eff_addr[39:13] ^ tlb_hi[tlb_j][39:13]) & ~tlb_mask[tlb_j][39:13]) == 27'd0 &&
                    df_eff_addr[63:62] == tlb_hi[tlb_j][63:62] &&
                    ((tlb_lo0[tlb_j][0] & tlb_lo1[tlb_j][0]) ||
                     (tlb_hi[tlb_j][7:0] == cp0_entryhi[7:0]))) begin
                    tlb_match_d = 1'b1;
                    if (odd_page_sel(df_eff_addr, tlb_mask[tlb_j]))
                        tlb_lo_sel_d = tlb_lo1[tlb_j];
                    else
                        tlb_lo_sel_d = tlb_lo0[tlb_j];
                    tlb_v_d = tlb_lo_sel_d[1];
                    tlb_d_d = tlb_lo_sel_d[2];
                    tlb_paddr_d = tlb_phys_addr(df_eff_addr, tlb_lo_sel_d, tlb_mask[tlb_j]);
                    tlb_mask_d = tlb_mask[tlb_j];
                    tlb_match_d_idx = tlb_j[5:0];
                end
            end
        end else begin
            tlb_match_d = 1'b1;
            tlb_v_d     = 1'b1;
            tlb_d_d     = 1'b1;
            tlb_paddr_d = unmapped_paddr(df_eff_addr);
        end
    end
`endif

    // DF exception detection
    wire df_has_early_exc = df_exc_iadel || df_exc_itlb || df_exc_syscall ||
                            df_exc_break || df_exc_ri || df_exc_cpu ||
                            df_exc_overflow || df_exc_trap || df_exc_fpe;

    wire df_align_half  = (df_opcode == `OP_LH || df_opcode == `OP_LHU || df_opcode == `OP_SH);
    wire df_align_word  = (df_opcode == `OP_LW || df_opcode == `OP_LWU || df_opcode == `OP_SW ||
                           df_opcode == `OP_LL || df_opcode == `OP_SC ||
                           df_opcode == `OP_LWC1 || df_opcode == `OP_SWC1);
    wire df_align_dword = (df_opcode == `OP_LD || df_opcode == `OP_SD ||
                           df_opcode == `OP_LLD || df_opcode == `OP_SCD ||
                           df_opcode == `OP_LDC1 || df_opcode == `OP_SDC1);
    wire df_misaligned  = (df_align_half && df_eff_addr[0]) ||
                          (df_align_word && |df_eff_addr[1:0]) ||
                          (df_align_dword && |df_eff_addr[2:0]);
    // Address error: alignment OR invalid 64-bit address range
    wire df_bad_addr_range = (df_do_load || df_do_store) &&
                             (addr_error_64(df_eff_addr) ||
                              addr_privilege_error(df_eff_addr, cp0_status));
    wire df_exc_adel = !df_has_early_exc && (df_misaligned || df_bad_addr_range) && df_do_load;
    wire df_exc_ades = !df_has_early_exc && (df_misaligned || df_bad_addr_range) && df_do_store;

    wire df_addr_ok = !df_misaligned && !df_bad_addr_range;
    wire df_mem_access = df_do_load || df_do_store;
    wire df_mapped_mem_access = df_addr_ok && df_mem_access && df_is_mapped;
    wire df_dtlb_check = !df_has_early_exc && df_mapped_mem_access;
    wire df_dtlb_miss    = df_dtlb_check && !tlb_match_d;
    wire df_dtlb_invalid = df_dtlb_check && tlb_match_d && !tlb_v_d;
    wire df_dtlb_mod     = !df_has_early_exc && df_addr_ok && df_do_store && df_is_mapped && tlb_match_d && tlb_v_d && !tlb_d_d;
    wire df_exc_dtlb     = df_dtlb_miss || df_dtlb_invalid || df_dtlb_mod;
    // Uncached: kseg1 is always uncached. For TLB-mapped addresses, use
    // the C field from the TLB entry (C=2 = uncached per R4000 convention).
    wire df_uncached     = df_erl_xuseg_unmapped ||
                           (df_addr_segment == 3'd1) ||
                           (df_is_mapped && tlb_match_d && tlb_lo_sel_d[5:3] == 3'd2);

    wire df_exc_watch_data = !df_has_early_exc && !df_misaligned && !df_exc_dtlb &&
                        ((df_do_load && cp0_watchlo[1] && df_watch_data_match) ||
                         (df_do_store && cp0_watchlo[0] && df_watch_data_match));
    wire df_exc_watch = (df_exc_watch_data || df_exc_watch_insn) &&
                        !cp0_status[1] && !cp0_status[2];

    // Store data positioning (in DF stage)
    wire [2:0] df_byte_offset = df_eff_addr[2:0];
    reg [63:0] df_store_data;
    reg [ 7:0] df_store_mask;

    always @(*) begin
        df_store_data = 64'd0;
        df_store_mask = 8'h00;
        case (df_mem_size)
            `MEM_BYTE: begin
                df_store_mask = 8'h80 >> df_byte_offset;
                df_store_data = {8{df_rt_val[7:0]}};
            end
            `MEM_HALF: begin
                case (df_byte_offset[2:1])
                    2'd0: begin df_store_mask = 8'hC0; df_store_data = {4{df_rt_val[15:0]}}; end
                    2'd1: begin df_store_mask = 8'h30; df_store_data = {4{df_rt_val[15:0]}}; end
                    2'd2: begin df_store_mask = 8'h0C; df_store_data = {4{df_rt_val[15:0]}}; end
                    2'd3: begin df_store_mask = 8'h03; df_store_data = {4{df_rt_val[15:0]}}; end
                endcase
            end
            `MEM_WORD: begin
                if (df_byte_offset[2]) begin df_store_mask = 8'h0F; df_store_data = {32'd0, df_rt_val[31:0]}; end
                else begin df_store_mask = 8'hF0; df_store_data = {df_rt_val[31:0], 32'd0}; end
            end
            `MEM_DWORD: begin df_store_mask = 8'hFF; df_store_data = df_rt_val; end
            `MEM_WORDL: begin
                case (df_byte_offset[1:0])
                    2'd0: if (df_byte_offset[2]) begin df_store_mask = 8'h0F; df_store_data = {32'd0, df_rt_val[31:0]}; end
                          else begin df_store_mask = 8'hF0; df_store_data = {df_rt_val[31:0], 32'd0}; end
                    2'd1: if (df_byte_offset[2]) begin df_store_mask = 8'h07; df_store_data = {32'd0, 8'd0, df_rt_val[31:8]}; end
                          else begin df_store_mask = 8'h70; df_store_data = {8'd0, df_rt_val[31:8], 32'd0}; end
                    2'd2: if (df_byte_offset[2]) begin df_store_mask = 8'h03; df_store_data = {32'd0, 16'd0, df_rt_val[31:16]}; end
                          else begin df_store_mask = 8'h30; df_store_data = {16'd0, df_rt_val[31:16], 32'd0}; end
                    2'd3: if (df_byte_offset[2]) begin df_store_mask = 8'h01; df_store_data = {56'd0, df_rt_val[31:24]}; end
                          else begin df_store_mask = 8'h10; df_store_data = {24'd0, df_rt_val[31:24], 32'd0}; end
                endcase
            end
            `MEM_WORDR: begin
                case (df_byte_offset[1:0])
                    2'd0: if (df_byte_offset[2]) begin df_store_mask = 8'h08; df_store_data = {32'd0, df_rt_val[7:0], 24'd0}; end
                          else begin df_store_mask = 8'h80; df_store_data = {df_rt_val[7:0], 56'd0}; end
                    2'd1: if (df_byte_offset[2]) begin df_store_mask = 8'h0C; df_store_data = {32'd0, df_rt_val[15:0], 16'd0}; end
                          else begin df_store_mask = 8'hC0; df_store_data = {df_rt_val[15:0], 48'd0}; end
                    2'd2: if (df_byte_offset[2]) begin df_store_mask = 8'h0E; df_store_data = {32'd0, df_rt_val[23:0], 8'd0}; end
                          else begin df_store_mask = 8'hE0; df_store_data = {df_rt_val[23:0], 40'd0}; end
                    2'd3: if (df_byte_offset[2]) begin df_store_mask = 8'h0F; df_store_data = {32'd0, df_rt_val[31:0]}; end
                          else begin df_store_mask = 8'hF0; df_store_data = {df_rt_val[31:0], 32'd0}; end
                endcase
            end
            `MEM_DWORDL: begin
                case (df_byte_offset)
                    3'd0: begin df_store_mask = 8'hFF; df_store_data = df_rt_val; end
                    3'd1: begin df_store_mask = 8'h7F; df_store_data = { 8'd0, df_rt_val[63: 8]}; end
                    3'd2: begin df_store_mask = 8'h3F; df_store_data = {16'd0, df_rt_val[63:16]}; end
                    3'd3: begin df_store_mask = 8'h1F; df_store_data = {24'd0, df_rt_val[63:24]}; end
                    3'd4: begin df_store_mask = 8'h0F; df_store_data = {32'd0, df_rt_val[63:32]}; end
                    3'd5: begin df_store_mask = 8'h07; df_store_data = {40'd0, df_rt_val[63:40]}; end
                    3'd6: begin df_store_mask = 8'h03; df_store_data = {48'd0, df_rt_val[63:48]}; end
                    3'd7: begin df_store_mask = 8'h01; df_store_data = {56'd0, df_rt_val[63:56]}; end
                endcase
            end
            `MEM_DWORDR: begin
                case (df_byte_offset)
                    3'd0: begin df_store_mask = 8'h80; df_store_data = {df_rt_val[ 7:0], 56'd0}; end
                    3'd1: begin df_store_mask = 8'hC0; df_store_data = {df_rt_val[15:0], 48'd0}; end
                    3'd2: begin df_store_mask = 8'hE0; df_store_data = {df_rt_val[23:0], 40'd0}; end
                    3'd3: begin df_store_mask = 8'hF0; df_store_data = {df_rt_val[31:0], 32'd0}; end
                    3'd4: begin df_store_mask = 8'hF8; df_store_data = {df_rt_val[39:0], 24'd0}; end
                    3'd5: begin df_store_mask = 8'hFC; df_store_data = {df_rt_val[47:0], 16'd0}; end
                    3'd6: begin df_store_mask = 8'hFE; df_store_data = {df_rt_val[55:0],  8'd0}; end
                    3'd7: begin df_store_mask = 8'hFF; df_store_data = df_rt_val; end
                endcase
            end
        endcase
    end

    // DF forwarding: same as EX for non-load results
    wire df_has_any_exc = df_has_early_exc || df_exc_adel || df_exc_ades || df_exc_dtlb || df_exc_watch;
    // DF forwarding only forwards non-memory results; loads/LLSC are excluded
    // below. Keep late DTLB/data-watch exception logic out of the RF forwarding
    // select path so a data-side translation result does not feed the RF stage.
    wire df_fwd_blocks = df_has_early_exc ||
                         (df_exc_watch_insn && !cp0_status[1] && !cp0_status[2]);
    wire df_fwd_valid = df_valid && !df_fwd_blocks && (df_wb_reg != 5'd0);
    wire df_fwd_ready = df_fwd_valid && (df_wb_sel != `WB_MEM) && (df_wb_sel != `WB_LLSC);
    reg [63:0] df_fwd_data;
    always @(*) begin
        case (df_wb_sel)
            `WB_ALU:  df_fwd_data = df_alu_result;
            `WB_LINK: df_fwd_data = df_link_addr;
            `WB_CP0:  df_fwd_data = df_cp0_rdata;
            `WB_HI:   df_fwd_data = df_alu_result;  // captured at EX→DF
            `WB_LO:   df_fwd_data = df_alu_result;  // captured at EX→DF
            `WB_MDLO: df_fwd_data = df_alu_result;  // MUL result passed through alu_result
            default:  df_fwd_data = df_alu_result;
        endcase
    end

    // ===================================================================
    // DF/DS Pipeline Register
    // ===================================================================
    reg        ds_valid;
    reg [63:0] ds_pc;
    reg [31:0] ds_insn;
    reg        ds_in_delay_slot;
    reg [63:0] ds_alu_result;
    reg [ 4:0] ds_wb_reg;
    reg [ 2:0] ds_wb_sel;
    reg        ds_do_load, ds_do_store;
    reg [ 2:0] ds_mem_size;
    reg        ds_mem_unsigned;
    reg [63:0] ds_rs_val, ds_rt_val;
    reg [63:0] ds_eff_addr;
    reg [5:0]  ds_opcode;
    reg        ds_md_start;
    reg        ds_wr_hi, ds_wr_lo;
    reg [63:0] ds_hi_wdata, ds_lo_wdata;
    reg [63:0] ds_link_addr;
    reg [63:0] ds_cp0_rdata;
    reg [ 4:0] ds_rd_idx;
    reg        ds_exc_iadel, ds_exc_itlb;
    reg        ds_itlb_miss;
    reg        ds_exc_watch_insn;
    reg        ds_exc_syscall, ds_exc_break, ds_exc_trap, ds_exc_ri, ds_exc_cpu, ds_exc_fpe;
    reg [1:0]  ds_exc_cpu_ce;
    reg        ds_exc_overflow;
    reg        ds_do_eret;
    reg        ds_is_mtc0, ds_is_dmtc0;
    reg        ds_is_cop0_co;
    reg [5:0]  ds_cop0_funct;
    reg        ds_is_cache;
    reg [ 2:0] ds_cache_op;
    reg [ 1:0] ds_cache_type;
    reg        ds_fpu_wr, ds_fpu_wr_word, ds_fpu_cmp, ds_fpu_cmp_result;
    reg [ 2:0] ds_fpu_cmp_cc;
    reg [ 4:0] ds_fpu_fd;
    reg [63:0] ds_fpu_result;
    reg [ 4:0] ds_fpu_exc_flags;
    reg        ds_fpu_exc_unimpl;
    reg        ds_fpu_mtc1, ds_fpu_ctc1;
    reg [31:0] ds_fpu_ctc1_data;
    reg        ds_fpu_early_wr_done, ds_fpu_early_fcsr_done;
    reg        ds_fpu_lwc1, ds_fpu_swc1, ds_fpu_ldc1, ds_fpu_sdc1;
    reg        ds_fpu_fr_mode;
    reg [31:0] ds_paddr;
    reg        ds_uncached;
    reg [ 7:0] ds_store_mask;
    reg [63:0] ds_store_data;
    reg        ds_exc_adel, ds_exc_ades;
    reg        ds_exc_dtlb;
    reg        ds_dtlb_miss, ds_dtlb_invalid, ds_dtlb_mod;
    reg        ds_dtlb_match_valid;
    reg [5:0]  ds_dtlb_match_idx;
    reg [63:0] ds_dtlb_lo_sel;
    reg [63:0] ds_dtlb_mask;
    reg        ds_exc_watch;
    reg        ds_exc_watch_data;
    reg        ds_ll_snoop_killed;

    // ===================================================================
    // STAGE 6: DS — Data Select (memory interface, load extraction, SC)
    // ===================================================================
    wire ds_has_early_exc = ds_exc_iadel || ds_exc_itlb || ds_exc_syscall ||
                            ds_exc_break || ds_exc_ri || ds_exc_cpu ||
                            ds_exc_overflow || ds_exc_trap || ds_exc_fpe;
    wire ds_has_any_exc = ds_has_early_exc || ds_exc_adel || ds_exc_ades ||
                          ds_exc_dtlb || ds_exc_watch;

    // Interrupt check in DS stage (to suppress memory access)
    // Older writes to interrupt-visible CP0 state must be visible before a DS
    // memory operation is allowed to issue. Stall briefly instead of forwarding
    // those writes through the D-cache request/stall path.
    wire tc_writes_cause = tc_valid &&
                           (tc_is_mtc0 || tc_is_dmtc0) && (tc_rd_idx == `CP0_CAUSE);
    wire wb_writes_cause = wb_valid &&
                           (wb_is_mtc0 || wb_is_dmtc0) && (wb_rd_idx == `CP0_CAUSE);
    wire wb_writes_compare = wb_valid &&
                              (wb_is_mtc0 || wb_is_dmtc0) && (wb_rd_idx == `CP0_COMPARE);
    wire tc_writes_compare = tc_valid && (tc_is_mtc0 || tc_is_dmtc0) && (tc_rd_idx == `CP0_COMPARE);
    wire tc_writes_status = tc_valid &&
                            (tc_is_mtc0 || tc_is_dmtc0) && (tc_rd_idx == `CP0_STATUS);
    wire wb_writes_status = wb_valid &&
                            (wb_is_mtc0 || wb_is_dmtc0) && (wb_rd_idx == `CP0_STATUS);
    wire ds_int_state_hazard = ds_valid && (ds_do_load || ds_do_store) &&
                               (tc_writes_cause || wb_writes_cause ||
                                tc_writes_compare || wb_writes_compare ||
                                tc_writes_status || wb_writes_status);
`ifndef FPGA_NO_INTERRUPTS
    wire ds_int_enabled = cp0_status[0] && !cp0_status[1] && !cp0_status[2];
    wire [7:0] ds_int_pending = cp0_cause[15:8] & cp0_status[15:8];
    wire ds_exc_int = ds_int_enabled && (ds_int_pending != 8'd0);
`else
    wire ds_exc_int = 1'b0;
`endif

    // Suppress memory operations at DS if DS itself has exceptions,
    // or if an older instruction in TC/WB might take an exception (would flush this store)
    wire tc_might_except = tc_valid && tc_has_any_exc;
    wire tc_might_redirect = tc_valid && tc_do_eret;
    wire ds_suppress_mem = ds_has_any_exc || (ds_valid && ds_exc_int) ||
                           ds_int_state_hazard ||
                           tc_might_except || tc_might_redirect ||
                           wb_take_exception;

    // SC success check — must forward llbit from pending LL in pipeline.
    //
    // CC-1 (per R4000 UM 8.4.1): llbit clears only when the LL'd cache line
    // is invalidated, NOT on any local store. A local store to a different
    // cache line must not fail a subsequent SC. Cache-line size used here
    // is 128 bytes (L2 line), matching the S-cache granularity. A same-line
    // local store is conservatively treated as invalidating the line.
    //
    // Forward ll_addr from pending LL in pipeline (uncommitted LL is tracked
    // at TC/WB; committed LL is in ll_addr reg).
    wire [63:0] effective_ll_addr = (wb_valid && !wb_take_exception && !wb_do_eret &&
                                     (wb_opcode == `OP_LL || wb_opcode == `OP_LLD)) ? wb_eff_addr :
                                    (tc_valid && !tc_has_any_exc &&
                                     (tc_opcode == `OP_LL || tc_opcode == `OP_LLD)) ? tc_eff_addr :
                                    ll_addr;

    // Same-line check: LL line == store line, 128-byte line size.
    wire wb_store_same_line = (wb_eff_addr[63:7] == effective_ll_addr[63:7]);
    wire tc_store_same_line = (tc_eff_addr[63:7] == effective_ll_addr[63:7]);

    wire wb_clears_llbit = (wb_valid && !wb_take_exception && !wb_do_eret && wb_do_store && wb_store_same_line) ||
                           (wb_valid && wb_do_eret);
    wire tc_clears_llbit = tc_valid && !tc_has_any_exc && tc_do_store && tc_store_same_line;
    // Forward llbit from pending LL in TC/WB (not yet committed)
    wire ds_has_ll = ds_valid && !ds_has_any_exc &&
                     (ds_opcode == `OP_LL || ds_opcode == `OP_LLD);
    wire tc_has_ll = tc_valid && !tc_has_any_exc &&
                     (tc_opcode == `OP_LL || tc_opcode == `OP_LLD);
    wire wb_has_ll = wb_valid && !wb_take_exception && !wb_do_eret &&
                     (wb_opcode == `OP_LL || wb_opcode == `OP_LLD);
    wire snoop_hits_ds_ll = snoop_valid && ds_has_ll &&
                            (snoop_addr[31:7] == ds_paddr[31:7]);
    wire snoop_hits_tc_ll = snoop_valid && tc_has_ll &&
                            (snoop_addr[31:7] == tc_paddr[31:7]);
    wire snoop_hits_wb_ll = snoop_valid && wb_has_ll &&
                            (snoop_addr[31:7] == wb_paddr[31:7]);
    wire snoop_hits_committed_ll = snoop_valid && llbit &&
                                   (snoop_addr[31:7] == ll_paddr[31:7]);
    wire tc_sets_llbit = tc_has_ll && !tc_ll_snoop_killed &&
                         !snoop_hits_tc_ll;
    wire wb_sets_llbit = wb_has_ll && !wb_ll_snoop_killed &&
                         !snoop_hits_wb_ll;
    wire effective_llbit = (llbit || tc_sets_llbit || wb_sets_llbit) &&
                           !wb_clears_llbit && !tc_clears_llbit &&
                           !snoop_hits_committed_ll;
    wire ds_sc_success = effective_llbit && (ds_eff_addr == effective_ll_addr);
    wire ds_sc_insn = (ds_opcode == `OP_SC || ds_opcode == `OP_SCD);

    // Access ack: once an uncached access has been accepted by the external
    // device, suppress re-issue while the cache holds its response valid.
    // Cached loads complete when cache_stall drops; uncached loads complete on
    // uncached_accepted because the S-cache may keep cache_stall asserted until
    // the core consumes its held response.
    reg ds_store_acked;
    reg ds_uncached_device_issued;
    reg ds_cached_store_visible;
    reg ds_load_acked;
    reg ds_sc_checked;
    reg ds_sc_success_acked;
    reg [63:0] ds_load_ack_data;
    // Once an uncached load has reached the external device, an interrupt may
    // defer its architectural boundary but must not suppress response capture.
    // Otherwise DS advances with the previous load's registered data while the
    // correctly tagged response is discarded by the cache hierarchy.
    wire ds_load_active = ds_do_load && ds_valid && !flush_all &&
                          (!ds_suppress_mem ||
                           (dmem_uncached && ds_uncached_device_issued)) &&
                          !(ds_is_cache && ds_cache_type[1]);
    wire ds_load_capture_pending = ds_load_active && !ds_load_acked;
    wire ds_sc_check_active = ds_do_store && ds_sc_insn && !ds_suppress_mem &&
                              ds_valid && !flush_all;
    wire ds_sc_check_pending = ds_sc_check_active && !ds_sc_checked;
    wire ds_sc_store_success = ds_sc_checked && ds_sc_success_acked;
    // A pending interrupt suppresses a younger DS memory operation before the
    // architectural WB interrupt boundary.  If an older accepted device
    // access is still deferring that boundary, keep the not-yet-issued memory
    // operation in DS.  It either executes when the level interrupt drops, or
    // advances with a deferred-interrupt token once the older side effect has
    // retired and is replayed after exception entry.
    wire ds_irq_mem_hold = ds_valid && (ds_do_load || ds_do_store) &&
                           !ds_has_any_exc && ds_exc_int &&
                           !ds_uncached_device_issued &&
                           wb_int_device_barrier;
    wire pipeline_mem_stall = ds_int_state_hazard || ds_irq_mem_hold ||
                              cache_stall || ds_load_capture_pending ||
                              ds_sc_check_pending;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ds_store_acked <= 1'b0;
            ds_uncached_device_issued <= 1'b0;
            ds_cached_store_visible <= 1'b0;
            ds_load_acked <= 1'b0;
            ds_sc_checked <= 1'b0;
            ds_sc_success_acked <= 1'b0;
            ds_load_ack_data <= 64'd0;
        end else begin
            if (flush_all) begin
                ds_store_acked <= 1'b0;
                ds_uncached_device_issued <= 1'b0;
                ds_cached_store_visible <= 1'b0;
                ds_load_acked <= 1'b0;
                ds_sc_checked <= 1'b0;
                ds_sc_success_acked <= 1'b0;
                ds_load_ack_data <= 64'd0;
            end else if (!pipeline_mem_stall) begin
                ds_store_acked <= 1'b0;   // pipeline advancing — clear for next insn
                ds_uncached_device_issued <= 1'b0;
                ds_cached_store_visible <= 1'b0;
                ds_load_acked <= 1'b0;
                ds_sc_checked <= 1'b0;
                ds_sc_success_acked <= 1'b0;
            end else if (ds_sc_check_pending) begin
                ds_sc_checked <= 1'b1;
                ds_sc_success_acked <= ds_sc_success;
            end else if (ds_load_capture_pending && dmem_uncached && uncached_accepted) begin
                ds_load_acked <= 1'b1;
                ds_load_ack_data <= dmem_rdata;
            end else if (ds_load_capture_pending && !dmem_uncached && !cache_stall) begin
                ds_load_acked <= 1'b1;
                ds_load_ack_data <= dmem_rdata;
            end else if (uncached_accepted) begin
                if (ds_do_store)
                    ds_store_acked <= 1'b1;   // device accepted — don't re-issue
            end
            // A presented uncached request is not necessarily issued: the
            // S-cache also stalls it during its post-response cooldown.  Only
            // the request-buffer start pulse proves that the operation will
            // survive later interrupt suppression and reach the device.
            if (!flush_all && uncached_started)
                ds_uncached_device_issued <= 1'b1;
            if (!flush_all && pipeline_mem_stall && cached_store_visible)
                ds_cached_store_visible <= 1'b1;
        end
    end

    // Memory interface
    wire ds_store_active = ds_do_store && !ds_suppress_mem && ds_valid &&
                           (!ds_sc_insn || ds_sc_store_success) &&
                           !flush_all;
    // A DS store is a level request, but each architectural store may become
    // visible only once while the pipeline is held.  Uncached stores already
    // use the external acceptance token.  Cached stores have an equivalent
    // D-cache mutation token; honoring it prevents a resident hit from being
    // replayed after a coherence hold admits and completes a newer DMA snoop.
    wire ds_store_present = ds_store_active && !ds_store_acked &&
                            !ds_cached_store_visible;
    always @(*) begin
        // Suppress D-cache access for CACHE instructions targeting S-cache (type 2/3).
        // Without this, the D-cache sees a miss and triggers an S-cache fill that
        // overwrites the CACHE instruction's tag operation (breaks S-cache size probe).
        dmem_ren   = ds_load_capture_pending;
        dmem_wen   = ds_store_present;
        dmem_addr  = {32'd0, ds_paddr};
        dmem_wmask = ds_store_present ? ds_store_mask : 8'h00;
        dmem_wdata = ds_store_data;
    end
    assign dmem_uncached = ds_uncached | force_uncached;
    assign dmem_lookup_addr = {32'd0, tlb_paddr_d};
    assign dmem_lookup_advance = !pipeline_mem_stall;

    // Load data extraction from the registered memory response. Loads hold DS
    // for one cycle after the cache/memory response is available, so TC never
    // captures data through the live external-memory mux.
    wire [2:0] ds_byte_offset = ds_eff_addr[2:0];
    wire [63:0] ds_mem_rdata = ds_load_ack_data;
    reg [63:0] ds_load_data;

    always @(*) begin
        ds_load_data = 64'd0;
        case (ds_mem_size)
            `MEM_BYTE: begin
                case (ds_byte_offset)
                    3'd0: ds_load_data = ds_mem_unsigned ? {56'd0, ds_mem_rdata[63:56]} : {{56{ds_mem_rdata[63]}}, ds_mem_rdata[63:56]};
                    3'd1: ds_load_data = ds_mem_unsigned ? {56'd0, ds_mem_rdata[55:48]} : {{56{ds_mem_rdata[55]}}, ds_mem_rdata[55:48]};
                    3'd2: ds_load_data = ds_mem_unsigned ? {56'd0, ds_mem_rdata[47:40]} : {{56{ds_mem_rdata[47]}}, ds_mem_rdata[47:40]};
                    3'd3: ds_load_data = ds_mem_unsigned ? {56'd0, ds_mem_rdata[39:32]} : {{56{ds_mem_rdata[39]}}, ds_mem_rdata[39:32]};
                    3'd4: ds_load_data = ds_mem_unsigned ? {56'd0, ds_mem_rdata[31:24]} : {{56{ds_mem_rdata[31]}}, ds_mem_rdata[31:24]};
                    3'd5: ds_load_data = ds_mem_unsigned ? {56'd0, ds_mem_rdata[23:16]} : {{56{ds_mem_rdata[23]}}, ds_mem_rdata[23:16]};
                    3'd6: ds_load_data = ds_mem_unsigned ? {56'd0, ds_mem_rdata[15: 8]} : {{56{ds_mem_rdata[15]}}, ds_mem_rdata[15: 8]};
                    3'd7: ds_load_data = ds_mem_unsigned ? {56'd0, ds_mem_rdata[ 7: 0]} : {{56{ds_mem_rdata[ 7]}}, ds_mem_rdata[ 7: 0]};
                endcase
            end
            `MEM_HALF: begin
                case (ds_byte_offset[2:1])
                    2'd0: ds_load_data = ds_mem_unsigned ? {48'd0, ds_mem_rdata[63:48]} : {{48{ds_mem_rdata[63]}}, ds_mem_rdata[63:48]};
                    2'd1: ds_load_data = ds_mem_unsigned ? {48'd0, ds_mem_rdata[47:32]} : {{48{ds_mem_rdata[47]}}, ds_mem_rdata[47:32]};
                    2'd2: ds_load_data = ds_mem_unsigned ? {48'd0, ds_mem_rdata[31:16]} : {{48{ds_mem_rdata[31]}}, ds_mem_rdata[31:16]};
                    2'd3: ds_load_data = ds_mem_unsigned ? {48'd0, ds_mem_rdata[15: 0]} : {{48{ds_mem_rdata[15]}}, ds_mem_rdata[15: 0]};
                endcase
            end
            `MEM_WORD: begin
                if (ds_byte_offset[2])
                    ds_load_data = ds_mem_unsigned ? {32'd0, ds_mem_rdata[31:0]} : {{32{ds_mem_rdata[31]}}, ds_mem_rdata[31:0]};
                else
                    ds_load_data = ds_mem_unsigned ? {32'd0, ds_mem_rdata[63:32]} : {{32{ds_mem_rdata[63]}}, ds_mem_rdata[63:32]};
            end
            `MEM_DWORD: ds_load_data = ds_mem_rdata;
            `MEM_WORDL: begin
                case (ds_byte_offset[1:0])
                    2'd0: ds_load_data = ds_byte_offset[2] ? {{32{ds_mem_rdata[31]}}, ds_mem_rdata[31:0]} : {{32{ds_mem_rdata[63]}}, ds_mem_rdata[63:32]};
                    2'd1: ds_load_data = ds_byte_offset[2] ? {{32{ds_mem_rdata[23]}}, ds_mem_rdata[23:0], ds_rt_val[7:0]} : {{32{ds_mem_rdata[55]}}, ds_mem_rdata[55:32], ds_rt_val[7:0]};
                    2'd2: ds_load_data = ds_byte_offset[2] ? {{32{ds_mem_rdata[15]}}, ds_mem_rdata[15:0], ds_rt_val[15:0]} : {{32{ds_mem_rdata[47]}}, ds_mem_rdata[47:32], ds_rt_val[15:0]};
                    2'd3: ds_load_data = ds_byte_offset[2] ? {{32{ds_mem_rdata[7]}}, ds_mem_rdata[7:0], ds_rt_val[23:0]} : {{32{ds_mem_rdata[39]}}, ds_mem_rdata[39:32], ds_rt_val[23:0]};
                endcase
            end
            `MEM_WORDR: begin
                case (ds_byte_offset[1:0])
                    2'd0: ds_load_data = ds_byte_offset[2] ? {{32{ds_rt_val[31]}}, ds_rt_val[31:8], ds_mem_rdata[31:24]} : {{32{ds_rt_val[31]}}, ds_rt_val[31:8], ds_mem_rdata[63:56]};
                    2'd1: ds_load_data = ds_byte_offset[2] ? {{32{ds_rt_val[31]}}, ds_rt_val[31:16], ds_mem_rdata[31:16]} : {{32{ds_rt_val[31]}}, ds_rt_val[31:16], ds_mem_rdata[63:48]};
                    2'd2: ds_load_data = ds_byte_offset[2] ? {{32{ds_rt_val[31]}}, ds_rt_val[31:24], ds_mem_rdata[31:8]} : {{32{ds_rt_val[31]}}, ds_rt_val[31:24], ds_mem_rdata[63:40]};
                    2'd3: ds_load_data = ds_byte_offset[2] ? {{32{ds_mem_rdata[31]}}, ds_mem_rdata[31:0]} : {{32{ds_mem_rdata[63]}}, ds_mem_rdata[63:32]};
                endcase
            end
            `MEM_DWORDL: begin
                case (ds_byte_offset)
                    3'd0: ds_load_data = ds_mem_rdata;
                    3'd1: ds_load_data = {ds_mem_rdata[55: 0], ds_rt_val[ 7:0]};
                    3'd2: ds_load_data = {ds_mem_rdata[47: 0], ds_rt_val[15:0]};
                    3'd3: ds_load_data = {ds_mem_rdata[39: 0], ds_rt_val[23:0]};
                    3'd4: ds_load_data = {ds_mem_rdata[31: 0], ds_rt_val[31:0]};
                    3'd5: ds_load_data = {ds_mem_rdata[23: 0], ds_rt_val[39:0]};
                    3'd6: ds_load_data = {ds_mem_rdata[15: 0], ds_rt_val[47:0]};
                    3'd7: ds_load_data = {ds_mem_rdata[ 7: 0], ds_rt_val[55:0]};
                endcase
            end
            `MEM_DWORDR: begin
                case (ds_byte_offset)
                    3'd0: ds_load_data = {ds_rt_val[63: 8], ds_mem_rdata[63:56]};
                    3'd1: ds_load_data = {ds_rt_val[63:16], ds_mem_rdata[63:48]};
                    3'd2: ds_load_data = {ds_rt_val[63:24], ds_mem_rdata[63:40]};
                    3'd3: ds_load_data = {ds_rt_val[63:32], ds_mem_rdata[63:32]};
                    3'd4: ds_load_data = {ds_rt_val[63:40], ds_mem_rdata[63:24]};
                    3'd5: ds_load_data = {ds_rt_val[63:48], ds_mem_rdata[63:16]};
                    3'd6: ds_load_data = {ds_rt_val[63:56], ds_mem_rdata[63: 8]};
                    3'd7: ds_load_data = ds_mem_rdata;
                endcase
            end
        endcase
    end

    wire ds_fwd_valid = ds_valid && !ds_has_any_exc && (ds_wb_reg != 5'd0);
    // FPGA timing: DS-stage memory/LLSC data still depends on cache/SC
    // combinational logic. Let dependent users wait one more cycle and forward
    // the registered TC value instead.
    wire ds_fwd_ready = ds_fwd_valid &&
                        (ds_wb_sel != `WB_MEM) &&
                        (ds_wb_sel != `WB_LLSC);
    reg [63:0] ds_fwd_data;
    always @(*) begin
        case (ds_wb_sel)
            `WB_LINK: ds_fwd_data = ds_link_addr;
            `WB_CP0:  ds_fwd_data = ds_cp0_rdata;
            default:  ds_fwd_data = ds_alu_result;
        endcase
    end

    // ===================================================================
    // DS/TC Pipeline Register
    // ===================================================================
    reg        tc_valid;
    reg [63:0] tc_pc;
    reg [31:0] tc_insn;
    reg        tc_in_delay_slot;
    reg [63:0] tc_alu_result;
    reg [ 4:0] tc_wb_reg;
    reg [ 2:0] tc_wb_sel;
    reg        tc_do_load, tc_do_store;
    reg [63:0] tc_rt_val;
    reg [63:0] tc_eff_addr;
    reg [5:0]  tc_opcode;
    reg        tc_md_start;
    reg        tc_wr_hi, tc_wr_lo;
    reg [63:0] tc_hi_wdata, tc_lo_wdata;
    reg [63:0] tc_link_addr;
    reg [63:0] tc_cp0_rdata;
    reg [ 4:0] tc_rd_idx;
    reg        tc_exc_iadel, tc_exc_itlb;
    reg        tc_itlb_miss;
    reg        tc_exc_watch_insn;
    reg        tc_exc_syscall, tc_exc_break, tc_exc_trap, tc_exc_ri, tc_exc_cpu, tc_exc_fpe;
    reg [1:0]  tc_exc_cpu_ce;
    reg        tc_exc_overflow;
    reg        tc_do_eret;
    reg        tc_is_mtc0, tc_is_dmtc0;
    reg        tc_is_cop0_co;
    reg [5:0]  tc_cop0_funct;
    reg        tc_is_cache;
    reg [ 2:0] tc_cache_op;
    reg [ 1:0] tc_cache_type;
    reg [31:0] tc_paddr;
    reg        tc_fpu_wr, tc_fpu_wr_word, tc_fpu_cmp, tc_fpu_cmp_result;
    reg [ 2:0] tc_fpu_cmp_cc;
    reg [ 4:0] tc_fpu_fd;
    reg [63:0] tc_fpu_result;
    reg [ 4:0] tc_fpu_exc_flags;
    reg        tc_fpu_exc_unimpl;
    reg        tc_fpu_mtc1, tc_fpu_ctc1;
    reg [31:0] tc_fpu_ctc1_data;
    reg        tc_fpu_early_wr_done, tc_fpu_early_fcsr_done;
    reg        tc_fpu_lwc1, tc_fpu_ldc1;
    reg        tc_fpu_fr_mode;
    reg        tc_exc_adel, tc_exc_ades;
    reg        tc_exc_dtlb;
    reg        tc_dtlb_miss, tc_dtlb_invalid, tc_dtlb_mod;
    reg        tc_exc_watch;
    reg        tc_exc_watch_data;
    reg [ 7:0] tc_store_mask;
    reg [63:0] tc_store_data;
    reg        tc_device_side_effect;
    reg        tc_cached_store_visible;
    reg        tc_deferred_mem_irq;
    reg        tc_sc_insn;
    reg        tc_sc_success;
    reg [63:0] tc_load_data;
    reg        tc_ll_snoop_killed;

    // ===================================================================
    // STAGE 7: TC — Tag Check (L2 handled by r4000_scache at memory interface)
    // ===================================================================
    wire tc_has_early_exc = tc_exc_iadel || tc_exc_itlb || tc_exc_syscall ||
                            tc_exc_break || tc_exc_ri || tc_exc_cpu ||
                            tc_exc_overflow || tc_exc_trap || tc_exc_fpe;
    wire tc_has_any_exc = tc_has_early_exc || tc_exc_adel || tc_exc_ades ||
                          tc_exc_dtlb || tc_exc_watch || tc_deferred_mem_irq;
    wire tc_fwd_valid = tc_valid && !tc_has_any_exc && (tc_wb_reg != 5'd0);

    // Build the TC forwarding/WB result from TC pipeline registers. This keeps
    // live D-cache data and LL/SC success logic off the DS->TC register input.
    reg [63:0] tc_wb_result_mux;
    always @(*) begin
        case (tc_wb_sel)
            `WB_ALU:  tc_wb_result_mux = tc_alu_result;
            `WB_MEM:  tc_wb_result_mux = tc_load_data;
            `WB_LINK: tc_wb_result_mux = tc_link_addr;
            `WB_CP0: begin
                // Index Load Tag from the FPGA S-cache completes after its
                // synchronous metadata read.  A following MFC0 TagLo may
                // already be in flight, so select the completed/current
                // architectural TagLo at TC instead of the stale RF snapshot.
                if (tc_rd_idx == `CP0_TAGLO)
                    tc_wb_result_mux = cache_cop_taglo_update_valid ?
                        {32'd0, cache_cop_taglo_update_data} : cp0_taglo;
                else
                    tc_wb_result_mux = tc_cp0_rdata;
            end
            `WB_LLSC: tc_wb_result_mux = {63'd0, tc_sc_success};
            `WB_MDLO: tc_wb_result_mux = tc_alu_result;
            default:  tc_wb_result_mux = tc_alu_result;
        endcase
    end
    wire [63:0] tc_wb_result = tc_wb_result_mux;

    // ===================================================================
    // TC/WB Pipeline Register
    // ===================================================================
    reg        wb_valid;
    reg [63:0] wb_pc;
    reg [31:0] wb_insn;
    reg        wb_in_delay_slot;
    reg [ 4:0] wb_wb_reg;
    reg [63:0] wb_wb_result;
    reg        wb_wr_hi, wb_wr_lo;
    reg [63:0] wb_hi_wdata, wb_lo_wdata;
    reg [63:0] wb_rt_val;
    reg [ 4:0] wb_rd_idx;
    reg [5:0]  wb_opcode;
    reg [63:0] wb_eff_addr;
    reg        wb_do_load, wb_do_store;
    reg        wb_sc_insn;
    reg        wb_sc_success;
    reg        wb_device_side_effect;
    reg        wb_cached_store_visible;
    reg        wb_deferred_mem_irq;
    reg [63:0] wb_link_addr;
    reg [63:0] wb_cp0_rdata;
    reg        wb_exc_iadel, wb_exc_itlb;
    reg        wb_itlb_miss;
    reg        wb_exc_watch_insn;
    reg        wb_exc_syscall, wb_exc_break, wb_exc_trap, wb_exc_ri, wb_exc_cpu, wb_exc_fpe;
    reg [1:0]  wb_exc_cpu_ce;
    reg        wb_exc_overflow;
    reg        wb_exc_adel, wb_exc_ades;
    reg        wb_exc_dtlb;
    reg        wb_dtlb_miss, wb_dtlb_invalid, wb_dtlb_mod;
    reg        wb_exc_watch;
    reg        wb_exc_watch_data;
    reg        wb_do_eret;
    reg        wb_is_mtc0, wb_is_dmtc0;
    reg        wb_is_cop0_co;
    reg [5:0]  wb_cop0_funct;
    reg        wb_is_cache;
    reg [ 2:0] wb_cache_op;
    reg [ 1:0] wb_cache_type;
    reg [31:0] wb_paddr;
    reg        wb_fpu_wr, wb_fpu_wr_word, wb_fpu_cmp, wb_fpu_cmp_result;
    reg [ 2:0] wb_fpu_cmp_cc;
    reg [ 4:0] wb_fpu_fd;
    reg [63:0] wb_fpu_result;
    reg [ 4:0] wb_fpu_exc_flags;
    reg        wb_fpu_exc_unimpl;
    reg        wb_fpu_mtc1, wb_fpu_ctc1;
    reg [31:0] wb_fpu_ctc1_data;
    reg        wb_fpu_early_wr_done, wb_fpu_early_fcsr_done;
    reg        wb_fpu_lwc1, wb_fpu_ldc1;
    reg        wb_fpu_fr_mode;
    reg [63:0] wb_load_data;
    reg [ 7:0] wb_store_mask;
    reg [63:0] wb_store_data;
    reg [ 2:0] wb_wb_sel;
    reg        wb_ll_snoop_killed;

    // ===================================================================
    // HI/LO Forwarding — only check pipeline for MTHI/MTLO
    // With multi-cycle muldiv, md_done writes HI/LO directly.
    // MFHI/MFLO stalls while md_busy, so hi_reg/lo_reg are current when MFHI proceeds.
    // ===================================================================
    wire ex_updates_hi = ex_valid && !ex_has_early_exc && ex_wr_hi;
    wire ex_updates_lo = ex_valid && !ex_has_early_exc && ex_wr_lo;
    wire df_updates_hi = df_valid && !df_has_early_exc && df_wr_hi;
    wire df_updates_lo = df_valid && !df_has_early_exc && df_wr_lo;
    wire ds_updates_hi = ds_valid && ds_wr_hi;
    wire ds_updates_lo = ds_valid && ds_wr_lo;
    wire tc_updates_hi = tc_valid && tc_wr_hi;
    wire tc_updates_lo = tc_valid && tc_wr_lo;
    wire wb_updates_hi = wb_valid && !wb_take_exception && !wb_do_eret && wb_wr_hi;
    wire wb_updates_lo = wb_valid && !wb_take_exception && !wb_do_eret && wb_wr_lo;

    assign effective_hi = ex_updates_hi ? ex_hi_wdata :
                          df_updates_hi ? df_hi_wdata :
                          ds_updates_hi ? ds_hi_wdata :
                          tc_updates_hi ? tc_hi_wdata :
                          wb_updates_hi ? wb_hi_wdata :
                          hi_reg;
    assign effective_lo = ex_updates_lo ? ex_lo_wdata :
                          df_updates_lo ? df_lo_wdata :
                          ds_updates_lo ? ds_lo_wdata :
                          tc_updates_lo ? tc_lo_wdata :
                          wb_updates_lo ? wb_lo_wdata :
                          lo_reg;

    // ===================================================================
    // STAGE 8: WB — Writeback + Exception Handling (COMMIT point)
    // ===================================================================

    // Interrupt detection (in WB stage)
    // When MTC0/DMTC0 to Status is at WB, take the most conservative view:
    // an interrupt fires only if BOTH old and new Status would allow it.
    // This prevents lost wakeup (splhi masking is atomic) WITHOUT causing
    // livelock when splx re-enables interrupts (deferred to next insn).
    wire wb_mtc0_sr = inflight_mtc0_status_wb;
`ifndef FPGA_NO_INTERRUPTS
    wire wb_int_ie  = wb_mtc0_sr ? (wb_rt_val[0] & cp0_status[0]) : cp0_status[0];
    wire wb_int_exl = wb_mtc0_sr ? (wb_rt_val[1] | cp0_status[1]) : cp0_status[1];
    wire wb_int_erl = wb_mtc0_sr ? (wb_rt_val[2] | cp0_status[2]) : cp0_status[2];
    wire wb_int_enabled = wb_int_ie && !wb_int_exl && !wb_int_erl;
    wire [7:0] wb_eff_im = wb_mtc0_sr ? (wb_rt_val[15:8] & cp0_status[15:8]) : cp0_status[15:8];
    wire [7:0] wb_int_pending = cp0_cause[15:8] & wb_eff_im;
    // Uncached device accesses become externally visible from DS, before they reach
    // the WB instruction boundary where asynchronous interrupts are
    // recognized. Track only accesses whose device request was actually
    // issued: a pending interrupt suppresses younger DS requests, and treating
    // those suppressed instructions as side effects can starve the interrupt
    // (and let a suppressed access retire without ever executing).
    // A read can itself have a side effect (for example, the WD33C93 indirect
    // data port advances its register pointer).  If an interrupt flushes that
    // accepted read before retirement, ERET replays it and advances the device
    // twice.  The registered issued bit covers the external request/response
    // interval without feeding the WB exception decision combinationally back
    // into the DS request path; TC/WB bits cover the retirement interval.
    wire wb_int_device_barrier =
        ds_uncached_device_issued ||
        (tc_valid && tc_device_side_effect) ||
        (wb_valid && wb_device_side_effect);
    // Cached stores also mutate architecturally visible state in DS.  Carry a
    // distinct token rather than conflating RAM writes with device semantics.
    // This lets the accepted store (especially SC) retire before an async IRQ.
    wire wb_int_cached_store_barrier =
        ds_cached_store_visible ||
        (tc_valid && tc_cached_store_visible) ||
        (wb_valid && wb_cached_store_visible);
    wire wb_exc_int = wb_int_enabled &&
                      ((wb_int_pending != 8'd0) || wb_deferred_mem_irq) &&
                      !wb_int_device_barrier &&
                      !wb_int_cached_store_barrier;
`else
    wire [7:0] wb_int_pending = 8'd0;
    wire wb_exc_int = 1'b0;
`endif

    // Machine Check: invalid PageMask on TLBWI/TLBWR.
    // Valid PageMask[24:13] patterns are pairs of bits contiguous from bit 13.
    function cp0_pagemask_bits_valid;
        input [11:0] bits;
        begin
            cp0_pagemask_bits_valid =
                (bits == 12'h000) || (bits == 12'h003) ||
                (bits == 12'h00F) || (bits == 12'h03F) ||
                (bits == 12'h0FF) || (bits == 12'h3FF) ||
                (bits == 12'hFFF);
        end
    endfunction

    wire wb_exc_mcheck = wb_valid && !wb_take_exception_pre && wb_is_cop0_co &&
                         (wb_cop0_funct == `C0FN_TLBWI || wb_cop0_funct == `C0FN_TLBWR) &&
                         !cp0_pagemask_valid;

    wire wb_take_exception_pre = wb_valid && (
        wb_exc_iadel || wb_exc_itlb || wb_exc_adel || wb_exc_ades || wb_exc_dtlb ||
        wb_exc_watch || wb_exc_overflow ||
        wb_exc_syscall || wb_exc_break || wb_exc_trap || wb_exc_ri || wb_exc_cpu ||
        wb_exc_fpe || wb_exc_int);
    wire wb_take_exception = wb_take_exception_pre || wb_exc_mcheck;

    wire [4:0] wb_exc_code =
        wb_exc_iadel    ? `EXC_ADEL :
        wb_exc_itlb     ? `EXC_TLBL :
        wb_exc_adel     ? `EXC_ADEL :
        wb_exc_ades     ? `EXC_ADES :
        wb_dtlb_mod     ? `EXC_MOD  :
        (wb_dtlb_miss || wb_dtlb_invalid) && wb_do_store ? `EXC_TLBS :
        (wb_dtlb_miss || wb_dtlb_invalid) ? `EXC_TLBL :
        wb_exc_watch    ? `EXC_WATCH :
        wb_exc_ri       ? `EXC_RI  :
        wb_exc_cpu      ? `EXC_CPU :
        wb_exc_syscall  ? `EXC_SYS :
        wb_exc_break    ? `EXC_BP  :
        wb_exc_overflow ? `EXC_OV  :
        wb_exc_trap     ? `EXC_TR  :
        wb_exc_fpe      ? `EXC_FPE :
        wb_exc_mcheck   ? `EXC_MCHECK :
                           `EXC_INT ;

    wire wb_is_itlb_refill = wb_itlb_miss && !wb_exc_iadel;
    wire wb_is_tlb_refill = (wb_is_itlb_refill || wb_dtlb_miss) &&
                            !(wb_exc_iadel) && !wb_dtlb_invalid && !wb_dtlb_mod;

    wire wb_bev = cp0_status[22];
    wire wb_use_refill_vector = wb_is_tlb_refill && !cp0_status[1];
    wire [63:0] wb_exc_vector = wb_use_refill_vector ?
        (wb_bev ? 64'hFFFF_FFFF_BFC0_0200 : 64'hFFFF_FFFF_8000_0000) :
        (wb_bev ? 64'hFFFF_FFFF_BFC0_0380 : 64'hFFFF_FFFF_8000_0180);

    // WB forwarding
    wire wb_fwd_valid = wb_valid && !wb_take_exception && (wb_wb_reg != 5'd0);

    // ===================================================================
    // CP0 Forwarding: EX > DF > DS > TC > WB > architectural
    // ===================================================================
    wire cp0_fwd_match_ex = ex_valid && !ex_has_early_exc &&
                         (ex_is_mtc0 || ex_is_dmtc0) && (ex_rd_idx == rf_rd_idx);
    wire cp0_fwd_match_df = df_valid && !df_has_early_exc &&
                         (df_is_mtc0 || df_is_dmtc0) && (df_rd_idx == rf_rd_idx);
    wire cp0_fwd_match_ds = ds_valid &&
                         (ds_is_mtc0 || ds_is_dmtc0) && (ds_rd_idx == rf_rd_idx);
    wire cp0_fwd_match_tc = tc_valid &&
                         (tc_is_mtc0 || tc_is_dmtc0) && (tc_rd_idx == rf_rd_idx);
    // CACHE ILT forwarding: when a CACHE Index Load Tag is at WB stage and
    // writes cp0_taglo, forward the result to any MFC0 reading TagLo in RF.
    // This is needed because the ILT writes cp0_taglo via non-blocking at WB,
    // but MFC0 reads at RF — with 3 nops between them, MFC0 arrives at RF
    // one cycle before the ILT's WB write takes effect.
    wire cp0_fwd_cache_ilt = wb_cache_commit && wb_cache_op == 3'd1 &&
                              rf_rd_idx == `CP0_TAGLO;
    wire cp0_fwd_cache_ilt_update = cache_cop_taglo_update_valid &&
                                     rf_opcode == `OP_COP0 &&
                                     (rf_rs_idx == `COP0_MFC0 ||
                                      rf_rs_idx == `COP0_DMFC0) &&
                                     rf_rd_idx == `CP0_TAGLO;

    // Cross-register side effect: MTC0 to Compare clears IP7 (bit 15) in Cause.
    // Detect pending Compare writes in any pipeline stage for forwarding to MFC0 Cause.
    wire pending_compare_write =
        (ex_valid && !ex_has_early_exc && (ex_is_mtc0 || ex_is_dmtc0) && (ex_rd_idx == `CP0_COMPARE)) ||
        (df_valid && !df_has_early_exc && (df_is_mtc0 || df_is_dmtc0) && (df_rd_idx == `CP0_COMPARE)) ||
        (ds_valid && (ds_is_mtc0 || ds_is_dmtc0) && (ds_rd_idx == `CP0_COMPARE)) ||
        (tc_valid && (tc_is_mtc0 || tc_is_dmtc0) && (tc_rd_idx == `CP0_COMPARE));

    wire [63:0] rf_cp0_rdata_raw = cp0_fwd_cache_ilt_update ?
                                        {32'd0, cache_cop_taglo_update_data} :
                                    cp0_fwd_match_ex ? ex_rt_val :
                                    cp0_fwd_match_df ? df_rt_val :
                                    cp0_fwd_match_ds ? ds_rt_val :
                                    cp0_fwd_match_tc ? tc_rt_val :
                                    cp0_fwd_cache_ilt ? {32'd0, cache_cop_taglo_in} :
                                    rf_cp0_rdata_arch;

    // Apply Compare→Cause IP7 side effect: clear bit 15 when reading Cause
    // and a Compare write is pending in the pipeline
    assign rf_cp0_rdata = (rf_rd_idx == `CP0_CAUSE && pending_compare_write) ?
                           {rf_cp0_rdata_raw[63:16], 1'b0, rf_cp0_rdata_raw[14:0]} :
                           rf_cp0_rdata_raw;

    // ===================================================================
    // Forwarding Logic: EX > DF > DS > TC > register file
    // ===================================================================
    // Avoid same-cycle WB forwarding in the RF operand mux.  On this FPGA that
    // path is timing-critical; RF instead stalls for one cycle below and then
    // reads the architectural value committed by WB.
    assign rf_rs_val = (rf_rs_idx != 5'd0 && rf_rs_idx == ex_wb_reg && ex_fwd_ready) ? ex_fwd_data :
                       (rf_rs_idx != 5'd0 && rf_rs_idx == df_wb_reg && df_fwd_ready) ? df_fwd_data :
                       (rf_rs_idx != 5'd0 && rf_rs_idx == ds_wb_reg && ds_fwd_ready) ? ds_fwd_data :
                       (rf_rs_idx != 5'd0 && rf_rs_idx == tc_wb_reg && tc_fwd_valid) ? tc_wb_result :
                       rf_rs_raw;
    assign rf_rt_val = (rf_rt_idx != 5'd0 && rf_rt_idx == ex_wb_reg && ex_fwd_ready) ? ex_fwd_data :
                       (rf_rt_idx != 5'd0 && rf_rt_idx == df_wb_reg && df_fwd_ready) ? df_fwd_data :
                       (rf_rt_idx != 5'd0 && rf_rt_idx == ds_wb_reg && ds_fwd_ready) ? ds_fwd_data :
                       (rf_rt_idx != 5'd0 && rf_rt_idx == tc_wb_reg && tc_fwd_valid) ? tc_wb_result :
                       rf_rt_raw;

    // RF control-flow redirects feed PC. Keep EX result data out of that path;
    // an exact EX dependency stalls one cycle below and then forwards from DF.
    assign rf_rs_ctrl_val = (rf_rs_idx != 5'd0 && rf_rs_idx == df_wb_reg && df_fwd_ready) ? df_fwd_data :
                            (rf_rs_idx != 5'd0 && rf_rs_idx == ds_wb_reg && ds_fwd_ready) ? ds_fwd_data :
                            (rf_rs_idx != 5'd0 && rf_rs_idx == tc_wb_reg && tc_fwd_valid) ? tc_wb_result :
                            rf_rs_raw;
    assign rf_rt_ctrl_val = (rf_rt_idx != 5'd0 && rf_rt_idx == df_wb_reg && df_fwd_ready) ? df_fwd_data :
                            (rf_rt_idx != 5'd0 && rf_rt_idx == ds_wb_reg && ds_fwd_ready) ? ds_fwd_data :
                            (rf_rt_idx != 5'd0 && rf_rt_idx == tc_wb_reg && tc_fwd_valid) ? tc_wb_result :
                            rf_rt_raw;

    // ===================================================================
    // Hazard Detection & Stall/Flush Control
    // ===================================================================

    // Load-use hazard: load in EX/DF/DS, RF needs the result → stall until TC.
    wire load_use_hazard = rf_valid && (
        (ex_valid && ex_do_load && ex_wb_reg != 5'd0 &&
         ((rf_uses_rs && rf_rs_idx == ex_wb_reg) || (rf_uses_rt && rf_rt_idx == ex_wb_reg))) ||
        (df_valid && df_do_load && df_wb_reg != 5'd0 &&
         ((rf_uses_rs && rf_rs_idx == df_wb_reg) || (rf_uses_rt && rf_rt_idx == df_wb_reg))) ||
        (ds_valid && ds_do_load && ds_wb_reg != 5'd0 &&
         ((rf_uses_rs && rf_rs_idx == ds_wb_reg) || (rf_uses_rt && rf_rt_idx == ds_wb_reg)))
    );

    // SC-use hazard: SC in EX/DF/DS, RF needs the result (wb_sel == WB_LLSC).
    wire sc_use_hazard = rf_valid && (
        (ex_valid && (ex_wb_sel == `WB_LLSC) && ex_wb_reg != 5'd0 &&
         ((rf_uses_rs && rf_rs_idx == ex_wb_reg) || (rf_uses_rt && rf_rt_idx == ex_wb_reg))) ||
        (df_valid && (df_wb_sel == `WB_LLSC) && df_wb_reg != 5'd0 &&
         ((rf_uses_rs && rf_rs_idx == df_wb_reg) || (rf_uses_rt && rf_rt_idx == df_wb_reg))) ||
        (ds_valid && (ds_wb_sel == `WB_LLSC) && ds_wb_reg != 5'd0 &&
         ((rf_uses_rs && rf_rs_idx == ds_wb_reg) || (rf_uses_rt && rf_rt_idx == ds_wb_reg)))
    );

    wire control_ex_result_hazard = rf_valid && (rf_do_jump || rf_do_branch) &&
        ex_fwd_ready &&
        ((rf_uses_rs && rf_rs_idx == ex_wb_reg) ||
         (rf_uses_rt && rf_rt_idx == ex_wb_reg));

    wire wb_gpr_read_hazard = rf_valid && wb_fwd_valid &&
        ((rf_uses_rs && rf_rs_idx == wb_wb_reg) ||
         (rf_uses_rt && rf_rt_idx == wb_wb_reg));

    // Muldiv stall: MFHI/MFLO must wait for muldiv to complete
    // Also stall new MULT/DIV/MUL while unit is busy
    wire muldiv_stall = rf_valid && (
        ((rf_wb_sel == `WB_HI || rf_wb_sel == `WB_LO) && (md_busy || md_start_wire)) ||
        (rf_md_start && (md_busy || md_start_wire))
    );

    // CTC1 writes a control register, not a normal ALU/FPR result. It is rare
    // enough to wait for its GPR source to be architectural, which keeps the
    // forwarded GPR result mux out of the RF->EX FCSR control path.
    wire ctc1_rt_raw_hazard = rf_valid && rf_fpu_ctc1 && (rf_rt_idx != 5'd0) && (
        (ex_valid && (ex_wb_reg == rf_rt_idx) && (ex_wb_reg != 5'd0)) ||
        (df_valid && (df_wb_reg == rf_rt_idx) && (df_wb_reg != 5'd0)) ||
        (ds_valid && (ds_wb_reg == rf_rt_idx) && (ds_wb_reg != 5'd0)) ||
        (tc_valid && (tc_wb_reg == rf_rt_idx) && (tc_wb_reg != 5'd0)) ||
        (wb_valid && (wb_wb_reg == rf_rt_idx) && (wb_wb_reg != 5'd0))
    );

    // CP0 CO hazard: TLBP writes Index, TLBR writes EntryHi/EntryLo0/EntryLo1/PageMask.
    // CP0 register readers at RF must stall until these operations commit at WB.
    // TLBWI/TLBWR/TLBP/TLBR also consume architectural CP0 TLB state; keep them
    // behind older MTC0/DMTC0 writes and older TLB CO side effects they read.
    wire rf_reads_cp0 = rf_valid && (rf_opcode == `OP_COP0) &&
                        (rf_rs_idx == `COP0_MFC0 || rf_rs_idx == `COP0_DMFC0);
    wire rf_tlbwi = rf_valid && rf_is_cop0_co && rf_cop0_funct == `C0FN_TLBWI;
    wire rf_tlbwr = rf_valid && rf_is_cop0_co && rf_cop0_funct == `C0FN_TLBWR;
    wire rf_tlbp  = rf_valid && rf_is_cop0_co && rf_cop0_funct == `C0FN_TLBP;
    wire rf_tlbr  = rf_valid && rf_is_cop0_co && rf_cop0_funct == `C0FN_TLBR;
    wire rf_tlbop_reads_index    = rf_tlbwi || rf_tlbr;
    wire rf_tlbop_reads_entryhi  = rf_tlbwi || rf_tlbwr || rf_tlbp;
    wire rf_tlbop_reads_entrylo0 = rf_tlbwi || rf_tlbwr;
    wire rf_tlbop_reads_entrylo1 = rf_tlbwi || rf_tlbwr;
    wire rf_tlbop_reads_pagemask = rf_tlbwi || rf_tlbwr;
    wire rf_tlbop_reads_wired    = rf_tlbwr;
    wire pending_tlbp = (ex_valid && ex_is_cop0_co && ex_cop0_funct == `C0FN_TLBP) ||
                        (df_valid && df_is_cop0_co && df_cop0_funct == `C0FN_TLBP) ||
                        (ds_valid && ds_is_cop0_co && ds_cop0_funct == `C0FN_TLBP) ||
                        (tc_valid && tc_is_cop0_co && tc_cop0_funct == `C0FN_TLBP) ||
                        (wb_valid && !wb_take_exception && wb_is_cop0_co && wb_cop0_funct == `C0FN_TLBP);
    wire pending_tlbr = (ex_valid && ex_is_cop0_co && ex_cop0_funct == `C0FN_TLBR) ||
                        (df_valid && df_is_cop0_co && df_cop0_funct == `C0FN_TLBR) ||
                        (ds_valid && ds_is_cop0_co && ds_cop0_funct == `C0FN_TLBR) ||
                        (tc_valid && tc_is_cop0_co && tc_cop0_funct == `C0FN_TLBR) ||
                        (wb_valid && !wb_take_exception && wb_is_cop0_co && wb_cop0_funct == `C0FN_TLBR);

    wire tlbop_mtc0_hazard_ex = ex_valid && !ex_has_early_exc &&
                                (ex_is_mtc0 || ex_is_dmtc0) && (
        (rf_tlbop_reads_index    && ex_rd_idx == `CP0_INDEX) ||
        (rf_tlbop_reads_entryhi  && ex_rd_idx == `CP0_ENTRYHI) ||
        (rf_tlbop_reads_entrylo0 && ex_rd_idx == `CP0_ENTRYLO0) ||
        (rf_tlbop_reads_entrylo1 && ex_rd_idx == `CP0_ENTRYLO1) ||
        (rf_tlbop_reads_pagemask && ex_rd_idx == `CP0_PAGEMASK) ||
        (rf_tlbop_reads_wired    && ex_rd_idx == `CP0_WIRED)
    );
    wire tlbop_mtc0_hazard_df = df_valid && !df_has_early_exc &&
                                (df_is_mtc0 || df_is_dmtc0) && (
        (rf_tlbop_reads_index    && df_rd_idx == `CP0_INDEX) ||
        (rf_tlbop_reads_entryhi  && df_rd_idx == `CP0_ENTRYHI) ||
        (rf_tlbop_reads_entrylo0 && df_rd_idx == `CP0_ENTRYLO0) ||
        (rf_tlbop_reads_entrylo1 && df_rd_idx == `CP0_ENTRYLO1) ||
        (rf_tlbop_reads_pagemask && df_rd_idx == `CP0_PAGEMASK) ||
        (rf_tlbop_reads_wired    && df_rd_idx == `CP0_WIRED)
    );
    wire tlbop_mtc0_hazard_ds = ds_valid && (ds_is_mtc0 || ds_is_dmtc0) && (
        (rf_tlbop_reads_index    && ds_rd_idx == `CP0_INDEX) ||
        (rf_tlbop_reads_entryhi  && ds_rd_idx == `CP0_ENTRYHI) ||
        (rf_tlbop_reads_entrylo0 && ds_rd_idx == `CP0_ENTRYLO0) ||
        (rf_tlbop_reads_entrylo1 && ds_rd_idx == `CP0_ENTRYLO1) ||
        (rf_tlbop_reads_pagemask && ds_rd_idx == `CP0_PAGEMASK) ||
        (rf_tlbop_reads_wired    && ds_rd_idx == `CP0_WIRED)
    );
    wire tlbop_mtc0_hazard_tc = tc_valid && (tc_is_mtc0 || tc_is_dmtc0) && (
        (rf_tlbop_reads_index    && tc_rd_idx == `CP0_INDEX) ||
        (rf_tlbop_reads_entryhi  && tc_rd_idx == `CP0_ENTRYHI) ||
        (rf_tlbop_reads_entrylo0 && tc_rd_idx == `CP0_ENTRYLO0) ||
        (rf_tlbop_reads_entrylo1 && tc_rd_idx == `CP0_ENTRYLO1) ||
        (rf_tlbop_reads_pagemask && tc_rd_idx == `CP0_PAGEMASK) ||
        (rf_tlbop_reads_wired    && tc_rd_idx == `CP0_WIRED)
    );
    wire tlbop_mtc0_hazard_wb = wb_valid && !wb_take_exception && !wb_do_eret &&
                                (wb_is_mtc0 || wb_is_dmtc0) && (
        (rf_tlbop_reads_index    && wb_rd_idx == `CP0_INDEX) ||
        (rf_tlbop_reads_entryhi  && wb_rd_idx == `CP0_ENTRYHI) ||
        (rf_tlbop_reads_entrylo0 && wb_rd_idx == `CP0_ENTRYLO0) ||
        (rf_tlbop_reads_entrylo1 && wb_rd_idx == `CP0_ENTRYLO1) ||
        (rf_tlbop_reads_pagemask && wb_rd_idx == `CP0_PAGEMASK) ||
        (rf_tlbop_reads_wired    && wb_rd_idx == `CP0_WIRED)
    );
    wire tlbop_mtc0_hazard = tlbop_mtc0_hazard_ex || tlbop_mtc0_hazard_df ||
                             tlbop_mtc0_hazard_ds || tlbop_mtc0_hazard_tc ||
                             tlbop_mtc0_hazard_wb;
    wire tlbop_co_hazard =
        (pending_tlbp && rf_tlbop_reads_index) ||
        (pending_tlbr && (rf_tlbop_reads_entryhi || rf_tlbop_reads_entrylo0 ||
                          rf_tlbop_reads_entrylo1 || rf_tlbop_reads_pagemask));
    wire cp0_co_read_hazard = rf_reads_cp0 && (
        (pending_tlbp && rf_rd_idx == `CP0_INDEX) ||
        (pending_tlbr && (rf_rd_idx == `CP0_ENTRYHI || rf_rd_idx == `CP0_ENTRYLO0 ||
                          rf_rd_idx == `CP0_ENTRYLO1 || rf_rd_idx == `CP0_PAGEMASK))
    );
    wire cp0_mtc0_read_hazard_wb = rf_reads_cp0 && wb_valid &&
        !wb_take_exception && !wb_do_eret && (wb_is_mtc0 || wb_is_dmtc0) &&
        ((wb_rd_idx == rf_rd_idx) ||
         (rf_rd_idx == `CP0_CAUSE && wb_rd_idx == `CP0_COMPARE));
    wire cp0_co_hazard = cp0_co_read_hazard || cp0_mtc0_read_hazard_wb ||
                         tlbop_mtc0_hazard || tlbop_co_hazard;

    // FPR write-use hazard: FPR writes commit at WB, so RF must not read a
    // pending FPR result from an older pipeline stage.
    function [4:0] fpr_phys_idx;
        input [4:0] idx;
        input       fr_mode_in;
        begin
            fpr_phys_idx = (!fr_mode_in && idx[0]) ? {idx[4:1], 1'b0} : idx;
        end
    endfunction

    wire rf_reads_fpr_fs = rf_valid && (rf_do_fpop || rf_madd_op != 3'd0 || rf_fpu_wr_gpr);
    wire rf_reads_fpr_ft = rf_valid && (rf_do_fpop || rf_madd_op != 3'd0 || rf_fpu_swc1 || rf_fpu_sdc1);
    wire rf_reads_fpr_fr = rf_valid && (rf_madd_op != 3'd0);
    wire [4:0] rf_fpr_fs_idx = fpr_phys_idx(fpu_fs, cp1_fr);
    wire [4:0] rf_fpr_ft_idx = fpr_phys_idx(fpu_ft, cp1_fr);
    wire [4:0] rf_fpr_fr_idx = fpr_phys_idx(rf_rs_idx, cp1_fr);
    wire       ex_fpu_writes_fpr = ex_valid && (ex_fpu_wr || ex_fpu_mtc1 || ex_fpu_lwc1 || ex_fpu_ldc1);
    wire       df_fpu_writes_fpr = df_valid && (df_fpu_wr || df_fpu_mtc1 || df_fpu_lwc1 || df_fpu_ldc1);
    wire       ds_fpu_writes_fpr = ds_valid && (ds_fpu_wr || ds_fpu_mtc1 || ds_fpu_lwc1 || ds_fpu_ldc1);
    wire       tc_fpu_writes_fpr = tc_valid && (tc_fpu_wr || tc_fpu_mtc1 || tc_fpu_lwc1 || tc_fpu_ldc1);
    wire       wb_fpu_writes_fpr = wb_valid && (wb_fpu_wr || wb_fpu_mtc1 || wb_fpu_lwc1 || wb_fpu_ldc1);
    wire [4:0] ex_fpu_write_idx = fpr_phys_idx((ex_fpu_lwc1 || ex_fpu_ldc1) ? ex_insn[20:16] : ex_fpu_fd,
                                                ex_fpu_fr_mode);
    wire [4:0] df_fpu_write_idx = fpr_phys_idx((df_fpu_lwc1 || df_fpu_ldc1) ? df_insn[20:16] : df_fpu_fd,
                                                df_fpu_fr_mode);
    wire [4:0] ds_fpu_write_idx = fpr_phys_idx((ds_fpu_lwc1 || ds_fpu_ldc1) ? ds_insn[20:16] : ds_fpu_fd,
                                                ds_fpu_fr_mode);
    wire [4:0] tc_fpu_write_idx = fpr_phys_idx((tc_fpu_lwc1 || tc_fpu_ldc1) ? tc_insn[20:16] : tc_fpu_fd,
                                                tc_fpu_fr_mode);
    wire [4:0] wb_fpu_write_idx = fpr_phys_idx((wb_fpu_lwc1 || wb_fpu_ldc1) ? wb_insn[20:16] : wb_fpu_fd,
                                                wb_fpu_fr_mode);
	    wire fpr_write_hazard = (
	        (ex_fpu_writes_fpr &&
	         ((rf_reads_fpr_fs && rf_fpr_fs_idx == ex_fpu_write_idx) ||
	          (rf_reads_fpr_ft && rf_fpr_ft_idx == ex_fpu_write_idx) ||
	          (rf_reads_fpr_fr && rf_fpr_fr_idx == ex_fpu_write_idx))) ||
	        (df_fpu_writes_fpr &&
	         ((rf_reads_fpr_fs && rf_fpr_fs_idx == df_fpu_write_idx) ||
	          (rf_reads_fpr_ft && rf_fpr_ft_idx == df_fpu_write_idx) ||
	          (rf_reads_fpr_fr && rf_fpr_fr_idx == df_fpu_write_idx))) ||
	        (ds_fpu_writes_fpr &&
	         ((rf_reads_fpr_fs && rf_fpr_fs_idx == ds_fpu_write_idx) ||
	          (rf_reads_fpr_ft && rf_fpr_ft_idx == ds_fpu_write_idx) ||
	          (rf_reads_fpr_fr && rf_fpr_fr_idx == ds_fpu_write_idx))) ||
	        (tc_fpu_writes_fpr &&
	         ((rf_reads_fpr_fs && rf_fpr_fs_idx == tc_fpu_write_idx) ||
	          (rf_reads_fpr_ft && rf_fpr_ft_idx == tc_fpu_write_idx) ||
	          (rf_reads_fpr_fr && rf_fpr_fr_idx == tc_fpu_write_idx))) ||
	        (wb_fpu_writes_fpr &&
	         ((rf_reads_fpr_fs && rf_fpr_fs_idx == wb_fpu_write_idx) ||
	          (rf_reads_fpr_ft && rf_fpr_ft_idx == wb_fpu_write_idx) ||
	          (rf_reads_fpr_fr && rf_fpr_fr_idx == wb_fpu_write_idx)))
	    );

    // FCSR writes now commit at WB, so consumers that read FCSR/FCC/RM/FS
    // wait until older FCSR updates have reached the FPU control register.
    wire ex_fpu_writes_fcsr = ex_valid && (ex_fpu_wr || ex_fpu_cmp || ex_fpu_ctc1 || ex_exc_fpe);
    wire df_fpu_writes_fcsr = df_valid && (df_fpu_wr || df_fpu_cmp || df_fpu_ctc1 || df_exc_fpe);
    wire ds_fpu_writes_fcsr = ds_valid && (ds_fpu_wr || ds_fpu_cmp || ds_fpu_ctc1 || ds_exc_fpe);
    wire tc_fpu_writes_fcsr = tc_valid && (tc_fpu_wr || tc_fpu_cmp || tc_fpu_ctc1 || tc_exc_fpe);
    wire wb_fpu_writes_fcsr = wb_valid && (wb_fpu_wr || wb_fpu_cmp || wb_fpu_ctc1 || wb_exc_fpe);
    wire rf_uses_fcsr =
        rf_valid && (rf_do_fpop || rf_madd_op != 3'd0 || rf_fpu_ctc1 ||
                     (rf_opcode == `OP_COP1 && rf_rs_idx == `COP1_CFC1) ||
                     (rf_opcode == `OP_COP1 && rf_rs_idx == `COP1_BC1) ||
                     (rf_opcode == `OP_SPECIAL && rf_funct == `FN_MOVCI));
    wire fcsr_write_hazard = rf_uses_fcsr &&
                              (ex_fpu_writes_fcsr || df_fpu_writes_fcsr ||
                               ds_fpu_writes_fcsr || tc_fpu_writes_fcsr ||
                               wb_fpu_writes_fcsr);

    // CACHE ILT → MFC0 TagLo hazard: MFC0 reads cp0_taglo at RF, but CACHE ILT
    // writes it at WB. With 3 nops between them, MFC0's RF arrives one cycle
    // before ILT's WB. Stall MFC0 for one cycle so ILT can commit first.
    wire cache_ilt_taglo_hazard = tc_valid && tc_is_cache && tc_cache_op == 3'd1 &&
                                  rf_valid && rf_insn[31:26] == `OP_COP0 &&
                                  rf_insn[25:21] == `COP0_MFC0 &&
                                  rf_insn[15:11] == `CP0_TAGLO;

    wire rf_uses_cp1_status = rf_valid &&
        (rf_opcode == `OP_COP1 || rf_opcode == `OP_LWC1 ||
         rf_opcode == `OP_SWC1 || rf_opcode == `OP_LDC1 ||
         rf_opcode == `OP_SDC1);
    wire cp1_status_hazard = rf_uses_cp1_status && inflight_mtc0_status_any;

    // CC-5: sync — drain pipe before sync advances out of RF. Older ops ahead
    // of sync must commit (all of EX/DF/DS/TC/WB empty) so subsequent ops
    // cannot observe pre-sync effects out of order. Cache misses hold the
    // pipeline via cache_stall separately.
    wire sync_hazard = rf_valid && rf_is_sync &&
                       (ex_valid || df_valid || ds_valid || tc_valid || wb_valid);

    // CACHE operations are ordered with respect to following instructions.
    // A younger access must not run before the selected cache operation
    // reaches WB and its cache state machine completes. Hold the next RF
    // instruction while the older operation drains; cache_stall then holds it
    // through multi-cycle cache work. This includes Primary-D: otherwise a
    // following Secondary writeback can commit before dirty Primary-D data is
    // published, which violates the cross-primary cache lifecycle.
    wire cache_order_hazard = rf_valid &&
                              ((ex_valid && ex_is_cache) ||
                               (df_valid && df_is_cache) ||
                               (ds_valid && ds_is_cache) ||
                               (tc_valid && tc_is_cache) ||
                               (wb_valid && wb_is_cache));

    wire stall_rf_pre_fpu = load_use_hazard || sc_use_hazard ||
                            control_ex_result_hazard || wb_gpr_read_hazard ||
                            muldiv_stall ||
                            ctc1_rt_raw_hazard ||
                            cp0_co_hazard || fpr_write_hazard ||
                            fcsr_write_hazard || cache_ilt_taglo_hazard ||
                            cp1_status_hazard ||
                            sync_hazard || cache_order_hazard || stall_ex;
    wire rf_fpu_can_issue = rf_valid && !stall_rf_pre_fpu;
    // The memory/cache stall freezes RF->EX commit, but FPU early writes are
    // disabled, so a waiting RF-stage FPU op can compute its result while an
    // older memory op is stalled.  The acked result advances only when the
    // pipeline freeze releases.
    wire rf_fpu_issue = rf_fpu_can_issue;
    wire rf_fpu_is_cvt = rf_do_fpop &&
        (((rf_fpu_fmt == `COP1_FMT_S) &&
          (rf_funct == `FPU_CVT_D || rf_funct == `FPU_CVT_W || rf_funct == `FPU_CVT_L ||
           rf_funct == `FPU_ROUND_W || rf_funct == `FPU_TRUNC_W ||
           rf_funct == `FPU_CEIL_W  || rf_funct == `FPU_FLOOR_W ||
           rf_funct == `FPU_ROUND_L || rf_funct == `FPU_TRUNC_L ||
           rf_funct == `FPU_CEIL_L  || rf_funct == `FPU_FLOOR_L)) ||
         ((rf_fpu_fmt == `COP1_FMT_D) &&
          (rf_funct == `FPU_CVT_S || rf_funct == `FPU_CVT_W || rf_funct == `FPU_CVT_L ||
           rf_funct == `FPU_ROUND_W || rf_funct == `FPU_TRUNC_W ||
           rf_funct == `FPU_CEIL_W  || rf_funct == `FPU_FLOOR_W ||
           rf_funct == `FPU_ROUND_L || rf_funct == `FPU_TRUNC_L ||
           rf_funct == `FPU_CEIL_L  || rf_funct == `FPU_FLOOR_L)) ||
         ((rf_fpu_fmt == `COP1_FMT_W) &&
          (rf_funct == `FPU_CVT_S || rf_funct == `FPU_CVT_D)) ||
         ((rf_fpu_fmt == `COP1_FMT_L) &&
          (rf_funct == `FPU_CVT_S || rf_funct == `FPU_CVT_D)));
    wire rf_fpu_multicycle = rf_fpu_can_issue &&
        ((rf_do_fpop && (rf_funct == `FPU_DIV || rf_funct == `FPU_SQRT ||
                         rf_funct == `FPU_MUL || rf_funct == `FPU_ADD ||
                         rf_funct == `FPU_SUB || rf_fpu_is_cvt)) ||
         rf_madd_op != 3'd0);
    wire rf_fpu_result_op = rf_valid && (rf_do_fpop || rf_madd_op != 3'd0);
    wire fpu_multicycle_stall = rf_fpu_multicycle && !fpu_result_valid &&
                                !rf_fpu_result_acked;
    wire rf_fpu_result_ready = !rf_fpu_multicycle || fpu_result_valid;
    wire rf_fpu_capture_pending = rf_fpu_can_issue && rf_fpu_result_op &&
                                  rf_fpu_result_ready && !rf_fpu_result_acked;
    wire rf_fpu_start = rf_fpu_issue &&
                        (!rf_fpu_result_op || !rf_fpu_result_acked);

    wire stall_rf = stall_rf_pre_fpu || fpu_multicycle_stall || rf_fpu_capture_pending;
    wire stall_is = stall_rf;
    wire stall_if = stall_is;

`ifdef FPGA_SYNC_IMEM
    wire sync_imem_hold_resp = (pipeline_mem_stall || stall_is) && imem_q_valid;
    assign imem_addr = {32'd0, sync_imem_hold_resp ? imem_q_paddr : is_paddr};
    assign imem_valid =
        sync_imem_hold_resp ?
            (imem_q_valid && !imem_q_exc_iadel && !imem_q_exc_itlb) :
            (is_valid && !is_exc_iadel && !is_exc_itlb);
    assign imem_uncached = sync_imem_hold_resp ? imem_q_uncached : is_uncached;
`endif

    // Flush signals
    wire flush_all = wb_take_exception || (wb_valid && wb_do_eret);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rf_fpu_result_acked <= 1'b0;
            rf_fpu_result_ack_data <= 64'd0;
            rf_fpu_exc_flags_ack <= 5'd0;
            rf_fpu_exc_trap_ack <= 1'b0;
            rf_fpu_exc_unimpl_ack <= 1'b0;
        end else if (flush_all) begin
            rf_fpu_result_acked <= 1'b0;
            rf_fpu_result_ack_data <= 64'd0;
            rf_fpu_exc_flags_ack <= 5'd0;
            rf_fpu_exc_trap_ack <= 1'b0;
            rf_fpu_exc_unimpl_ack <= 1'b0;
        end else if (rf_fpu_capture_pending) begin
            rf_fpu_result_acked <= 1'b1;
            rf_fpu_result_ack_data <= fpu_fp_result;
            rf_fpu_exc_flags_ack <= fpu_fp_exc_flags;
            rf_fpu_exc_trap_ack <= rf_fpu_exc_trap;
            rf_fpu_exc_unimpl_ack <= fpu_fp_exc_unimpl;
        end else if (!pipeline_mem_stall && !stall_ex && !stall_rf) begin
            rf_fpu_result_acked <= 1'b0;
        end
    end

    // rf_has_delay_slot: the instruction currently in IS is a delay slot
    // (because the branch/jump currently in RF will consume it)
    wire rf_has_delay_slot = rf_valid && !stall_rf && (
        rf_do_jump ||
        (rf_do_branch && rf_branch_taken) ||
        (rf_do_branch && !rf_branch_taken && !rf_branch_likely)
    );

    // Squash IF→IS: taken branch/jump means IF has wrong-path instruction
    wire squash_if = rf_valid && !stall_rf && (
        rf_do_jump ||
        (rf_do_branch && rf_branch_taken));
    wire rf_taken_control_flow = rf_valid && !stall_rf && (
        rf_do_jump ||
        (rf_do_branch && rf_branch_taken));
    wire rf_control_flow_debug_track =
        rf_taken_control_flow &&
        // The low exception vector redirects into the kernel handler after a
        // fault. Keep the pre-fault producer instead of overwriting it there.
        ((rf_pc[31:16] != 16'h8000) ||
         (rf_pc[15:8] != 8'h01));

    // Annul IS→RF: Branch Likely not-taken means delay slot should not execute
    wire annul_is = rf_valid && !stall_rf && rf_do_branch && rf_branch_likely && !rf_branch_taken;

    // ===================================================================
    // Next PC Logic
    // ===================================================================
    reg [63:0] next_pc;
    wire pc_redirect = wb_take_exception || (wb_valid && wb_do_eret);
    wire pc_hold = stall_if || pipeline_mem_stall;
    assign imem_fetch_advance = !pc_hold;

    always @(*) begin
        if (wb_take_exception) begin
            next_pc = wb_exc_vector;
        end else if (wb_valid && wb_do_eret) begin
            if (cp0_status[2])
                next_pc = cp0_errorepc;
            else
                next_pc = cp0_epc;
        end else if (rf_valid && rf_do_jump && !stall_rf) begin
            next_pc = rf_jump_target;
        end else if (rf_valid && rf_do_branch && rf_branch_taken && !stall_rf) begin
            next_pc = rf_branch_target;
        end else begin
            next_pc = pc + 64'd4;
        end
    end

    // ===================================================================
    // Sequential Update — Pipeline registers + architectural state
    // ===================================================================
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc             <= 64'hFFFF_FFFF_BFC0_0000;
            hi_reg         <= 64'd0;
            lo_reg         <= 64'd0;
            llbit          <= 1'b0;
            ll_addr        <= 64'd0;
            ll_paddr       <= 32'd0;
            just_did_eret  <= 1'b0;
            last_cf_pc     <= 32'd0;
            last_cf_target <= 32'd0;
            mfi_valid      <= 1'b0;
            mfi_order      <= 64'd0;
            mfi_pc         <= 64'd0;
            mfi_insn       <= 32'd0;
            mfi_trap       <= 1'b0;
            mfi_rd_addr    <= 5'd0;
            mfi_rd_wdata   <= 64'd0;
            mfi_hilo_wen   <= 2'b00;
            mfi_hi_wdata   <= 64'd0;
            mfi_lo_wdata   <= 64'd0;
            mfi_mem_addr   <= 64'd0;
            mfi_mem_rmask  <= 8'h00;
            mfi_mem_wmask  <= 8'h00;
            mfi_mem_rdata  <= 64'd0;
            mfi_mem_wdata  <= 64'd0;
`ifndef SYNTHESIS
            tlbw_pending   <= 1'b0;
`endif
`ifndef FPGA_NO_MFI
            insn_count     <= 64'd0;
`endif

            for (i = 1; i < 32; i = i + 1)
                gpr[i] <= 64'd0;

            // CP0 initial values
            cp0_index    <= 64'd0;
            cp0_random   <= 64'd47;
            cp0_entrylo0 <= 64'd0;
            cp0_entrylo1 <= 64'd0;
            cp0_context  <= 64'd0;
            cp0_pagemask <= 64'd0;
            cp0_pagemask_valid <= 1'b1;
            cp0_wired    <= 64'd0;
            cp0_badvaddr <= 64'd0;
            cp0_count    <= 64'd0;
            cp0_count_phase <= 1'b0;
            cp0_entryhi  <= 64'd0;
            cp0_compare  <= 64'd0;
            cp0_status   <= 64'h0000_0000_0440_0004; // BEV=1, FR=1, ERL=1
            cp0_cause    <= 64'd0;
            cp0_epc      <= 64'd0;
            cp0_prid     <= CP0_PRID_VAL;
            cp0_config   <= CP0_CONFIG_RESET;  // M=1, BE=1, IC=8K, DC=8K, IB=16B, DB=32B, SB=128B, SC=0
            cp0_lladdr   <= 64'd0;
            cp0_watchlo  <= 64'd0;
            cp0_watchhi  <= 64'd0;
            cp0_xcontext <= 64'd0;
            cp0_ecc      <= 64'd0;
            cp0_cacheerr <= 64'd0;
            cp0_taglo    <= 64'd0;
            cp0_taghi    <= 64'd0;
            cp0_errorepc <= 64'd0;

`ifndef SYNTHESIS
            for (i = 0; i < 48; i = i + 1) begin
                tlb_hi[i]   <= {2'b11, 24'hFFFFFF, i[5:0], 32'd0};
                tlb_lo0[i]  <= 64'd0;
                tlb_lo1[i]  <= 64'd0;
                tlb_mask[i] <= 64'd0;
            end
`endif
            // (Under SYNTHESIS, TLB storage reset lives inside u_tlb.)

            // Pipeline registers
            is_valid  <= 1'b0;
            is_paddr  <= 32'h1FC0_0000;  // reset physical address (kseg1 0xBFC00000)
            is_uncached <= 1'b1;          // kseg1 is uncached
`ifdef FPGA_SYNC_IMEM
            imem_q_valid <= 1'b0;
            imem_q_paddr <= 32'h1FC0_0000;
            imem_q_uncached <= 1'b1;
`endif
            rf_valid  <= 1'b0;
            ex_valid  <= 1'b0;
            df_valid  <= 1'b0;
            df_watch_data_match <= 1'b0;
            ds_valid  <= 1'b0;
            ds_dtlb_match_valid <= 1'b0;
            ds_dtlb_match_idx <= 6'd63;
            ds_dtlb_lo_sel <= 64'd0;
            ds_dtlb_mask <= 64'd0;
            ds_ll_snoop_killed <= 1'b0;
            tc_valid  <= 1'b0;
            wb_valid  <= 1'b0;
            tc_device_side_effect <= 1'b0;
            wb_device_side_effect <= 1'b0;
            tc_cached_store_visible <= 1'b0;
            wb_cached_store_visible <= 1'b0;
            tc_deferred_mem_irq <= 1'b0;
            wb_deferred_mem_irq <= 1'b0;
            tc_ll_snoop_killed <= 1'b0;
            wb_ll_snoop_killed <= 1'b0;
            debug_tlbwr_valid_q <= 1'b0;
            debug_tlbwr_random_q <= 1'b0;
            debug_tlbwr_idx_q <= 6'd0;
            debug_tlbwr_random_idx_q <= 6'd47;
            debug_tlbwr_wired_q <= 6'd0;
            debug_tlbwr_hi_q <= 32'd0;
            debug_tlbwr_lo0_q <= 32'd0;
            debug_tlbwr_lo1_q <= 32'd0;
            debug_tlbwr_mask_q <= 32'd0;

        end else begin
            // -------------------------------------------------------
            // Deferred TLB write disabled — caused 1-cycle delay that broke
            // kernel TLB refill handler. Using inline writes with $write("")
            // to prevent simulator optimization instead.
`ifndef SYNTHESIS
            if (tlbw_pending) tlbw_pending <= 1'b0;
`endif

            // Architecturally fixed CP0 bits are driven every cycle so
            // Quartus does not build storage for reset-only fields.
            cp0_prid             <= CP0_PRID_VAL;
            cp0_cacheerr         <= 64'd0;
            cp0_config[63:3]     <= CP0_CONFIG_RESET[63:3];
            cp0_cause[63:32]     <= 32'd0;
            cp0_cause[30]        <= 1'b0;
            cp0_cause[27:16]     <= 12'd0;
            cp0_cause[7]         <= 1'b0;
            cp0_cause[1:0]       <= 2'b00;
            if (tlbw_commit_en) begin
                debug_tlbwr_valid_q <= 1'b1;
                debug_tlbwr_random_q <= tlbw_commit_random;
                debug_tlbwr_idx_q <= tlbw_commit_idx;
                debug_tlbwr_random_idx_q <= tlb_random_idx;
                debug_tlbwr_wired_q <= tlb_wired_idx;
                debug_tlbwr_hi_q <= tlbw_commit_hi[31:0];
                debug_tlbwr_lo0_q <= tlbw_commit_lo0[31:0];
                debug_tlbwr_lo1_q <= tlbw_commit_lo1[31:0];
                debug_tlbwr_mask_q <= tlbw_commit_mask[31:0];
            end

            // IF → IS pipeline register update
            // -------------------------------------------------------
            if (flush_all) begin
                is_valid <= 1'b0;
            end else if (pipeline_mem_stall) begin
                // freeze — hold all registers
            end else if (stall_if) begin
                // stall — hold (no flush_all check needed, handled above)
            end else if (squash_if) begin
                is_valid <= 1'b0;
            end else begin
                is_valid           <= 1'b1;
                is_pc              <= pc;
                is_paddr           <= tlb_paddr_i;
                is_uncached        <= if_uncached;
                is_in_delay_slot   <= 1'b0;  // set in IS→RF below
                is_exc_iadel       <= if_exc_iadel;
                is_exc_itlb        <= if_exc_itlb;
                is_itlb_miss       <= if_itlb_miss;
                is_exc_watch_insn  <= if_exc_watch_insn;
            end

            // -------------------------------------------------------
            // IS → RF pipeline register update
            // -------------------------------------------------------
`ifdef FPGA_SYNC_IMEM
            if (flush_all) begin
                imem_q_valid <= 1'b0;
            end else if (pipeline_mem_stall) begin
                // freeze — hold all registers
            end else if (stall_is) begin
                // stall — hold
            end else if (squash_if) begin
                imem_q_valid <= 1'b0;
            end else if (annul_is) begin
                imem_q_valid <= 1'b0;
            end else begin
                imem_q_valid          <= is_valid;
                imem_q_pc             <= is_pc;
                imem_q_paddr          <= is_paddr;
                imem_q_uncached       <= is_uncached;
                imem_q_in_delay_slot  <= rf_has_delay_slot;
                imem_q_exc_iadel      <= is_exc_iadel;
                imem_q_exc_itlb       <= is_exc_itlb;
                imem_q_itlb_miss      <= is_itlb_miss;
                imem_q_exc_watch_insn <= is_exc_watch_insn;
            end

            if (flush_all) begin
                rf_valid <= 1'b0;
            end else if (pipeline_mem_stall) begin
                // freeze — hold all registers
            end else if (stall_is) begin
                // stall — hold
            end else if (imem_q_valid && !annul_is) begin
                rf_valid           <= 1'b1;
                rf_pc              <= imem_q_pc;
                rf_insn            <= is_insn;
                rf_in_delay_slot   <= imem_q_in_delay_slot;
                rf_exc_iadel       <= imem_q_exc_iadel;
                rf_exc_itlb        <= imem_q_exc_itlb;
                rf_itlb_miss       <= imem_q_itlb_miss;
                rf_exc_watch_insn  <= imem_q_exc_watch_insn;
            end else begin
                rf_valid <= 1'b0;
            end
`else
            if (flush_all) begin
                rf_valid <= 1'b0;
            end else if (pipeline_mem_stall) begin
                // freeze — hold all registers
            end else if (stall_is) begin
                // stall — hold
            end else if (annul_is) begin
                rf_valid <= 1'b0;
            end else if (is_valid) begin
                rf_valid           <= 1'b1;
                rf_pc              <= is_pc;
                rf_insn            <= is_insn;
                rf_in_delay_slot   <= rf_has_delay_slot;  // delay slot in IS when branch in RF
                rf_exc_iadel       <= is_exc_iadel;
                rf_exc_itlb        <= is_exc_itlb;
                rf_itlb_miss       <= is_itlb_miss;
                rf_exc_watch_insn  <= is_exc_watch_insn;
            end else begin
                rf_valid <= 1'b0;
            end
`endif

            // -------------------------------------------------------
            // RF → EX pipeline register update
            // -------------------------------------------------------
            if (flush_all) begin
                ex_valid <= 1'b0;
            end else if (pipeline_mem_stall) begin
                // freeze — hold all registers
            end else if (stall_ex) begin
                // EX stall (MUL waiting for muldiv) — hold EX, don't overwrite
            end else if (stall_rf) begin
                // RF stall (load-use, etc.) — insert bubble into EX
                ex_valid <= 1'b0;
            end else if (rf_valid) begin
                ex_valid           <= 1'b1;
                ex_pc              <= rf_pc;
                ex_insn            <= rf_insn;
                ex_in_delay_slot   <= rf_in_delay_slot;
                ex_alu_op          <= rf_alu_op;
                ex_alu_a           <= rf_alu_a;
                ex_alu_b           <= rf_alu_b;
                ex_alu_word        <= rf_alu_word;
                ex_check_overflow  <= rf_check_overflow;
                ex_wb_reg          <= rf_wb_reg;
                ex_wb_sel          <= rf_wb_sel;
                ex_do_load         <= rf_do_load;
                ex_do_store        <= rf_do_store;
                ex_mem_size        <= rf_mem_size;
                ex_mem_unsigned    <= rf_mem_unsigned;
                ex_rs_val          <= rf_rs_val;
                ex_rt_val          <= (rf_fpu_swc1 || rf_fpu_sdc1) ? fpu_ft_val : rf_rt_val;
                ex_rs_idx          <= rf_rs_idx;
                ex_rt_idx          <= rf_rt_idx;
                ex_rd_idx          <= rf_rd_idx;
                ex_opcode          <= rf_opcode;
                ex_se_imm          <= rf_se_imm;
                ex_md_start        <= rf_md_start;
                ex_md_op           <= rf_md_op;
                ex_wr_hi           <= rf_wr_hi;
                ex_wr_lo           <= rf_wr_lo;
                ex_hi_wdata        <= rf_hi_wdata;
                ex_lo_wdata        <= rf_lo_wdata;
                ex_exc_iadel       <= rf_exc_iadel;
                ex_exc_itlb        <= rf_exc_itlb;
                ex_itlb_miss       <= rf_itlb_miss;
                ex_exc_watch_insn  <= rf_exc_watch_insn;
                ex_exc_syscall     <= rf_exc_syscall;
                ex_exc_break       <= rf_exc_break;
                ex_exc_trap        <= rf_exc_trap;
                ex_exc_ri          <= rf_exc_ri;
                ex_exc_cpu         <= rf_exc_cpu;
                ex_exc_cpu_ce      <= rf_exc_cpu_ce;
                ex_exc_fpe         <= rf_fpu_result_op ? rf_fpu_exc_trap_ack : 1'b0;
                ex_do_eret         <= rf_do_eret;
                ex_is_mtc0         <= rf_is_mtc0;
                ex_is_dmtc0        <= rf_is_dmtc0;
                ex_is_cop0_co      <= rf_is_cop0_co;
                ex_cop0_funct      <= rf_cop0_funct;
                ex_is_cache        <= rf_is_cache;
                ex_cache_op        <= rf_cache_op;
                ex_cache_type      <= rf_cache_type;
                ex_link_addr       <= rf_link_addr;
                ex_cp0_rdata       <= rf_cp0_rdata;
                // FPU
                ex_fpu_wr          <= rf_fpu_wr;
                ex_fpu_wr_word     <= rf_fpu_wr_word || (rf_fpu_mtc1 && rf_rs_idx == `COP1_MTC1);
                ex_fpu_fd          <= rf_fpu_mtc1 ? fpu_fs : rf_fpu_fd;
                ex_fpu_exc_flags   <= rf_fpu_result_op ? rf_fpu_exc_flags_ack : 5'd0;
                ex_fpu_exc_unimpl  <= rf_fpu_result_op ? rf_fpu_exc_unimpl_ack : 1'b0;
                ex_fpu_cmp         <= rf_fpu_cmp;
                ex_fpu_cmp_result  <= rf_do_fpop && rf_fpu_cmp ? rf_fpu_result_ack_data[0] : 1'b0;
                ex_fpu_cmp_cc      <= rf_fpu_cmp_cc;
                ex_fpu_mtc1        <= rf_fpu_mtc1;
                ex_fpu_ctc1        <= rf_fpu_ctc1;
                ex_fpu_ctc1_data   <= rf_fpu_ctc1 ? rf_rt_raw[31:0] : 32'd0;
                ex_fpu_early_wr_done   <= rf_fpu_early_wr;
                ex_fpu_early_fcsr_done <= rf_fpu_early_fcsr_wr;
                ex_fpu_lwc1        <= rf_fpu_lwc1;
                ex_fpu_swc1        <= rf_fpu_swc1;
                ex_fpu_ldc1        <= rf_fpu_ldc1;
                ex_fpu_sdc1        <= rf_fpu_sdc1;
                ex_fpu_fr_mode     <= cp1_fr;
            end else begin
                ex_valid <= 1'b0;
            end
            if (!pipeline_mem_stall && !stall_ex) begin
                ex_fpu_result <= (!stall_rf && rf_valid) ?
                    (rf_fpu_result_op ? rf_fpu_result_ack_data :
                     rf_fpu_mtc1 ? rf_rt_val : 64'd0) :
                    64'd0;
            end

            // -------------------------------------------------------
            // EX → DF pipeline register update
            // -------------------------------------------------------
            if (flush_all) begin
                df_valid <= 1'b0;
            end else if (pipeline_mem_stall) begin
                // freeze — hold all registers
            end else if (stall_ex) begin
                // MUL waiting for muldiv — hold EX, insert bubble into DF
                df_valid <= 1'b0;
            end else if (ex_valid) begin
                df_valid           <= 1'b1;
                df_pc              <= ex_pc;
                df_insn            <= ex_insn;
                df_in_delay_slot   <= ex_in_delay_slot;
                df_alu_result      <= ex_fwd_data;  // captures ALU, LINK, CP0, HI, LO, MDLO correctly
                df_wb_reg          <= ex_wb_reg;
                df_wb_sel          <= ex_wb_sel;
                df_do_load         <= ex_do_load;
                df_do_store        <= ex_do_store;
                df_mem_size        <= ex_mem_size;
                df_mem_unsigned    <= ex_mem_unsigned;
                df_rs_val          <= ex_rs_val;
                df_rt_val          <= ex_rt_val;
                df_eff_addr        <= ex_eff_addr;
                df_opcode          <= ex_opcode;
                df_se_imm          <= ex_se_imm;
                df_md_start        <= ex_md_start;
                df_wr_hi           <= ex_wr_hi;
                df_wr_lo           <= ex_wr_lo;
                df_hi_wdata        <= ex_hi_wdata;
                df_lo_wdata        <= ex_lo_wdata;
                df_link_addr       <= ex_link_addr;
                df_cp0_rdata       <= ex_cp0_rdata;
                df_rd_idx          <= ex_rd_idx;
                df_exc_iadel       <= ex_exc_iadel;
                df_exc_itlb        <= ex_exc_itlb;
                df_itlb_miss       <= ex_itlb_miss;
                df_exc_watch_insn  <= ex_exc_watch_insn;
                df_watch_data_match <= ex_watch_data_match;
                df_exc_syscall     <= ex_exc_syscall;
                df_exc_break       <= ex_exc_break;
                df_exc_trap        <= ex_exc_trap;
                df_exc_ri          <= ex_exc_ri;
                df_exc_cpu         <= ex_exc_cpu;
                df_exc_cpu_ce      <= ex_exc_cpu_ce;
                df_exc_overflow    <= ex_exc_overflow;
                df_exc_fpe         <= ex_exc_fpe;
                df_do_eret         <= ex_do_eret;
                df_is_mtc0         <= ex_is_mtc0;
                df_is_dmtc0        <= ex_is_dmtc0;
                df_is_cop0_co      <= ex_is_cop0_co;
                df_cop0_funct      <= ex_cop0_funct;
                df_is_cache        <= ex_is_cache;
                df_cache_op        <= ex_cache_op;
                df_cache_type      <= ex_cache_type;
                df_fpu_wr          <= ex_fpu_wr;
                df_fpu_wr_word     <= ex_fpu_wr_word;
                df_fpu_fd          <= ex_fpu_fd;
                df_fpu_result      <= ex_fpu_result;
                df_fpu_exc_flags   <= ex_fpu_exc_flags;
                df_fpu_exc_unimpl  <= ex_fpu_exc_unimpl;
                df_fpu_cmp         <= ex_fpu_cmp;
                df_fpu_cmp_result  <= ex_fpu_cmp_result;
                df_fpu_cmp_cc      <= ex_fpu_cmp_cc;
                df_fpu_mtc1        <= ex_fpu_mtc1;
                df_fpu_ctc1        <= ex_fpu_ctc1;
                df_fpu_ctc1_data   <= ex_fpu_ctc1_data;
                df_fpu_early_wr_done   <= ex_fpu_early_wr_done;
                df_fpu_early_fcsr_done <= ex_fpu_early_fcsr_done;
                df_fpu_lwc1        <= ex_fpu_lwc1;
                df_fpu_swc1        <= ex_fpu_swc1;
                df_fpu_ldc1        <= ex_fpu_ldc1;
                df_fpu_sdc1        <= ex_fpu_sdc1;
                df_fpu_fr_mode     <= ex_fpu_fr_mode;
            end else begin
                df_valid <= 1'b0;
            end

            // -------------------------------------------------------
            // DF → DS pipeline register update
            // -------------------------------------------------------
            if (flush_all) begin
                ds_valid <= 1'b0;
                ds_ll_snoop_killed <= 1'b0;
            end else if (pipeline_mem_stall) begin
                // freeze — hold all registers
                // LL can remain in DS for several cache-response cycles.
                // Remember any matching DMA write until this instruction
                // advances; a one-cycle snoop pulse must not be lost.
                if (snoop_hits_ds_ll)
                    ds_ll_snoop_killed <= 1'b1;
            end else if (df_valid) begin
                ds_valid           <= 1'b1;
                ds_ll_snoop_killed <= 1'b0;
                ds_pc              <= df_pc;
                ds_insn            <= df_insn;
                ds_in_delay_slot   <= df_in_delay_slot;
                ds_alu_result      <= df_alu_result;
                ds_wb_reg          <= df_wb_reg;
                ds_wb_sel          <= df_wb_sel;
                ds_do_load         <= df_do_load;
                ds_do_store        <= df_do_store;
                ds_mem_size        <= df_mem_size;
                ds_mem_unsigned    <= df_mem_unsigned;
                ds_rs_val          <= df_rs_val;
                ds_rt_val          <= df_rt_val;
                ds_eff_addr        <= df_eff_addr;
                ds_opcode          <= df_opcode;
                ds_wr_hi           <= df_wr_hi;
                ds_wr_lo           <= df_wr_lo;
                ds_hi_wdata        <= df_hi_wdata;
                ds_lo_wdata        <= df_lo_wdata;
                ds_link_addr       <= df_link_addr;
                ds_cp0_rdata       <= df_cp0_rdata;
                ds_rd_idx          <= df_rd_idx;
                ds_exc_iadel       <= df_exc_iadel;
                ds_exc_itlb        <= df_exc_itlb;
                ds_itlb_miss       <= df_itlb_miss;
                ds_exc_watch_insn  <= df_exc_watch_insn;
                ds_exc_syscall     <= df_exc_syscall;
                ds_exc_break       <= df_exc_break;
                ds_exc_trap        <= df_exc_trap;
                ds_exc_ri          <= df_exc_ri;
                ds_exc_cpu         <= df_exc_cpu;
                ds_exc_cpu_ce      <= df_exc_cpu_ce;
                ds_exc_overflow    <= df_exc_overflow;
                ds_exc_fpe         <= df_exc_fpe;
                ds_do_eret         <= df_do_eret;
                ds_is_mtc0         <= df_is_mtc0;
                ds_is_dmtc0        <= df_is_dmtc0;
                ds_is_cop0_co      <= df_is_cop0_co;
                ds_cop0_funct      <= df_cop0_funct;
                ds_is_cache        <= df_is_cache;
                ds_cache_op        <= df_cache_op;
                ds_cache_type      <= df_cache_type;
                ds_fpu_wr          <= df_fpu_wr;
                ds_fpu_wr_word     <= df_fpu_wr_word;
                ds_fpu_fd          <= df_fpu_fd;
                ds_fpu_result      <= df_fpu_result;
                ds_fpu_exc_flags   <= df_fpu_exc_flags;
                ds_fpu_exc_unimpl  <= df_fpu_exc_unimpl;
                ds_fpu_cmp         <= df_fpu_cmp;
                ds_fpu_cmp_result  <= df_fpu_cmp_result;
                ds_fpu_cmp_cc      <= df_fpu_cmp_cc;
                ds_fpu_mtc1        <= df_fpu_mtc1;
                ds_fpu_ctc1        <= df_fpu_ctc1;
                ds_fpu_ctc1_data   <= df_fpu_ctc1_data;
                ds_fpu_early_wr_done   <= df_fpu_early_wr_done;
                ds_fpu_early_fcsr_done <= df_fpu_early_fcsr_done;
                ds_fpu_lwc1        <= df_fpu_lwc1;
                ds_fpu_swc1        <= df_fpu_swc1;
                ds_fpu_ldc1        <= df_fpu_ldc1;
                ds_fpu_sdc1        <= df_fpu_sdc1;
                ds_fpu_fr_mode     <= df_fpu_fr_mode;
                ds_paddr           <= tlb_paddr_d;
                ds_uncached        <= df_uncached;
                ds_store_mask      <= df_store_mask;
                ds_store_data      <= df_store_data;
                ds_exc_adel        <= df_exc_adel;
                ds_exc_ades        <= df_exc_ades;
                ds_exc_dtlb        <= df_exc_dtlb;
                ds_dtlb_miss       <= df_dtlb_miss;
                ds_dtlb_invalid    <= df_dtlb_invalid;
                ds_dtlb_mod        <= df_dtlb_mod;
                ds_dtlb_match_valid <= df_is_mapped && tlb_match_d;
                ds_dtlb_match_idx   <= tlb_match_d_idx;
                ds_dtlb_lo_sel      <= tlb_lo_sel_d;
                ds_dtlb_mask        <= tlb_mask_d;
                ds_exc_watch       <= df_exc_watch;
                ds_exc_watch_data  <= df_exc_watch_data;
                ds_md_start        <= df_md_start;
            end else begin
                ds_valid <= 1'b0;
                ds_ll_snoop_killed <= 1'b0;
                ds_dtlb_match_valid <= 1'b0;
                ds_dtlb_match_idx <= 6'd63;
                ds_dtlb_lo_sel <= 64'd0;
                ds_dtlb_mask <= 64'd0;
            end

            // -------------------------------------------------------
            // DS → TC pipeline register update
            // -------------------------------------------------------
            if (flush_all) begin
                tc_valid <= 1'b0;
                tc_ll_snoop_killed <= 1'b0;
                tc_device_side_effect <= 1'b0;
                tc_cached_store_visible <= 1'b0;
                tc_deferred_mem_irq <= 1'b0;
            end else if (pipeline_mem_stall) begin
                tc_valid <= 1'b0; // bubble — let WB drain
                tc_ll_snoop_killed <= 1'b0;
                tc_device_side_effect <= 1'b0;
                tc_cached_store_visible <= 1'b0;
                tc_deferred_mem_irq <= 1'b0;
            end else if (ds_valid) begin
                tc_valid           <= 1'b1;
                tc_pc              <= ds_pc;
                tc_insn            <= ds_insn;
                tc_in_delay_slot   <= ds_in_delay_slot;
                tc_alu_result      <= ds_alu_result;
                tc_wb_reg          <= ds_wb_reg;
                tc_wb_sel          <= ds_wb_sel;
                tc_do_load         <= ds_do_load;
                tc_do_store        <= ds_do_store;
                tc_rt_val          <= ds_rt_val;
                tc_eff_addr        <= ds_eff_addr;
                tc_opcode          <= ds_opcode;
                tc_wr_hi           <= ds_wr_hi;
                tc_wr_lo           <= ds_wr_lo;
                tc_hi_wdata        <= ds_hi_wdata;
                tc_lo_wdata        <= ds_lo_wdata;
                tc_link_addr       <= ds_link_addr;
                tc_cp0_rdata       <= ds_cp0_rdata;
                tc_rd_idx          <= ds_rd_idx;
                tc_exc_iadel       <= ds_exc_iadel;
                tc_exc_itlb        <= ds_exc_itlb;
                tc_itlb_miss       <= ds_itlb_miss;
                tc_exc_watch_insn  <= ds_exc_watch_insn;
                tc_exc_syscall     <= ds_exc_syscall;
                tc_exc_break       <= ds_exc_break;
                tc_exc_trap        <= ds_exc_trap;
                tc_exc_ri          <= ds_exc_ri;
                tc_exc_cpu         <= ds_exc_cpu;
                tc_exc_cpu_ce      <= ds_exc_cpu_ce;
                tc_exc_overflow    <= ds_exc_overflow;
                tc_exc_fpe         <= ds_exc_fpe;
                tc_do_eret         <= ds_do_eret;
                tc_is_mtc0         <= ds_is_mtc0;
                tc_is_dmtc0        <= ds_is_dmtc0;
                tc_is_cop0_co      <= ds_is_cop0_co;
                tc_cop0_funct      <= ds_cop0_funct;
                tc_is_cache        <= ds_is_cache;
                tc_cache_op        <= ds_cache_op;
                tc_cache_type      <= ds_cache_type;
                tc_paddr           <= ds_paddr;
                tc_fpu_wr          <= ds_fpu_wr;
                tc_fpu_wr_word     <= ds_fpu_wr_word;
                tc_fpu_fd          <= ds_fpu_fd;
                tc_fpu_result      <= ds_fpu_result;
                tc_fpu_exc_flags   <= ds_fpu_exc_flags;
                tc_fpu_exc_unimpl  <= ds_fpu_exc_unimpl;
                tc_fpu_cmp         <= ds_fpu_cmp;
                tc_fpu_cmp_result  <= ds_fpu_cmp_result;
                tc_fpu_cmp_cc      <= ds_fpu_cmp_cc;
                tc_fpu_mtc1        <= ds_fpu_mtc1;
                tc_fpu_ctc1        <= ds_fpu_ctc1;
                tc_fpu_ctc1_data   <= ds_fpu_ctc1_data;
                tc_fpu_early_wr_done   <= ds_fpu_early_wr_done;
                tc_fpu_early_fcsr_done <= ds_fpu_early_fcsr_done;
                tc_fpu_lwc1        <= ds_fpu_lwc1;
                tc_fpu_ldc1        <= ds_fpu_ldc1;
                tc_fpu_fr_mode     <= ds_fpu_fr_mode;
                tc_exc_adel        <= ds_exc_adel;
                tc_exc_ades        <= ds_exc_ades;
                tc_exc_dtlb        <= ds_exc_dtlb;
                tc_dtlb_miss       <= ds_dtlb_miss;
                tc_dtlb_invalid    <= ds_dtlb_invalid;
                tc_dtlb_mod        <= ds_dtlb_mod;
                tc_exc_watch       <= ds_exc_watch;
                tc_exc_watch_data  <= ds_exc_watch_data;
                tc_store_mask      <= ds_store_mask;
                tc_store_data      <= ds_store_data;
                tc_device_side_effect <= ds_uncached_device_issued ||
                                         (dmem_uncached &&
                                          (dmem_wen || dmem_ren));
                tc_cached_store_visible <= ds_cached_store_visible ||
                                           cached_store_visible;
                tc_deferred_mem_irq <= ds_exc_int &&
                                       (ds_do_load || ds_do_store) &&
                                       !ds_has_any_exc &&
                                       !ds_uncached_device_issued;
                tc_sc_insn         <= ds_sc_insn;
                tc_sc_success      <= ds_sc_store_success;
                tc_load_data       <= ds_load_data;
                tc_md_start        <= ds_md_start;
                tc_ll_snoop_killed <= ds_ll_snoop_killed ||
                                      snoop_hits_ds_ll;
            end else begin
                tc_valid <= 1'b0;
                tc_ll_snoop_killed <= 1'b0;
                tc_device_side_effect <= 1'b0;
                tc_cached_store_visible <= 1'b0;
                tc_deferred_mem_irq <= 1'b0;
            end
            // -------------------------------------------------------
            // TC → WB pipeline register update
            // -------------------------------------------------------
            if (flush_all) begin
                wb_valid <= 1'b0;
                wb_ll_snoop_killed <= 1'b0;
                wb_device_side_effect <= 1'b0;
                wb_cached_store_visible <= 1'b0;
                wb_deferred_mem_irq <= 1'b0;
            end else if (tc_valid) begin
                wb_valid            <= 1'b1;
                wb_pc               <= tc_pc;
                wb_insn             <= tc_insn;
                wb_in_delay_slot    <= tc_in_delay_slot;
                wb_wb_reg           <= tc_wb_reg;
                wb_wb_result        <= tc_wb_result;
                wb_wr_hi            <= tc_wr_hi;
                wb_wr_lo            <= tc_wr_lo;
                wb_hi_wdata         <= tc_hi_wdata;
                wb_lo_wdata         <= tc_lo_wdata;
                wb_rt_val           <= tc_rt_val;
                wb_rd_idx           <= tc_rd_idx;
                wb_opcode           <= tc_opcode;
                wb_eff_addr         <= tc_eff_addr;
                wb_do_load          <= tc_do_load;
                wb_do_store         <= tc_do_store;
                wb_device_side_effect <= tc_device_side_effect;
                wb_cached_store_visible <= tc_cached_store_visible;
                wb_deferred_mem_irq <= tc_deferred_mem_irq;
                wb_sc_insn          <= tc_sc_insn;
                wb_sc_success       <= tc_sc_success;
                wb_link_addr        <= tc_link_addr;
                wb_cp0_rdata        <= tc_cp0_rdata;
                wb_exc_iadel        <= tc_exc_iadel;
                wb_exc_itlb         <= tc_exc_itlb;
                wb_itlb_miss        <= tc_itlb_miss;
                wb_exc_watch_insn   <= tc_exc_watch_insn;
                wb_exc_syscall      <= tc_exc_syscall;
                wb_exc_break        <= tc_exc_break;
                wb_exc_trap         <= tc_exc_trap;
                wb_exc_ri           <= tc_exc_ri;
                wb_exc_cpu          <= tc_exc_cpu;
                wb_exc_cpu_ce       <= tc_exc_cpu_ce;
                wb_exc_overflow     <= tc_exc_overflow;
                wb_exc_fpe          <= tc_exc_fpe;
                wb_exc_adel         <= tc_exc_adel;
                wb_exc_ades         <= tc_exc_ades;
                wb_exc_dtlb         <= tc_exc_dtlb;
                wb_dtlb_miss        <= tc_dtlb_miss;
                wb_dtlb_invalid     <= tc_dtlb_invalid;
                wb_dtlb_mod         <= tc_dtlb_mod;
                wb_exc_watch        <= tc_exc_watch;
                wb_exc_watch_data   <= tc_exc_watch_data;
                wb_do_eret          <= tc_do_eret;
                wb_is_mtc0          <= tc_is_mtc0;
                wb_is_dmtc0         <= tc_is_dmtc0;
                wb_is_cop0_co       <= tc_is_cop0_co;
                wb_cop0_funct       <= tc_cop0_funct;
                wb_is_cache         <= tc_is_cache;
                wb_cache_op         <= tc_cache_op;
                wb_cache_type       <= tc_cache_type;
                wb_paddr            <= tc_paddr;
                wb_fpu_wr           <= tc_fpu_wr;
                wb_fpu_wr_word      <= tc_fpu_wr_word;
                wb_fpu_fd           <= tc_fpu_fd;
                wb_fpu_result       <= tc_fpu_result;
                wb_fpu_exc_flags    <= tc_fpu_exc_flags;
                wb_fpu_exc_unimpl   <= tc_fpu_exc_unimpl;
                wb_fpu_cmp          <= tc_fpu_cmp;
                wb_fpu_cmp_result   <= tc_fpu_cmp_result;
                wb_fpu_cmp_cc       <= tc_fpu_cmp_cc;
                wb_fpu_mtc1         <= tc_fpu_mtc1;
                wb_fpu_ctc1         <= tc_fpu_ctc1;
                wb_fpu_ctc1_data    <= tc_fpu_ctc1_data;
                wb_fpu_early_wr_done    <= tc_fpu_early_wr_done;
                wb_fpu_early_fcsr_done  <= tc_fpu_early_fcsr_done;
                wb_fpu_lwc1         <= tc_fpu_lwc1;
                wb_fpu_ldc1         <= tc_fpu_ldc1;
                wb_fpu_fr_mode      <= tc_fpu_fr_mode;
                wb_load_data        <= tc_load_data;
                wb_store_mask       <= tc_store_mask;
                wb_store_data       <= tc_store_data;
                wb_wb_sel           <= tc_wb_sel;
                wb_ll_snoop_killed  <= tc_ll_snoop_killed ||
                                       snoop_hits_tc_ll;
            end else begin
                wb_valid <= 1'b0;
                wb_ll_snoop_killed <= 1'b0;
                wb_device_side_effect <= 1'b0;
                wb_cached_store_visible <= 1'b0;
                wb_deferred_mem_irq <= 1'b0;
            end

            // -------------------------------------------------------
            // WB stage writeback (COMMIT point)
            // -------------------------------------------------------
            // Debug: removed

            if (wb_valid && wb_take_exception) begin
                // Exception entry
                if (!cp0_status[1]) begin
                    if (wb_in_delay_slot) begin
                        cp0_epc <= wb_pc - 64'd4;
                        cp0_cause[31] <= 1'b1;
                    end else begin
                        cp0_epc <= wb_pc;
                        cp0_cause[31] <= 1'b0;
                    end
                end
                cp0_cause[6:2] <= wb_exc_code;
                if (wb_exc_cpu) cp0_cause[29:28] <= wb_exc_cpu_ce;
                cp0_status[1] <= 1'b1;

                // Telemetry: log interrupt exceptions with pending IP bits
`ifndef SYNTHESIS
                if (wb_exc_int && $test$plusargs("trace_cpu_int"))
                    $display("CPU_INT: IP=%02x PC=%08x",
                        wb_int_pending, wb_pc[31:0]);
`endif

                if (wb_exc_iadel)
                    cp0_badvaddr <= wb_pc;
                else if (wb_exc_adel || wb_exc_ades)
                    cp0_badvaddr <= wb_eff_addr;
                else if (wb_exc_itlb)
                    cp0_badvaddr <= wb_pc;
                else if (wb_exc_dtlb)
                    cp0_badvaddr <= wb_eff_addr;

                if (wb_exc_itlb) begin
                    cp0_context[22:4] <= wb_pc[31:13];
                    cp0_xcontext[31:4] <= {1'b0, wb_pc[39:13]};
                    cp0_xcontext[33:32] <= wb_pc[63:62];
                    cp0_entryhi[63:13] <= wb_pc[63:13];
                end else if (wb_exc_dtlb) begin
                    cp0_context[22:4] <= wb_eff_addr[31:13];
                    cp0_xcontext[31:4] <= {1'b0, wb_eff_addr[39:13]};
                    cp0_xcontext[33:32] <= wb_eff_addr[63:62];
                    cp0_entryhi[63:13] <= wb_eff_addr[63:13];
                end

                llbit <= 1'b0; // Exception clears llbit

            end else if (wb_valid && wb_do_eret) begin
                if (cp0_status[2])
                    cp0_status[2] <= 1'b0;
                else
                    cp0_status[1] <= 1'b0;
                llbit <= 1'b0;

            end else if (wb_valid) begin
                // Normal execution
                if (wb_wb_reg != 5'd0)
                    gpr[wb_wb_reg] <= wb_wb_result;

                // FPU register writes at commit
                // (fpu_wen is driven combinationally below)

                if (wb_wr_hi) hi_reg <= wb_hi_wdata;
                if (wb_wr_lo) lo_reg <= wb_lo_wdata;

                if (wb_opcode == `OP_LL || wb_opcode == `OP_LLD) begin
                    llbit <= 1'b1;
                    ll_addr <= wb_eff_addr;
                    ll_paddr <= wb_paddr;      // CC-2: track physical line for DMA snoop
                    cp0_lladdr <= wb_eff_addr;
                end else if (wb_do_store && (wb_eff_addr[63:7] == ll_addr[63:7]))
                    // CC-1: clear llbit only on store to the LL'd cache line
                    // (128 B granularity). See RTL_CANONICAL_CORRECTNESS.md.
                    llbit <= 1'b0;

                // CP0 writes
                if (wb_is_mtc0 || wb_is_dmtc0) begin
                    case (wb_rd_idx)
                        `CP0_INDEX:    cp0_index    <= wb_rt_val;
                        `CP0_ENTRYLO0: cp0_entrylo0 <= wb_rt_val;
                        `CP0_ENTRYLO1: cp0_entrylo1 <= wb_rt_val;
                        `CP0_CONTEXT:  cp0_context  <= wb_rt_val;
                        `CP0_PAGEMASK: begin
                            cp0_pagemask <= wb_rt_val;
                            cp0_pagemask_valid <= cp0_pagemask_bits_valid(wb_rt_val[24:13]);
                        end
                        `CP0_WIRED: begin
                            cp0_wired  <= {58'd0, wb_rt_val[5:0]};
                            cp0_random <= 64'd47;
                        end
                        `CP0_ENTRYHI:  cp0_entryhi  <= wb_rt_val;
                        // Count is written below, where the architectural
                        // write has priority over its free-running divider.
                        `CP0_COUNT:    ;
                        `CP0_COMPARE: begin
                            cp0_compare  <= wb_rt_val;
                            $display("CP0_COMPARE: set to 0x%08x (count=0x%08x delta=%0d)",
                                wb_rt_val[31:0], cp0_count[31:0], wb_rt_val[31:0] - cp0_count[31:0]);
                        end
                        `CP0_STATUS:   cp0_status   <= wb_rt_val;
                        `CP0_CAUSE:    cp0_cause[9:8] <= wb_rt_val[9:8];
                        `CP0_EPC:      cp0_epc      <= wb_rt_val;
                        `CP0_CONFIG:   cp0_config[2:0] <= wb_rt_val[2:0]; // Only K0 is writable
                        `CP0_LLADDR: begin
                            cp0_lladdr <= wb_rt_val;
                            // CC-7: MTC0 LLADDR is used by kernel context
                            // restore; keep ll_addr/ll_paddr in sync so a
                            // subsequent SC compares against the restored
                            // line. Does NOT set llbit — that stays per
                            // the LL/SC protocol.
                            ll_addr  <= wb_rt_val;
                            ll_paddr <= wb_rt_val[31:0];
                        end
                        `CP0_WATCHLO:  cp0_watchlo  <= wb_rt_val;
                        `CP0_WATCHHI:  cp0_watchhi  <= wb_rt_val;
                        `CP0_XCONTEXT: cp0_xcontext <= wb_rt_val;
                        `CP0_ECC:      cp0_ecc      <= wb_rt_val;
                        `CP0_TAGLO:    cp0_taglo    <= wb_rt_val;
                        `CP0_TAGHI:    cp0_taghi    <= wb_rt_val;
                        `CP0_ERROREPC: cp0_errorepc <= wb_rt_val;
                        default: ;
                    endcase
                end

                // TLB operations (suppressed on Machine Check)
`ifdef SYNTHESIS
                // Synthesis: TLBWI/TLBWR commit through u_tlb's combinational
                // write port (driven from tlbw_en/tlbw_widx/tlbw_w*). Only the
                // CP0-register-side effects of TLBR/TLBP are handled here.
                if (wb_is_cop0_co && !wb_exc_mcheck) begin
                    case (wb_cop0_funct)
                        `C0FN_TLBR: begin
                            cp0_entryhi  <= tlb_read_hi;
                            cp0_entrylo0 <= {34'd0, tlb_read_lo0[29:0]};
                            cp0_entrylo1 <= {34'd0, tlb_read_lo1[29:0]};
                            cp0_pagemask <= tlb_read_mask;
                            cp0_pagemask_valid <= cp0_pagemask_bits_valid(tlb_read_mask[24:13]);
                        end
                        `C0FN_TLBP: begin
                            cp0_index <= tlb_probe_match ? {58'd0, tlb_probe_idx}
                                                         : 64'h0000_0000_8000_0000;
                        end
                        default: ;
                    endcase
                end
`else
                if (wb_is_cop0_co && !wb_exc_mcheck) begin
                    case (wb_cop0_funct)
                        `C0FN_TLBWI: begin
                            tlb_hi  [cp0_index[5:0]] <= cp0_entryhi;
                            tlb_lo0 [cp0_index[5:0]] <= {34'd0, cp0_entrylo0[29:1], cp0_entrylo0[0] & cp0_entrylo1[0]};
                            tlb_lo1 [cp0_index[5:0]] <= {34'd0, cp0_entrylo1[29:1], cp0_entrylo0[0] & cp0_entrylo1[0]};
                            tlb_mask[cp0_index[5:0]] <= cp0_pagemask;
                            // Keep array observable for simulator.
                            if (cp0_index[5:0] == 6'd63) $write("");
                        end
                        `C0FN_TLBWR: begin
                            tlb_hi  [tlb_random_idx] <= cp0_entryhi;
                            tlb_lo0 [tlb_random_idx] <= {34'd0, cp0_entrylo0[29:1], cp0_entrylo0[0] & cp0_entrylo1[0]};
                            tlb_lo1 [tlb_random_idx] <= {34'd0, cp0_entrylo1[29:1], cp0_entrylo0[0] & cp0_entrylo1[0]};
                            tlb_mask[tlb_random_idx] <= cp0_pagemask;
                            if (cp0_random[5:0] == 6'd63) $write("");
                        end
                        `C0FN_TLBR: begin
                            cp0_entryhi  <= tlb_hi  [cp0_index[5:0]];
                            // Return only meaningful bits [29:0] of EntryLo
                            cp0_entrylo0 <= {34'd0, tlb_lo0[cp0_index[5:0]][29:0]};
                            cp0_entrylo1 <= {34'd0, tlb_lo1[cp0_index[5:0]][29:0]};
                            cp0_pagemask <= tlb_mask[cp0_index[5:0]];
                            cp0_pagemask_valid <= cp0_pagemask_bits_valid(tlb_mask[cp0_index[5:0]][24:13]);
                        end
                        `C0FN_TLBP: begin
                            cp0_index <= 64'h0000_0000_8000_0000;
                        end
                        default: ;
                    endcase
                end
`endif
            end

            // Muldiv completion — writes HI/LO directly (not through pipeline)
            // Lower priority than WB MTHI/MTLO writes
            if (md_done) begin
                if (!(wb_valid && !wb_take_exception && !wb_do_eret && wb_wr_hi))
                    hi_reg <= md_hi;
                if (!(wb_valid && !wb_take_exception && !wb_do_eret && wb_wr_lo))
                    lo_reg <= md_lo;
            end

            // TLBP search (cosim only; SYNTHESIS handles TLBP via u_tlb.probe_*)
`ifndef SYNTHESIS
            if (wb_valid && !wb_take_exception && !wb_do_eret &&
                wb_is_cop0_co && wb_cop0_funct == `C0FN_TLBP) begin
                for (i = 0; i < 48; i = i + 1) begin
                    if (((cp0_entryhi[39:13] ^ tlb_hi[i][39:13]) & ~tlb_mask[i][39:13]) == 27'd0 &&
                        cp0_entryhi[63:62] == tlb_hi[i][63:62] &&
                        ((tlb_lo0[i][0] & tlb_lo1[i][0]) ||
                         (tlb_hi[i][7:0] == cp0_entryhi[7:0])))
                        cp0_index <= {58'd0, i[5:0]};
                end
            end
`endif

            // Track ERET for 1-cycle watch suppression
            just_did_eret <= wb_valid && wb_do_eret;

            // Update PC
            if (pc_redirect || !pc_hold)
                pc <= next_pc;
            if (rf_control_flow_debug_track) begin
                last_cf_pc     <= rf_pc[31:0];
                last_cf_target <= rf_do_jump ? rf_jump_target[31:0] :
                                               rf_branch_target[31:0];
            end
            // R4000 Count advances at half the maximum instruction-issue
            // rate, independent of pipeline progress.  A committed Count
            // write must win over both this divider and the simulation-only
            // idle fast-forward path.
            cp0_count_phase <= ~cp0_count_phase;
            if (wb_valid && !wb_take_exception && !wb_do_eret &&
                (wb_is_mtc0 || wb_is_dmtc0) && wb_rd_idx == `CP0_COUNT)
                cp0_count <= wb_rt_val;
            // sim_idle_boost skips idle Count→Compare waits.  Only skip when
            // the gap is a normal timer interval (< 2^31), not a full-counter
            // wrap wait (Compare=0, delta near 2^32).
            else if (sim_idle_boost &&
                (cp0_compare[31:0] - cp0_count[31:0]) > 32'd1000 &&
                (cp0_compare[31:0] - cp0_count[31:0]) < 32'h80000000)
                cp0_count <= {32'd0, cp0_compare[31:0] - 32'd16};
            else if (cp0_count_phase)
                cp0_count <= cp0_count + 64'd1;

`ifndef FPGA_NO_INTERRUPTS
            // Hardware interrupt pins → Cause IP[6:2] (level-sensitive)
            cp0_cause[14:10] <= hw_int[4:0];

            // Timer interrupt: IP[7] (Cause bit 15) — internal Count==Compare only
            if (cp0_count[31:0] == cp0_compare[31:0])
                cp0_cause[15] <= 1'b1;
`else
            cp0_cause[15:10] <= 6'd0;
`endif

            // Compare write clears IP[7] (takes priority over Count==Compare match)
            if (wb_valid && !wb_take_exception && !wb_do_eret &&
                (wb_is_mtc0 || wb_is_dmtc0) && wb_rd_idx == `CP0_COMPARE)
                cp0_cause[15] <= 1'b0;

            // CACHE Index Load Tag → update cp0_taglo from cache readback.
            // FPGA S-cache metadata is synchronous, so its completed result
            // can arrive after the CACHE instruction has left WB.
            if (cache_cop_taglo_update_valid)
                cp0_taglo <= {32'd0, cache_cop_taglo_update_data};
            else if (wb_cache_commit && wb_cache_op == 3'd1)
                cp0_taglo <= {32'd0, cache_cop_taglo_in};

            // Random is the replacement pointer consumed by TLBWR. Advancing
            // it on committed replacement, not every pipeline clock, avoids
            // refill-handler phase lock in this non-cycle-accurate core while
            // preserving the wired/random partition.
            if (wb_valid && !wb_take_exception && !wb_do_eret &&
                wb_is_cop0_co && wb_cop0_funct == `C0FN_TLBWR)
                cp0_random <= {58'd0, tlb_random_next};

            // MFI outputs — generated from WB stage. The FPGA wrapper does not
            // observe MFI, so the DE2 build ties it off before synthesis.
`ifdef FPGA_NO_MFI
            mfi_valid     <= 1'b0;
            mfi_order     <= 64'd0;
            mfi_pc        <= 64'd0;
            mfi_insn      <= 32'd0;
            mfi_trap      <= 1'b0;
            mfi_rd_addr   <= 5'd0;
            mfi_rd_wdata  <= 64'd0;
            mfi_hilo_wen  <= 2'b00;
            mfi_hi_wdata  <= 64'd0;
            mfi_lo_wdata  <= 64'd0;
            mfi_mem_addr  <= 64'd0;
            mfi_mem_rmask <= 8'h00;
            mfi_mem_wmask <= 8'h00;
            mfi_mem_rdata <= 64'd0;
            mfi_mem_wdata <= 64'd0;
`else
            if (wb_valid) begin
                mfi_valid    <= 1'b1;
                mfi_pc       <= wb_pc;
                mfi_insn     <= wb_insn;
                mfi_order    <= insn_count;
                mfi_trap     <= wb_take_exception;
                mfi_rd_addr  <= wb_take_exception ? 5'd0 : wb_wb_reg;
                mfi_rd_wdata <= wb_wb_result;
                // HI/LO MFI: only report MTHI/MTLO from pipeline
                // md_done writes are NOT reported here (they happen asynchronously)
                mfi_hilo_wen <= wb_take_exception ? 2'b00 : {wb_wr_hi, wb_wr_lo};
                mfi_hi_wdata <= wb_wr_hi ? wb_hi_wdata : 64'd0;
                mfi_lo_wdata <= wb_wr_lo ? wb_lo_wdata : 64'd0;
                mfi_mem_addr <= wb_eff_addr;
                mfi_mem_rmask <= (wb_do_load && !wb_take_exception) ? 8'hFF : 8'h00;
                mfi_mem_wmask <= (wb_do_store && !wb_take_exception &&
                                  (!wb_sc_insn || wb_sc_success)) ?
                                 wb_store_mask : 8'h00;
                mfi_mem_rdata <= wb_load_data;
                mfi_mem_wdata <= wb_store_data;

                insn_count <= insn_count + 64'd1;

            end else begin
                mfi_valid <= 1'b0;
            end
`endif

            // CC-2: a DMA write to the LL'd 128-byte S-cache line clears the
            // reservation.  Cover both an already committed LL and an LL
            // committing on this edge.  The DS/TC/WB poison bits carry any
            // one-cycle snoop that arrived while the LL was still in flight.
            if (snoop_hits_committed_ll || snoop_hits_wb_ll ||
                (wb_has_ll && wb_ll_snoop_killed))
                llbit <= 1'b0;

            // CC-3: CACHE instruction targeting the LL'd line clears llbit.
            // Per R4000 UM, any CACHE op that invalidates or writes back the
            // LL'd cache line must clear LLbit. We apply this to D$ (type=1)
            // and S$ (type[1]), at 128 B (L2 line) granularity. Exclude op 1
            // (Index_Load_Tag) since it is a pure read. I-cache ops (type=0)
            // are excluded — LL/SC is defined on data, not instructions.
            if (wb_cache_commit && wb_cache_op != 3'd1 && wb_cache_type != 2'd0 &&
                llbit && (wb_paddr[31:7] == ll_paddr[31:7]))
                llbit <= 1'b0;

            // CC-4: S-cache line eviction of the LL'd line clears llbit.
            // The S-cache raises evict_valid/evict_addr on ANY valid-line
            // replacement (conflict miss, fill, CACHE invalidate). If the
            // evicted line backs our ll_paddr (128 B granularity), the SC
            // must fail — the data we LL'd is no longer cached.
            if (evict_valid && llbit && (evict_addr[31:7] == ll_paddr[31:7]))
                llbit <= 1'b0;
        end
    end

    // ---------------------------------------------------------------
    // FPU write control — combinational, driven by WB stage
    // ---------------------------------------------------------------
    // CACHE instruction commit
    wire wb_cache_commit = wb_valid && !wb_take_exception && !wb_do_eret && wb_is_cache;
    assign retirement_tail_empty = !tc_valid && !wb_valid;
    assign cache_cop_valid = wb_cache_commit;
    assign cache_cop_op    = wb_cache_op;
    assign cache_cop_type  = wb_cache_type;
    assign cache_cop_paddr = wb_paddr;
    assign cache_cop_taglo_out = cp0_taglo[31:0]; // for Index Store Tag

    wire wb_fpu_commit = wb_valid && !wb_take_exception && !wb_do_eret;

    always @(*) begin
        fpu_wen = 1'b0;
        fpu_fcsr_wen = 1'b0;
        fpu_fcsr_wdata = fpu_fcsr_rdata;
        fpu_fd_wr = wb_fpu_fd;
        fpu_fpr_wdata = wb_fpu_result;
        fpu_fpr_wdata_word = wb_fpu_wr_word;

        if (wb_valid && wb_exc_fpe) begin
            // Trapping FPU exception: write cause bits only, not sticky flags/FPR.
            fpu_fcsr_wen = 1'b1;
            fpu_fcsr_wdata[17:12] = wb_fpu_exc_unimpl ?
                                     6'b100000 :
                                     {1'b0, wb_fpu_exc_flags};
        end else if (wb_fpu_commit) begin
            if (wb_fpu_cmp) begin
                fpu_fcsr_wen = 1'b1;
                fpu_fcsr_wdata = fcsr_set_fcc(fpu_fcsr_rdata, wb_fpu_cmp_cc,
                                               wb_fpu_cmp_result);
            end else if (wb_fpu_ctc1) begin
                fpu_fcsr_wen = 1'b1;
                case (wb_insn[15:11])
                    5'd25: begin // FCCR - FCC bits
                        fpu_fcsr_wdata[31:25] = wb_fpu_ctc1_data[7:1];
                        fpu_fcsr_wdata[23] = wb_fpu_ctc1_data[0];
                    end
                    5'd26: begin // FEXR - exception flags
                        fpu_fcsr_wdata[17:12] = wb_fpu_ctc1_data[17:12];
                        fpu_fcsr_wdata[6:2] = wb_fpu_ctc1_data[6:2];
                    end
                    5'd28: begin // FENR - enables + FS + RM
                        fpu_fcsr_wdata[11:7] = wb_fpu_ctc1_data[11:7];
                        fpu_fcsr_wdata[24] = wb_fpu_ctc1_data[2];
                        fpu_fcsr_wdata[1:0] = wb_fpu_ctc1_data[1:0];
                    end
                    default: // 31 = FCSR - full register
                        fpu_fcsr_wdata = wb_fpu_ctc1_data;
                endcase
            end else if (wb_fpu_wr && wb_fpu_exc_flags != 5'd0) begin
                fpu_fcsr_wen = 1'b1;
                fpu_fcsr_wdata[6:2] = fpu_fcsr_rdata[6:2] | wb_fpu_exc_flags;
                fpu_fcsr_wdata[16:12] = wb_fpu_exc_flags;
            end
        end

        if (wb_fpu_commit) begin
            // FPR late write: only when early write was suppressed (pipeline exception)
            if ((wb_fpu_wr || wb_fpu_mtc1) && !wb_fpu_early_wr_done) begin
                fpu_wen = 1'b1;
            end

            // LWC1/LDC1: always write at WB (data not available until WB)
            if (wb_fpu_lwc1) begin
                fpu_wen = 1'b1;
                fpu_fd_wr = wb_insn[20:16]; // ft field for LWC1
                fpu_fpr_wdata = wb_load_data;
                fpu_fpr_wdata_word = 1'b1;
            end

            if (wb_fpu_ldc1) begin
                fpu_wen = 1'b1;
                fpu_fd_wr = wb_insn[20:16]; // ft field for LDC1
                fpu_fpr_wdata = wb_load_data;
                fpu_fpr_wdata_word = 1'b0;
            end

        end
    end

    assign debug_pc       = pc[31:0];
    assign debug_is_pc    = is_pc[31:0];
    assign debug_rf_pc    = rf_pc[31:0];
    assign debug_rf_insn  = rf_insn;
    assign debug_wb_pc    = wb_pc[31:0];
    assign debug_wb_insn  = wb_insn;
    assign debug_epc      = cp0_epc[31:0];
    assign debug_cause    = cp0_cause[31:0];
    assign debug_badvaddr = cp0_badvaddr[31:0];
    assign debug_status   = cp0_status[31:0];
    assign debug_context  = cp0_context[31:0];
    assign debug_entryhi  = cp0_entryhi[31:0];
    assign debug_gpr_v0   = gpr[2][31:0];
    assign debug_gpr_v1   = gpr[3][31:0];
    assign debug_gpr_at   = gpr[1][31:0];
    assign debug_gpr_t0   = gpr[8][31:0];
    assign debug_gpr_t1   = gpr[9][31:0];
    assign debug_gpr_a0   = gpr[4][31:0];
    assign debug_gpr_a1   = gpr[5][31:0];
    assign debug_gpr_a2   = gpr[6][31:0];
    assign debug_gpr_t8   = gpr[24][31:0];
    assign debug_gpr_s4   = gpr[20][31:0];
    assign debug_gpr_s5   = gpr[21][31:0];
    assign debug_gpr_s6   = gpr[22][31:0];
    assign debug_gpr_t2   = gpr[10][31:0];
    assign debug_gpr_t3   = gpr[11][31:0];
    assign debug_gpr_a3   = gpr[7][31:0];
    assign debug_gpr_s0   = gpr[16][31:0];
    assign debug_gpr_s1   = gpr[17][31:0];
    assign debug_gpr_s2   = gpr[18][31:0];
    assign debug_gpr_s3   = gpr[19][31:0];
    assign debug_gpr_t4   = gpr[12][31:0];
    assign debug_gpr_t6   = gpr[14][31:0];
    assign debug_gpr_t7   = gpr[15][31:0];
    assign debug_gpr_t9   = gpr[25][31:0];
    assign debug_gpr_sp   = gpr[29][31:0];
    assign debug_gpr_ra   = gpr[31][31:0];
    assign debug_last_cf_pc     = last_cf_pc;
    assign debug_last_cf_target = last_cf_target;
    assign debug_wb_eff_addr = wb_eff_addr[31:0];
    assign debug_wb_load_data = wb_load_data[31:0];
    assign debug_dmem_pc = ds_pc[31:0];
    assign debug_dmem_insn = ds_insn;
    assign debug_dmem_vaddr = ds_eff_addr[31:0];
    assign debug_dtlb_paddr = ds_paddr;
    assign debug_dtlb_lo = ds_dtlb_lo_sel[31:0];
    assign debug_dtlb_mask = ds_dtlb_mask[31:0];
    assign debug_dtlb_state = {
        4'hD,
        ds_valid,
        ds_do_load,
        ds_do_store,
        ds_suppress_mem,
        ds_uncached,
        ds_exc_dtlb,
        ds_dtlb_miss,
        ds_dtlb_invalid,
        ds_dtlb_mod,
        dmem_ren,
        dmem_wen,
        ds_load_acked,
        ds_store_acked,
        flush_all,
        cache_stall,
        is_mapped(ds_eff_addr),
        addr_segment(ds_eff_addr),
        2'd0,
        ds_dtlb_match_valid,
        ds_dtlb_match_idx
    };
    assign debug_tlbwr_meta = {
        8'h01,
        debug_tlbwr_valid_q,
        debug_tlbwr_random_q,
        debug_tlbwr_idx_q,
        debug_tlbwr_random_idx_q,
        debug_tlbwr_wired_q,
        4'd0
    };
    assign debug_tlbwr_hi    = debug_tlbwr_hi_q;
    assign debug_tlbwr_lo0   = debug_tlbwr_lo0_q;
    assign debug_tlbwr_lo1   = debug_tlbwr_lo1_q;
    assign debug_tlbwr_mask  = debug_tlbwr_mask_q;
`ifdef SYNTHESIS
    assign debug_tlb_hi    = tlb_debug_hi[31:0];
    assign debug_tlb_lo0   = tlb_debug_lo0[31:0];
    assign debug_tlb_lo1   = tlb_debug_lo1[31:0];
    assign debug_tlb_mask  = tlb_debug_mask[31:0];
`else
    assign debug_tlb_hi    = tlb_hi[debug_tlb_idx][31:0];
    assign debug_tlb_lo0   = tlb_lo0[debug_tlb_idx][31:0];
    assign debug_tlb_lo1   = tlb_lo1[debug_tlb_idx][31:0];
    assign debug_tlb_mask  = tlb_mask[debug_tlb_idx][31:0];
`endif
    assign debug_flags = {
        4'h0,
        cache_stall,
        stall_if,
        stall_rf,
        flush_all,
        is_valid,
        rf_valid,
        ex_valid,
        df_valid,
        ds_valid,
        tc_valid,
        wb_valid,
        wb_take_exception,
        wb_do_eret,
        wb_exc_code,
        imem_uncached,
        dmem_uncached,
        dmem_ren,
        dmem_wen,
        wb_do_load,
        wb_do_store,
        wb_exc_itlb,
        wb_dtlb_miss,
        wb_dtlb_invalid,
        wb_dtlb_mod
    };

endmodule
