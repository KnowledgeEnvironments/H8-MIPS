// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_top.v — Top-level R4000SC wrapper with L1 + L2 caches
// Core → L1 I$/D$ → L2 S-cache → external memory
// S-cache is transparent: on hit, zero extra latency.
// On S-cache miss, L1 is frozen while S-cache fills from external memory.

`default_nettype none

module r4000_top #(
    parameter SCACHE_LINES = 1024,
    parameter FULL_ICACHE_INCLUSION = 0,
    // The registered I-cache lookup overlap is retained for focused CPU
    // evaluation, but a board integration may disable it without removing
    // the cache island.  Disabled is cycle-identical to the admitted e1
    // lookup path: tag/data RAMs are addressed by the current IS request.
    parameter ICACHE_LOOKAHEAD = 1
) (
    input  wire        clk,
    input  wire        rst_n,

    // Physical instruction memory interface (to testbench/SoC)
    output wire [31:0] imem_paddr,
    output wire        imem_ren,
    input  wire [31:0] imem_rdata,

    // Physical data memory interface (to testbench/SoC)
    output wire        dmem_ren,
    output wire        dmem_wen,
    output wire [31:0] dmem_paddr,
    output wire [ 7:0] dmem_wmask,
    output wire [63:0] dmem_wdata,
    input  wire [63:0] dmem_rdata,
    input  wire        dmem_ready,     // ext memory response valid (for FPGA/multi-cycle)
    output wire        dmem_uncached_wr, // ext_dmem_wen is from uncached store (not writeback)
    output wire        dmem_cache_owned, // S-cache fill/writeback owns external port

    // MFI passthrough
    output wire        mfi_valid,
    output wire [63:0] mfi_order,
    output wire [63:0] mfi_pc,
    output wire [31:0] mfi_insn,
    output wire        mfi_trap,
    output wire [ 4:0] mfi_rd_addr,
    output wire [63:0] mfi_rd_wdata,
    output wire [ 1:0] mfi_hilo_wen,
    output wire [63:0] mfi_hi_wdata,
    output wire [63:0] mfi_lo_wdata,
    output wire [63:0] mfi_mem_addr,
    output wire [ 7:0] mfi_mem_rmask,
    output wire [ 7:0] mfi_mem_wmask,
    output wire [63:0] mfi_mem_rdata,
    output wire [63:0] mfi_mem_wdata,

    // Hardware interrupts (active-high, directly mapped to Cause IP[6:2])
    input  wire [5:0]  hw_int,

    // Simulation: skip Count forward to trigger timer interrupt sooner
    input  wire        sim_idle_boost,

    // Diagnostic: force every CPU instruction and data access to be uncached
    // (bypass I$/D$/S$).  Select this before reset and leave it asserted for
    // the whole run so the diagnostic does not change cache mode in flight.
    input  wire        force_uncached,

    // DMA snoop port (for S-cache coherency with bus-master DMA)
    input  wire        snoop_valid,
    input  wire [31:0] snoop_addr,
    input  wire [63:0] snoop_wdata,
    input  wire [ 7:0] snoop_wmask,
    output wire        snoop_wr_ready,
    output wire        snoop_wr_done,

    // Stop architectural retirement for a DMA write coherence window while
    // allowing cache controllers and older external transactions to drain.
    input  wire        external_hold,
    output wire        external_hold_quiesced,

    // DMA read snoop port (MC snoops S-cache on bus-master reads)
    input  wire        snoop_rd_valid,
    input  wire [31:0] snoop_rd_addr,
    output wire        snoop_rd_resp_valid,
    output wire        snoop_rd_hit,
    output wire [63:0] snoop_rd_data,

    output wire [31:0] debug_core_pc,
    output wire [31:0] debug_core_is_pc,
    output wire [31:0] debug_core_rf_pc,
    output wire [31:0] debug_core_rf_insn,
    output wire [31:0] debug_core_wb_pc,
    output wire [31:0] debug_core_wb_insn,
    output wire [31:0] debug_core_epc,
    output wire [31:0] debug_core_cause,
    output wire [31:0] debug_core_badvaddr,
    output wire [31:0] debug_core_status,
    output wire [31:0] debug_core_context,
    output wire [31:0] debug_core_entryhi,
    output wire [31:0] debug_core_flags,
    output wire [31:0] debug_core_gpr_v0,
    output wire [31:0] debug_core_gpr_v1,
    output wire [31:0] debug_core_gpr_at,
    output wire [31:0] debug_core_gpr_t0,
    output wire [31:0] debug_core_gpr_t1,
    output wire [31:0] debug_core_gpr_a0,
    output wire [31:0] debug_core_gpr_a1,
    output wire [31:0] debug_core_gpr_a2,
    output wire [31:0] debug_core_gpr_t8,
    output wire [31:0] debug_core_gpr_s4,
    output wire [31:0] debug_core_gpr_s5,
    output wire [31:0] debug_core_gpr_s6,
    output wire [31:0] debug_core_gpr_t2,
    output wire [31:0] debug_core_gpr_t3,
    output wire [31:0] debug_core_gpr_a3,
    output wire [31:0] debug_core_gpr_s0,
    output wire [31:0] debug_core_gpr_s1,
    output wire [31:0] debug_core_gpr_s2,
    output wire [31:0] debug_core_gpr_s3,
    output wire [31:0] debug_core_gpr_t4,
    output wire [31:0] debug_core_gpr_t6,
    output wire [31:0] debug_core_gpr_t7,
    output wire [31:0] debug_core_gpr_t9,
    output wire [31:0] debug_core_gpr_sp,
    output wire [31:0] debug_core_gpr_ra,
    output wire [31:0] debug_core_last_cf_pc,
    output wire [31:0] debug_core_last_cf_target,
    output wire [31:0] debug_core_wb_eff_addr,
    output wire [31:0] debug_core_wb_load_data,
    output wire [31:0] debug_cache_status,
    output wire [31:0] debug_cache_addr,
    output wire [383:0] debug_scache_probe,
    output wire [255:0] debug_icache_capture,
    output wire        debug_core_store_commit,
    output wire [31:0] debug_core_store_pc,
    output wire [31:0] debug_core_store_insn,
    output wire [31:0] debug_core_store_paddr,
    output wire [31:0] debug_core_store_vaddr,
    output wire [ 7:0] debug_core_store_wmask,
    output wire [63:0] debug_core_store_wdata,
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
    // Core ↔ L1 cache wires
    // ---------------------------------------------------------------
    wire [63:0] core_imem_addr;
    wire        core_imem_valid;
    wire [63:0] core_imem_fetch_addr;
    wire        core_imem_fetch_advance;
    wire        core_imem_uncached;
    wire        effective_imem_uncached /* verilator public_flat_rd */ =
                    core_imem_uncached | force_uncached;
    wire [31:0] icache_to_core_rdata;
    wire        icache_stall /* verilator public_flat_rd */;

    wire        core_dmem_ren;
    wire        core_dmem_wen;
    wire [63:0] core_dmem_addr;
    wire [63:0] core_dmem_lookup_addr;
    wire        core_dmem_lookup_advance;
    wire [ 7:0] core_dmem_wmask;
    wire [63:0] core_dmem_wdata;
    wire        core_dmem_uncached;
    wire [63:0] dcache_to_core_rdata;
    wire        dcache_stall /* verilator public_flat_rd */;
    wire        dcache_store_visible;
    wire [31:0] core_debug_dmem_pc;
    wire [31:0] core_debug_dmem_insn;

    wire        scache_stall /* verilator public_flat_rd */;
    wire        scache_uncached_started;
    wire        scache_uncached_accepted;
    wire        sc_backinv_valid;
    wire [31:0] sc_backinv_addr;
    wire        sc_backinv_cacheop /* verilator public_flat_rd */;
    wire        sc_backinv_icache /* verilator public_flat_rd */;
    wire        sc_backinv_writeback;
    wire        sc_backinv_ready;
    wire        dcache_backinv_done;
`ifdef FPGA_CORE_ONLY
    wire        cache_stall_raw /* verilator public_flat_rd */ =
        (core_dmem_ren || core_dmem_wen) && !dmem_ready;
`else
    wire        cache_stall_raw /* verilator public_flat_rd */ =
        icache_stall || dcache_stall || scache_stall;
`endif
    wire        cache_stall = cache_stall_raw || external_hold;
    wire        snoop_commit = snoop_valid && snoop_wr_ready;
    // A coherence hold freezes younger architectural work in the core while
    // cache controllers continue to drain.  Ack only after no cache/CACHE-op
    // or external-memory side effect from the older boundary remains.
    assign external_hold_quiesced = external_hold &&
                                     core_retirement_tail_empty &&
                                     !cache_stall_raw &&
                                     !dmem_ren && !dmem_wen &&
                                     !core_cop_valid;

    // CACHE instruction signals from core
    wire        core_cop_valid;
    wire        core_retirement_tail_empty;
    wire [ 2:0] core_cop_op;
    wire [ 1:0] core_cop_type;
    wire [31:0] core_cop_paddr;
    wire [31:0] core_cop_taglo;     // TagLo value (for Index Store Tag)
    wire [31:0] scache_cop_taglo;   // S-cache tag readback (for Index Load Tag)
    wire        scache_cop_taglo_valid;
    wire [31:0] icache_cop_taglo;   // I-cache tag readback
    wire        icache_cop_taglo_valid;
    wire [31:0] dcache_cop_taglo;   // D-cache tag readback

    wire        core_mfi_valid;
    wire [63:0] core_mfi_order;
    wire [63:0] core_mfi_pc;
    wire [31:0] core_mfi_insn;
    wire        core_mfi_trap;
    wire [ 4:0] core_mfi_rd_addr;
    wire [63:0] core_mfi_rd_wdata;
    wire [ 1:0] core_mfi_hilo_wen;
    wire [63:0] core_mfi_hi_wdata;
    wire [63:0] core_mfi_lo_wdata;
    wire [63:0] core_mfi_mem_addr;
    wire [ 7:0] core_mfi_mem_rmask;
    wire [ 7:0] core_mfi_mem_wmask;
    wire [63:0] core_mfi_mem_rdata;
    wire [63:0] core_mfi_mem_wdata;

`ifdef FPGA_NO_MFI
    assign mfi_valid     = 1'b0;
    assign mfi_order     = 64'd0;
    assign mfi_pc        = 64'd0;
    assign mfi_insn      = 32'd0;
    assign mfi_trap      = 1'b0;
    assign mfi_rd_addr   = 5'd0;
    assign mfi_rd_wdata  = 64'd0;
    assign mfi_hilo_wen  = 2'b00;
    assign mfi_hi_wdata  = 64'd0;
    assign mfi_lo_wdata  = 64'd0;
    assign mfi_mem_addr  = 64'd0;
    assign mfi_mem_rmask = 8'h00;
    assign mfi_mem_wmask = 8'h00;
    assign mfi_mem_rdata = 64'd0;
    assign mfi_mem_wdata = 64'd0;
`else
    assign mfi_valid     = core_mfi_valid;
    assign mfi_order     = core_mfi_order;
    assign mfi_pc        = core_mfi_pc;
    assign mfi_insn      = core_mfi_insn;
    assign mfi_trap      = core_mfi_trap;
    assign mfi_rd_addr   = core_mfi_rd_addr;
    assign mfi_rd_wdata  = core_mfi_rd_wdata;
    assign mfi_hilo_wen  = core_mfi_hilo_wen;
    assign mfi_hi_wdata  = core_mfi_hi_wdata;
    assign mfi_lo_wdata  = core_mfi_lo_wdata;
    assign mfi_mem_addr  = core_mfi_mem_addr;
    assign mfi_mem_rmask = core_mfi_mem_rmask;
    assign mfi_mem_wmask = core_mfi_mem_wmask;
    assign mfi_mem_rdata = core_mfi_mem_rdata;
    assign mfi_mem_wdata = core_mfi_mem_wdata;
`endif

    // Mux tag readback based on CACHE instruction cache type
    wire [31:0] muxed_cop_taglo = (core_cop_type == 2'd0) ? icache_cop_taglo :
                                  (core_cop_type == 2'd1) ? dcache_cop_taglo :
                                                            scache_cop_taglo;
    wire cop_taglo_update_valid = icache_cop_taglo_valid ||
                                  scache_cop_taglo_valid;
    wire [31:0] cop_taglo_update_data = icache_cop_taglo_valid ?
                                        icache_cop_taglo : scache_cop_taglo;

    // Demux CACHE ops to I$/D$/S$ based on cache type
    wire icache_cop_valid = core_cop_valid && (core_cop_type == 2'd0);
    wire dcache_cop_valid = core_cop_valid && (core_cop_type == 2'd1);
    wire scache_cop_valid = core_cop_valid && core_cop_type[1]; // type 2 or 3

    // ---------------------------------------------------------------
    // L1 ↔ S-cache wires
    // ---------------------------------------------------------------
    // I-cache memory side → S-cache
    wire [31:0] icache_mem_addr;
    wire        icache_need_fill;
    wire [31:0] scache_to_icache_rdata;
    wire [31:0] scache_to_icache_fill_rdata;
    wire        icache_freeze;
    wire        icache_mem_wen;
    wire [63:0] icache_mem_wdata;
    // A primary-I Fill/Hit-Writeback carries the CACHE operation's target,
    // independently of the instruction stream executing that operation.  In
    // particular, boot firmware may run uncached while moving a cached line.
    wire        icache_mem_uncached = effective_imem_uncached &&
                                       !icache_need_fill &&
                                       !icache_mem_wen;
    wire [127:0] icache_debug_capture;

    // D-cache memory side → S-cache
    wire [31:0] dcache_mem_addr;
    wire        dcache_mem_ren;
    wire        dcache_mem_wen;
    wire [ 7:0] dcache_mem_wmask;
    wire [63:0] dcache_mem_wdata;
    wire [63:0] scache_to_dcache_rdata;
    wire        dcache_freeze;
    wire [31:0] scache_ext_imem_addr;
    wire [127:0] dcache_debug_capture;
    wire [127:0] scache_debug_i_capture;

    // S-cache back-invalidation. D-cache receives every replacement/CACHE-op
    // invalidation for strict inclusion. I-cache receives explicit CACHE-op
    // invalidations and ordinary replacements only for secondary lines whose
    // contents were modified or snooped after fill.

`ifdef FPGA_CORE_ONLY
    assign icache_to_core_rdata = imem_rdata;
    assign dcache_to_core_rdata = dmem_rdata;
    assign scache_uncached_started = (core_dmem_ren || core_dmem_wen) &&
                                     dmem_ready;
    assign scache_uncached_accepted = (core_dmem_ren || core_dmem_wen) && dmem_ready;

    assign icache_stall = 1'b0;
    assign dcache_stall = 1'b0;
    assign dcache_store_visible = core_dmem_wen && dmem_ready &&
                                  !core_dmem_uncached;
    assign scache_stall = 1'b0;
    assign icache_cop_taglo = 32'd0;
    assign icache_cop_taglo_valid = 1'b0;
    assign dcache_cop_taglo = 32'd0;
    assign scache_cop_taglo = 32'd0;
    assign scache_cop_taglo_valid = 1'b0;
    assign sc_backinv_valid = 1'b0;
    assign sc_backinv_addr = 32'd0;
    assign sc_backinv_cacheop = 1'b0;
    assign sc_backinv_icache = 1'b0;
    assign sc_backinv_writeback = 1'b0;
    assign sc_backinv_ready = 1'b0;
    assign dcache_backinv_done = 1'b0;
`endif

    // ---------------------------------------------------------------
    // Core instance
    // ---------------------------------------------------------------
    r4000_core u_core (
        .clk          (clk),
        .rst_n        (rst_n),

        .imem_addr    (core_imem_addr),
        .imem_valid   (core_imem_valid),
        .imem_rdata   (icache_to_core_rdata),
        .imem_fetch_addr(core_imem_fetch_addr),
        .imem_fetch_advance(core_imem_fetch_advance),

        .dmem_ren     (core_dmem_ren),
        .dmem_wen     (core_dmem_wen),
        .dmem_addr    (core_dmem_addr),
        .dmem_lookup_addr(core_dmem_lookup_addr),
        .dmem_lookup_advance(core_dmem_lookup_advance),
        .dmem_wmask   (core_dmem_wmask),
        .dmem_wdata   (core_dmem_wdata),
        .dmem_rdata   (dcache_to_core_rdata),

        .cache_stall   (cache_stall),
        .uncached_started  (scache_uncached_started),
        .uncached_accepted (scache_uncached_accepted),
        .cached_store_visible(dcache_store_visible),
        .imem_uncached (core_imem_uncached),
        .dmem_uncached (core_dmem_uncached),
        .force_uncached(force_uncached),

        .hw_int        (hw_int),
        .sim_idle_boost(sim_idle_boost),

        // CC-2: DMA snoop → core llbit clear
        .snoop_valid   (snoop_commit),
        .snoop_addr    (snoop_addr),

        // CC-4: S-cache line eviction → core llbit clear (uses the existing
        // S-cache back-invalidate signal that already feeds I$/D$ for
        // inclusion maintenance)
        .evict_valid   (sc_backinv_valid),
        .evict_addr    (sc_backinv_addr),

        .cache_cop_valid(core_cop_valid),
        .cache_cop_op   (core_cop_op),
        .cache_cop_type (core_cop_type),
        .cache_cop_paddr(core_cop_paddr),
        .cache_cop_taglo_out(core_cop_taglo),
        .cache_cop_taglo_in(muxed_cop_taglo),
        .cache_cop_taglo_update_valid(cop_taglo_update_valid),
        .cache_cop_taglo_update_data(cop_taglo_update_data),
        .retirement_tail_empty(core_retirement_tail_empty),

        .mfi_valid    (core_mfi_valid),
        .mfi_order    (core_mfi_order),
        .mfi_pc       (core_mfi_pc),
        .mfi_insn     (core_mfi_insn),
        .mfi_trap     (core_mfi_trap),
        .mfi_rd_addr  (core_mfi_rd_addr),
        .mfi_rd_wdata (core_mfi_rd_wdata),
        .mfi_hilo_wen (core_mfi_hilo_wen),
        .mfi_hi_wdata (core_mfi_hi_wdata),
        .mfi_lo_wdata (core_mfi_lo_wdata),
        .mfi_mem_addr (core_mfi_mem_addr),
        .mfi_mem_rmask(core_mfi_mem_rmask),
        .mfi_mem_wmask(core_mfi_mem_wmask),
        .mfi_mem_rdata(core_mfi_mem_rdata),
        .mfi_mem_wdata(core_mfi_mem_wdata),

        .debug_pc      (debug_core_pc),
        .debug_is_pc   (debug_core_is_pc),
        .debug_rf_pc   (debug_core_rf_pc),
        .debug_rf_insn (debug_core_rf_insn),
        .debug_wb_pc   (debug_core_wb_pc),
        .debug_wb_insn (debug_core_wb_insn),
        .debug_epc     (debug_core_epc),
        .debug_cause   (debug_core_cause),
        .debug_badvaddr(debug_core_badvaddr),
        .debug_status  (debug_core_status),
        .debug_context (debug_core_context),
        .debug_entryhi (debug_core_entryhi),
        .debug_flags   (debug_core_flags),
        .debug_gpr_v0  (debug_core_gpr_v0),
        .debug_gpr_v1  (debug_core_gpr_v1),
        .debug_gpr_at  (debug_core_gpr_at),
        .debug_gpr_t0  (debug_core_gpr_t0),
        .debug_gpr_t1  (debug_core_gpr_t1),
        .debug_gpr_a0  (debug_core_gpr_a0),
        .debug_gpr_a1  (debug_core_gpr_a1),
        .debug_gpr_a2  (debug_core_gpr_a2),
        .debug_gpr_t8  (debug_core_gpr_t8),
        .debug_gpr_s4  (debug_core_gpr_s4),
        .debug_gpr_s5  (debug_core_gpr_s5),
        .debug_gpr_s6  (debug_core_gpr_s6),
        .debug_gpr_t2  (debug_core_gpr_t2),
        .debug_gpr_t3  (debug_core_gpr_t3),
        .debug_gpr_a3  (debug_core_gpr_a3),
        .debug_gpr_s0  (debug_core_gpr_s0),
        .debug_gpr_s1  (debug_core_gpr_s1),
        .debug_gpr_s2  (debug_core_gpr_s2),
        .debug_gpr_s3  (debug_core_gpr_s3),
        .debug_gpr_t4  (debug_core_gpr_t4),
        .debug_gpr_t6  (debug_core_gpr_t6),
        .debug_gpr_t7  (debug_core_gpr_t7),
        .debug_gpr_t9  (debug_core_gpr_t9),
        .debug_gpr_sp  (debug_core_gpr_sp),
        .debug_gpr_ra  (debug_core_gpr_ra),
        .debug_last_cf_pc     (debug_core_last_cf_pc),
        .debug_last_cf_target (debug_core_last_cf_target),
        .debug_wb_eff_addr(debug_core_wb_eff_addr),
        .debug_wb_load_data(debug_core_wb_load_data),
        .debug_dmem_pc(core_debug_dmem_pc),
        .debug_dmem_insn(core_debug_dmem_insn),
        .debug_dmem_vaddr(debug_dmem_vaddr),
        .debug_dtlb_state(debug_dtlb_state),
        .debug_dtlb_paddr(debug_dtlb_paddr),
        .debug_dtlb_lo(debug_dtlb_lo),
        .debug_dtlb_mask(debug_dtlb_mask),
        .debug_tlb_idx (debug_tlb_idx),
        .debug_tlb_hi  (debug_tlb_hi),
        .debug_tlb_lo0 (debug_tlb_lo0),
        .debug_tlb_lo1 (debug_tlb_lo1),
        .debug_tlb_mask(debug_tlb_mask),
        .debug_tlbwr_meta(debug_tlbwr_meta),
        .debug_tlbwr_hi(debug_tlbwr_hi),
        .debug_tlbwr_lo0(debug_tlbwr_lo0),
        .debug_tlbwr_lo1(debug_tlbwr_lo1),
        .debug_tlbwr_mask(debug_tlbwr_mask)
    );

`ifndef FPGA_CORE_ONLY
    // ---------------------------------------------------------------
    // I-cache (L1) — memory side connects to S-cache
    // ---------------------------------------------------------------
    r4000_icache u_icache (
        .clk        (clk),
        .rst_n      (rst_n),
        .core_addr  (core_imem_addr[31:0]),
        .core_valid (core_imem_valid),
        .fetch_addr (core_imem_fetch_addr[31:0]),
        .fetch_advance((ICACHE_LOOKAHEAD != 0) && core_imem_fetch_advance),
        .uncached   (effective_imem_uncached),
        .core_rdata (icache_to_core_rdata),
        .stall      (icache_stall),
        .cop_valid  (icache_cop_valid),
        .cop_op     (core_cop_op),
        .cop_paddr  (core_cop_paddr),
        .cop_taglo  (core_cop_taglo),
        .freeze     (icache_freeze),
        .backinv_valid (sc_backinv_valid && sc_backinv_icache),
        .backinv_addr  (sc_backinv_addr),
        .snoop_valid   (snoop_commit),
        .snoop_addr    (snoop_addr),
        .mem_addr   (icache_mem_addr),
        .need_fill  (icache_need_fill),
        .mem_rdata  (scache_to_icache_rdata),
        .fill_rdata (scache_to_icache_fill_rdata),
        .mem_wen    (icache_mem_wen),
        .mem_wdata  (icache_mem_wdata),
        .debug_capture (icache_debug_capture),
        .cop_taglo_out (icache_cop_taglo),
        .cop_taglo_valid(icache_cop_taglo_valid)
    );

    // ---------------------------------------------------------------
    // D-cache (L1) — memory side connects to S-cache
    // ---------------------------------------------------------------
    r4000_dcache u_dcache (
        .clk        (clk),
        .rst_n      (rst_n),
        .core_addr  (core_dmem_addr[31:0]),
        .lookup_addr(core_dmem_lookup_addr[31:0]),
        .lookup_advance(core_dmem_lookup_advance),
        .core_ren   (core_dmem_ren),
        .core_wen   (core_dmem_wen),
        .core_wmask (core_dmem_wmask),
        .core_wdata (core_dmem_wdata),
        .uncached   (core_dmem_uncached),
        .core_rdata (dcache_to_core_rdata),
        .stall      (dcache_stall),
        .core_store_visible(dcache_store_visible),
        .cop_valid  (dcache_cop_valid),
        .cop_op     (core_cop_op),
        .cop_paddr  (core_cop_paddr),
        .freeze     (dcache_freeze),
        .mem_addr   (dcache_mem_addr),
        .mem_ren    (dcache_mem_ren),
        .mem_wen    (dcache_mem_wen),
        .mem_wmask  (dcache_mem_wmask),
        .mem_wdata  (dcache_mem_wdata),
        .mem_rdata  (scache_to_dcache_rdata),
        .snoop_valid   (snoop_commit),
        .snoop_addr    (snoop_addr),
        .backinv_valid (sc_backinv_valid),
        .backinv_addr  (sc_backinv_addr),
        .backinv_writeback(sc_backinv_writeback),
        .backinv_ready (sc_backinv_ready),
        .backinv_done  (dcache_backinv_done),
        .debug_capture (dcache_debug_capture),
        .cop_taglo_out (dcache_cop_taglo)
    );

    // ---------------------------------------------------------------
    // Secondary cache (L2) — sits between L1 and external memory
    // ---------------------------------------------------------------
    r4000_scache #(
        .SCACHE_LINES(SCACHE_LINES),
        .FULL_ICACHE_INCLUSION(FULL_ICACHE_INCLUSION)
    ) u_scache (
        .clk            (clk),
        .rst_n          (rst_n),

        // I-cache memory-side
        .i_addr         (icache_mem_addr),
        .i_valid        (core_imem_valid),
        .i_need_fill    (icache_need_fill),
        .i_uncached     (icache_mem_uncached),
        .i_rdata        (scache_to_icache_rdata),
        .i_fill_rdata   (scache_to_icache_fill_rdata),
        .i_freeze       (icache_freeze),
        .i_wen          (icache_mem_wen),
        .i_wdata        (icache_mem_wdata),

        // D-cache memory-side
        .d_addr         (dcache_mem_addr),
        .d_ren          (dcache_mem_ren),
        .d_wen          (dcache_mem_wen),
        .d_wmask        (dcache_mem_wmask),
        .d_wdata        (dcache_mem_wdata),
        .d_uncached     (core_dmem_uncached),
        .d_mem_pc       (core_debug_dmem_pc),
        .force_uncached (force_uncached),
        .d_rdata        (scache_to_dcache_rdata),
        .d_freeze       (dcache_freeze),

        // CACHE instruction (type 2/3)
        .cop_valid      (scache_cop_valid),
        .cop_op         (core_cop_op),
        .cop_paddr      (core_cop_paddr),
        .cop_taglo_in   (core_cop_taglo),
        .cop_taglo_out  (scache_cop_taglo),
        .cop_taglo_valid(scache_cop_taglo_valid),

        // External instruction memory (32-bit, uncached I reads)
        .ext_imem_addr  (scache_ext_imem_addr),
        .ext_imem_rdata (imem_rdata),

        // External data memory (64-bit, S-cache fills/WB + uncached D)
        .ext_dmem_addr  (dmem_paddr),
        .ext_dmem_ren   (dmem_ren),
        .ext_dmem_wen   (dmem_wen),
        .ext_dmem_wmask (dmem_wmask),
        .ext_dmem_wdata (dmem_wdata),
        .ext_dmem_rdata (dmem_rdata),
        .ext_dmem_ready (dmem_ready),

        .stall          (scache_stall),
        .uncached_started (scache_uncached_started),
        .uncached_accepted (scache_uncached_accepted),
        .ext_dmem_uncached_wr (dmem_uncached_wr),
        .ext_dmem_cache_owned (dmem_cache_owned),

        .snoop_valid    (snoop_commit),
        .snoop_addr     (snoop_addr),
        .snoop_wdata    (snoop_wdata),
        .snoop_wmask    (snoop_wmask),
        .snoop_wr_ready (snoop_wr_ready),
        .snoop_wr_done  (snoop_wr_done),

        .snoop_rd_valid (snoop_rd_valid),
        .snoop_rd_addr  (snoop_rd_addr),
        .snoop_rd_resp_valid(snoop_rd_resp_valid),
        .snoop_rd_hit   (snoop_rd_hit),
        .snoop_rd_data  (snoop_rd_data),

        .backinv_valid  (sc_backinv_valid),
        .backinv_addr   (sc_backinv_addr),
        .backinv_cacheop(sc_backinv_cacheop),
        .backinv_icache (sc_backinv_icache),
        .backinv_writeback(sc_backinv_writeback),
        .backinv_ready  (sc_backinv_ready),
        .backinv_done   (dcache_backinv_done),
        .debug_status   (debug_cache_status),
        .debug_i_capture(scache_debug_i_capture),
        .debug_scache_probe(debug_scache_probe),
        .debug_addr     (debug_cache_addr)
    );
`else
    assign dmem_ren = core_dmem_ren;
    assign dmem_wen = core_dmem_wen;
    assign dmem_paddr = core_dmem_addr[31:0];
    assign dmem_wmask = core_dmem_wen ? core_dmem_wmask : 8'h00;
    assign dmem_wdata = core_dmem_wdata;
    assign dmem_uncached_wr = core_dmem_wen && core_dmem_uncached;
    assign dmem_cache_owned = 1'b0;
    assign snoop_wr_ready = 1'b1;
    assign snoop_wr_done = snoop_valid;

    assign snoop_rd_resp_valid = snoop_rd_valid;
    assign snoop_rd_hit = 1'b0;
    assign snoop_rd_data = 64'd0;
    assign debug_cache_status = 32'd0;
    assign debug_cache_addr = 32'd0;
    assign debug_scache_probe = 384'd0;
`endif

`ifdef FPGA_CORE_ONLY
    assign imem_paddr = core_imem_addr[31:0];
    assign debug_icache_capture = {224'd0, imem_rdata};
`else
    assign imem_paddr = scache_ext_imem_addr;
    assign debug_icache_capture = {icache_debug_capture, scache_debug_i_capture};
`endif
    assign imem_ren = 1'b1; // I-side ext port always reads (combinational)

    assign debug_core_store_commit = core_dmem_wen && !cache_stall;
    assign debug_core_store_pc = core_debug_dmem_pc;
    assign debug_core_store_insn = core_debug_dmem_insn;
    assign debug_core_store_paddr = core_dmem_addr[31:0];
    assign debug_core_store_vaddr = debug_dmem_vaddr;
    assign debug_core_store_wmask = core_dmem_wmask;
    assign debug_core_store_wdata = core_dmem_wdata;

endmodule
