// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// Production-RTL out-of-context timing wrapper for r4000_core plus L1 caches.
//
// This wrapper deliberately stops at the existing L1 memory-side contracts.
// It does not model or replace any product behavior; the complete r4000_top
// inventory separately measures the production S-cache hierarchy.

`default_nettype none

module r4000_core_l1_timing_top (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] imem_rdata,
    input  wire [31:0] ifill_rdata,
    input  wire        imem_freeze,
    input  wire [63:0] dmem_rdata,
    input  wire        dmem_freeze,
    input  wire [5:0]  hw_int,
    input  wire        force_uncached,
    input  wire        snoop_valid,
    input  wire [31:0] snoop_addr,
    input  wire        backinv_valid,
    input  wire [31:0] backinv_addr,
    input  wire        backinv_writeback,
    input  wire [5:0]  debug_tlb_idx,
    output wire [31:0] imem_addr,
    output wire        imem_need_fill,
    output wire        imem_wen,
    output wire [63:0] imem_wdata,
    output wire [31:0] dmem_addr,
    output wire        dmem_ren,
    output wire        dmem_wen,
    output wire [7:0]  dmem_wmask,
    output wire [63:0] dmem_wdata,
    output wire        dmem_backinv_ready,
    output wire        dmem_backinv_done,
    output wire [31:0] debug_pc,
    output wire [31:0] debug_cache_control,
    output wire [127:0] debug_icache_capture,
    output wire [127:0] debug_dcache_capture
);
    wire [63:0] core_imem_addr;
    wire        core_imem_valid;
    wire [63:0] core_imem_fetch_addr;
    wire        core_imem_fetch_advance;
    wire [31:0] core_imem_rdata;
    wire        core_imem_uncached;
    wire        effective_imem_uncached = core_imem_uncached | force_uncached;
    wire        icache_stall;

    wire        core_dmem_ren;
    wire        core_dmem_wen;
    wire [63:0] core_dmem_addr;
    wire [63:0] core_dmem_lookup_addr;
    wire        core_dmem_lookup_advance;
    wire [7:0]  core_dmem_wmask;
    wire [63:0] core_dmem_wdata;
    wire [63:0] core_dmem_rdata;
    wire        core_dmem_uncached;
    wire        dcache_stall;
    wire        dcache_store_visible;

    wire        core_cop_valid;
    wire [2:0]  core_cop_op;
    wire [1:0]  core_cop_type;
    wire [31:0] core_cop_paddr;
    wire [31:0] core_cop_taglo;
    wire [31:0] icache_cop_taglo;
    wire        icache_cop_taglo_valid;
    wire [31:0] dcache_cop_taglo;
    wire        icache_cop_valid = core_cop_valid && (core_cop_type == 2'd0);
    wire        dcache_cop_valid = core_cop_valid && (core_cop_type == 2'd1);
    wire [31:0] muxed_cop_taglo =
        (core_cop_type == 2'd0) ? icache_cop_taglo :
        (core_cop_type == 2'd1) ? dcache_cop_taglo : 32'd0;
    wire        cache_stall = icache_stall || dcache_stall;
    wire        uncached_started = (dmem_ren || dmem_wen) && !dmem_freeze;
    wire        uncached_accepted = uncached_started;

    r4000_core u_core (
        .clk(clk),
        .rst_n(rst_n),
        .imem_addr(core_imem_addr),
        .imem_valid(core_imem_valid),
        .imem_rdata(core_imem_rdata),
        .imem_fetch_addr(core_imem_fetch_addr),
        .imem_fetch_advance(core_imem_fetch_advance),
        .dmem_ren(core_dmem_ren),
        .dmem_wen(core_dmem_wen),
        .dmem_addr(core_dmem_addr),
        .dmem_lookup_addr(core_dmem_lookup_addr),
        .dmem_lookup_advance(core_dmem_lookup_advance),
        .dmem_wmask(core_dmem_wmask),
        .dmem_wdata(core_dmem_wdata),
        .dmem_rdata(core_dmem_rdata),
        .cache_stall(cache_stall),
        .uncached_started(uncached_started),
        .uncached_accepted(uncached_accepted),
        .cached_store_visible(dcache_store_visible),
        .imem_uncached(core_imem_uncached),
        .dmem_uncached(core_dmem_uncached),
        .force_uncached(force_uncached),
        .hw_int(hw_int),
        .sim_idle_boost(1'b0),
        .snoop_valid(snoop_valid),
        .snoop_addr(snoop_addr),
        .evict_valid(backinv_valid),
        .evict_addr(backinv_addr),
        .cache_cop_valid(core_cop_valid),
        .cache_cop_op(core_cop_op),
        .cache_cop_type(core_cop_type),
        .cache_cop_paddr(core_cop_paddr),
        .cache_cop_taglo_out(core_cop_taglo),
        .cache_cop_taglo_in(muxed_cop_taglo),
        .cache_cop_taglo_update_valid(icache_cop_taglo_valid),
        .cache_cop_taglo_update_data(icache_cop_taglo),
        .debug_pc(debug_pc),
        .debug_tlb_idx(debug_tlb_idx)
    );

    r4000_icache u_icache (
        .clk(clk),
        .rst_n(rst_n),
        .core_addr(core_imem_addr[31:0]),
        .core_valid(core_imem_valid),
        .fetch_addr(core_imem_fetch_addr[31:0]),
        .fetch_advance(core_imem_fetch_advance),
        .uncached(effective_imem_uncached),
        .core_rdata(core_imem_rdata),
        .stall(icache_stall),
        .cop_valid(icache_cop_valid),
        .cop_op(core_cop_op),
        .cop_paddr(core_cop_paddr),
        .cop_taglo(core_cop_taglo),
        .freeze(imem_freeze),
        .backinv_valid(backinv_valid),
        .backinv_addr(backinv_addr),
        .snoop_valid(snoop_valid),
        .snoop_addr(snoop_addr),
        .mem_addr(imem_addr),
        .need_fill(imem_need_fill),
        .mem_rdata(imem_rdata),
        .fill_rdata(ifill_rdata),
        .mem_wen(imem_wen),
        .mem_wdata(imem_wdata),
        .debug_capture(debug_icache_capture),
        .cop_taglo_out(icache_cop_taglo),
        .cop_taglo_valid(icache_cop_taglo_valid)
    );

    r4000_dcache u_dcache (
        .clk(clk),
        .rst_n(rst_n),
        .core_addr(core_dmem_addr[31:0]),
        .lookup_addr(core_dmem_lookup_addr[31:0]),
        .lookup_advance(core_dmem_lookup_advance),
        .core_ren(core_dmem_ren),
        .core_wen(core_dmem_wen),
        .core_wmask(core_dmem_wmask),
        .core_wdata(core_dmem_wdata),
        .uncached(core_dmem_uncached),
        .core_rdata(core_dmem_rdata),
        .stall(dcache_stall),
        .core_store_visible(dcache_store_visible),
        .cop_valid(dcache_cop_valid),
        .cop_op(core_cop_op),
        .cop_paddr(core_cop_paddr),
        .freeze(dmem_freeze),
        .mem_addr(dmem_addr),
        .mem_ren(dmem_ren),
        .mem_wen(dmem_wen),
        .mem_wmask(dmem_wmask),
        .mem_wdata(dmem_wdata),
        .mem_rdata(dmem_rdata),
        .snoop_valid(snoop_valid),
        .snoop_addr(snoop_addr),
        .backinv_valid(backinv_valid),
        .backinv_addr(backinv_addr),
        .backinv_writeback(backinv_writeback),
        .backinv_ready(dmem_backinv_ready),
        .backinv_done(dmem_backinv_done),
        .debug_capture(debug_dcache_capture),
        .cop_taglo_out(dcache_cop_taglo)
    );

    assign debug_cache_control = {
        6'd0,
        core_cop_taglo[7:0],
        core_cop_paddr[7:0],
        cache_stall,
        core_imem_uncached,
        core_dmem_uncached,
        dcache_store_visible,
        core_cop_valid,
        core_cop_type,
        core_cop_op
    };
endmodule

`default_nettype wire
