// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_tlb.v — R4000 TLB: 48-entry storage + ITLB/DTLB lookup + TLBP probe
//                          + TLBR read + TLBWI/TLBWR write + reset.
//
// Encapsulates everything that touches the 48-entry TLB storage
// so r4000_core.v can be free of the in-line procedural priority for-loops
// that stalled Quartus 18.1 SE Lite logic synthesis.
//
// Lookup implementation: per-entry parallel match comparators in a generate
// block + two-level explicit priority selection (highest-index wins, which
// matches the original blocking-loop semantics) + grouped structural muxes
// for the selected entry's mask/lo. No procedural for-loops, no integer-
// indexed wire reads from inside an always block — patterns that bit us in
// the in-line refactor.
//
// Verification gate: same outputs as the original in-line code for all
// (vaddr, asid, tlb_*) inputs. Verified by passing cheritest 478/478 +
// testfloat 92/92 with `make regress` after r4000_core.v adoption.

`ifndef CYAN_R4000_TLB_V
`define CYAN_R4000_TLB_V

`default_nettype none

module r4000_tlb (
    input  wire        clk,
    input  wire        rst_n,

    // Current ASID (cp0_entryhi[7:0]) for match
    input  wire [7:0]  asid,

    // ITLB lookup — combinational. Outputs are valid for any vaddr; caller
    // gates on is_mapped(vaddr) externally and uses i_paddr_unmapped for the
    // unmapped path. (We keep the unmapped path out of this module so
    // is_mapped/unmapped_paddr stay in r4000_core where addr-classification
    // helpers live.)
    input  wire [63:0] i_vaddr,
    output wire        i_match,
    output wire        i_v,
    output wire [31:0] i_paddr,
    output wire [63:0] i_lo_sel,

    // DTLB lookup — combinational
    input  wire [63:0] d_vaddr,
    output wire        d_match,
    output wire        d_v,
    output wire        d_d,
    output wire [31:0] d_paddr,
    output wire [63:0] d_lo_sel,
    output wire [63:0] d_mask,
    output wire [5:0]  d_match_idx,

    // TLBP probe — combinational. Caller passes cp0_entryhi as probe_vaddr.
    input  wire [63:0] probe_vaddr,
    output wire        probe_match,
    output wire [5:0]  probe_idx,

    // TLBR read — combinational read by index.
    input  wire [5:0]  read_idx,
    output wire [63:0] read_hi,
    output wire [63:0] read_lo0,
    output wire [63:0] read_lo1,
    output wire [63:0] read_mask,

    // Debug read — independent combinational read port for the FPGA ISSP TLB
    // dumper. This must not disturb CP0 TLBR's architectural read index.
    input  wire [5:0]  debug_read_idx,
    output wire [63:0] debug_read_hi,
    output wire [63:0] debug_read_lo0,
    output wire [63:0] debug_read_lo1,
    output wire [63:0] debug_read_mask,

    // TLBW write — sequential. Caller drives write_idx from cp0_index (TLBWI)
    // or from the random-index logic (TLBWR), and write_en when the WB-stage
    // CP0 op is committing.
    input  wire        write_en,
    input  wire [5:0]  write_idx,
    input  wire [63:0] write_hi,
    input  wire [63:0] write_lo0,
    input  wire [63:0] write_lo1,
    input  wire [63:0] write_mask
);

    // ================================================================
    // Storage
    // ================================================================
    // Store only the architectural bits this core uses:
    //   EntryHi:  region[63:62], VPN2[39:13], ASID[7:0]
    //   EntryLo:  low 30 bits (core already canonicalizes the G bit)
    //   PageMask: mask[24:13]
    reg [36:0] tlb_hi   [0:47];
    reg [29:0] tlb_lo0  [0:47];
    reg [29:0] tlb_lo1  [0:47];
    reg [11:0] tlb_mask [0:47];

    function [36:0] pack_entryhi;
        input [63:0] entry_hi;
        begin
            pack_entryhi = {entry_hi[63:62], entry_hi[39:13], entry_hi[7:0]};
        end
    endfunction

    function [36:0] reset_entryhi;
        input [5:0] entry_idx;
        begin
            reset_entryhi = {2'b11, 2'b11, entry_idx, 19'd0, 8'd0};
        end
    endfunction

    function [63:0] unpack_entryhi;
        input [36:0] entry_hi;
        begin
            unpack_entryhi = {entry_hi[36:35], 22'd0, entry_hi[34:8], 5'd0, entry_hi[7:0]};
        end
    endfunction

    function [63:0] unpack_entrylo;
        input [29:0] entry_lo;
        begin
            unpack_entrylo = {34'd0, entry_lo};
        end
    endfunction

    function [63:0] unpack_pagemask;
        input [11:0] page_mask;
        begin
            unpack_pagemask = {39'd0, page_mask, 13'd0};
        end
    endfunction

    // Reset + write port. Async-reset to match r4000_core's convention
    // (avoids SYNCASYNCNET warnings when cosim drives rst_n asynchronously).
    integer rst_i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (rst_i = 0; rst_i < 48; rst_i = rst_i + 1) begin
                tlb_hi[rst_i]   <= reset_entryhi(rst_i[5:0]);
                tlb_lo0[rst_i]  <= 30'd0;
                tlb_lo1[rst_i]  <= 30'd0;
                tlb_mask[rst_i] <= 12'd0;
            end
        end else if (write_en) begin
            tlb_hi  [write_idx] <= pack_entryhi(write_hi);
            tlb_lo0 [write_idx] <= write_lo0[29:0];
            tlb_lo1 [write_idx] <= write_lo1[29:0];
            tlb_mask[write_idx] <= write_mask[24:13];
`ifndef SYNTHESIS
            // Keep array observable for simulator (avoids X-prop / DCE in
            // some Verilator versions when entry 63 is written — cargo-cult
            // from the original deferred-write workaround).
            if (write_idx == 6'd63) $write("");
`endif
        end
    end

    // ================================================================
    // TLBR read port — combinational (used in CP0 WB to populate
    // cp0_entryhi/lo0/lo1/pagemask).
    // ================================================================
    assign read_hi   = unpack_entryhi(tlb_hi[read_idx]);
    assign read_lo0  = unpack_entrylo(tlb_lo0[read_idx]);
    assign read_lo1  = unpack_entrylo(tlb_lo1[read_idx]);
    assign read_mask = unpack_pagemask(tlb_mask[read_idx]);

    assign debug_read_hi  = unpack_entryhi(tlb_hi[debug_read_idx]);
    assign debug_read_lo0 = unpack_entrylo(tlb_lo0[debug_read_idx]);
    assign debug_read_lo1 = unpack_entrylo(tlb_lo1[debug_read_idx]);
    assign debug_read_mask = unpack_pagemask(tlb_mask[debug_read_idx]);

    // ================================================================
    // Helper functions (private to TLB module)
    // ================================================================
    function odd_page_sel;
        input [63:0] vaddr;
        input [11:0] pmask;
        begin
            if      (pmask[11]) odd_page_sel = vaddr[24];
            else if (pmask[9])  odd_page_sel = vaddr[22];
            else if (pmask[7])  odd_page_sel = vaddr[20];
            else if (pmask[5])  odd_page_sel = vaddr[18];
            else if (pmask[3])  odd_page_sel = vaddr[16];
            else if (pmask[1])  odd_page_sel = vaddr[14];
            else                odd_page_sel = vaddr[12];
        end
    endfunction

    function [31:0] tlb_phys_addr;
        input [63:0] vaddr;
        input [29:0] entry_lo;
        input [11:0] pmask;
        reg   [31:0] pfn_shifted, offset_mask;
        begin
            pfn_shifted = {entry_lo[25:6], 12'b0};
            offset_mask = {8'b0, pmask, 12'hFFF};
            tlb_phys_addr = (pfn_shifted & ~offset_mask) | (vaddr[31:0] & offset_mask);
        end
    endfunction

    // ================================================================
    // ITLB lookup (combinational, dataflow form)
    //
    //   match_vec[k] = match condition for entry k (per the R4000 TLB
    //                  hit rule: VPN match modulo per-entry pagemask AND
    //                  segment match AND (global bit OR ASID match)).
    //   group_match  = OR-reduce per group of 8 entries.
    //   any_match    = OR-reduce of match_vec.
    //   mask_e/lo_e  = the matched entry's mask + (odd ? lo1 : lo0),
    //                  selected first within each group, then across groups.
    //
    // Highest-index priority preserves the original blocking for-loop's
    // last-write-wins semantics (real R4000 TLB matches should be unique,
    // but if duplicates exist we match the original behavior bit-for-bit).
    // ================================================================
    wire [47:0] i_match_vec;
    genvar gi;
    generate
        for (gi = 0; gi < 48; gi = gi + 1) begin : g_itlb_match
            assign i_match_vec[gi] =
                (((i_vaddr[39:13] ^ tlb_hi[gi][34:8]) & ~{15'd0, tlb_mask[gi]}) == 27'd0) &&
                (i_vaddr[63:62] == tlb_hi[gi][36:35]) &&
                ((tlb_lo0[gi][0] & tlb_lo1[gi][0]) ||
                 (tlb_hi[gi][7:0] == asid));
        end
    endgenerate

    wire [5:0] i_group_match = {
        |i_match_vec[47:40], |i_match_vec[39:32], |i_match_vec[31:24],
        |i_match_vec[23:16], |i_match_vec[15:8],  |i_match_vec[7:0]
    };

    wire        i_any_match = |i_group_match;

    wire [11:0] i_mask_g0 =
        i_match_vec[7] ? tlb_mask[7] : i_match_vec[6] ? tlb_mask[6] :
        i_match_vec[5] ? tlb_mask[5] : i_match_vec[4] ? tlb_mask[4] :
        i_match_vec[3] ? tlb_mask[3] : i_match_vec[2] ? tlb_mask[2] :
        i_match_vec[1] ? tlb_mask[1] : tlb_mask[0];
    wire [11:0] i_mask_g1 =
        i_match_vec[15] ? tlb_mask[15] : i_match_vec[14] ? tlb_mask[14] :
        i_match_vec[13] ? tlb_mask[13] : i_match_vec[12] ? tlb_mask[12] :
        i_match_vec[11] ? tlb_mask[11] : i_match_vec[10] ? tlb_mask[10] :
        i_match_vec[9]  ? tlb_mask[9]  : tlb_mask[8];
    wire [11:0] i_mask_g2 =
        i_match_vec[23] ? tlb_mask[23] : i_match_vec[22] ? tlb_mask[22] :
        i_match_vec[21] ? tlb_mask[21] : i_match_vec[20] ? tlb_mask[20] :
        i_match_vec[19] ? tlb_mask[19] : i_match_vec[18] ? tlb_mask[18] :
        i_match_vec[17] ? tlb_mask[17] : tlb_mask[16];
    wire [11:0] i_mask_g3 =
        i_match_vec[31] ? tlb_mask[31] : i_match_vec[30] ? tlb_mask[30] :
        i_match_vec[29] ? tlb_mask[29] : i_match_vec[28] ? tlb_mask[28] :
        i_match_vec[27] ? tlb_mask[27] : i_match_vec[26] ? tlb_mask[26] :
        i_match_vec[25] ? tlb_mask[25] : tlb_mask[24];
    wire [11:0] i_mask_g4 =
        i_match_vec[39] ? tlb_mask[39] : i_match_vec[38] ? tlb_mask[38] :
        i_match_vec[37] ? tlb_mask[37] : i_match_vec[36] ? tlb_mask[36] :
        i_match_vec[35] ? tlb_mask[35] : i_match_vec[34] ? tlb_mask[34] :
        i_match_vec[33] ? tlb_mask[33] : tlb_mask[32];
    wire [11:0] i_mask_g5 =
        i_match_vec[47] ? tlb_mask[47] : i_match_vec[46] ? tlb_mask[46] :
        i_match_vec[45] ? tlb_mask[45] : i_match_vec[44] ? tlb_mask[44] :
        i_match_vec[43] ? tlb_mask[43] : i_match_vec[42] ? tlb_mask[42] :
        i_match_vec[41] ? tlb_mask[41] : tlb_mask[40];

    wire [29:0] i_lo0_g0 =
        i_match_vec[7] ? tlb_lo0[7] : i_match_vec[6] ? tlb_lo0[6] :
        i_match_vec[5] ? tlb_lo0[5] : i_match_vec[4] ? tlb_lo0[4] :
        i_match_vec[3] ? tlb_lo0[3] : i_match_vec[2] ? tlb_lo0[2] :
        i_match_vec[1] ? tlb_lo0[1] : tlb_lo0[0];
    wire [29:0] i_lo0_g1 =
        i_match_vec[15] ? tlb_lo0[15] : i_match_vec[14] ? tlb_lo0[14] :
        i_match_vec[13] ? tlb_lo0[13] : i_match_vec[12] ? tlb_lo0[12] :
        i_match_vec[11] ? tlb_lo0[11] : i_match_vec[10] ? tlb_lo0[10] :
        i_match_vec[9]  ? tlb_lo0[9]  : tlb_lo0[8];
    wire [29:0] i_lo0_g2 =
        i_match_vec[23] ? tlb_lo0[23] : i_match_vec[22] ? tlb_lo0[22] :
        i_match_vec[21] ? tlb_lo0[21] : i_match_vec[20] ? tlb_lo0[20] :
        i_match_vec[19] ? tlb_lo0[19] : i_match_vec[18] ? tlb_lo0[18] :
        i_match_vec[17] ? tlb_lo0[17] : tlb_lo0[16];
    wire [29:0] i_lo0_g3 =
        i_match_vec[31] ? tlb_lo0[31] : i_match_vec[30] ? tlb_lo0[30] :
        i_match_vec[29] ? tlb_lo0[29] : i_match_vec[28] ? tlb_lo0[28] :
        i_match_vec[27] ? tlb_lo0[27] : i_match_vec[26] ? tlb_lo0[26] :
        i_match_vec[25] ? tlb_lo0[25] : tlb_lo0[24];
    wire [29:0] i_lo0_g4 =
        i_match_vec[39] ? tlb_lo0[39] : i_match_vec[38] ? tlb_lo0[38] :
        i_match_vec[37] ? tlb_lo0[37] : i_match_vec[36] ? tlb_lo0[36] :
        i_match_vec[35] ? tlb_lo0[35] : i_match_vec[34] ? tlb_lo0[34] :
        i_match_vec[33] ? tlb_lo0[33] : tlb_lo0[32];
    wire [29:0] i_lo0_g5 =
        i_match_vec[47] ? tlb_lo0[47] : i_match_vec[46] ? tlb_lo0[46] :
        i_match_vec[45] ? tlb_lo0[45] : i_match_vec[44] ? tlb_lo0[44] :
        i_match_vec[43] ? tlb_lo0[43] : i_match_vec[42] ? tlb_lo0[42] :
        i_match_vec[41] ? tlb_lo0[41] : tlb_lo0[40];

    wire [29:0] i_lo1_g0 =
        i_match_vec[7] ? tlb_lo1[7] : i_match_vec[6] ? tlb_lo1[6] :
        i_match_vec[5] ? tlb_lo1[5] : i_match_vec[4] ? tlb_lo1[4] :
        i_match_vec[3] ? tlb_lo1[3] : i_match_vec[2] ? tlb_lo1[2] :
        i_match_vec[1] ? tlb_lo1[1] : tlb_lo1[0];
    wire [29:0] i_lo1_g1 =
        i_match_vec[15] ? tlb_lo1[15] : i_match_vec[14] ? tlb_lo1[14] :
        i_match_vec[13] ? tlb_lo1[13] : i_match_vec[12] ? tlb_lo1[12] :
        i_match_vec[11] ? tlb_lo1[11] : i_match_vec[10] ? tlb_lo1[10] :
        i_match_vec[9]  ? tlb_lo1[9]  : tlb_lo1[8];
    wire [29:0] i_lo1_g2 =
        i_match_vec[23] ? tlb_lo1[23] : i_match_vec[22] ? tlb_lo1[22] :
        i_match_vec[21] ? tlb_lo1[21] : i_match_vec[20] ? tlb_lo1[20] :
        i_match_vec[19] ? tlb_lo1[19] : i_match_vec[18] ? tlb_lo1[18] :
        i_match_vec[17] ? tlb_lo1[17] : tlb_lo1[16];
    wire [29:0] i_lo1_g3 =
        i_match_vec[31] ? tlb_lo1[31] : i_match_vec[30] ? tlb_lo1[30] :
        i_match_vec[29] ? tlb_lo1[29] : i_match_vec[28] ? tlb_lo1[28] :
        i_match_vec[27] ? tlb_lo1[27] : i_match_vec[26] ? tlb_lo1[26] :
        i_match_vec[25] ? tlb_lo1[25] : tlb_lo1[24];
    wire [29:0] i_lo1_g4 =
        i_match_vec[39] ? tlb_lo1[39] : i_match_vec[38] ? tlb_lo1[38] :
        i_match_vec[37] ? tlb_lo1[37] : i_match_vec[36] ? tlb_lo1[36] :
        i_match_vec[35] ? tlb_lo1[35] : i_match_vec[34] ? tlb_lo1[34] :
        i_match_vec[33] ? tlb_lo1[33] : tlb_lo1[32];
    wire [29:0] i_lo1_g5 =
        i_match_vec[47] ? tlb_lo1[47] : i_match_vec[46] ? tlb_lo1[46] :
        i_match_vec[45] ? tlb_lo1[45] : i_match_vec[44] ? tlb_lo1[44] :
        i_match_vec[43] ? tlb_lo1[43] : i_match_vec[42] ? tlb_lo1[42] :
        i_match_vec[41] ? tlb_lo1[41] : tlb_lo1[40];

    wire [29:0] i_lo_sel_g0 = odd_page_sel(i_vaddr, i_mask_g0) ? i_lo1_g0 : i_lo0_g0;
    wire [29:0] i_lo_sel_g1 = odd_page_sel(i_vaddr, i_mask_g1) ? i_lo1_g1 : i_lo0_g1;
    wire [29:0] i_lo_sel_g2 = odd_page_sel(i_vaddr, i_mask_g2) ? i_lo1_g2 : i_lo0_g2;
    wire [29:0] i_lo_sel_g3 = odd_page_sel(i_vaddr, i_mask_g3) ? i_lo1_g3 : i_lo0_g3;
    wire [29:0] i_lo_sel_g4 = odd_page_sel(i_vaddr, i_mask_g4) ? i_lo1_g4 : i_lo0_g4;
    wire [29:0] i_lo_sel_g5 = odd_page_sel(i_vaddr, i_mask_g5) ? i_lo1_g5 : i_lo0_g5;

    wire [31:0] i_paddr_g0 = {i_lo_sel_g0[25:18],
                              (i_lo_sel_g0[17:6] & ~i_mask_g0) | (i_vaddr[23:12] & i_mask_g0),
                              i_vaddr[11:0]};
    wire [31:0] i_paddr_g1 = {i_lo_sel_g1[25:18],
                              (i_lo_sel_g1[17:6] & ~i_mask_g1) | (i_vaddr[23:12] & i_mask_g1),
                              i_vaddr[11:0]};
    wire [31:0] i_paddr_g2 = {i_lo_sel_g2[25:18],
                              (i_lo_sel_g2[17:6] & ~i_mask_g2) | (i_vaddr[23:12] & i_mask_g2),
                              i_vaddr[11:0]};
    wire [31:0] i_paddr_g3 = {i_lo_sel_g3[25:18],
                              (i_lo_sel_g3[17:6] & ~i_mask_g3) | (i_vaddr[23:12] & i_mask_g3),
                              i_vaddr[11:0]};
    wire [31:0] i_paddr_g4 = {i_lo_sel_g4[25:18],
                              (i_lo_sel_g4[17:6] & ~i_mask_g4) | (i_vaddr[23:12] & i_mask_g4),
                              i_vaddr[11:0]};
    wire [31:0] i_paddr_g5 = {i_lo_sel_g5[25:18],
                              (i_lo_sel_g5[17:6] & ~i_mask_g5) | (i_vaddr[23:12] & i_mask_g5),
                              i_vaddr[11:0]};

    wire [29:0] i_lo_sel_e =
        i_group_match[5] ? i_lo_sel_g5 : i_group_match[4] ? i_lo_sel_g4 :
        i_group_match[3] ? i_lo_sel_g3 : i_group_match[2] ? i_lo_sel_g2 :
        i_group_match[1] ? i_lo_sel_g1 : i_lo_sel_g0;
    wire [31:0] i_paddr_e =
        i_group_match[5] ? i_paddr_g5 : i_group_match[4] ? i_paddr_g4 :
        i_group_match[3] ? i_paddr_g3 : i_group_match[2] ? i_paddr_g2 :
        i_group_match[1] ? i_paddr_g1 : i_paddr_g0;

    assign i_match  = i_any_match;
    assign i_lo_sel = i_any_match ? unpack_entrylo(i_lo_sel_e) : 64'd0;
    assign i_v      = i_any_match & i_lo_sel_e[1];
    assign i_paddr  = i_any_match ? i_paddr_e : 32'd0;

    // ================================================================
    // DTLB lookup (same dataflow form as ITLB, plus dirty bit output)
    // ================================================================
    wire [47:0] d_match_vec;
    genvar gj;
    generate
        for (gj = 0; gj < 48; gj = gj + 1) begin : g_dtlb_match
            assign d_match_vec[gj] =
                (((d_vaddr[39:13] ^ tlb_hi[gj][34:8]) & ~{15'd0, tlb_mask[gj]}) == 27'd0) &&
                (d_vaddr[63:62] == tlb_hi[gj][36:35]) &&
                ((tlb_lo0[gj][0] & tlb_lo1[gj][0]) ||
                 (tlb_hi[gj][7:0] == asid));
        end
    endgenerate

    wire [5:0] d_group_match = {
        |d_match_vec[47:40], |d_match_vec[39:32], |d_match_vec[31:24],
        |d_match_vec[23:16], |d_match_vec[15:8],  |d_match_vec[7:0]
    };

    wire        d_any_match = |d_group_match;

    wire [11:0] d_mask_g0 =
        d_match_vec[7] ? tlb_mask[7] : d_match_vec[6] ? tlb_mask[6] :
        d_match_vec[5] ? tlb_mask[5] : d_match_vec[4] ? tlb_mask[4] :
        d_match_vec[3] ? tlb_mask[3] : d_match_vec[2] ? tlb_mask[2] :
        d_match_vec[1] ? tlb_mask[1] : tlb_mask[0];
    wire [11:0] d_mask_g1 =
        d_match_vec[15] ? tlb_mask[15] : d_match_vec[14] ? tlb_mask[14] :
        d_match_vec[13] ? tlb_mask[13] : d_match_vec[12] ? tlb_mask[12] :
        d_match_vec[11] ? tlb_mask[11] : d_match_vec[10] ? tlb_mask[10] :
        d_match_vec[9]  ? tlb_mask[9]  : tlb_mask[8];
    wire [11:0] d_mask_g2 =
        d_match_vec[23] ? tlb_mask[23] : d_match_vec[22] ? tlb_mask[22] :
        d_match_vec[21] ? tlb_mask[21] : d_match_vec[20] ? tlb_mask[20] :
        d_match_vec[19] ? tlb_mask[19] : d_match_vec[18] ? tlb_mask[18] :
        d_match_vec[17] ? tlb_mask[17] : tlb_mask[16];
    wire [11:0] d_mask_g3 =
        d_match_vec[31] ? tlb_mask[31] : d_match_vec[30] ? tlb_mask[30] :
        d_match_vec[29] ? tlb_mask[29] : d_match_vec[28] ? tlb_mask[28] :
        d_match_vec[27] ? tlb_mask[27] : d_match_vec[26] ? tlb_mask[26] :
        d_match_vec[25] ? tlb_mask[25] : tlb_mask[24];
    wire [11:0] d_mask_g4 =
        d_match_vec[39] ? tlb_mask[39] : d_match_vec[38] ? tlb_mask[38] :
        d_match_vec[37] ? tlb_mask[37] : d_match_vec[36] ? tlb_mask[36] :
        d_match_vec[35] ? tlb_mask[35] : d_match_vec[34] ? tlb_mask[34] :
        d_match_vec[33] ? tlb_mask[33] : tlb_mask[32];
    wire [11:0] d_mask_g5 =
        d_match_vec[47] ? tlb_mask[47] : d_match_vec[46] ? tlb_mask[46] :
        d_match_vec[45] ? tlb_mask[45] : d_match_vec[44] ? tlb_mask[44] :
        d_match_vec[43] ? tlb_mask[43] : d_match_vec[42] ? tlb_mask[42] :
        d_match_vec[41] ? tlb_mask[41] : tlb_mask[40];

    wire [29:0] d_lo0_g0 =
        d_match_vec[7] ? tlb_lo0[7] : d_match_vec[6] ? tlb_lo0[6] :
        d_match_vec[5] ? tlb_lo0[5] : d_match_vec[4] ? tlb_lo0[4] :
        d_match_vec[3] ? tlb_lo0[3] : d_match_vec[2] ? tlb_lo0[2] :
        d_match_vec[1] ? tlb_lo0[1] : tlb_lo0[0];
    wire [29:0] d_lo0_g1 =
        d_match_vec[15] ? tlb_lo0[15] : d_match_vec[14] ? tlb_lo0[14] :
        d_match_vec[13] ? tlb_lo0[13] : d_match_vec[12] ? tlb_lo0[12] :
        d_match_vec[11] ? tlb_lo0[11] : d_match_vec[10] ? tlb_lo0[10] :
        d_match_vec[9]  ? tlb_lo0[9]  : tlb_lo0[8];
    wire [29:0] d_lo0_g2 =
        d_match_vec[23] ? tlb_lo0[23] : d_match_vec[22] ? tlb_lo0[22] :
        d_match_vec[21] ? tlb_lo0[21] : d_match_vec[20] ? tlb_lo0[20] :
        d_match_vec[19] ? tlb_lo0[19] : d_match_vec[18] ? tlb_lo0[18] :
        d_match_vec[17] ? tlb_lo0[17] : tlb_lo0[16];
    wire [29:0] d_lo0_g3 =
        d_match_vec[31] ? tlb_lo0[31] : d_match_vec[30] ? tlb_lo0[30] :
        d_match_vec[29] ? tlb_lo0[29] : d_match_vec[28] ? tlb_lo0[28] :
        d_match_vec[27] ? tlb_lo0[27] : d_match_vec[26] ? tlb_lo0[26] :
        d_match_vec[25] ? tlb_lo0[25] : tlb_lo0[24];
    wire [29:0] d_lo0_g4 =
        d_match_vec[39] ? tlb_lo0[39] : d_match_vec[38] ? tlb_lo0[38] :
        d_match_vec[37] ? tlb_lo0[37] : d_match_vec[36] ? tlb_lo0[36] :
        d_match_vec[35] ? tlb_lo0[35] : d_match_vec[34] ? tlb_lo0[34] :
        d_match_vec[33] ? tlb_lo0[33] : tlb_lo0[32];
    wire [29:0] d_lo0_g5 =
        d_match_vec[47] ? tlb_lo0[47] : d_match_vec[46] ? tlb_lo0[46] :
        d_match_vec[45] ? tlb_lo0[45] : d_match_vec[44] ? tlb_lo0[44] :
        d_match_vec[43] ? tlb_lo0[43] : d_match_vec[42] ? tlb_lo0[42] :
        d_match_vec[41] ? tlb_lo0[41] : tlb_lo0[40];

    wire [29:0] d_lo1_g0 =
        d_match_vec[7] ? tlb_lo1[7] : d_match_vec[6] ? tlb_lo1[6] :
        d_match_vec[5] ? tlb_lo1[5] : d_match_vec[4] ? tlb_lo1[4] :
        d_match_vec[3] ? tlb_lo1[3] : d_match_vec[2] ? tlb_lo1[2] :
        d_match_vec[1] ? tlb_lo1[1] : tlb_lo1[0];
    wire [29:0] d_lo1_g1 =
        d_match_vec[15] ? tlb_lo1[15] : d_match_vec[14] ? tlb_lo1[14] :
        d_match_vec[13] ? tlb_lo1[13] : d_match_vec[12] ? tlb_lo1[12] :
        d_match_vec[11] ? tlb_lo1[11] : d_match_vec[10] ? tlb_lo1[10] :
        d_match_vec[9]  ? tlb_lo1[9]  : tlb_lo1[8];
    wire [29:0] d_lo1_g2 =
        d_match_vec[23] ? tlb_lo1[23] : d_match_vec[22] ? tlb_lo1[22] :
        d_match_vec[21] ? tlb_lo1[21] : d_match_vec[20] ? tlb_lo1[20] :
        d_match_vec[19] ? tlb_lo1[19] : d_match_vec[18] ? tlb_lo1[18] :
        d_match_vec[17] ? tlb_lo1[17] : tlb_lo1[16];
    wire [29:0] d_lo1_g3 =
        d_match_vec[31] ? tlb_lo1[31] : d_match_vec[30] ? tlb_lo1[30] :
        d_match_vec[29] ? tlb_lo1[29] : d_match_vec[28] ? tlb_lo1[28] :
        d_match_vec[27] ? tlb_lo1[27] : d_match_vec[26] ? tlb_lo1[26] :
        d_match_vec[25] ? tlb_lo1[25] : tlb_lo1[24];
    wire [29:0] d_lo1_g4 =
        d_match_vec[39] ? tlb_lo1[39] : d_match_vec[38] ? tlb_lo1[38] :
        d_match_vec[37] ? tlb_lo1[37] : d_match_vec[36] ? tlb_lo1[36] :
        d_match_vec[35] ? tlb_lo1[35] : d_match_vec[34] ? tlb_lo1[34] :
        d_match_vec[33] ? tlb_lo1[33] : tlb_lo1[32];
    wire [29:0] d_lo1_g5 =
        d_match_vec[47] ? tlb_lo1[47] : d_match_vec[46] ? tlb_lo1[46] :
        d_match_vec[45] ? tlb_lo1[45] : d_match_vec[44] ? tlb_lo1[44] :
        d_match_vec[43] ? tlb_lo1[43] : d_match_vec[42] ? tlb_lo1[42] :
        d_match_vec[41] ? tlb_lo1[41] : tlb_lo1[40];

    wire [11:0] d_mask_e =
        d_group_match[5] ? d_mask_g5 : d_group_match[4] ? d_mask_g4 :
        d_group_match[3] ? d_mask_g3 : d_group_match[2] ? d_mask_g2 :
        d_group_match[1] ? d_mask_g1 : d_mask_g0;
    wire [29:0] d_lo0_e =
        d_group_match[5] ? d_lo0_g5 : d_group_match[4] ? d_lo0_g4 :
        d_group_match[3] ? d_lo0_g3 : d_group_match[2] ? d_lo0_g2 :
        d_group_match[1] ? d_lo0_g1 : d_lo0_g0;
    wire [29:0] d_lo1_e =
        d_group_match[5] ? d_lo1_g5 : d_group_match[4] ? d_lo1_g4 :
        d_group_match[3] ? d_lo1_g3 : d_group_match[2] ? d_lo1_g2 :
        d_group_match[1] ? d_lo1_g1 : d_lo1_g0;
    wire [29:0] d_lo_sel_e = odd_page_sel(d_vaddr, d_mask_e) ? d_lo1_e : d_lo0_e;
    wire [2:0] d_idx_g0 =
        d_match_vec[7] ? 3'd7 : d_match_vec[6] ? 3'd6 :
        d_match_vec[5] ? 3'd5 : d_match_vec[4] ? 3'd4 :
        d_match_vec[3] ? 3'd3 : d_match_vec[2] ? 3'd2 :
        d_match_vec[1] ? 3'd1 : 3'd0;
    wire [2:0] d_idx_g1 =
        d_match_vec[15] ? 3'd7 : d_match_vec[14] ? 3'd6 :
        d_match_vec[13] ? 3'd5 : d_match_vec[12] ? 3'd4 :
        d_match_vec[11] ? 3'd3 : d_match_vec[10] ? 3'd2 :
        d_match_vec[9]  ? 3'd1 : 3'd0;
    wire [2:0] d_idx_g2 =
        d_match_vec[23] ? 3'd7 : d_match_vec[22] ? 3'd6 :
        d_match_vec[21] ? 3'd5 : d_match_vec[20] ? 3'd4 :
        d_match_vec[19] ? 3'd3 : d_match_vec[18] ? 3'd2 :
        d_match_vec[17] ? 3'd1 : 3'd0;
    wire [2:0] d_idx_g3 =
        d_match_vec[31] ? 3'd7 : d_match_vec[30] ? 3'd6 :
        d_match_vec[29] ? 3'd5 : d_match_vec[28] ? 3'd4 :
        d_match_vec[27] ? 3'd3 : d_match_vec[26] ? 3'd2 :
        d_match_vec[25] ? 3'd1 : 3'd0;
    wire [2:0] d_idx_g4 =
        d_match_vec[39] ? 3'd7 : d_match_vec[38] ? 3'd6 :
        d_match_vec[37] ? 3'd5 : d_match_vec[36] ? 3'd4 :
        d_match_vec[35] ? 3'd3 : d_match_vec[34] ? 3'd2 :
        d_match_vec[33] ? 3'd1 : 3'd0;
    wire [2:0] d_idx_g5 =
        d_match_vec[47] ? 3'd7 : d_match_vec[46] ? 3'd6 :
        d_match_vec[45] ? 3'd5 : d_match_vec[44] ? 3'd4 :
        d_match_vec[43] ? 3'd3 : d_match_vec[42] ? 3'd2 :
        d_match_vec[41] ? 3'd1 : 3'd0;
    wire [5:0] d_match_idx_e =
        d_group_match[5] ? {3'd5, d_idx_g5} :
        d_group_match[4] ? {3'd4, d_idx_g4} :
        d_group_match[3] ? {3'd3, d_idx_g3} :
        d_group_match[2] ? {3'd2, d_idx_g2} :
        d_group_match[1] ? {3'd1, d_idx_g1} :
                           {3'd0, d_idx_g0};

    assign d_match  = d_any_match;
    assign d_lo_sel = d_any_match ? unpack_entrylo(d_lo_sel_e) : 64'd0;
    assign d_mask   = d_any_match ? unpack_pagemask(d_mask_e) : 64'd0;
    assign d_v      = d_any_match & d_lo_sel_e[1];
    wire [31:0] d_paddr_e = {d_lo_sel_e[25:18],
                             (d_lo_sel_e[17:6] & ~d_mask_e) | (d_vaddr[23:12] & d_mask_e),
                             d_vaddr[11:0]};

    assign d_d      = d_any_match & d_lo_sel_e[2];
    assign d_paddr  = d_any_match ? d_paddr_e : 32'd0;
    assign d_match_idx = d_any_match ? d_match_idx_e : 6'd63;

    // ================================================================
    // TLBP probe (same comparator + priority encoder, against probe_vaddr).
    // The original code used a procedural for-loop over a sequential always
    // block; here it is combinational so the result is available the cycle
    // TLBP commits — caller registers the result into cp0_index.
    // ================================================================
    wire [47:0] p_match_vec;
    genvar gp;
    generate
        for (gp = 0; gp < 48; gp = gp + 1) begin : g_probe_match
            assign p_match_vec[gp] =
                (((probe_vaddr[39:13] ^ tlb_hi[gp][34:8]) & ~{15'd0, tlb_mask[gp]}) == 27'd0) &&
                (probe_vaddr[63:62] == tlb_hi[gp][36:35]) &&
                ((tlb_lo0[gp][0] & tlb_lo1[gp][0]) ||
                 (tlb_hi[gp][7:0] == asid));
        end
    endgenerate

    wire [5:0] p_group_match = {
        |p_match_vec[47:40], |p_match_vec[39:32], |p_match_vec[31:24],
        |p_match_vec[23:16], |p_match_vec[15:8],  |p_match_vec[7:0]
    };

    wire [2:0] p_idx_g0 =
        p_match_vec[7] ? 3'd7 : p_match_vec[6] ? 3'd6 :
        p_match_vec[5] ? 3'd5 : p_match_vec[4] ? 3'd4 :
        p_match_vec[3] ? 3'd3 : p_match_vec[2] ? 3'd2 :
        p_match_vec[1] ? 3'd1 : 3'd0;
    wire [2:0] p_idx_g1 =
        p_match_vec[15] ? 3'd7 : p_match_vec[14] ? 3'd6 :
        p_match_vec[13] ? 3'd5 : p_match_vec[12] ? 3'd4 :
        p_match_vec[11] ? 3'd3 : p_match_vec[10] ? 3'd2 :
        p_match_vec[9]  ? 3'd1 : 3'd0;
    wire [2:0] p_idx_g2 =
        p_match_vec[23] ? 3'd7 : p_match_vec[22] ? 3'd6 :
        p_match_vec[21] ? 3'd5 : p_match_vec[20] ? 3'd4 :
        p_match_vec[19] ? 3'd3 : p_match_vec[18] ? 3'd2 :
        p_match_vec[17] ? 3'd1 : 3'd0;
    wire [2:0] p_idx_g3 =
        p_match_vec[31] ? 3'd7 : p_match_vec[30] ? 3'd6 :
        p_match_vec[29] ? 3'd5 : p_match_vec[28] ? 3'd4 :
        p_match_vec[27] ? 3'd3 : p_match_vec[26] ? 3'd2 :
        p_match_vec[25] ? 3'd1 : 3'd0;
    wire [2:0] p_idx_g4 =
        p_match_vec[39] ? 3'd7 : p_match_vec[38] ? 3'd6 :
        p_match_vec[37] ? 3'd5 : p_match_vec[36] ? 3'd4 :
        p_match_vec[35] ? 3'd3 : p_match_vec[34] ? 3'd2 :
        p_match_vec[33] ? 3'd1 : 3'd0;
    wire [2:0] p_idx_g5 =
        p_match_vec[47] ? 3'd7 : p_match_vec[46] ? 3'd6 :
        p_match_vec[45] ? 3'd5 : p_match_vec[44] ? 3'd4 :
        p_match_vec[43] ? 3'd3 : p_match_vec[42] ? 3'd2 :
        p_match_vec[41] ? 3'd1 : 3'd0;

    wire [2:0] p_group_idx =
        p_group_match[5] ? 3'd5 : p_group_match[4] ? 3'd4 :
        p_group_match[3] ? 3'd3 : p_group_match[2] ? 3'd2 :
        p_group_match[1] ? 3'd1 : 3'd0;
    wire [2:0] p_idx_low =
        p_group_match[5] ? p_idx_g5 : p_group_match[4] ? p_idx_g4 :
        p_group_match[3] ? p_idx_g3 : p_group_match[2] ? p_idx_g2 :
        p_group_match[1] ? p_idx_g1 : p_idx_g0;
    wire [5:0] p_idx = {p_group_idx, p_idx_low};

    assign probe_match = |p_group_match;
    assign probe_idx   = p_idx;

endmodule
`default_nettype wire
`endif
