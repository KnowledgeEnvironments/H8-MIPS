// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_scache.v — parameterized direct-mapped secondary cache
// Default geometry is 1024 lines x 128 bytes/line = 128KB.
// The line size is fixed at the R4000 128-byte secondary-cache line.
// Write-back with dirty bits
// Transparent memory layer between L1 caches and external memory
// On S-cache hit: data provided combinationally (zero extra latency)
// On S-cache miss: freezes requesting L1, fills from external memory

`default_nettype none

`ifdef FPGA_SCACHE
/* verilator lint_off DECLFILENAME */
/* verilator lint_off VARHIDDEN */
module r4000_scache_ram8 #(
    parameter ADDR_W = 10,
    parameter NUMWORDS = 1024
) (
    input  wire              clk,
    input  wire [ADDR_W-1:0] raddr,
    output wire [7:0]        rdata,
    input  wire              we,
    input  wire [ADDR_W-1:0] waddr,
    input  wire [7:0]        wdata
);
`ifdef VERILATOR
    reg [7:0] mem [0:NUMWORDS-1];
    reg [7:0] rdata_q;
    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`elsif FPGA_XILINX
    (* ram_style = "block" *) reg [7:0] mem [0:NUMWORDS-1];
    reg [7:0] rdata_q;
    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`else
    altsyncram ram (
        .clock0    (clk),
        .address_a (waddr),
        .data_a    (wdata),
        .wren_a    (we),
        .address_b (raddr),
        .q_b       (rdata)
    );
    defparam ram.operation_mode = "DUAL_PORT";
    defparam ram.width_a = 8;
    defparam ram.widthad_a = ADDR_W;
    defparam ram.numwords_a = NUMWORDS;
    defparam ram.width_b = 8;
    defparam ram.widthad_b = ADDR_W;
    defparam ram.numwords_b = NUMWORDS;
    defparam ram.address_aclr_a = "NONE";
    defparam ram.address_aclr_b = "NONE";
    defparam ram.address_reg_b = "CLOCK0";
    defparam ram.outdata_reg_b = "UNREGISTERED";
    defparam ram.outdata_aclr_b = "NONE";
    defparam ram.indata_aclr_a = "NONE";
    defparam ram.wrcontrol_aclr_a = "NONE";
    defparam ram.read_during_write_mode_mixed_ports = "OLD_DATA";
`ifdef FPGA_CYCLONE_IV_GX
    defparam ram.intended_device_family = "Cyclone IV GX";
`else
    defparam ram.intended_device_family = "Cyclone IV E";
`endif
    defparam ram.lpm_type = "altsyncram";
`endif
endmodule

module r4000_scache_bank64 #(
    parameter ADDR_W = 10,
    parameter NUMWORDS = 1024
) (
    input  wire        clk,
    input  wire [ADDR_W-1:0] raddr,
    output wire [63:0] rdata,
    input  wire        we,
    input  wire [7:0]  be,
    input  wire [ADDR_W-1:0] waddr,
    input  wire [63:0] wdata
);
`ifdef VERILATOR
    reg [7:0] b0 [0:NUMWORDS-1];
    reg [7:0] b1 [0:NUMWORDS-1];
    reg [7:0] b2 [0:NUMWORDS-1];
    reg [7:0] b3 [0:NUMWORDS-1];
    reg [7:0] b4 [0:NUMWORDS-1];
    reg [7:0] b5 [0:NUMWORDS-1];
    reg [7:0] b6 [0:NUMWORDS-1];
    reg [7:0] b7 [0:NUMWORDS-1];
    reg [63:0] rdata_q;
    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= {b7[raddr], b6[raddr], b5[raddr], b4[raddr],
                    b3[raddr], b2[raddr], b1[raddr], b0[raddr]};
        if (we) begin
            if (be[0]) b0[waddr] <= wdata[7:0];
            if (be[1]) b1[waddr] <= wdata[15:8];
            if (be[2]) b2[waddr] <= wdata[23:16];
            if (be[3]) b3[waddr] <= wdata[31:24];
            if (be[4]) b4[waddr] <= wdata[39:32];
            if (be[5]) b5[waddr] <= wdata[47:40];
            if (be[6]) b6[waddr] <= wdata[55:48];
            if (be[7]) b7[waddr] <= wdata[63:56];
        end
    end
`else
    wire [7:0] q0, q1, q2, q3, q4, q5, q6, q7;

    assign rdata = {q7, q6, q5, q4, q3, q2, q1, q0};

    r4000_scache_ram8 #(.ADDR_W(ADDR_W), .NUMWORDS(NUMWORDS)) b0 (.clk(clk), .raddr(raddr), .rdata(q0), .we(we && be[0]), .waddr(waddr), .wdata(wdata[7:0]));
    r4000_scache_ram8 #(.ADDR_W(ADDR_W), .NUMWORDS(NUMWORDS)) b1 (.clk(clk), .raddr(raddr), .rdata(q1), .we(we && be[1]), .waddr(waddr), .wdata(wdata[15:8]));
    r4000_scache_ram8 #(.ADDR_W(ADDR_W), .NUMWORDS(NUMWORDS)) b2 (.clk(clk), .raddr(raddr), .rdata(q2), .we(we && be[2]), .waddr(waddr), .wdata(wdata[23:16]));
    r4000_scache_ram8 #(.ADDR_W(ADDR_W), .NUMWORDS(NUMWORDS)) b3 (.clk(clk), .raddr(raddr), .rdata(q3), .we(we && be[3]), .waddr(waddr), .wdata(wdata[31:24]));
    r4000_scache_ram8 #(.ADDR_W(ADDR_W), .NUMWORDS(NUMWORDS)) b4 (.clk(clk), .raddr(raddr), .rdata(q4), .we(we && be[4]), .waddr(waddr), .wdata(wdata[39:32]));
    r4000_scache_ram8 #(.ADDR_W(ADDR_W), .NUMWORDS(NUMWORDS)) b5 (.clk(clk), .raddr(raddr), .rdata(q5), .we(we && be[5]), .waddr(waddr), .wdata(wdata[47:40]));
    r4000_scache_ram8 #(.ADDR_W(ADDR_W), .NUMWORDS(NUMWORDS)) b6 (.clk(clk), .raddr(raddr), .rdata(q6), .we(we && be[6]), .waddr(waddr), .wdata(wdata[55:48]));
    r4000_scache_ram8 #(.ADDR_W(ADDR_W), .NUMWORDS(NUMWORDS)) b7 (.clk(clk), .raddr(raddr), .rdata(q7), .we(we && be[7]), .waddr(waddr), .wdata(wdata[63:56]));
`endif
endmodule

module r4000_scache_ram64 #(
    parameter ADDR_W = 14,
    parameter NUMWORDS = 16384
) (
    input  wire        clk,
    input  wire [ADDR_W-1:0] raddr,
    output wire [63:0] rdata,
    input  wire        we,
    input  wire [7:0]  be,
    input  wire [ADDR_W-1:0] waddr,
    input  wire [63:0] wdata
);
// Full-CPU simulation fails with a large 16Kx64 reg-array RAM model in this
// submodule. Use the proven banked behavioral model for simulation while
// leaving synthesis on the single altsyncram path.
`ifdef VERILATOR
`ifndef FPGA_SCACHE_4BANK
`ifndef FPGA_SCACHE_16BANK
`define FPGA_SCACHE_16BANK
`endif
`endif
`endif

`ifdef FPGA_SCACHE_4BANK
    wire [63:0] q0, q1, q2, q3;
    wire [1:0] rbank = raddr[1:0];
    wire [1:0] wbank = waddr[1:0];
    reg  [1:0] rbank_q;

    always @(posedge clk)
        rbank_q <= rbank;

    assign rdata = (rbank_q == 2'd0) ? q0 :
                   (rbank_q == 2'd1) ? q1 :
                   (rbank_q == 2'd2) ? q2 : q3;

    r4000_scache_bank64 #(.ADDR_W(ADDR_W-2), .NUMWORDS(NUMWORDS/4)) b0 (.clk(clk), .raddr(raddr[ADDR_W-1:2]), .rdata(q0), .we(we && wbank == 2'd0), .be(be), .waddr(waddr[ADDR_W-1:2]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-2), .NUMWORDS(NUMWORDS/4)) b1 (.clk(clk), .raddr(raddr[ADDR_W-1:2]), .rdata(q1), .we(we && wbank == 2'd1), .be(be), .waddr(waddr[ADDR_W-1:2]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-2), .NUMWORDS(NUMWORDS/4)) b2 (.clk(clk), .raddr(raddr[ADDR_W-1:2]), .rdata(q2), .we(we && wbank == 2'd2), .be(be), .waddr(waddr[ADDR_W-1:2]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-2), .NUMWORDS(NUMWORDS/4)) b3 (.clk(clk), .raddr(raddr[ADDR_W-1:2]), .rdata(q3), .we(we && wbank == 2'd3), .be(be), .waddr(waddr[ADDR_W-1:2]), .wdata(wdata));
`elsif FPGA_SCACHE_16BANK
    wire [63:0] q0, q1, q2, q3, q4, q5, q6, q7;
    wire [63:0] q8, q9, q10, q11, q12, q13, q14, q15;
    wire [3:0] rbank = raddr[3:0];
    wire [3:0] wbank = waddr[3:0];
    reg  [3:0] rbank_q;

    always @(posedge clk)
        rbank_q <= rbank;

    assign rdata = (rbank_q == 4'd0)  ? q0  :
                   (rbank_q == 4'd1)  ? q1  :
                   (rbank_q == 4'd2)  ? q2  :
                   (rbank_q == 4'd3)  ? q3  :
                   (rbank_q == 4'd4)  ? q4  :
                   (rbank_q == 4'd5)  ? q5  :
                   (rbank_q == 4'd6)  ? q6  :
                   (rbank_q == 4'd7)  ? q7  :
                   (rbank_q == 4'd8)  ? q8  :
                   (rbank_q == 4'd9)  ? q9  :
                   (rbank_q == 4'd10) ? q10 :
                   (rbank_q == 4'd11) ? q11 :
                   (rbank_q == 4'd12) ? q12 :
                   (rbank_q == 4'd13) ? q13 :
                   (rbank_q == 4'd14) ? q14 : q15;

    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b0  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q0),  .we(we && wbank == 4'd0),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b1  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q1),  .we(we && wbank == 4'd1),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b2  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q2),  .we(we && wbank == 4'd2),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b3  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q3),  .we(we && wbank == 4'd3),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b4  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q4),  .we(we && wbank == 4'd4),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b5  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q5),  .we(we && wbank == 4'd5),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b6  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q6),  .we(we && wbank == 4'd6),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b7  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q7),  .we(we && wbank == 4'd7),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b8  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q8),  .we(we && wbank == 4'd8),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b9  (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q9),  .we(we && wbank == 4'd9),  .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b10 (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q10), .we(we && wbank == 4'd10), .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b11 (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q11), .we(we && wbank == 4'd11), .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b12 (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q12), .we(we && wbank == 4'd12), .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b13 (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q13), .we(we && wbank == 4'd13), .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b14 (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q14), .we(we && wbank == 4'd14), .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
    r4000_scache_bank64 #(.ADDR_W(ADDR_W-4), .NUMWORDS(NUMWORDS/16)) b15 (.clk(clk), .raddr(raddr[ADDR_W-1:4]), .rdata(q15), .we(we && wbank == 4'd15), .be(be), .waddr(waddr[ADDR_W-1:4]), .wdata(wdata));
`else
`ifdef VERILATOR
    reg [63:0] mem [0:NUMWORDS-1];
    reg [63:0] rdata_q;
    integer b;

    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we) begin
            for (b = 0; b < 8; b = b + 1) begin
                if (be[b])
                    mem[waddr][b*8 +: 8] <= wdata[b*8 +: 8];
            end
        end
    end
`else
    r4000_scache_bank64 #(
        .ADDR_W(ADDR_W),
        .NUMWORDS(NUMWORDS)
    ) flat (
        .clk(clk),
        .raddr(raddr),
        .rdata(rdata),
        .we(we),
        .be(be),
        .waddr(waddr),
        .wdata(wdata)
    );
`endif
`endif
endmodule

module r4000_scache_tag_port #(
    parameter ADDR_W = 10,
    parameter TAG_W  = 15,
    parameter NUMWORDS = 1024
) (
    input  wire              clk,
    input  wire [ADDR_W-1:0] raddr,
    output wire [TAG_W-1:0]  rdata,
    input  wire              we,
    input  wire [ADDR_W-1:0] waddr,
    input  wire [TAG_W-1:0]  wdata
);
`ifdef VERILATOR
    reg [TAG_W-1:0] mem [0:NUMWORDS-1];
    reg [TAG_W-1:0] rdata_q;

    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`elsif FPGA_XILINX
    (* ram_style = "block" *) reg [TAG_W-1:0] mem [0:NUMWORDS-1];
    reg [TAG_W-1:0] rdata_q;

    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`else
    altsyncram ram (
        .clock0    (clk),
        .address_a (waddr),
        .data_a    (wdata),
        .wren_a    (we),
        .address_b (raddr),
        .q_b       (rdata)
    );
    defparam ram.operation_mode = "DUAL_PORT";
    defparam ram.width_a = TAG_W;
    defparam ram.widthad_a = ADDR_W;
    defparam ram.numwords_a = NUMWORDS;
    defparam ram.width_b = TAG_W;
    defparam ram.widthad_b = ADDR_W;
    defparam ram.numwords_b = NUMWORDS;
    defparam ram.address_aclr_a = "NONE";
    defparam ram.address_aclr_b = "NONE";
    defparam ram.address_reg_b = "CLOCK0";
    defparam ram.outdata_reg_b = "UNREGISTERED";
    defparam ram.outdata_aclr_b = "NONE";
    defparam ram.indata_aclr_a = "NONE";
    defparam ram.wrcontrol_aclr_a = "NONE";
    defparam ram.read_during_write_mode_mixed_ports = "OLD_DATA";
`ifdef FPGA_CYCLONE_IV_GX
    defparam ram.intended_device_family = "Cyclone IV GX";
`else
    defparam ram.intended_device_family = "Cyclone IV E";
`endif
    defparam ram.lpm_type = "altsyncram";
`endif
endmodule

// Valid/dirty metadata has five independent asynchronous lookup ports but one
// canonical update stream.  Replicating this small LUT RAM is far cheaper than
// synthesizing five 8192-to-1 mux trees from resettable flip-flop arrays.
module r4000_scache_meta_port #(
    parameter ADDR_W = 10,
    parameter NUMWORDS = 1024
) (
    input  wire              clk,
    input  wire [ADDR_W-1:0] raddr,
    output wire [1:0]        rdata,
    input  wire              we,
    input  wire [ADDR_W-1:0] waddr,
    input  wire [1:0]        wdata
);
    // Encoding is {valid, dirty}.  The array is deliberately not reset;
    // r4000_scache sweeps the common write port after every reset.
    (* ram_style = "distributed" *) reg [1:0] mem [0:NUMWORDS-1];

    assign rdata = mem[raddr];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
    end
endmodule
/* verilator lint_on DECLFILENAME */
/* verilator lint_on VARHIDDEN */
`endif

module r4000_scache #(
    parameter SCACHE_LINES = 1024,
    parameter FULL_ICACHE_INCLUSION = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    // I-cache memory-side interface (replaces external imem from I-cache's perspective)
    input  wire [31:0] i_addr,        // from I-cache mem_addr
    input  wire        i_valid,       // IS-stage instruction fetch is valid
    input  wire        i_need_fill,   // from I-cache: has miss or actively filling
    input  wire        i_uncached,    // bypass S-cache
    output wire [31:0] i_rdata,       // to I-cache fetch/uncached path
    output wire [31:0] i_fill_rdata,  // to I-cache fill RAM write path
    output wire        i_freeze,      // hold I-cache state machine

    // I-cache writeback interface (for CACHE PI|HWB)
    input  wire        i_wen,         // I-cache writeback write enable
    input  wire [63:0] i_wdata,       // I-cache writeback data (64-bit doubleword)

    // D-cache memory-side interface (replaces external dmem from D-cache's perspective)
    input  wire [31:0] d_addr,        // from D-cache mem_addr
    input  wire        d_ren,         // from D-cache mem_ren
    input  wire        d_wen,         // from D-cache mem_wen
    input  wire [ 7:0] d_wmask,       // from D-cache mem_wmask
    input  wire [63:0] d_wdata,       // from D-cache mem_wdata
    input  wire        d_uncached,    // bypass S-cache
    input  wire [31:0] d_mem_pc,      // PC of the D-side memory instruction
    input  wire        force_uncached, // Gate A diag: suppress L2 snoop on uncached too
    output wire [63:0] d_rdata,       // to D-cache mem_rdata
    output wire        d_freeze,      // hold D-cache state machine

    // CACHE instruction (type 2/3 = secondary cache)
    input  wire        cop_valid,
    input  wire [2:0]  cop_op,
    input  wire [31:0] cop_paddr,
    input  wire [31:0] cop_taglo_in,   // TagLo value for Index Store Tag
    output wire [31:0] cop_taglo_out,  // Tag readback for Index Load Tag
    output wire        cop_taglo_valid, // Readback is ready for CP0 TagLo

    // External instruction memory (32-bit, for uncached I reads)
    output wire [31:0] ext_imem_addr,
    input  wire [31:0] ext_imem_rdata,

    // External data memory (64-bit, for S-cache fills/writebacks and uncached D)
    output reg  [31:0] ext_dmem_addr,
    output reg         ext_dmem_ren,
    output reg         ext_dmem_wen,
    output reg  [ 7:0] ext_dmem_wmask,
    output reg  [63:0] ext_dmem_wdata,
    input  wire [63:0] ext_dmem_rdata,
    input  wire        ext_dmem_ready,  // memory response valid (for FPGA/multi-cycle)

    // Stall output (contributes to cache_stall)
    output wire        stall,

    // Uncached write indicator: high when ext_dmem_wen is from a CPU uncached
    // store (not from S-cache writeback). Used by MC snoop to invalidate stale
    // S-cache entries when uncached writes update RAM directly.
    output wire        ext_dmem_uncached_wr,

    // High only for an external fill/writeback already serialized by this
    // cache controller. The system arbiter uses it to drain cache-owned work
    // ahead of a coherence probe without admitting a new CPU transaction.
    output wire        ext_dmem_cache_owned,

    // Uncached request-buffer start pulse.  Unlike stall/pending, this proves
    // that a request will survive core-side interrupt suppression.
    output wire        uncached_started,

    // Uncached transaction accepted pulse (for core to suppress re-issue)
    output wire        uncached_accepted,

    // DMA snoop port: keep S-cache coherent with bus-master writes.
    input  wire        snoop_valid,
    input  wire [31:0] snoop_addr,
    input  wire [63:0] snoop_wdata,
    input  wire [ 7:0] snoop_wmask,
    output wire        snoop_wr_ready,
    output reg         snoop_wr_done,

    // DMA read snoop port: return S-cache data for bus-master reads (MC snooping)
    // When a bus master reads from RAM, the MC checks S-cache for dirty data.
    // If the requested address is dirty in S-cache, the cache data is returned
    // instead of (stale) RAM data.
    input  wire        snoop_rd_valid,
    input  wire [31:0] snoop_rd_addr,
    output wire        snoop_rd_resp_valid,
    output wire        snoop_rd_hit,
    output wire [63:0] snoop_rd_data,

    // D-cache back-invalidation port (S-cache inclusion maintenance)
    // Pulses when S-cache evicts/invalidates a valid line, so D-cache
    // can invalidate any matching entries and maintain inclusion.
    output reg         backinv_valid,
    output reg  [31:0] backinv_addr,
    output reg         backinv_cacheop,
    output reg         backinv_icache,
    output reg         backinv_writeback,
    output wire        backinv_ready,
    input  wire        backinv_done,

    output wire [31:0] debug_status,
    output wire [127:0] debug_i_capture,
    output wire [383:0] debug_scache_probe,
    output wire [31:0] debug_addr
);

    function integer clog2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (clog2 = 0; v > 0; clog2 = clog2 + 1)
                v = v >> 1;
        end
    endfunction

    localparam LINE_DWS = 16;
    localparam IDX_W = clog2(SCACHE_LINES);
    localparam TAG_W = 32 - 7 - IDX_W;
    localparam DATA_ADDR_W = IDX_W + 4;
    localparam SCACHE_DATA_DWS = SCACHE_LINES * LINE_DWS;

`ifdef FPGA_SCACHE_BYPASS
    assign backinv_ready = 1'b0;
    assign snoop_wr_ready = 1'b1;
    // Synthesis bring-up path: no secondary-cache storage. This keeps the
    // frozen module interface and lets Quartus report CPU fit/timing while
    // the 128KB FPGA S-cache RAM implementation is isolated separately.
    assign ext_imem_addr = i_addr;
    assign i_rdata = ext_imem_rdata;
    assign i_fill_rdata = ext_imem_rdata;
    assign i_freeze = 1'b0;

    assign d_rdata = ext_dmem_rdata;
    assign d_freeze = (d_ren || d_wen) && !ext_dmem_ready;
    assign stall = d_freeze;
    assign ext_dmem_uncached_wr = d_wen;
    assign ext_dmem_cache_owned = 1'b0;
    assign uncached_started = uncached_accepted;
    assign uncached_accepted = d_uncached && (d_ren || d_wen) && ext_dmem_ready;

    assign cop_taglo_out = 32'd0;
    assign cop_taglo_valid = 1'b0;
    assign snoop_rd_resp_valid = snoop_rd_valid;
    assign snoop_rd_hit = 1'b0;
    assign snoop_rd_data = 64'd0;
    assign debug_status = {4'h0, 22'd0, d_freeze, stall,
                           ext_dmem_ren, ext_dmem_wen,
                           ext_dmem_ready, ext_dmem_uncached_wr};
    assign debug_i_capture = 128'd0;
    assign debug_scache_probe = 384'd0;
    assign debug_addr = ext_dmem_addr;

    always @(*) begin
        ext_dmem_addr  = d_addr;
        ext_dmem_ren   = d_ren;
        ext_dmem_wen   = d_wen;
        ext_dmem_wmask = d_wen ? d_wmask : 8'h00;
        ext_dmem_wdata = d_wdata;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            backinv_valid <= 1'b0;
            backinv_addr  <= 32'd0;
            backinv_cacheop <= 1'b0;
            backinv_icache <= 1'b0;
            backinv_writeback <= 1'b0;
            snoop_wr_done <= 1'b0;
        end else begin
            backinv_valid <= 1'b0;
            backinv_addr  <= 32'd0;
            backinv_cacheop <= 1'b0;
            backinv_icache <= 1'b0;
            backinv_writeback <= 1'b0;
            snoop_wr_done <= snoop_valid;
        end
    end
`elsif FPGA_SCACHE
    // Quartus 13.1 cannot map the simulation S-cache's zero-latency,
    // multi-read asynchronous data array into Cyclone IV M9Ks.  Keep the
    // FPGA implementation uses one parameterized synchronous RAM. D-side
    // reads, writeback, and DMA snoop reads take priority on the read port;
    // D-side stores use the RAM write port and do not by themselves block
    // I-side fills.
    localparam NLINES = SCACHE_LINES;

    localparam [3:0]
        S_IDLE      = 4'd0,
        S_WB_PREP   = 4'd1,
        S_WB        = 4'd2,
        S_DFILL     = 4'd3,
        S_IFILL     = 4'd4,
        S_DONE      = 4'd5,
        S_META_WAIT = 4'd6,
        S_CACHE_WB  = 4'd7,
        S_IUNCACHED = 4'd8,
        S_CDE_ZERO  = 4'd9,
        S_BACKINV_WAIT = 4'd10;

    reg [3:0] state;
    reg [3:0] fill_cnt;
    reg [31:0] fill_addr;
    reg        fill_is_i;
    reg [3:0] wb_cnt;
    reg [IDX_W-1:0] wb_idx;
    reg [TAG_W-1:0] wb_tag;
    reg       wb_is_cache_op;
    reg       cache_wb_keep_valid;
    reg       cache_wb_install_cde;
    reg       backinv_pending_valid;
    reg [31:0] backinv_pending_addr;
    reg       backinv_pending_cacheop;
    reg       backinv_pending_icache;
    reg       backinv_victim_dirty;
    reg       backinv_intervention_dirty;
    reg [1:0] fill_first;
    reg [31:0] uncached_i_addr;
    reg [63:0] uncached_i_rdata;
    reg        uncached_i_valid;

    // Metadata RAMs cannot have an asynchronous array reset.  Hold the cache
    // stalled after reset and clear one line per clock through their common
    // write port.
    reg meta_init_active;
    reg [IDX_W-1:0] meta_init_idx;

    wire i_addr_high_lowalias_vector_page = (i_addr[31:12] == 20'h08000);
    wire [31:0] i_scache_addr =
        i_addr_high_lowalias_vector_page ? {20'd0, i_addr[11:0]} : i_addr;
    wire d_addr_low_vector_page = (d_addr[31:12] == 20'h00000);
    wire d_addr_high_lowalias_vector_page = (d_addr[31:12] == 20'h08000);
    wire d_addr_ip20_vector_page =
        d_addr_low_vector_page || d_addr_high_lowalias_vector_page;
    wire [31:0] d_scache_addr =
        d_addr_high_lowalias_vector_page ? {20'd0, d_addr[11:0]} : d_addr;

    wire [IDX_W-1:0] i_idx    = i_scache_addr[IDX_W+6:7];
    wire [TAG_W-1:0] i_tag    = i_scache_addr[31:IDX_W+7];
    wire [3:0]       i_dw_off = i_scache_addr[6:3];
    wire [IDX_W-1:0] d_idx    = d_scache_addr[IDX_W+6:7];
    wire [TAG_W-1:0] d_tag    = d_scache_addr[31:IDX_W+7];
    wire [3:0]       d_dw_off = d_scache_addr[6:3];

    // CACHE operations are WB-stage pulses from the core.  Hold one until the
    // FPGA S-cache metadata port and state machine can actually accept it.
    reg              cop_latched;
    reg [2:0]        cop_latched_op;
    reg [31:0]       cop_latched_paddr;
    reg [31:0]       cop_latched_taglo;
    wire [2:0]       cop_eff_op = cop_latched ? cop_latched_op : cop_op;
    wire [31:0]      cop_eff_paddr = cop_latched ? cop_latched_paddr : cop_paddr;
    wire [31:0]      cop_eff_taglo = cop_latched ? cop_latched_taglo : cop_taglo_in;
    // A platform may map low vectors through a high physical alias, while
    // primary I-cache lines for exception fetches are tagged with low paddr.
    // Normalize that alias for S-cache lookup/fill/CACHE-op state so cached
    // vector edits through 0x08000xxx cannot strand dirty data under a high tag.
    wire cop_addr_high_lowalias_vector_page =
        (cop_eff_paddr[31:12] == 20'h08000);
    wire [31:0] cop_scache_addr =
        cop_addr_high_lowalias_vector_page ?
        {20'd0, cop_eff_paddr[11:0]} : cop_eff_paddr;
    wire [IDX_W-1:0] cop_idx  = cop_scache_addr[IDX_W+6:7];
    wire [TAG_W-1:0] cop_tag  = cop_scache_addr[31:IDX_W+7];
    wire [31:0] cop_target_backinv_addr =
        {cop_tag, cop_idx, 7'b0};
    wire [IDX_W-1:0] snoop_idx = snoop_addr[IDX_W+6:7];
    wire [TAG_W-1:0] snoop_tag = snoop_addr[31:IDX_W+7];
    wire [3:0]       snoop_dw  = snoop_addr[6:3];
    wire [IDX_W-1:0] snoop_rd_idx = snoop_rd_addr[IDX_W+6:7];
    wire [TAG_W-1:0] snoop_rd_tag = snoop_rd_addr[31:IDX_W+7];
    wire [3:0]       snoop_rd_dw  = snoop_rd_addr[6:3];

    wire [IDX_W-1:0] fill_idx = fill_addr[IDX_W+6:7];
    reg              meta_we;
    reg [IDX_W-1:0]  meta_waddr;
    reg [1:0]        meta_wdata;
    wire [1:0] i_meta_now;
    wire [1:0] d_meta_now;
    wire [1:0] cop_meta_now;
    wire [1:0] snoop_meta_now;
    wire [1:0] snoop_rd_meta_now;

    r4000_scache_meta_port #(.ADDR_W(IDX_W), .NUMWORDS(NLINES)) u_meta_i (
        .clk(clk), .raddr(i_idx), .rdata(i_meta_now),
        .we(meta_we), .waddr(meta_waddr), .wdata(meta_wdata)
    );
    r4000_scache_meta_port #(.ADDR_W(IDX_W), .NUMWORDS(NLINES)) u_meta_d (
        .clk(clk), .raddr(d_idx), .rdata(d_meta_now),
        .we(meta_we), .waddr(meta_waddr), .wdata(meta_wdata)
    );
    r4000_scache_meta_port #(.ADDR_W(IDX_W), .NUMWORDS(NLINES)) u_meta_cop (
        .clk(clk), .raddr(cop_idx), .rdata(cop_meta_now),
        .we(meta_we), .waddr(meta_waddr), .wdata(meta_wdata)
    );
    r4000_scache_meta_port #(.ADDR_W(IDX_W), .NUMWORDS(NLINES)) u_meta_snoop (
        .clk(clk), .raddr(snoop_idx), .rdata(snoop_meta_now),
        .we(meta_we), .waddr(meta_waddr), .wdata(meta_wdata)
    );
    r4000_scache_meta_port #(.ADDR_W(IDX_W), .NUMWORDS(NLINES)) u_meta_snoop_rd (
        .clk(clk), .raddr(snoop_rd_idx), .rdata(snoop_rd_meta_now),
        .we(meta_we), .waddr(meta_waddr), .wdata(meta_wdata)
    );

    wire i_meta_valid_now        = i_meta_now[1];
    wire i_meta_dirty_now        = i_meta_now[0];
    wire d_meta_valid_now        = d_meta_now[1];
    wire d_meta_dirty_now        = d_meta_now[0];
    wire cop_meta_valid_now      = cop_meta_now[1];
    wire cop_meta_dirty_now      = cop_meta_now[0];
    wire snoop_meta_valid_now    = snoop_meta_now[1];
    wire snoop_rd_meta_valid_now = snoop_rd_meta_now[1];
    wire snoop_rd_meta_dirty_now = snoop_rd_meta_now[0];

    // r4000_top suppresses i_uncached whenever either request is present, so
    // retaining it here creates a false I-cache-tag feedback dependency.
    wire i_need_sc = i_need_fill || i_wen;
    wire d_need_sc = (d_ren || d_wen) && !d_uncached;
    // CPU uncached reads bypass S-cache.  Uncached writes still do a tag
    // lookup so a matching cached line can be kept coherent with RAM.
    wire d_lookup_sc = (d_ren || d_wen) && (!d_uncached || (!force_uncached && d_wen));

    wire [TAG_W-1:0] i_meta_line_tag_q;
    wire [TAG_W-1:0] d_meta_line_tag_q;
    wire [TAG_W-1:0] cop_meta_line_tag_q;
    wire [TAG_W-1:0] snoop_meta_line_tag_q;
    wire [TAG_W-1:0] snoop_rd_meta_line_tag_q;

    reg              tag_we;
    reg [IDX_W-1:0]  tag_waddr;
    reg [TAG_W-1:0]  tag_wdata;

    r4000_scache_tag_port #(.ADDR_W(IDX_W), .TAG_W(TAG_W), .NUMWORDS(NLINES)) u_tag_i (
        .clk(clk), .raddr(i_idx), .rdata(i_meta_line_tag_q),
        .we(tag_we), .waddr(tag_waddr), .wdata(tag_wdata)
    );
    r4000_scache_tag_port #(.ADDR_W(IDX_W), .TAG_W(TAG_W), .NUMWORDS(NLINES)) u_tag_d (
        .clk(clk), .raddr(d_idx), .rdata(d_meta_line_tag_q),
        .we(tag_we), .waddr(tag_waddr), .wdata(tag_wdata)
    );
    r4000_scache_tag_port #(.ADDR_W(IDX_W), .TAG_W(TAG_W), .NUMWORDS(NLINES)) u_tag_cop (
        .clk(clk), .raddr(cop_idx), .rdata(cop_meta_line_tag_q),
        .we(tag_we), .waddr(tag_waddr), .wdata(tag_wdata)
    );
    r4000_scache_tag_port #(.ADDR_W(IDX_W), .TAG_W(TAG_W), .NUMWORDS(NLINES)) u_tag_snoop (
        .clk(clk), .raddr(snoop_idx), .rdata(snoop_meta_line_tag_q),
        .we(tag_we), .waddr(tag_waddr), .wdata(tag_wdata)
    );
    r4000_scache_tag_port #(.ADDR_W(IDX_W), .TAG_W(TAG_W), .NUMWORDS(NLINES)) u_tag_snoop_rd (
        .clk(clk), .raddr(snoop_rd_idx), .rdata(snoop_rd_meta_line_tag_q),
        .we(tag_we), .waddr(tag_waddr), .wdata(tag_wdata)
    );

    always @(*) begin
        tag_we    = 1'b0;
        tag_waddr = {IDX_W{1'b0}};
        tag_wdata = {TAG_W{1'b0}};

        if (rst_n) begin
            if (state == S_IDLE && cop_accept) begin
                if (cop_eff_op == 3'd2) begin
                    tag_we    = 1'b1;
                    tag_waddr = cop_idx;
                    tag_wdata = cop_eff_taglo[31 -: TAG_W];
                end
            end else if (state == S_CDE_ZERO && fill_cnt == 4'd15) begin
                tag_we    = 1'b1;
                tag_waddr = fill_idx;
                tag_wdata = fill_addr[31:IDX_W+7];
            end else if (state == S_DONE) begin
                tag_we    = 1'b1;
                tag_waddr = fill_idx;
                tag_wdata = fill_addr[31:IDX_W+7];
            end
        end
    end

    reg              i_meta_sample_q;
    reg [IDX_W-1:0]  i_meta_idx_q;
    reg [TAG_W-1:0]  i_meta_addr_tag_q;
    reg              i_meta_line_valid_q;
    reg              i_meta_line_dirty_q;
    reg              i_dirty_pending_q;
    reg [IDX_W-1:0]  i_dirty_idx_q;
    reg [TAG_W-1:0]  i_dirty_tag_q;

    reg              d_meta_sample_q;
    reg [IDX_W-1:0]  d_meta_idx_q;
    reg [TAG_W-1:0]  d_meta_addr_tag_q;
    reg              d_meta_line_valid_q;
    reg              d_meta_line_dirty_q;

    reg              cop_meta_sample_q;
    reg [IDX_W-1:0]  cop_meta_idx_q;
    reg [TAG_W-1:0]  cop_meta_addr_tag_q;
    reg              cop_meta_line_valid_q;
    reg              cop_meta_line_dirty_q;
    reg              cop_accepted_q;
    reg [2:0]        cop_accepted_op_q;
    reg [31:0]       cop_accepted_paddr_q;
    reg [31:0]       cop_accepted_taglo_q;

    reg              snoop_meta_sample_q;
    reg              snoop_meta_req_q;
    reg [IDX_W-1:0]  snoop_meta_idx_q;
    reg [TAG_W-1:0]  snoop_meta_addr_tag_q;
    reg              snoop_meta_line_valid_q;
    reg [3:0]        snoop_meta_dw_q;
    reg [63:0]       snoop_meta_wdata_q;
    reg [7:0]        snoop_meta_wmask_q;
    reg              snoop_fill_match_q;

    reg              snoop_rd_meta_sample_q;
    reg              snoop_rd_meta_req_q;
    reg [IDX_W-1:0]  snoop_rd_meta_idx_q;
    reg [TAG_W-1:0]  snoop_rd_meta_addr_tag_q;
    reg              snoop_rd_meta_line_valid_q;
    reg              snoop_rd_meta_line_dirty_q;
    reg [3:0]        snoop_rd_meta_dw_q;

    wire snoop_meta_hit_q = snoop_meta_sample_q && snoop_meta_req_q &&
                            snoop_meta_line_valid_q &&
                            (snoop_meta_line_tag_q == snoop_meta_addr_tag_q);
    wire meta_aux_write_busy = snoop_meta_hit_q || i_dirty_pending_q;

    wire uncached_d_req = d_uncached && (d_ren || d_wen);

    wire i_meta_addr_ready = i_meta_sample_q &&
                             (i_meta_idx_q == i_idx) &&
                             (i_meta_addr_tag_q == i_tag);
    wire i_meta_ready = i_need_sc && i_meta_addr_ready;
    wire d_meta_ready = d_lookup_sc && d_meta_sample_q &&
                        (d_meta_idx_q == d_idx) &&
                        (d_meta_addr_tag_q == d_tag);
    wire cop_replay = cop_accepted_q && cop_valid &&
                      (cop_accepted_op_q == cop_op) &&
                      (cop_accepted_paddr_q == cop_paddr) &&
                      (cop_accepted_taglo_q == cop_taglo_in);
    wire cop_pending = cop_latched || (cop_valid && !cop_replay);
    wire cop_meta_ready = cop_pending && cop_meta_sample_q &&
                          (cop_meta_idx_q == cop_idx) &&
                          (cop_meta_addr_tag_q == cop_tag);
    wire cop_accept = (state == S_IDLE) && !backinv_pending_valid &&
                      !meta_aux_write_busy &&
                      !uncached_d_req &&
                      cop_pending && cop_meta_ready;

    wire i_sc_line_hit = i_meta_addr_ready && i_meta_line_valid_q &&
                         (i_meta_line_tag_q == i_tag);
    wire i_sc_hit = i_need_sc && i_sc_line_hit;
    wire d_sc_hit = d_meta_ready && d_meta_line_valid_q && (d_meta_line_tag_q == d_tag);
    wire i_sc_miss = i_need_sc && i_meta_ready && !i_sc_hit;
    wire d_sc_miss = d_need_sc && d_meta_ready && !d_sc_hit;
    wire i_meta_wait = i_need_sc && !i_meta_ready;
    wire d_meta_wait = d_lookup_sc && !d_meta_ready;
    wire cop_meta_wait = cop_pending && !cop_meta_ready;
    wire i_uncached_bypass = i_uncached && !i_need_fill && !i_wen;
    wire i_uncached_prom = i_uncached_bypass && (i_addr[31:19] == 13'h3F8);
    wire i_uncached_ext_req = i_valid && i_uncached_bypass && !i_uncached_prom;
    wire i_uncached_ext_hit = i_uncached_ext_req && uncached_i_valid &&
                              (uncached_i_addr[31:3] == i_addr[31:3]);
    wire i_uncached_ext_miss = i_uncached_ext_req && !i_uncached_ext_hit;
    wire [31:0] i_uncached_ext_word =
        i_addr[2] ? uncached_i_rdata[31:0] : uncached_i_rdata[63:32];

    wire [63:0] cache_dw_q;
    wire [63:0] i_sc_dw_q = cache_dw_q;
    wire [63:0] d_sc_dw_q = cache_dw_q;
    wire [DATA_ADDR_W-1:0] i_cache_rd_addr = {i_idx, i_dw_off};
    wire [DATA_ADDR_W-1:0] d_cache_rd_addr = {d_idx, d_dw_off};
    wire d_read_sc = d_ren && d_lookup_sc;
    // A Primary-I write names a cached CACHE-operation target regardless of
    // the issuing instruction's fetch mode.  i_wen therefore qualifies both
    // the lookup and write without i_need_fill/i_uncached feedback.
    wire i_write_sc = (state == S_IDLE) && i_wen && i_sc_line_hit;
    wire d_write_sc = (state == S_IDLE) && d_wen && !d_uncached && d_sc_hit;
    wire d_intervention_write_sc = (state == S_BACKINV_WAIT) &&
                                    d_wen &&
                                    ({d_scache_addr[31:7], 7'b0} ==
                                     {wb_tag, wb_idx, 7'b0});
    wire i_write_hits_d_victim = i_write_sc && (i_idx == d_idx);
    wire d_write_hits_i_victim = d_write_sc && (d_idx == i_idx);
    wire i_dirty_pending_hits_d_victim =
        i_dirty_pending_q && (i_dirty_idx_q == d_idx) &&
        (i_dirty_tag_q == d_meta_line_tag_q);
    wire i_dirty_pending_hits_i_victim =
        i_dirty_pending_q && (i_dirty_idx_q == i_idx) &&
        (i_dirty_tag_q == i_meta_line_tag_q);
    wire d_victim_dirty = d_meta_line_valid_q &&
                          (d_meta_line_dirty_q || i_write_hits_d_victim ||
                           i_dirty_pending_hits_d_victim);
    wire i_victim_dirty = i_meta_line_valid_q &&
                          (i_meta_line_dirty_q || d_write_hits_i_victim ||
                           i_dirty_pending_hits_i_victim);
    wire [31:0] d_victim_addr = {d_meta_line_tag_q, d_idx, 7'b0};
    wire [31:0] i_victim_addr = {i_meta_line_tag_q, i_idx, 7'b0};
    wire d_victim_low_vector =
        d_meta_line_valid_q && (d_victim_addr[31:12] == 20'd0);
    wire i_victim_low_vector =
        i_meta_line_valid_q && (i_victim_addr[31:12] == 20'd0);
    wire d_mem_pc_low_vector =
        d_mem_pc[31] && (d_mem_pc[28:12] == 17'd0);
    wire d_victim_dmem_vector_line =
        d_victim_low_vector && d_mem_pc_low_vector &&
        (d_victim_addr[11:7] == d_mem_pc[11:7]);
    // A D-side miss can alias the S-cache index of the instruction that
    // issued it.  Invalidating that live I-cache line while the pipeline is
    // frozen creates an inclusion livelock: the I refill evicts the pending
    // D line, the retried D access evicts the I line, and the instruction can
    // never retire.  The low-vector path has always suppressed this case.
    // Use the current physical I-side line to provide the same safeguard for
    // ordinary kernel/user text when full inclusion is enabled.  Unrelated
    // victims (including reused executable pages) are still back-invalidated.
    wire d_victim_current_i_line =
        d_meta_line_valid_q && i_valid && !i_uncached &&
        (d_victim_addr[31:7] == i_scache_addr[31:7]);
    // Every valid S-cache victim invalidates a matching primary I-cache line
    // only when full inclusion is enabled.  Uniprocessor R4000SC systems use
    // NoMPmode instead: ordinary secondary replacement does not invalidate a
    // primary line.  Explicit cache operations and DMA snoops provide the
    // required invalidation paths without creating an I/D alias livelock.
    wire d_victim_icache_stale =
        (FULL_ICACHE_INCLUSION != 0) ?
            (d_meta_line_valid_q && !d_victim_current_i_line) :
        (d_victim_low_vector && !d_victim_dmem_vector_line);
    wire i_victim_icache_stale =
        (FULL_ICACHE_INCLUSION != 0) ? i_meta_line_valid_q : i_victim_low_vector;
    wire snoop_rd_tag_match_q = snoop_rd_meta_line_valid_q &&
                                (snoop_rd_meta_line_tag_q ==
                                 snoop_rd_meta_addr_tag_q);
    // A read snoop needs the shared data RAM only when the installed line is
    // dirty and therefore newer than external memory.  Clean hits and tag
    // misses are metadata-only probes: MC can read RAM directly without
    // displacing a CPU I/D cache read from this single synchronous port.
    wire snoop_rd_needs_data_q = snoop_rd_meta_sample_q &&
                                 snoop_rd_meta_req_q &&
                                 snoop_rd_tag_match_q &&
                                 snoop_rd_meta_line_dirty_q;
    wire d_port_busy = d_read_sc || snoop_rd_needs_data_q ||
                       (state == S_WB_PREP) || (state == S_WB) ||
                       (state == S_CACHE_WB) || (state == S_CDE_ZERO);
    wire [DATA_ADDR_W-1:0] cache_rd_addr =
        (state == S_WB_PREP) ? {wb_idx, 4'd0} :
        ((state == S_WB) || (state == S_CACHE_WB)) ? {wb_idx, wb_cnt} :
        snoop_rd_needs_data_q ? {snoop_rd_meta_idx_q,
                                 snoop_rd_meta_dw_q} :
        d_read_sc            ? d_cache_rd_addr :
                               i_cache_rd_addr;
    reg [DATA_ADDR_W-1:0] cache_rd_addr_q;
    reg        cache_rd_valid_q;
    reg        cache_rd_write_q;
    reg [DATA_ADDR_W-1:0] cache_rd_write_addr_q;
    wire cache_rd_same_write_q = cache_rd_write_q &&
                                 (cache_rd_write_addr_q == cache_rd_addr_q);
    // During fills, data RAM at fill_idx already contains the new line while
    // tag/status metadata can still describe the evicted line.
    wire d_fill_conflict = (state == S_DFILL || state == S_IFILL ||
                            state == S_DONE || state == S_META_WAIT) &&
                           (fill_idx == d_idx);
    wire i_sc_data_ready = cache_rd_valid_q &&
                           (cache_rd_addr_q == i_cache_rd_addr);
    wire d_sc_data_ready_raw = cache_rd_valid_q &&
                               (cache_rd_addr_q == d_cache_rd_addr);
    wire d_sc_data_ready = d_sc_data_ready_raw && !d_fill_conflict;
    wire i_sync_wait = i_need_sc && i_sc_hit && !i_sc_data_ready;
    wire d_sync_wait = d_ren && d_lookup_sc && d_sc_hit && !d_sc_data_ready;

    wire [31:0] i_sc_word = i_addr[2] ? i_sc_dw_q[31:0] : i_sc_dw_q[63:32];
    assign ext_imem_addr = i_addr;
    assign i_fill_rdata = (i_sc_hit && i_sc_data_ready) ? i_sc_word : 32'd0;
    assign i_rdata = i_uncached_ext_hit ? i_uncached_ext_word :
                     i_uncached_bypass ? ext_imem_rdata :
                     i_fill_rdata;
    wire d_sc_data_usable = !d_uncached && d_lookup_sc && d_sc_hit &&
                            d_sc_data_ready;
    wire cop_taglo_data_valid = (cop_meta_ready || cop_replay);
    wire cop_taglo_line_valid = cop_taglo_data_valid && cop_meta_line_valid_q;
    wire [1:0] cop_sc_state = cop_taglo_line_valid ?
        (cop_meta_line_dirty_q ? 2'b10 : 2'b01) : 2'b00;
    assign cop_taglo_out = cop_taglo_line_valid ?
        {cop_meta_line_tag_q, {(24-TAG_W){1'b0}}, cop_sc_state, 6'b0} :
        32'd0;
    // The FPGA metadata RAM is synchronous.  CACHE reaches core WB before
    // the selected tag is available, so explicitly return the completed
    // Index Load Tag result when the held operation is accepted.
    assign cop_taglo_valid = cop_accept && (cop_eff_op == 3'd1);

    wire snoop_rd_fill_conflict = (state == S_DFILL || state == S_IFILL ||
                                   state == S_DONE || state == S_META_WAIT) &&
                                  (fill_idx == snoop_rd_meta_idx_q);
    // The DMA requester holds its address until MC accepts the transfer.
    // Do not claim a dirty hit until the shared synchronous data port is
    // returning that exact word.  In particular, an older CPU writeback keeps
    // ownership of the port and retires before the held DMA read is observed.
    wire snoop_rd_data_ready = cache_rd_valid_q &&
                               (cache_rd_addr_q ==
                                {snoop_rd_meta_idx_q,
                                 snoop_rd_meta_dw_q});
    wire snoop_rd_meta_transition_conflict =
        (state == S_CDE_ZERO || state == S_DONE || state == S_META_WAIT) &&
        (fill_idx == snoop_rd_meta_idx_q);
    wire snoop_rd_data_write_conflict =
        ram_we && (ram_waddr ==
                   {snoop_rd_meta_idx_q, snoop_rd_meta_dw_q});
    // Response-valid is independent of hit: a clean/missing line still lets
    // the board proceed to MC RAM.  Hold it off while the selected data or
    // metadata can be changing so the board latches one coherent result.
    assign snoop_rd_resp_valid = snoop_rd_meta_req_q &&
                                 (!snoop_rd_needs_data_q ||
                                  snoop_rd_data_ready) &&
                                 !meta_init_active &&
                                 !meta_aux_write_busy &&
                                 !snoop_rd_fill_conflict &&
                                 !snoop_rd_meta_transition_conflict &&
                                 !snoop_valid &&
                                 !snoop_meta_req_q &&
                                 !snoop_update_pending &&
                                 (!snoop_rd_needs_data_q ||
                                  !snoop_rd_data_write_conflict);
    assign snoop_rd_hit = snoop_rd_needs_data_q &&
                          !snoop_rd_fill_conflict &&
                          snoop_rd_data_ready;
    assign snoop_rd_data = d_sc_dw_q;
    wire sc_busy = meta_init_active || meta_aux_write_busy ||
                   (state != S_IDLE) || backinv_pending_valid;
    wire d_read_hit_can_run_during_ifill =
        (state == S_IFILL) && d_ren && !d_wen && !d_uncached &&
        d_lookup_sc && d_sc_hit && d_sc_data_ready && !d_fill_conflict;
    assign i_freeze = sc_busy || i_meta_wait || i_sc_miss ||
                      i_uncached_ext_miss || d_port_busy || i_sync_wait ||
                      snoop_update_pending;
    assign d_freeze = (sc_busy && !d_read_hit_can_run_during_ifill) ||
                      d_meta_wait || d_sc_miss || d_sync_wait ||
                      snoop_update_pending;

    localparam [31:0] KMEM_HEAD0 = 32'h0815_F1C4;
    localparam [31:0] KMEM_HEAD1 = 32'h0815_F1D0;
    localparam [31:0] KMEM_HEAD2 = 32'h0815_F1DC;
    localparam [31:0] KMEM_HEAD3 = 32'h0815_F1E8;
    localparam [31:0] KMEM_HEAD4 = 32'h0815_F1F4;
    localparam [31:0] KMEM_HEAD5 = 32'h0815_F200;
    localparam [31:0] KMEM_HEAD6 = 32'h0815_F20C;
    localparam [31:0] KMEM_HEAD7 = 32'h0815_F218;
    localparam [31:0] KMEM_HEAD8 = 32'h0815_F224;
    localparam [31:0] KMEM_HEAD9 = 32'h0815_F230;
    localparam [31:0] KMEM_HEAD10 = 32'h0815_F23C;

    localparam [3:0]
        DBG_SRC_NONE       = 4'h0,
        DBG_SRC_FILL       = 4'h1,
        DBG_SRC_SNOOP_WR   = 4'h2,
        DBG_SRC_SNOOP_INV  = 4'h3,
        DBG_SRC_I_WRITE    = 4'h4,
        DBG_SRC_D_WRITE    = 4'h5,
        DBG_SRC_D_UNCACHED = 4'h6,
        DBG_SRC_D_SUPPLY   = 4'h7,
        DBG_SRC_COP        = 4'h8,
        DBG_SRC_CDE_ZERO   = 4'h9,
        DBG_SRC_EXT_WB     = 4'hA;

    function kmem_head_word_addr;
        input [31:0] addr;
        begin
            case (addr)
                KMEM_HEAD0, KMEM_HEAD1, KMEM_HEAD2, KMEM_HEAD3,
                KMEM_HEAD4, KMEM_HEAD5, KMEM_HEAD6, KMEM_HEAD7,
                KMEM_HEAD8, KMEM_HEAD9, KMEM_HEAD10:
                    kmem_head_word_addr = 1'b1;
                default:
                    kmem_head_word_addr = 1'b0;
            endcase
        end
    endfunction

    function kmem_head_dw_addr;
        input [31:0] addr;
        begin
            case (addr & 32'hFFFF_FFF8)
                32'h0815_F1C0, 32'h0815_F1D0, 32'h0815_F1D8,
                32'h0815_F1E8, 32'h0815_F1F0, 32'h0815_F200,
                32'h0815_F208, 32'h0815_F218, 32'h0815_F220,
                32'h0815_F230, 32'h0815_F238:
                    kmem_head_dw_addr = 1'b1;
                default:
                    kmem_head_dw_addr = 1'b0;
            endcase
        end
    endfunction

    function [7:0] kmem_head_mask_for_dw;
        input [31:0] addr;
        begin
            case (addr & 32'hFFFF_FFF8)
                32'h0815_F1C0, 32'h0815_F1D8, 32'h0815_F1F0,
                32'h0815_F208, 32'h0815_F220, 32'h0815_F238:
                    kmem_head_mask_for_dw = 8'h0F;
                32'h0815_F1D0, 32'h0815_F1E8, 32'h0815_F200,
                32'h0815_F218, 32'h0815_F230:
                    kmem_head_mask_for_dw = 8'hF0;
                default:
                    kmem_head_mask_for_dw = 8'h00;
            endcase
        end
    endfunction

    function [31:0] kmem_head_word_from_dw;
        input [31:0] addr;
        input [63:0] data;
        begin
            case (kmem_head_mask_for_dw(addr))
                8'hF0: kmem_head_word_from_dw = data[63:32];
                8'h0F: kmem_head_word_from_dw = data[31:0];
                default: kmem_head_word_from_dw = 32'd0;
            endcase
        end
    endfunction

    wire ram_src_fill = (state == S_DFILL || state == S_IFILL) &&
                        ext_dmem_ready && (fill_first == 2'd0);
    wire ram_src_cde = (state == S_CDE_ZERO);
    wire debug_ram_src_fill = ram_src_fill;
    wire debug_ram_src_cde = ram_src_cde;
    wire debug_ram_src_snoop = snoop_update_pending;
    // Observe the admitted write predicate rather than rebuilding an
    // equivalent expression that can preserve a redundant physical cone.
    wire debug_ram_src_i = i_write_sc;
    wire debug_ram_src_d = (state == S_IDLE) && d_wen && !d_uncached &&
                           d_sc_hit;
    wire debug_ram_src_d_uncached = (state == S_IDLE) && d_wen &&
                                    d_uncached && !force_uncached &&
                                    d_sc_hit;
    wire [3:0] debug_ram_src_code =
        debug_ram_src_fill       ? DBG_SRC_FILL :
        debug_ram_src_cde        ? DBG_SRC_CDE_ZERO :
        debug_ram_src_snoop      ? DBG_SRC_SNOOP_WR :
        debug_ram_src_i          ? DBG_SRC_I_WRITE :
        debug_ram_src_d          ? DBG_SRC_D_WRITE :
        debug_ram_src_d_uncached ? DBG_SRC_D_UNCACHED :
                                  DBG_SRC_NONE;
    wire [31:0] debug_ram_src_addr =
        debug_ram_src_fill       ? {fill_addr[31:7], fill_cnt, 3'b000} :
        debug_ram_src_cde        ? {fill_addr[31:7], fill_cnt, 3'b000} :
        debug_ram_src_snoop      ? snoop_update_src_addr :
        debug_ram_src_i          ? {i_addr[31:3], 3'b000} :
        debug_ram_src_d          ? {d_addr[31:3], 3'b000} :
        debug_ram_src_d_uncached ? {d_addr[31:3], 3'b000} :
                                  32'd0;
    wire debug_watch_ram_write =
        ram_we && kmem_head_dw_addr(debug_ram_src_addr) &&
        ((ram_be & kmem_head_mask_for_dw(debug_ram_src_addr)) != 8'h00);
    wire debug_watch_snoop_req =
        snoop_valid && kmem_head_dw_addr(snoop_addr);
    wire debug_watch_snoop_hit =
        snoop_meta_hit_q && kmem_head_dw_addr({snoop_meta_addr_tag_q,
                                               snoop_meta_idx_q,
                                               snoop_meta_dw_q, 3'b000});
    wire debug_watch_snoop_inv =
        debug_watch_snoop_hit && (snoop_meta_wmask_q == 8'h00);
    wire debug_d_supply = d_ren && d_lookup_sc && d_sc_hit &&
                          d_sc_data_ready && kmem_head_word_addr(d_addr);
    wire [31:0] debug_d_supply_word =
        d_addr[2] ? d_sc_dw_q[31:0] : d_sc_dw_q[63:32];
    wire debug_d_supply_bad =
        debug_d_supply &&
        (debug_d_supply_word[31:16] == 16'd0) &&
        (debug_d_supply_word != 32'd0);
    wire debug_watch_cop =
        cop_accept && kmem_head_dw_addr(cop_eff_paddr);
    wire debug_watch_event =
        debug_watch_ram_write || debug_watch_snoop_inv ||
        debug_watch_snoop_req || debug_watch_cop;
    wire [3:0] debug_watch_src_code =
        debug_watch_ram_write ? debug_ram_src_code :
        debug_watch_snoop_inv ? DBG_SRC_SNOOP_INV :
        debug_watch_snoop_req ? DBG_SRC_SNOOP_WR :
        debug_watch_cop       ? DBG_SRC_COP :
                                DBG_SRC_NONE;
    wire [31:0] debug_watch_src_addr =
        debug_watch_ram_write ? debug_ram_src_addr :
        debug_watch_snoop_inv ? {snoop_meta_addr_tag_q, snoop_meta_idx_q,
                                 snoop_meta_dw_q, 3'b000} :
        debug_watch_snoop_req ? {snoop_addr[31:3], 3'b000} :
        debug_watch_cop       ? cop_eff_paddr :
                                32'd0;
    wire [63:0] debug_watch_src_data =
        debug_watch_ram_write ? ram_wdata :
        debug_watch_snoop_inv ? snoop_meta_wdata_q :
        debug_watch_snoop_req ? snoop_wdata :
        debug_watch_cop       ? {cop_eff_taglo, cop_eff_paddr} :
                                64'd0;

    localparam [24:0] SC20_PTE_LINE0 = 25'h010A800; // 0x08540000 >> 7
    localparam [24:0] SC20_PTE_LINE1 = 25'h010A400; // 0x08520000 >> 7

    function sc20_pte_line;
        input [31:0] addr;
        begin
            sc20_pte_line =
                (addr[31:7] == SC20_PTE_LINE0) ||
                (addr[31:7] == SC20_PTE_LINE1);
        end
    endfunction

    function sc20_lowvec_line;
        input [31:0] addr;
        begin
            sc20_lowvec_line =
                (addr[31:12] == 20'h00000) ||
                (addr[31:12] == 20'h08000);
        end
    endfunction

    function sc20_watch_line;
        input [31:0] addr;
        begin
            sc20_watch_line = sc20_pte_line(addr) || sc20_lowvec_line(addr);
        end
    endfunction

    function [31:0] sc20_word_at;
        input [31:0] addr;
        input [63:0] data;
        begin
            sc20_word_at = addr[2] ? data[31:0] : data[63:32];
        end
    endfunction

    wire sc20_d_supply_event =
        d_ren && d_lookup_sc && d_sc_hit && d_sc_data_ready &&
        sc20_pte_line(d_addr);
    wire [31:0] sc20_d_supply_word = sc20_word_at(d_addr, d_sc_dw_q);
    wire sc20_d_supply_bad = sc20_d_supply_event &&
                             (sc20_d_supply_word == 32'h4200_0018);
    wire sc20_ram_data_event =
        ram_we && sc20_watch_line(debug_ram_src_addr);
    wire sc20_ext_wb_event =
        ext_dmem_wen && ext_dmem_ready && sc20_watch_line(ext_dmem_addr);
    wire sc20_data_event = sc20_ram_data_event || sc20_ext_wb_event;
    wire [3:0] sc20_data_src_code =
        sc20_ram_data_event ? debug_ram_src_code : DBG_SRC_EXT_WB;
    wire [31:0] sc20_data_addr =
        sc20_ram_data_event ? debug_ram_src_addr : ext_dmem_addr;
    wire [63:0] sc20_data_data =
        sc20_ram_data_event ? ram_wdata : ext_dmem_wdata;
    wire [7:0] sc20_data_mask =
        sc20_ram_data_event ? ram_be : ext_dmem_wmask;
    wire [3:0] sc20_data_dw =
        sc20_ram_data_event ? ram_waddr[3:0] : ext_dmem_addr[6:3];
    wire sc20_cop_event =
        cop_accept &&
        ((cop_idx == {IDX_W{1'b0}}) ||
         sc20_pte_line(cop_eff_paddr) ||
         sc20_lowvec_line(cop_eff_paddr));
    wire [9:0] sc20_d_idx10 = d_addr[16:7];
    wire [9:0] sc20_cop_idx10 = cop_eff_paddr[16:7];

    reg [7:0]  sc20_d_count_q;
    reg [7:0]  sc20_data_count_q;
    reg [6:0]  sc20_cop_count_q;
    reg        sc20_d_bad_seen_q;
    reg [31:0] sc20_d_addr_q;
    reg [63:0] sc20_d_data_q;
    reg [31:0] sc20_d_info_q;
    reg [31:0] sc20_data_info_q;
    reg [31:0] sc20_data_addr_q;
    reg [63:0] sc20_data_data_q;
    reg [31:0] sc20_cop_info_q;
    reg [31:0] sc20_cop_addr_q;

    wire [31:0] sc20_meta = {
        8'h20,
        sc20_d_bad_seen_q,
        sc20_d_count_q,
        sc20_data_count_q,
        sc20_cop_count_q
    };
    assign debug_scache_probe = {
        32'h5343_3230, // "SC20"
        sc20_meta,
        sc20_d_addr_q,
        sc20_d_data_q[63:32],
        sc20_d_data_q[31:0],
        sc20_d_info_q,
        sc20_data_info_q,
        sc20_data_addr_q,
        sc20_data_data_q[63:32],
        sc20_data_data_q[31:0],
        sc20_cop_info_q,
        sc20_cop_addr_q
    };

    reg [3:0]  debug_src_code_q;
    reg [3:0]  debug_bad_state_q;
    reg [31:0] debug_src_addr_q;
    reg [63:0] debug_src_data_q;
    reg        debug_i_bad_seen;
    reg [31:0] debug_bad_word_q;

    assign debug_i_capture = {
        8'h53,
        debug_src_code_q,
        debug_i_bad_seen,
        debug_bad_state_q,
        debug_bad_word_q[14:0],
        debug_src_addr_q,
        debug_src_data_q[63:32],
        debug_src_data_q[31:0]
    };

    // Registered-response peripherals leave ready high for one cycle after
    // accepting an uncached access. Hold the next uncached request off the
    // external bus for two cycles so stale ready cannot acknowledge it.
    reg [1:0] uncached_cooldown;
    reg       uncached_d_inflight;
    reg       uncached_d_resp_valid /* verilator public_flat_rd */;
    reg [31:0] uncached_d_addr_q;
    reg        uncached_d_ren_q;
    reg        uncached_d_wen_q;
    reg [7:0]  uncached_d_wmask_q;
    reg [63:0] uncached_d_wdata_q;
    reg [63:0] uncached_d_resp_data_q;
    wire uncached_d_can_start /* verilator public_flat_rd */ =
        uncached_d_req && (uncached_cooldown == 2'd0) &&
        !uncached_d_inflight && !uncached_d_resp_valid && !sc_busy && !d_meta_wait &&
        !snoop_update_pending;
    wire uncached_d_issue = uncached_d_inflight;
    wire uncached_d_start = uncached_d_can_start;
    wire uncached_d_ext_accepted /* verilator public_flat_rd */ =
                                   uncached_d_issue && uncached_d_inflight &&
                                   ext_dmem_ready;
    wire uncached_d_resp_match =
        uncached_d_resp_valid && uncached_d_req &&
        d_ren && uncached_d_ren_q &&
        (d_addr == uncached_d_addr_q) &&
        (d_ren == uncached_d_ren_q) &&
        (d_wen == uncached_d_wen_q);
    wire uncached_d_complete = uncached_d_ext_accepted || uncached_d_resp_match;
    wire uncached_d_pending =
        (uncached_d_inflight && !uncached_d_complete) ||
        (uncached_d_req && !uncached_d_complete);
    wire [31:0] uncached_d_bus_addr =
        uncached_d_inflight ? uncached_d_addr_q : d_addr;
    wire        uncached_d_bus_ren =
        uncached_d_inflight ? uncached_d_ren_q : d_ren;
    wire        uncached_d_bus_wen =
        uncached_d_inflight ? uncached_d_wen_q : d_wen;
    wire [7:0]  uncached_d_bus_wmask =
        uncached_d_inflight ? uncached_d_wmask_q : d_wmask;
    wire [63:0] uncached_d_bus_wdata =
        uncached_d_inflight ? uncached_d_wdata_q : d_wdata;
    // Vector writes through either supported physical alias must evict stale
    // low-vector I-cache lines.  Uncached publication also invalidates a
    // matching normalized S-cache line so the subsequent I-cache refill
    // cannot recover stale vector data.
    wire [31:0] d_low_vector_backinv_addr =
        {d_scache_addr[31:7], 7'b0};
    wire [TAG_W-1:0] d_low_vector_backinv_tag =
        d_low_vector_backinv_addr[31:IDX_W+7];
    wire uncached_d_low_vector_write_start =
        uncached_d_start && d_wen && d_addr_ip20_vector_page;
    wire cached_d_start =
        (state == S_IDLE) && d_wen && !d_uncached && d_meta_ready;
    wire d_low_vector_write_start =
        d_wen && (uncached_d_start || cached_d_start) &&
        d_addr_ip20_vector_page;

    // Keep CACHE acceptance itself inside the busy interval.  Without this
    // term there is a one-cycle quiet gap after metadata becomes ready but
    // before a side-effecting CACHE operation enters BACKINV/WRITEBACK; a
    // coherence hold could acknowledge through that gap.
    assign stall    = sc_busy || cop_meta_wait || cop_accept ||
                      snoop_update_pending ||
                      uncached_d_pending || i_uncached_ext_miss;
    assign uncached_started = uncached_d_start;
    assign uncached_accepted = uncached_d_ext_accepted || uncached_d_resp_match;
    assign ext_dmem_uncached_wr = (state == S_IDLE) && uncached_d_issue &&
                                  uncached_d_bus_wen;
    assign ext_dmem_cache_owned = (state != S_IDLE) &&
                                  (ext_dmem_ren || ext_dmem_wen);
    assign d_rdata = d_sc_data_usable ? d_sc_dw_q :
                     (!d_uncached && d_lookup_sc) ? 64'd0 :
                     uncached_d_resp_match ? uncached_d_resp_data_q :
                                             ext_dmem_rdata;
    assign debug_status = {
        3'h1,
        state,
        fill_is_i,
        stall,
        i_freeze,
        d_freeze,
        i_sc_hit,
        d_sc_hit,
        i_sc_miss,
        d_sc_miss,
        cop_pending,
        cop_accept,
        uncached_d_issue,
        ext_dmem_ready,
        ext_dmem_ren,
        ext_dmem_wen,
        fill_first,
        fill_cnt,
        uncached_cooldown,
        backinv_valid,
        snoop_rd_hit,
        ext_dmem_uncached_wr
    };
    assign debug_addr = ext_dmem_addr;

    reg        ram_we;
    reg [7:0]  ram_be;
    reg [DATA_ADDR_W-1:0] ram_waddr;
    reg [63:0] ram_wdata;
    localparam SNOOP_UPDATE_DEPTH = 32;
    localparam SNOOP_UPDATE_PTR_W = 5;
    reg [DATA_ADDR_W-1:0] snoop_update_addr_fifo [0:SNOOP_UPDATE_DEPTH-1];
    reg [31:0] snoop_update_src_addr_fifo [0:SNOOP_UPDATE_DEPTH-1];
    reg [63:0] snoop_update_wdata_fifo [0:SNOOP_UPDATE_DEPTH-1];
    reg [7:0]  snoop_update_wmask_fifo [0:SNOOP_UPDATE_DEPTH-1];
    reg [SNOOP_UPDATE_PTR_W-1:0] snoop_update_rd_ptr;
    reg [SNOOP_UPDATE_PTR_W-1:0] snoop_update_wr_ptr;
    reg [SNOOP_UPDATE_PTR_W:0]   snoop_update_count;
    localparam [SNOOP_UPDATE_PTR_W:0] SNOOP_UPDATE_DEPTH_COUNT =
        SNOOP_UPDATE_DEPTH;
    wire snoop_update_pending = (snoop_update_count != 0);
    wire snoop_update_full = (snoop_update_count == SNOOP_UPDATE_DEPTH_COUNT);
    reg  snoop_wr_pending;
    // The split CPU/machine bridge admits one write commit at a time.  Start
    // only after every older registered lookup/merge has retired, so this
    // transaction cannot encounter the legacy full-queue drop behavior and
    // its completion can be identified without an address heuristic.
    // snoop_meta_hit_q implies snoop_meta_req_q, which is already excluded
    // here.  Name the only independent metadata writer explicitly so the
    // synchronous snoop-tag result cannot feed back into admission.
    assign snoop_wr_ready = !snoop_wr_pending && !snoop_meta_req_q &&
                            !snoop_update_pending && !meta_init_active &&
                            !i_dirty_pending_q;
    wire snoop_wr_accept = snoop_valid && snoop_wr_ready;
    wire [DATA_ADDR_W-1:0] snoop_update_addr =
        snoop_update_addr_fifo[snoop_update_rd_ptr];
    wire [31:0] snoop_update_src_addr =
        snoop_update_src_addr_fifo[snoop_update_rd_ptr];
    wire [63:0] snoop_update_wdata =
        snoop_update_wdata_fifo[snoop_update_rd_ptr];
    wire [7:0] snoop_update_wmask =
        snoop_update_wmask_fifo[snoop_update_rd_ptr];
    wire snoop_update_ram_write =
        snoop_update_pending && !ram_src_fill && !ram_src_cde;
    // A Primary-D intervention beat is accepted only when it owns the shared
    // data-RAM write port.  Queued DMA merges have priority in the mux below.
    assign backinv_ready = (state == S_BACKINV_WAIT) &&
                           !snoop_update_ram_write;
    // A DMA write can arrive after a fill has started but before the new tag
    // is installed.  Metadata lookup then cannot report a hit even though
    // already-returned fill words may be stale.  CDE zeroing has the same
    // ownership window: the target tag is not installed until the final zero
    // write.  Queue DMA bytes exactly as for an installed-line hit and apply
    // them after the fill/CDE RAM writes.
    wire snoop_fill_match =
        snoop_wr_accept &&
        (state == S_DFILL || state == S_IFILL || state == S_CDE_ZERO ||
         state == S_DONE) &&
        (snoop_addr[31:7] == fill_addr[31:7]);
    wire snoop_update_push =
        (snoop_meta_hit_q || snoop_fill_match_q) &&
        (snoop_meta_wmask_q != 8'h00);
    wire snoop_update_pop = snoop_update_ram_write;
    wire snoop_update_push_accept = snoop_update_push &&
                                    (!snoop_update_full || snoop_update_pop);
    // A completed DMA write is already visible in external RAM when its
    // registered S-cache lookup and queued merge arrive.  Hold an unaccepted
    // CPU writeback until that merge reaches cache RAM, then re-prime the
    // synchronous writeback word.  Raw pending DMA reads are deliberately not
    // part of this exclusion: MC may already own the CPU writeback.
    wire writeback_dma_write_block = snoop_wr_accept || snoop_meta_req_q ||
                                      snoop_update_pending;
    wire writeback_complete =
        (state == S_WB || state == S_CACHE_WB) &&
        ext_dmem_wen && ext_dmem_ready;

    always @(*) begin
        ext_dmem_addr  = d_addr;
        ext_dmem_ren   = 1'b0;
        ext_dmem_wen   = 1'b0;
        ext_dmem_wmask = 8'h00;
        ext_dmem_wdata = 64'd0;

        case (state)
            S_IDLE: begin
                if (uncached_d_issue) begin
                    ext_dmem_addr  = uncached_d_bus_addr;
                    ext_dmem_ren   = uncached_d_bus_ren;
                    ext_dmem_wen   = uncached_d_bus_wen;
                    ext_dmem_wmask = uncached_d_bus_wen ? uncached_d_bus_wmask : 8'h00;
                    ext_dmem_wdata = uncached_d_bus_wdata;
                end
            end
            S_WB, S_CACHE_WB: begin
                ext_dmem_addr = {wb_tag, wb_idx, wb_cnt, 3'b000};
                if (fill_first == 2'd0 && !writeback_dma_write_block) begin
                    ext_dmem_wen   = 1'b1;
                    ext_dmem_wmask = 8'hff;
                    ext_dmem_wdata = d_sc_dw_q;
                end
            end
            S_DFILL, S_IFILL: begin
                ext_dmem_addr = {fill_addr[31:7], fill_cnt, 3'b000};
                ext_dmem_ren  = (fill_first == 2'd0);
            end
            S_IUNCACHED: begin
                ext_dmem_addr = {uncached_i_addr[31:3], 3'b000};
                ext_dmem_ren  = (fill_first == 2'd0);
            end
            default: ;
        endcase
    end

    always @(*) begin
        ram_we    = 1'b0;
        ram_be    = 8'h00;
        ram_waddr = {DATA_ADDR_W{1'b0}};
        ram_wdata = 64'd0;

        if (ram_src_fill) begin
            ram_we    = 1'b1;
            ram_be    = 8'hff;
            ram_waddr = {fill_idx, fill_cnt};
            ram_wdata = ext_dmem_rdata;
        end else if (ram_src_cde) begin
            ram_we    = 1'b1;
            ram_be    = 8'hff;
            ram_waddr = {fill_idx, fill_cnt};
            ram_wdata = 64'd0;
        end else if (snoop_update_ram_write) begin
            ram_we    = 1'b1;
            ram_be    = snoop_update_wmask;
            ram_waddr = snoop_update_addr;
            ram_wdata = snoop_update_wdata;
        end else if (i_write_sc) begin
            ram_we    = 1'b1;
            ram_be    = 8'hff;
            ram_waddr = {i_idx, i_dw_off};
            ram_wdata = i_wdata;
        end else if (d_write_sc || d_intervention_write_sc) begin
            ram_we    = 1'b1;
            ram_be    = d_wmask;
            ram_waddr = {d_idx, d_dw_off};
            ram_wdata = d_wdata;
        end
    end

    r4000_scache_ram64 #(
        .ADDR_W(DATA_ADDR_W),
        .NUMWORDS(SCACHE_DATA_DWS)
    ) u_data (
        .clk   (clk),
        .raddr (cache_rd_addr),
        .rdata (cache_dw_q),
        .we    (ram_we),
        .be    (ram_be),
        .waddr (ram_waddr),
        .wdata (ram_wdata)
    );

    // Serialize all status changes through the metadata replicas' common
    // write port.  A snoop update wins first because it is an external pulse;
    // a deferred I-side dirty update remains pending until it commits.  State
    // transitions which require metadata are held while either is active.
    always @(*) begin
        meta_we    = 1'b0;
        meta_waddr = {IDX_W{1'b0}};
        meta_wdata = 2'b00;

        if (meta_init_active) begin
            meta_we    = 1'b1;
            meta_waddr = meta_init_idx;
            meta_wdata = 2'b00;
        end else if (snoop_meta_hit_q) begin
            meta_we    = 1'b1;
            meta_waddr = snoop_meta_idx_q;
            meta_wdata = (snoop_meta_wmask_q != 8'h00) ? 2'b11 : 2'b00;
        end else if (i_dirty_pending_q) begin
            meta_we    = 1'b1;
            meta_waddr = i_dirty_idx_q;
            meta_wdata = 2'b11;
        end else begin
            case (state)
                S_IDLE: begin
                    if (!backinv_pending_valid) begin
                        if (cop_accept) begin
                            case (cop_eff_op)
                                3'd0: begin
                                    if (!(cop_meta_line_valid_q &&
                                          cop_meta_line_dirty_q)) begin
                                        meta_we    = 1'b1;
                                        meta_waddr = cop_idx;
                                        meta_wdata = 2'b00;
                                    end
                                end
                                3'd2: begin
                                    meta_we    = 1'b1;
                                    meta_waddr = cop_idx;
                                    meta_wdata = {
                                        (cop_eff_taglo[7:6] != 2'b00),
                                        cop_eff_taglo[7]
                                    };
                                end
                                3'd3: begin
                                    // SD Create Dirty Exclusive on a hit
                                    // retains the matching secondary data and
                                    // changes only ownership/state. The primary
                                    // aliases are invalidated separately below.
                                    if (cop_meta_line_valid_q &&
                                        cop_meta_line_tag_q == cop_tag) begin
                                        meta_we    = 1'b1;
                                        meta_waddr = cop_idx;
                                        meta_wdata = 2'b11;
                                    end
                                end
                                3'd4: begin
                                    if (cop_meta_line_valid_q &&
                                        cop_meta_line_tag_q == cop_tag) begin
                                        meta_we    = 1'b1;
                                        meta_waddr = cop_idx;
                                        meta_wdata = 2'b00;
                                    end
                                end
                                3'd5: begin
                                    if (cop_meta_line_valid_q &&
                                        cop_meta_line_tag_q == cop_tag &&
                                        !cop_meta_line_dirty_q) begin
                                        meta_we    = 1'b1;
                                        meta_waddr = cop_idx;
                                        meta_wdata = 2'b00;
                                    end
                                end
                                default: ;
                            endcase
                        end else if (uncached_d_low_vector_write_start &&
                                     d_meta_line_valid_q &&
                                     d_meta_line_tag_q == d_low_vector_backinv_tag) begin
                            meta_we    = 1'b1;
                            meta_waddr = d_idx;
                            meta_wdata = 2'b00;
                        end else if (d_write_sc) begin
                            meta_we    = 1'b1;
                            meta_waddr = d_idx;
                            meta_wdata = 2'b11;
                        end
                    end
                end
                S_CACHE_WB: begin
                    if (writeback_complete && wb_cnt == 4'd15 &&
                        !cache_wb_install_cde) begin
                        meta_we    = 1'b1;
                        meta_waddr = wb_idx;
                        meta_wdata = cache_wb_keep_valid ? 2'b10 : 2'b00;
                    end
                end
                S_CDE_ZERO: begin
                    if (fill_cnt == 4'd15) begin
                        meta_we    = 1'b1;
                        meta_waddr = fill_idx;
                        meta_wdata = 2'b11;
                    end
                end
                S_DONE: begin
                    meta_we    = 1'b1;
                    meta_waddr = fill_idx;
                    meta_wdata = 2'b10;
                end
                default: ;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            fill_cnt <= 4'd0;
            fill_addr <= 32'd0;
            fill_is_i <= 1'b0;
            wb_cnt <= 4'd0;
            wb_idx <= {IDX_W{1'b0}};
            wb_tag <= {TAG_W{1'b0}};
            wb_is_cache_op <= 1'b0;
            cache_wb_keep_valid <= 1'b0;
            cache_wb_install_cde <= 1'b0;
            backinv_pending_valid <= 1'b0;
            backinv_pending_addr <= 32'd0;
            backinv_pending_cacheop <= 1'b0;
            backinv_pending_icache <= 1'b0;
            backinv_victim_dirty <= 1'b0;
            backinv_intervention_dirty <= 1'b0;
            fill_first <= 2'd0;
            uncached_cooldown <= 2'd0;
            uncached_d_inflight <= 1'b0;
            uncached_d_resp_valid <= 1'b0;
            uncached_d_addr_q <= 32'd0;
            uncached_d_ren_q <= 1'b0;
            uncached_d_wen_q <= 1'b0;
            uncached_d_wmask_q <= 8'd0;
            uncached_d_wdata_q <= 64'd0;
            uncached_d_resp_data_q <= 64'd0;
            cache_rd_addr_q <= {DATA_ADDR_W{1'b0}};
            cache_rd_valid_q <= 1'b0;
            cache_rd_write_q <= 1'b0;
            cache_rd_write_addr_q <= {DATA_ADDR_W{1'b0}};
            uncached_i_addr <= 32'd0;
            uncached_i_rdata <= 64'd0;
            uncached_i_valid <= 1'b0;
            i_meta_sample_q <= 1'b0;
            i_meta_idx_q <= {IDX_W{1'b0}};
            i_meta_addr_tag_q <= {TAG_W{1'b0}};
            i_meta_line_valid_q <= 1'b0;
            i_meta_line_dirty_q <= 1'b0;
            i_dirty_pending_q <= 1'b0;
            i_dirty_idx_q <= {IDX_W{1'b0}};
            i_dirty_tag_q <= {TAG_W{1'b0}};
            d_meta_sample_q <= 1'b0;
            d_meta_idx_q <= {IDX_W{1'b0}};
            d_meta_addr_tag_q <= {TAG_W{1'b0}};
            d_meta_line_valid_q <= 1'b0;
            d_meta_line_dirty_q <= 1'b0;
            cop_meta_sample_q <= 1'b0;
            cop_meta_idx_q <= {IDX_W{1'b0}};
            cop_meta_addr_tag_q <= {TAG_W{1'b0}};
            cop_meta_line_valid_q <= 1'b0;
            cop_meta_line_dirty_q <= 1'b0;
            cop_accepted_q <= 1'b0;
            cop_accepted_op_q <= 3'd0;
            cop_accepted_paddr_q <= 32'd0;
            cop_accepted_taglo_q <= 32'd0;
            cop_latched <= 1'b0;
            cop_latched_op <= 3'd0;
            cop_latched_paddr <= 32'd0;
            cop_latched_taglo <= 32'd0;
            snoop_meta_sample_q <= 1'b0;
            snoop_meta_req_q <= 1'b0;
            snoop_meta_idx_q <= {IDX_W{1'b0}};
            snoop_meta_addr_tag_q <= {TAG_W{1'b0}};
            snoop_meta_line_valid_q <= 1'b0;
            snoop_meta_dw_q <= 4'd0;
            snoop_meta_wdata_q <= 64'd0;
            snoop_meta_wmask_q <= 8'd0;
            snoop_fill_match_q <= 1'b0;
            snoop_rd_meta_sample_q <= 1'b0;
            snoop_rd_meta_req_q <= 1'b0;
            snoop_rd_meta_idx_q <= {IDX_W{1'b0}};
            snoop_rd_meta_addr_tag_q <= {TAG_W{1'b0}};
            snoop_rd_meta_line_valid_q <= 1'b0;
            snoop_rd_meta_line_dirty_q <= 1'b0;
            snoop_rd_meta_dw_q <= 4'd0;
            meta_init_active <= 1'b1;
            meta_init_idx <= {IDX_W{1'b0}};
            backinv_valid <= 1'b0;
            backinv_addr <= 32'd0;
            backinv_cacheop <= 1'b0;
            backinv_icache <= 1'b0;
            backinv_writeback <= 1'b0;
            snoop_update_rd_ptr <= {SNOOP_UPDATE_PTR_W{1'b0}};
            snoop_update_wr_ptr <= {SNOOP_UPDATE_PTR_W{1'b0}};
            snoop_update_count <= 0;
            snoop_wr_pending <= 1'b0;
            snoop_wr_done <= 1'b0;
            debug_src_code_q <= DBG_SRC_NONE;
            debug_bad_state_q <= 4'd0;
            debug_src_addr_q <= 32'd0;
            debug_src_data_q <= 64'd0;
            debug_i_bad_seen <= 1'b0;
            debug_bad_word_q <= 32'd0;
            sc20_d_count_q <= 8'd0;
            sc20_data_count_q <= 8'd0;
            sc20_cop_count_q <= 7'd0;
            sc20_d_bad_seen_q <= 1'b0;
            sc20_d_addr_q <= 32'd0;
            sc20_d_data_q <= 64'd0;
            sc20_d_info_q <= 32'd0;
            sc20_data_info_q <= 32'd0;
            sc20_data_addr_q <= 32'd0;
            sc20_data_data_q <= 64'd0;
            sc20_cop_info_q <= 32'd0;
            sc20_cop_addr_q <= 32'd0;
        end else if (meta_init_active) begin
            snoop_wr_done <= 1'b0;
            if (meta_init_idx == {IDX_W{1'b1}}) begin
                meta_init_active <= 1'b0;
                meta_init_idx <= {IDX_W{1'b0}};
            end else begin
                meta_init_idx <= meta_init_idx + 1'b1;
            end
        end else begin
            snoop_wr_done <= 1'b0;
            cache_rd_addr_q <= cache_rd_addr;
            // Mixed-port same-address read/write returns old data; reads from
            // other doublewords are still valid and must not stall I/D supply.
            cache_rd_valid_q <= !ram_we || (cache_rd_addr != ram_waddr);
            cache_rd_write_q <= ram_we;
            cache_rd_write_addr_q <= ram_waddr;

            if (uncached_d_ext_accepted) begin
                uncached_d_inflight <= 1'b0;
                if (uncached_d_ren_q) begin
                    uncached_d_resp_valid <= 1'b1;
                    uncached_d_resp_data_q <= ext_dmem_rdata;
                end
            end else if (uncached_d_resp_valid && !uncached_d_resp_match) begin
                uncached_d_resp_valid <= 1'b0;
            end else if (uncached_d_start) begin
                uncached_d_inflight <= 1'b1;
                uncached_d_addr_q <= d_addr;
                uncached_d_ren_q <= d_ren;
                uncached_d_wen_q <= d_wen;
                uncached_d_wmask_q <= d_wmask;
                uncached_d_wdata_q <= d_wdata;
            end

            if (uncached_d_ext_accepted)
                uncached_cooldown <= 2'd2;
            else if (uncached_cooldown != 2'd0)
                uncached_cooldown <= uncached_cooldown - 2'd1;
            if (fill_first != 2'd0)
                fill_first <= fill_first - 2'd1;
            if (writeback_dma_write_block &&
                (state == S_WB_PREP || state == S_WB || state == S_CACHE_WB))
                fill_first <= 2'd2;
            if (!cop_valid)
                cop_accepted_q <= 1'b0;
            if (cop_accept) begin
                cop_latched <= 1'b0;
            end else if (!cop_latched && cop_valid && !cop_replay) begin
                cop_latched <= 1'b1;
                cop_latched_op <= cop_op;
                cop_latched_paddr <= cop_paddr;
                cop_latched_taglo <= cop_taglo_in;
            end
            i_meta_sample_q <= 1'b1;
            i_meta_idx_q <= i_idx;
            i_meta_addr_tag_q <= i_tag;
            i_meta_line_valid_q <= i_meta_valid_now;
            i_meta_line_dirty_q <= i_meta_dirty_now;
            d_meta_sample_q <= 1'b1;
            d_meta_idx_q <= d_idx;
            d_meta_addr_tag_q <= d_tag;
            d_meta_line_valid_q <= d_meta_valid_now;
            d_meta_line_dirty_q <= d_meta_dirty_now;
            cop_meta_sample_q <= 1'b1;
            cop_meta_idx_q <= cop_idx;
            cop_meta_addr_tag_q <= cop_tag;
            cop_meta_line_valid_q <= cop_meta_valid_now;
            cop_meta_line_dirty_q <= cop_meta_dirty_now;
            snoop_meta_sample_q <= 1'b1;
            snoop_meta_req_q <= snoop_wr_accept;
            snoop_meta_idx_q <= snoop_idx;
            snoop_meta_addr_tag_q <= snoop_tag;
            snoop_meta_line_valid_q <= snoop_meta_valid_now;
            snoop_meta_dw_q <= snoop_dw;
            snoop_meta_wdata_q <= snoop_wdata;
            snoop_meta_wmask_q <= snoop_wmask;
            snoop_fill_match_q <= snoop_fill_match;
            if (snoop_wr_accept)
                snoop_wr_pending <= 1'b1;
            if (snoop_wr_pending && snoop_meta_req_q &&
                !snoop_update_push) begin
                snoop_wr_pending <= 1'b0;
                snoop_wr_done <= 1'b1;
            end else if (snoop_wr_pending && snoop_update_pop) begin
                // Admission required an empty update FIFO, so this pop is the
                // merge belonging to the outstanding bridge transaction.
                snoop_wr_pending <= 1'b0;
                snoop_wr_done <= 1'b1;
            end
            snoop_rd_meta_sample_q <= 1'b1;
            // The machine-side coordinator emits a launch pulse.  A dirty
            // hit needs an additional synchronous data-RAM cycle after its
            // metadata lookup, so retain the accepted request until the
            // response is actually visible.  Board builds also remain
            // compatible with the CPU-island bridge's held-valid request.
            if (snoop_rd_resp_valid)
                snoop_rd_meta_req_q <= 1'b0;
            else if (snoop_rd_valid)
                snoop_rd_meta_req_q <= 1'b1;
            snoop_rd_meta_idx_q <= snoop_rd_idx;
            snoop_rd_meta_addr_tag_q <= snoop_rd_tag;
            snoop_rd_meta_line_valid_q <= snoop_rd_meta_valid_now;
            snoop_rd_meta_line_dirty_q <= snoop_rd_meta_dirty_now;
            snoop_rd_meta_dw_q <= snoop_rd_dw;
            backinv_valid <= 1'b0;
            backinv_cacheop <= 1'b0;
            backinv_icache <= 1'b0;
            backinv_writeback <= 1'b0;
            if (i_dirty_pending_q && !snoop_meta_hit_q)
                i_dirty_pending_q <= 1'b0;
            // Preserve backinvalidations in issue order.  In particular, an
            // indexed Secondary operation may need to intervene the installed
            // line before issuing a defensive target-tag invalidation.
            if (backinv_pending_valid && state == S_IDLE) begin
                backinv_valid <= 1'b1;
                backinv_addr <= backinv_pending_addr;
                backinv_cacheop <= backinv_pending_cacheop;
                backinv_icache <= backinv_pending_icache;
                backinv_pending_valid <= 1'b0;
            end

            if (!debug_i_bad_seen && debug_watch_event) begin
                debug_src_code_q <= debug_watch_src_code;
                debug_src_addr_q <= debug_watch_src_addr;
                debug_src_data_q <= debug_watch_src_data;
            end
            if (!debug_i_bad_seen && debug_d_supply_bad) begin
                debug_bad_state_q <= state;
                debug_bad_word_q <= debug_d_supply_word;
                debug_i_bad_seen <= 1'b1;
            end
            if (sc20_d_supply_event && !sc20_d_bad_seen_q) begin
                sc20_d_count_q <= sc20_d_count_q + 8'd1;
                sc20_d_bad_seen_q <= sc20_d_bad_seen_q ||
                                     sc20_d_supply_bad;
                sc20_d_addr_q <= d_addr;
                sc20_d_data_q <= d_sc_dw_q;
                sc20_d_info_q <= {
                    4'hD,
                    state,
                    sc20_d_idx10,
                    d_dw_off,
                    d_sc_hit,
                    d_sc_miss,
                    d_sc_data_ready,
                    d_meta_line_valid_q,
                    d_meta_line_dirty_q,
                    sc20_d_supply_bad,
                    d_meta_line_tag_q[3:0]
                };
            end
            if (sc20_data_event && !sc20_d_bad_seen_q) begin
                sc20_data_count_q <= sc20_data_count_q + 8'd1;
                sc20_data_info_q <= {
                    4'h2,
                    sc20_data_src_code,
                    state,
                    sc20_ram_data_event,
                    sc20_ext_wb_event,
                    ext_dmem_ready,
                    tag_we,
                    sc20_data_mask,
                    sc20_data_dw,
                    4'h0
                };
                sc20_data_addr_q <= sc20_data_addr;
                sc20_data_data_q <= sc20_data_data;
            end
            if (sc20_cop_event && !sc20_d_bad_seen_q) begin
                sc20_cop_count_q <= sc20_cop_count_q + 7'd1;
                sc20_cop_info_q <= {
                    4'hC,
                    cop_eff_op,
                    cop_meta_line_valid_q,
                    cop_meta_line_dirty_q,
                    cop_eff_taglo[7:6],
                    sc20_cop_idx10,
                    cop_meta_line_tag_q[10:0]
                };
                sc20_cop_addr_q <= cop_eff_paddr;
            end

            if (snoop_update_pop)
                snoop_update_rd_ptr <= snoop_update_rd_ptr + 1'b1;
            if (snoop_update_push_accept) begin
                snoop_update_addr_fifo[snoop_update_wr_ptr] <=
                    {snoop_meta_idx_q, snoop_meta_dw_q};
                snoop_update_src_addr_fifo[snoop_update_wr_ptr] <=
                    {snoop_meta_addr_tag_q, snoop_meta_idx_q,
                     snoop_meta_dw_q, 3'b000};
                snoop_update_wdata_fifo[snoop_update_wr_ptr] <=
                    snoop_meta_wdata_q;
                snoop_update_wmask_fifo[snoop_update_wr_ptr] <=
                    snoop_meta_wmask_q;
                snoop_update_wr_ptr <= snoop_update_wr_ptr + 1'b1;
            end
            case ({snoop_update_push_accept, snoop_update_pop})
                2'b10: snoop_update_count <= snoop_update_count + 1'b1;
                2'b01: snoop_update_count <= snoop_update_count - 1'b1;
                default: ;
            endcase

            case (state)
                S_IDLE: begin
                    if (!backinv_pending_valid && !meta_aux_write_busy) begin
                        if (i_write_sc) begin
                            i_dirty_pending_q <= 1'b1;
                            i_dirty_idx_q <= i_idx;
                            i_dirty_tag_q <= i_tag;
                        end
                        if (d_low_vector_write_start) begin
                            backinv_valid <= 1'b1;
                            backinv_addr  <= d_low_vector_backinv_addr;
                            backinv_cacheop <= 1'b0;
                            backinv_icache <= 1'b1;
                        end

                        if (uncached_d_req || uncached_d_inflight) begin
                            // Keep the external memory port owned by the uncached
                            // D request until its response is drained.  The core
                            // can drop d_ren on an exception before the registered
                            // response returns; starting a fill here can consume
                            // that stale response as cached fill data.
                        end else if (cop_accept) begin
                            cop_accepted_q <= 1'b1;
                            cop_accepted_op_q <= cop_eff_op;
                            cop_accepted_paddr_q <= cop_eff_paddr;
                            cop_accepted_taglo_q <= cop_eff_taglo;
                            case (cop_eff_op)
                            3'd0: begin
                                // Index writeback-invalidate invalidates the
                                // line currently installed at this S-cache
                                // index.  In hardware bring-up we can also
                                // have a primary line for the requested target
                                // even when S-cache metadata holds another tag.
                                // Emit the indexed-line backinv first, then a
                                // target-address backinv if it differs.
                                if (cop_meta_line_valid_q) begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= {cop_meta_line_tag_q, cop_idx, 7'b0};
                                    backinv_cacheop <= 1'b1;
                                    backinv_icache <= 1'b1;
                                    if ({cop_meta_line_tag_q, cop_idx, 7'b0} != cop_target_backinv_addr) begin
                                        backinv_pending_valid <= 1'b1;
                                        backinv_pending_addr <= cop_target_backinv_addr;
                                        backinv_pending_cacheop <= 1'b1;
                                        backinv_pending_icache <= 1'b1;
                                    end
                                end else begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= cop_target_backinv_addr;
                                    backinv_cacheop <= 1'b1;
                                    backinv_icache <= 1'b1;
                                end
                                if (cop_meta_line_valid_q) begin
                                    wb_idx <= cop_idx;
                                    wb_tag <= cop_meta_line_tag_q;
                                    wb_is_cache_op <= 1'b1;
                                    cache_wb_keep_valid <= 1'b0;
                                    cache_wb_install_cde <= 1'b0;
                                    backinv_victim_dirty <= cop_meta_line_dirty_q;
                                    backinv_intervention_dirty <= 1'b0;
                                    // Index WBINV operates on the installed
                                    // line, independent of the operand tag.
                                    // Dirty Primary-D sublines must intervene
                                    // before that Secondary line is written
                                    // back and invalidated.
                                    backinv_writeback <= 1'b1;
                                    state <= S_BACKINV_WAIT;
                                end
                            end
                            3'd2: begin
                                if (cop_meta_line_valid_q) begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= {cop_meta_line_tag_q, cop_idx, 7'b0};
                                    backinv_cacheop <= 1'b1;
                                    backinv_icache <= 1'b1;
                                end
                            end
                            3'd3: begin
                                // Secondary Create Dirty Exclusive always
                                // invalidates matching primary blocks.  On a
                                // hit this is required to remove virtual
                                // aliases before the secondary line is
                                // repurposed; on a miss, invalidate the old
                                // indexed victim first and then the target in
                                // case the FPGA primary-subset invariant was
                                // already violated.
                                if (cop_meta_line_valid_q) begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= {cop_meta_line_tag_q, cop_idx, 7'b0};
                                    backinv_cacheop <= 1'b1;
                                    backinv_icache <= 1'b1;
                                    if ({cop_meta_line_tag_q, cop_idx, 7'b0} !=
                                        cop_target_backinv_addr) begin
                                        backinv_pending_valid <= 1'b1;
                                        backinv_pending_addr <= cop_target_backinv_addr;
                                        backinv_pending_cacheop <= 1'b1;
                                        backinv_pending_icache <= 1'b1;
                                    end
                                end else begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= cop_target_backinv_addr;
                                    backinv_cacheop <= 1'b1;
                                    backinv_icache <= 1'b1;
                                end
                                if (cop_meta_line_valid_q && cop_meta_line_dirty_q &&
                                    cop_meta_line_tag_q != cop_tag) begin
                                    wb_cnt <= 4'd0;
                                    wb_idx <= cop_idx;
                                    wb_tag <= cop_meta_line_tag_q;
                                    wb_is_cache_op <= 1'b1;
                                    cache_wb_keep_valid <= 1'b0;
                                    cache_wb_install_cde <= 1'b1;
                                    fill_addr <= cop_target_backinv_addr;
                                    fill_first <= 2'd2;
                                    state <= S_WB_PREP;
                                end else if (cop_meta_line_valid_q &&
                                             cop_meta_line_tag_q == cop_tag) begin
                                    // R4000 SD CDE hit: retain the existing
                                    // secondary data. Only the state becomes
                                    // Dirty Exclusive and matching primary
                                    // aliases are discarded. Zeroing a hit
                                    // destroys initialized shared-arena data.
                                    cache_wb_install_cde <= 1'b0;
                                    cache_wb_keep_valid <= 1'b0;
                                end else begin
                                    fill_addr <= cop_target_backinv_addr;
                                    fill_cnt <= 4'd0;
                                    cache_wb_install_cde <= 1'b1;
                                    cache_wb_keep_valid <= 1'b0;
                                    state <= S_CDE_ZERO;
                                end
                            end
                            3'd4: begin
                                // The FPGA model can temporarily violate the
                                // R4000 primary-subset property when L1
                                // writeback/refill traffic races a secondary
                                // hit invalidate.  Still issue a target-address
                                // backinv so primary caches can tag-check and
                                // drop stale matching lines.
                                backinv_valid <= 1'b1;
                                backinv_addr  <= cop_target_backinv_addr;
                                backinv_cacheop <= 1'b1;
                                backinv_icache <= 1'b1;
                            end
                            3'd5: begin
                                backinv_valid <= 1'b1;
                                backinv_addr  <= cop_target_backinv_addr;
                                backinv_cacheop <= 1'b1;
                                backinv_icache <= 1'b1;
                                if (cop_meta_line_valid_q && cop_meta_line_tag_q == cop_tag) begin
                                    wb_idx <= cop_idx;
                                    wb_tag <= cop_meta_line_tag_q;
                                    wb_is_cache_op <= 1'b1;
                                    cache_wb_keep_valid <= 1'b0;
                                    cache_wb_install_cde <= 1'b0;
                                    backinv_victim_dirty <= cop_meta_line_dirty_q;
                                    backinv_intervention_dirty <= 1'b0;
                                    backinv_writeback <= 1'b1;
                                    state <= S_BACKINV_WAIT;
                                end
                            end
                            3'd6: begin
                                if (cop_meta_line_valid_q && cop_meta_line_tag_q == cop_tag &&
                                    cop_meta_line_dirty_q) begin
                                    wb_cnt <= 4'd0;
                                    wb_idx <= cop_idx;
                                    wb_tag <= cop_meta_line_tag_q;
                                    wb_is_cache_op <= 1'b1;
                                    cache_wb_keep_valid <= 1'b1;
                                    cache_wb_install_cde <= 1'b0;
                                    fill_first <= 2'd2;
                                    state <= S_WB_PREP;
                                end
                            end
                            default: ;
                            endcase
                        end else if (d_sc_miss) begin
                            fill_addr <= d_scache_addr;
                            fill_cnt  <= 4'd0;
                            fill_is_i <= 1'b0;
                            fill_first <= 2'd2;
                            wb_is_cache_op <= 1'b0;
                            cache_wb_keep_valid <= 1'b0;
                            cache_wb_install_cde <= 1'b0;
                            if (d_meta_line_valid_q) begin
                                wb_idx <= d_idx;
                                wb_tag <= d_meta_line_tag_q;
                                backinv_victim_dirty <= d_victim_dirty;
                                backinv_intervention_dirty <= 1'b0;
                                if (d_low_vector_write_start &&
                                    ({d_meta_line_tag_q, d_idx, 7'b0} != d_low_vector_backinv_addr)) begin
                                    backinv_pending_valid <= 1'b1;
                                    backinv_pending_addr <= {d_meta_line_tag_q, d_idx, 7'b0};
                                    backinv_pending_cacheop <= 1'b0;
                                    backinv_pending_icache <= d_victim_icache_stale;
                                    if (d_victim_dirty) begin
                                        wb_cnt <= 4'd0;
                                        state <= S_WB_PREP;
                                    end else begin
                                        state <= S_DFILL;
                                    end
                                end else begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= {d_meta_line_tag_q, d_idx, 7'b0};
                                    backinv_cacheop <= 1'b0;
                                    backinv_icache <= d_victim_icache_stale;
                                    backinv_writeback <= 1'b1;
                                    state <= S_BACKINV_WAIT;
                                end
                            end else begin
                                state <= S_DFILL;
                            end
                        end else if (i_uncached_ext_miss) begin
                            uncached_i_addr <= i_addr;
                            uncached_i_valid <= 1'b0;
                            fill_first <= 2'd2;
                            state <= S_IUNCACHED;
                        end else if (i_sc_miss) begin
                            fill_addr <= i_scache_addr;
                            fill_cnt  <= 4'd0;
                            fill_is_i <= 1'b1;
                            fill_first <= 2'd2;
                            wb_is_cache_op <= 1'b0;
                            cache_wb_keep_valid <= 1'b0;
                            cache_wb_install_cde <= 1'b0;
                            if (i_meta_line_valid_q) begin
                                backinv_valid <= 1'b1;
                                backinv_addr  <= {i_meta_line_tag_q, i_idx, 7'b0};
                                backinv_cacheop <= 1'b0;
                                backinv_icache <= i_victim_icache_stale;
                                wb_idx <= i_idx;
                                wb_tag <= i_meta_line_tag_q;
                                backinv_writeback <= 1'b1;
                                backinv_victim_dirty <= i_victim_dirty;
                                backinv_intervention_dirty <= 1'b0;
                                state <= S_BACKINV_WAIT;
                            end else begin
                                state <= S_IFILL;
                            end
                        end
                    end
                end

                S_WB_PREP: begin
                    state <= wb_is_cache_op ? S_CACHE_WB : S_WB;
                end

                S_BACKINV_WAIT: begin
                    if (d_intervention_write_sc)
                        backinv_intervention_dirty <= 1'b1;
                    if (backinv_done) begin
                        if (backinv_victim_dirty ||
                            backinv_intervention_dirty ||
                            d_intervention_write_sc) begin
                            wb_cnt <= 4'd0;
                            fill_first <= 2'd2;
                            state <= S_WB_PREP;
                        end else begin
                            fill_first <= 2'd2;
                            if (wb_is_cache_op) begin
                                wb_is_cache_op <= 1'b0;
                                state <= S_META_WAIT;
                            end else begin
                                state <= fill_is_i ? S_IFILL : S_DFILL;
                            end
                        end
                    end
                end

                S_WB: begin
                    if (writeback_complete) begin
                        if (wb_cnt == 4'd15) begin
                            fill_cnt <= 4'd0;
                            fill_first <= 2'd2;
                            state <= fill_is_i ? S_IFILL : S_DFILL;
                        end else begin
                            wb_cnt <= wb_cnt + 4'd1;
                            fill_first <= 2'd1;
                        end
                    end
                end

                S_DFILL, S_IFILL: begin
                    if (ext_dmem_ready && fill_first == 2'd0) begin
                        if (fill_cnt == 4'd15)
                            state <= S_DONE;
                        fill_cnt <= fill_cnt + 4'd1;
                    end
                end

                S_IUNCACHED: begin
                    if (ext_dmem_ready && fill_first == 2'd0) begin
                        uncached_i_rdata <= ext_dmem_rdata;
                        uncached_i_valid <= 1'b1;
                        state <= S_IDLE;
                    end
                end

                S_CACHE_WB: begin
                    if (writeback_complete) begin
                        if (wb_cnt == 4'd15) begin
                            if (cache_wb_install_cde) begin
                                fill_cnt <= 4'd0;
                                state <= S_CDE_ZERO;
                                wb_is_cache_op <= 1'b0;
                                cache_wb_keep_valid <= 1'b0;
                            end else if (!meta_aux_write_busy) begin
                                state <= S_META_WAIT;
                                wb_is_cache_op <= 1'b0;
                                cache_wb_keep_valid <= 1'b0;
                            end
                        end else begin
                            wb_cnt <= wb_cnt + 4'd1;
                            fill_first <= 2'd1;
                        end
                    end
                end

                S_CDE_ZERO: begin
                    if (fill_cnt == 4'd15) begin
                        if (!meta_aux_write_busy) begin
                            cache_wb_install_cde <= 1'b0;
                            fill_cnt <= 4'd0;
                            state <= S_META_WAIT;
                        end
                    end else begin
                        fill_cnt <= fill_cnt + 4'd1;
                    end
                end

                S_DONE: begin
                    if (!meta_aux_write_busy)
                        state <= S_META_WAIT;
                end

                S_META_WAIT: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
`else
    assign backinv_ready = 1'b0;
    assign snoop_wr_ready = 1'b1;

    // ---------------------------------------------------------------
    // Address decomposition for parameterized 128-byte lines
    // [6:0]   byte offset within line (128 bytes)
    // [IDX_W+6:7] index
    // [31:IDX_W+7] tag
    // ---------------------------------------------------------------
    localparam NLINES = SCACHE_LINES;

    // Tag and status arrays
    // Packed (was unpacked) — see icache.v note.
    reg [NLINES-1:0] valid;
    reg [NLINES-1:0] dirty;
    reg [TAG_W-1:0]  tags  [0:NLINES-1];

    // Data array: SCACHE_LINES x 16 doublewords
    reg [63:0] data_array [0:SCACHE_DATA_DWS-1];

    // ---------------------------------------------------------------
    // I-side address decomposition
    // ---------------------------------------------------------------
    wire [IDX_W-1:0] i_idx    = i_addr[IDX_W+6:7];
    wire [TAG_W-1:0] i_tag    = i_addr[31:IDX_W+7];
    wire [3:0]       i_dw_off = i_addr[6:3];

    wire i_sc_hit = valid[i_idx] && (tags[i_idx] == i_tag);

    // Combinational read from S-cache data array
    wire [63:0] i_sc_dw = data_array[{i_idx, i_dw_off}];
    // Extract 32-bit word (big-endian: addr[2]=0 → upper word)
    wire [31:0] i_sc_word = i_addr[2] ? i_sc_dw[31:0] : i_sc_dw[63:32];

    // ---------------------------------------------------------------
    // D-side address decomposition
    // ---------------------------------------------------------------
    wire [IDX_W-1:0] d_idx    = d_addr[IDX_W+6:7];
    wire [TAG_W-1:0] d_tag    = d_addr[31:IDX_W+7];
    wire [3:0]       d_dw_off = d_addr[6:3];

    wire d_sc_hit = valid[d_idx] && (tags[d_idx] == d_tag);

    // Combinational read from S-cache data array
    wire [63:0] d_sc_dw = data_array[{d_idx, d_dw_off}];

    // ---------------------------------------------------------------
    // CACHE instruction address decomposition
    // ---------------------------------------------------------------
    wire [IDX_W-1:0] cop_idx = cop_paddr[IDX_W+6:7];
    wire [TAG_W-1:0] cop_tag = cop_paddr[31:IDX_W+7];

    // Combinational tag readback for Index Load Tag (cop_op=1)
    // TagLo format: tag in high bits, [7:6]=CacheState (00=inv,01=clean,10=dirty)
    wire [1:0] cop_sc_state = valid[cop_idx] ?
        (dirty[cop_idx] ? 2'b10 : 2'b01) : 2'b00;
    assign cop_taglo_out = valid[cop_idx] ?
        {tags[cop_idx], {(24-TAG_W){1'b0}}, cop_sc_state, 6'b0} :
        32'd0;
    assign cop_taglo_valid = cop_valid && (cop_op == 3'd1);

    // ---------------------------------------------------------------
    // DMA snoop address decomposition (write invalidation)
    // ---------------------------------------------------------------
    wire [IDX_W-1:0] snoop_idx = snoop_addr[IDX_W+6:7];
    wire [TAG_W-1:0] snoop_tag = snoop_addr[31:IDX_W+7];
    wire [3:0]       snoop_dw  = snoop_addr[6:3];

    // ---------------------------------------------------------------
    // DMA read snoop: combinational lookup for bus-master reads
    // Models the MC's bus snooping of the S-cache on DMA reads.
    // If the address matches a dirty S-cache line, return the cached
    // data (which may differ from stale RAM data).
    // ---------------------------------------------------------------
    wire [IDX_W-1:0] snoop_rd_idx = snoop_rd_addr[IDX_W+6:7];
    wire [TAG_W-1:0] snoop_rd_tag = snoop_rd_addr[31:IDX_W+7];
    wire [3:0]       snoop_rd_dw  = snoop_rd_addr[6:3]; // doubleword within line

    // During S_DFILL/S_IFILL/S_DDONE/S_IDONE, data_array entries at the fill
    // index are being overwritten with new fill data, but tags/valid/dirty still
    // reflect the OLD (evicted) line. A snoop hit would return corrupted fill
    // data instead of the original dirty data. The dirty line's writeback to RAM
    // completed before the fill started (S_DWB/S_IWB → S_DFILL/S_IFILL), so
    // RAM has the correct data — safe to let the DMA read fall through to RAM.
    wire snoop_fill_conflict = (state == S_DFILL || state == S_IFILL ||
                                state == S_DDONE || state == S_IDONE) &&
                               (fill_addr[IDX_W+6:7] == snoop_rd_idx);

    assign snoop_rd_resp_valid = snoop_rd_valid;
    assign snoop_rd_hit = snoop_rd_valid &&
                          valid[snoop_rd_idx] &&
                          (tags[snoop_rd_idx] == snoop_rd_tag) &&
                          dirty[snoop_rd_idx] &&
                          !snoop_fill_conflict;

    assign snoop_rd_data = data_array[{snoop_rd_idx, snoop_rd_dw}];

    // ---------------------------------------------------------------
    // Miss detection
    // ---------------------------------------------------------------
    // I-cache needs S-cache when it has a cached miss or is filling
    wire i_need_sc = i_need_fill && !i_uncached;
    wire i_sc_miss = i_need_sc && !i_sc_hit;

    // D-cache needs S-cache for cached reads AND writes.
    // Write misses trigger S-cache fill (write-allocate): the fill brings
    // the line from RAM, then the D-cache retries the store on the now-valid
    // S-cache line. This ensures write-through updates S-cache, not just RAM.
    wire d_need_sc = (d_ren || d_wen) && !d_uncached;
    wire d_sc_miss = d_need_sc && !d_sc_hit;

    // ---------------------------------------------------------------
    // State machine
    // ---------------------------------------------------------------
    localparam [3:0]
        S_IDLE     = 4'd0,
        S_DWB      = 4'd1,   // writeback dirty S-cache victim (for D miss)
        S_DFILL    = 4'd2,   // fill S-cache line from ext memory (for D miss)
        S_DDONE    = 4'd3,   // commit D fill metadata
        S_IWB      = 4'd4,   // writeback dirty S-cache victim (for I miss)
        S_IFILL    = 4'd5,   // fill S-cache line from ext memory (for I miss)
        S_IDONE    = 4'd6,   // commit I fill metadata
        S_CACHE_WB = 4'd7;   // CACHE instruction writeback

    reg [3:0]  state;
    reg [3:0]  fill_cnt;      // doubleword counter (0-15)
    reg [31:0] fill_addr;     // address triggering fill (determines line)
    reg [31:0] wb_addr;       // address of line being written back
    reg [63:0] wb_buf [0:15]; // writeback buffer (saves dirty victim)

    // CDE pending: after S_CACHE_WB completes, install new CDE tag instead of invalidating
    reg        cde_pending;
    // HWB (op 6): keep line valid after writeback (vs HWBINV which invalidates)
    reg        wb_keep_valid;
    reg [TAG_W-1:0] cde_tag;
    reg [IDX_W-1:0] cde_idx;

    // COP latch: holds a CACHE instruction that arrived during a miss.
    // Without this, CACHE ops are silently dropped when S-cache is busy with a fill.
    reg        cop_latched;
    reg [2:0]  cop_latched_op;
    reg [31:0] cop_latched_paddr;
    reg [31:0] cop_latched_taglo;

    // Guard against stale ext_dmem_ready on first cycles of fill/writeback.
    // When transitioning from S_IDLE to S_DFILL/S_IFILL/S_DWB/S_IWB, the MC
    // might still have cpu_ready=1 from a previous transaction (MC pipeline:
    // S_CPU_RAM sets ready=1, S_WAIT clears it next cycle). We need 2 cycles
    // for the stale ready to fully clear before accepting new responses.
    reg [1:0] fill_first;

    wire sc_busy = (state != S_IDLE);

    // ---------------------------------------------------------------
    // Freeze logic — freeze each L1 only on its own S-cache miss.
    // This prevents deadlock when both L1s miss at the same S-cache
    // index with different tags: each L1 can read from S-cache while
    // the other's miss is deferred (see deferral logic in S_IDLE).
    // For uncached D accesses, stall until ext_dmem_ready is asserted.
    // ---------------------------------------------------------------

    // Detect when each L1 is actively reading from S-cache
    // D-cache reads during its S_FILL state (d_ren=1 from D-cache mem interface)
    // I-cache reads during its S_FILL state (i_need_fill=1)
    wire d_reading_sc = d_ren && !d_uncached && d_sc_hit;
    wire i_reading_sc = i_need_sc && i_sc_hit;

    // Uncached cooldown: after each uncached transaction completes,
    // suppress ext_dmem_ren/wen for one cycle so peripherals see valid=0
    // and clear their ready. Without this, back-to-back uncached ops to
    // the same peripheral (e.g. store then load) keep valid high and the
    // peripheral never processes the second transaction.
    reg [1:0] uncached_cooldown;
    reg       uncached_d_inflight;
    reg [31:0] uncached_d_addr_q;
    reg        uncached_d_ren_q;
    reg        uncached_d_wen_q;
    reg [7:0]  uncached_d_wmask_q;
    reg [63:0] uncached_d_wdata_q;

    wire uncached_d_req    = d_uncached && (d_ren || d_wen);
    wire uncached_d_can_start /* verilator public_flat_rd */ = uncached_d_req &&
                                (uncached_cooldown == 2'd0) &&
                                !uncached_d_inflight && !sc_busy;
    wire uncached_d_issue  = uncached_d_inflight;
    wire uncached_d_start  = uncached_d_can_start;
    wire uncached_d_pending = (uncached_d_inflight && !uncached_accepted) ||
                              (uncached_d_req && !uncached_accepted);
    wire [31:0] uncached_d_bus_addr =
        uncached_d_inflight ? uncached_d_addr_q : d_addr;
    wire        uncached_d_bus_ren =
        uncached_d_inflight ? uncached_d_ren_q : d_ren;
    wire        uncached_d_bus_wen =
        uncached_d_inflight ? uncached_d_wen_q : d_wen;
    wire [7:0]  uncached_d_bus_wmask =
        uncached_d_inflight ? uncached_d_wmask_q : d_wmask;
    wire [63:0] uncached_d_bus_wdata =
        uncached_d_inflight ? uncached_d_wdata_q : d_wdata;

    assign i_freeze = sc_busy || i_sc_miss;
    assign d_freeze = sc_busy || d_sc_miss;
    assign stall    = sc_busy || uncached_d_pending;
    // Gate on !sc_busy: during S-cache fills, ext_dmem_ready reflects the
    // fill response (from RAM), not the uncached D response (from device).
    // Without this, uncached_accepted fires spuriously during I-cache fills
    // that trigger S-cache misses, causing ds_store_acked to suppress the
    // store before the device ever sees it.
    wire uncached_d_ext_accepted /* verilator public_flat_rd */ =
        uncached_d_issue && uncached_d_inflight && ext_dmem_ready;
    wire uncached_d_resp_valid /* verilator public_flat_rd */ = 1'b0;
    assign uncached_started = uncached_d_start;
    assign uncached_accepted = uncached_d_ext_accepted;

    // Uncached write indicator: ext_dmem_wen is from a CPU uncached store
    // (NOT from S-cache writeback or write-through miss)
    assign ext_dmem_uncached_wr = (state == S_IDLE) && uncached_d_issue &&
                                  uncached_d_bus_wen;
    assign ext_dmem_cache_owned = (state != S_IDLE) &&
                                  (ext_dmem_ren || ext_dmem_wen);
    assign debug_status = {
        4'h2,
        state,
        1'b0,
        stall,
        i_freeze,
        d_freeze,
        i_sc_hit,
        d_sc_hit,
        i_sc_miss,
        d_sc_miss,
        uncached_d_issue,
        ext_dmem_ready,
        ext_dmem_ren,
        ext_dmem_wen,
        fill_first,
        fill_cnt,
        uncached_cooldown,
        backinv_valid,
        3'b000
    };
    assign debug_i_capture = 128'd0;
    assign debug_scache_probe = 384'd0;
    assign debug_addr = ext_dmem_addr;

    // ---------------------------------------------------------------
    // Data outputs to L1 caches (combinational)
    // ---------------------------------------------------------------
    // I-cache: uncached → pass-through from ext imem; cached hit → S-cache data
    assign ext_imem_addr = i_addr;
    wire i_uncached_bypass = i_uncached && !i_need_fill && !i_wen;
    assign i_fill_rdata = i_sc_hit ? i_sc_word : 32'd0;
    assign i_rdata = i_uncached_bypass ? ext_imem_rdata :
                     i_fill_rdata;

    // Pre-compute write-through merge (prevents simulator optimization issue)
    wire [63:0] d_wt_merged = byte_merge(d_sc_dw, d_wdata, d_wmask);

    // CPU uncached reads bypass S-cache.  DMA dirty intervention uses the
    // separate snoop_rd_* path above.
    assign d_rdata = (!d_uncached && d_sc_hit) ? d_sc_dw :
                                                 ext_dmem_rdata;

    // ---------------------------------------------------------------
    // Byte merge function
    // ---------------------------------------------------------------
    function [63:0] byte_merge;
        input [63:0] old_val;
        input [63:0] new_val;
        input [7:0]  mask;
        integer b;
        begin
            for (b = 0; b < 8; b = b + 1)
                byte_merge[b*8 +: 8] = mask[b] ? new_val[b*8 +: 8] : old_val[b*8 +: 8];
        end
    endfunction

    // ---------------------------------------------------------------
    // External dmem port multiplexing (combinational)
    // ---------------------------------------------------------------
    always @(*) begin
        ext_dmem_addr  = d_addr;
        ext_dmem_ren   = 1'b0;
        ext_dmem_wen   = 1'b0;
        ext_dmem_wmask = 8'h00;
        ext_dmem_wdata = 64'd0;

        case (state)
            S_IDLE: begin
                // Uncached D pass-through (suppressed during cooldown)
                if (uncached_d_issue) begin
                    ext_dmem_addr  = uncached_d_bus_addr;
                    ext_dmem_ren   = uncached_d_bus_ren;
                    ext_dmem_wen   = uncached_d_bus_wen;
                    ext_dmem_wmask = uncached_d_bus_wen ? uncached_d_bus_wmask : 8'h00;
                    ext_dmem_wdata = uncached_d_bus_wdata;
                end
                // Write-through miss: write to RAM when S-cache doesn't have the line
                else if (ext_wt_miss_pending) begin
                    ext_dmem_addr  = ext_wt_miss_addr;
                    ext_dmem_wen   = 1'b1;
                    ext_dmem_wmask = ext_wt_miss_wmask;
                    ext_dmem_wdata = ext_wt_miss_data;
                end
            end

            S_DWB, S_IWB, S_CACHE_WB: begin
                // Writeback dirty S-cache line to external memory
                ext_dmem_addr  = {wb_addr[31:7], fill_cnt[3:0], 3'b000};
                ext_dmem_wen   = 1'b1;
                ext_dmem_wmask = 8'hFF;
                ext_dmem_wdata = wb_buf[fill_cnt];
            end

            S_DFILL, S_IFILL: begin
                // Fill S-cache line from external memory
                ext_dmem_addr = {fill_addr[31:7], fill_cnt[3:0], 3'b000};
                ext_dmem_ren  = 1'b1;
            end

            default: ;
        endcase
    end

    // ---------------------------------------------------------------
    // Main state machine
    // ---------------------------------------------------------------
    // Deferred IST tag write: separated from the case statement to work
    // around a simulator optimization issue where tag array writes via
    // variable index inside deeply nested case/if blocks get dropped.
    reg              ist_wr_pending;
    reg [IDX_W-1:0]  ist_wr_idx;
    reg [TAG_W-1:0]  ist_wr_tag;

    // Write-through miss: when D-cache writes through but S-cache doesn't
    // have the line, queue an external memory write to preserve the data.
    reg              ext_wt_miss_pending;
    reg [31:0]       ext_wt_miss_addr;
    reg [63:0]       ext_wt_miss_data;
    reg [7:0]        ext_wt_miss_wmask;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            ext_wt_miss_pending <= 1'b0;
            fill_cnt <= 4'd0;
            fill_first <= 2'd0;
            uncached_cooldown <= 2'd0;
            uncached_d_inflight <= 1'b0;
            uncached_d_addr_q <= 32'd0;
            uncached_d_ren_q <= 1'b0;
            uncached_d_wen_q <= 1'b0;
            uncached_d_wmask_q <= 8'd0;
            uncached_d_wdata_q <= 64'd0;
            cde_pending <= 1'b0;
            wb_keep_valid <= 1'b0;
            cop_latched <= 1'b0;
            cde_tag <= {TAG_W{1'b0}};
            cde_idx <= {IDX_W{1'b0}};
            backinv_valid <= 1'b0;
            backinv_addr  <= 32'd0;
            backinv_cacheop <= 1'b0;
            backinv_icache <= 1'b0;
            backinv_writeback <= 1'b0;
            ist_wr_pending <= 1'b0;
            ist_wr_idx <= {IDX_W{1'b0}};
            ist_wr_tag <= {TAG_W{1'b0}};
            valid <= {NLINES{1'b0}};
            dirty <= {NLINES{1'b0}};
            snoop_wr_done <= 1'b0;
        end else begin
            snoop_wr_done <= snoop_valid;
            // Apply deferred IST tag write
            if (ist_wr_pending) begin
                tags[ist_wr_idx] <= ist_wr_tag;
                ist_wr_pending   <= 1'b0;
            end
            // 2-cycle cooldown: on uncached_accepted, set to 2.
            // Counts down each cycle: 2→1→0. Pipeline stays stalled
            // while cooldown != 0, ensuring addr/data bus settles
            // before the next uncached transaction.
            if (uncached_accepted)
                uncached_cooldown <= 2'd2;
            else if (uncached_cooldown != 2'd0)
                uncached_cooldown <= uncached_cooldown - 2'd1;
            if (uncached_accepted)
                uncached_d_inflight <= 1'b0;
            else if (uncached_d_start) begin
                uncached_d_inflight <= 1'b1;
                uncached_d_addr_q <= d_addr;
                uncached_d_ren_q <= d_ren;
                uncached_d_wen_q <= d_wen;
                uncached_d_wmask_q <= d_wmask;
                uncached_d_wdata_q <= d_wdata;
            end

            // Self-snoop for uncached writes: runs EVERY cycle while an
            // uncached write is pending (not just at acceptance). If the
            // S-cache has a dirty line at the same physical address, update
            // the data in-place so any future writeback is consistent with RAM.
            // On real R4000, the SysAD bus interface snoops all transactions.
            if (d_wen && d_uncached && !force_uncached &&
                valid[d_idx] && tags[d_idx] == d_addr[31:IDX_W+7]) begin
                // Update the S-cache data to match the uncached store
                data_array[{d_idx, d_dw_off}] <= byte_merge(
                    data_array[{d_idx, d_dw_off}], d_wdata, d_wmask);
            end
            // Decrement fill_first counter each cycle
            if (fill_first != 2'd0) fill_first <= fill_first - 2'd1;
            // Back-invalidation is a single-cycle pulse (self-clearing)
            backinv_valid <= 1'b0;
            backinv_cacheop <= 1'b0;
            backinv_icache <= 1'b0;
            backinv_writeback <= 1'b0;

            // COP latch: capture CACHE instructions that arrive when S-cache is busy.
            // Must be OUTSIDE the state machine case so it works in any state.
            if (cop_valid && state != S_IDLE) begin
                cop_latched <= 1'b1;
                cop_latched_op <= cop_op;
                cop_latched_paddr <= cop_paddr;
                cop_latched_taglo <= cop_taglo_in;
            end

            case (state)
                S_IDLE: begin
                    // I-cache writeback to S-cache (CACHE PI|HWB)
                    if (i_wen && i_sc_hit) begin
                        data_array[{i_idx, i_dw_off}] <= i_wdata;
                        dirty[i_idx] <= 1'b1;
                    end
                    // D-cache write-through to S-cache: update data, mark dirty
                    if (d_wen && !d_uncached && d_sc_hit) begin : dwt_blk
                        reg [63:0] merged;
                        merged = d_wt_merged;
                        data_array[{d_idx, d_dw_off}] <= merged;
                        dirty[d_idx] <= 1'b1;
                    end
                    // Write-through miss: S-cache doesn't have the line (inclusion
                    // violated, e.g. by I-cache fill evicting S-cache line at same
                    // index). Write directly to external memory (RAM) so the data
                    // isn't lost. This models the MC handling a write-through miss.
                    if (d_wen && !d_uncached && !d_sc_hit && !d_sc_miss) begin
                        ext_wt_miss_pending <= 1'b1;
                        ext_wt_miss_addr    <= d_addr;
                        ext_wt_miss_data    <= d_wdata;
                        ext_wt_miss_wmask   <= d_wmask;
                    end
                    // Clear write-through miss after external write accepted.
                    // Only clear when the ext_dmem is actually driving the wt_miss
                    // (not when ready fires for an unrelated uncached request).
                    if (ext_wt_miss_pending && ext_dmem_ready &&
                        !(d_uncached && uncached_cooldown == 2'd0))
                        ext_wt_miss_pending <= 1'b0;

                    // D-cache miss has priority over I-cache miss.
                    // Deferral: don't start a fill that would evict a line
                    // being actively read by the other L1 cache. This
                    // prevents infinite thrashing when both L1s need
                    // different tags at the same S-cache index.
                    if (uncached_d_req || uncached_d_inflight) begin
                        // Drain an accepted uncached transaction before any
                        // cached fill can claim the external-memory response.
                    end else if (d_sc_miss) begin
                        // Defer D fill if I is reading from S-cache at same index
                        if (!i_reading_sc || (i_idx != d_idx)) begin
                            fill_addr <= d_addr;
                            fill_cnt  <= 4'd0;
                            fill_first <= 2'd2;
                            // Back-invalidate D-cache: evicting valid S-cache line
                            if (valid[d_idx]) begin
                                backinv_valid <= 1'b1;
                                backinv_addr  <= {tags[d_idx], d_idx, 7'b0};
                                backinv_cacheop <= 1'b0;
                                backinv_icache <= 1'b1;
                            end
                            if (valid[d_idx] && dirty[d_idx]) begin
                                // Dirty victim: writeback first
                                wb_addr <= {tags[d_idx], d_idx, 7'b0};
                                for (i = 0; i < LINE_DWS; i = i + 1)
                                    wb_buf[i] <= data_array[{d_idx, i[3:0]}];
                                state <= S_DWB;
                            end else begin
                                state <= S_DFILL;
                            end
                        end
                    end else if (i_sc_miss) begin
                        // Defer I fill if D is reading from S-cache at same index
                        if (!d_reading_sc || (d_idx != i_idx)) begin
                            fill_addr <= i_addr;
                            fill_cnt  <= 4'd0;
                            fill_first <= 2'd2;
                            // Back-invalidate D-cache: evicting valid S-cache line
                            if (valid[i_idx]) begin
                                backinv_valid <= 1'b1;
                                backinv_addr  <= {tags[i_idx], i_idx, 7'b0};
                                backinv_cacheop <= 1'b0;
                                backinv_icache <= 1'b1;
                            end
                            // Check if dirty — also account for simultaneous D-cache
                            // write-through that sets dirty on this same cycle.
                            // The write-through is non-blocking (dirty not yet 1),
                            // but we must treat the line as dirty for writeback.
                            if (valid[i_idx] && (dirty[i_idx] ||
                                (d_wen && !d_uncached && d_sc_hit && d_idx == i_idx))) begin
                                wb_addr <= {tags[i_idx], i_idx, 7'b0};
                                for (i = 0; i < LINE_DWS; i = i + 1) begin
                                    // Merge write-through data into writeback buffer
                                    if (d_wen && !d_uncached && d_sc_hit &&
                                        d_idx == i_idx && d_dw_off == i[3:0])
                                        wb_buf[i] <= byte_merge(d_sc_dw, d_wdata, d_wmask);
                                    else
                                        wb_buf[i] <= data_array[{i_idx, i[3:0]}];
                                end
                                state <= S_IWB;
                            end else begin
                                state <= S_IFILL;
                            end
                        end
                    end

                    // (COP latch is now at top level, before case(state))

                    // CACHE instruction (secondary cache operations)
                    // CACHE instruction: process or replay latched COP
                    if (!(uncached_d_req || uncached_d_inflight) &&
                        ((cop_valid && !d_sc_miss && !i_sc_miss) ||
                         (cop_latched && !d_sc_miss && !i_sc_miss))) begin
                        cop_latched <= 1'b0;
                        begin : cop_dispatch
                            // Select between live and latched COP values
                            reg cop_use_latch;
                            reg [2:0]  eff_cop_op;
                            reg [IDX_W-1:0] eff_cop_idx;
                            reg [TAG_W-1:0] eff_cop_tag;
                            reg [31:0] eff_cop_taglo;
                            cop_use_latch = cop_latched && !(cop_valid && !d_sc_miss && !i_sc_miss);
                            eff_cop_op    = cop_use_latch ? cop_latched_op    : cop_op;
                            eff_cop_idx   = cop_use_latch ? cop_latched_paddr[IDX_W+6:7] : cop_idx;
                            eff_cop_tag   = cop_use_latch ? cop_latched_paddr[31:IDX_W+7] : cop_tag;
                            eff_cop_taglo = cop_use_latch ? cop_latched_taglo : cop_taglo_in;
                        case (eff_cop_op)
                            3'd0: begin // Index Writeback Invalidate
                                if (valid[eff_cop_idx]) begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= {tags[eff_cop_idx], eff_cop_idx, 7'b0};
                                    backinv_cacheop <= 1'b1;
                                    backinv_icache <= 1'b1;
                                end
                                if (valid[eff_cop_idx] && dirty[eff_cop_idx]) begin
                                    wb_addr <= {tags[eff_cop_idx], eff_cop_idx, 7'b0};
                                    for (i = 0; i < LINE_DWS; i = i + 1)
                                        wb_buf[i] <= data_array[{eff_cop_idx, i[3:0]}];
                                    fill_cnt <= 4'd0;
                                    fill_first <= 2'd2;
                                    wb_keep_valid <= 1'b0;
                                    state <= S_CACHE_WB;
                                end else begin
                                    valid[eff_cop_idx] <= 1'b0;
                                    dirty[eff_cop_idx] <= 1'b0;
                                end
                            end
                            3'd4: begin // Hit Invalidate
                                // Preserve primary-cache inclusion even when
                                // the modeled S-cache metadata misses due to
                                // racing L1 writeback/refill traffic.
                                backinv_valid <= 1'b1;
                                backinv_addr  <= {eff_cop_tag, eff_cop_idx, 7'b0};
                                backinv_cacheop <= 1'b1;
                                backinv_icache <= 1'b1;
                                if (valid[eff_cop_idx] && tags[eff_cop_idx] == eff_cop_tag) begin
                                    valid[eff_cop_idx] <= 1'b0;
                                    dirty[eff_cop_idx] <= 1'b0;
                                end
                            end
                            3'd5: begin // Hit Writeback Invalidate
                                backinv_valid <= 1'b1;
                                backinv_addr  <= {eff_cop_tag, eff_cop_idx, 7'b0};
                                backinv_cacheop <= 1'b1;
                                backinv_icache <= 1'b1;
                                if (valid[eff_cop_idx] && tags[eff_cop_idx] == eff_cop_tag) begin
                                    if (dirty[eff_cop_idx]) begin
                                        wb_addr <= {tags[eff_cop_idx], eff_cop_idx, 7'b0};
                                        for (i = 0; i < LINE_DWS; i = i + 1)
                                            wb_buf[i] <= data_array[{eff_cop_idx, i[3:0]}];
                                        fill_cnt <= 4'd0;
                                        fill_first <= 2'd2;
                                        wb_keep_valid <= 1'b0;
                                        state <= S_CACHE_WB;
                                    end else begin
                                        valid[eff_cop_idx] <= 1'b0;
                                    end
                                end
                            end
                            3'd6: begin // Hit Writeback (no invalidate)
                                if (valid[eff_cop_idx] && tags[eff_cop_idx] == eff_cop_tag &&
                                    dirty[eff_cop_idx]) begin
                                    wb_addr <= {tags[eff_cop_idx], eff_cop_idx, 7'b0};
                                    for (i = 0; i < LINE_DWS; i = i + 1)
                                        wb_buf[i] <= data_array[{eff_cop_idx, i[3:0]}];
                                    fill_cnt <= 4'd0;
                                    fill_first <= 2'd2;
                                    wb_keep_valid <= 1'b1;
                                    state <= S_CACHE_WB;
                                end
                            end
                            3'd2: begin // Index Store Tag
                                if (valid[eff_cop_idx]) begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= {tags[eff_cop_idx], eff_cop_idx, 7'b0};
                                    backinv_cacheop <= 1'b1;
                                    backinv_icache <= 1'b1;
                                end
                                ist_wr_pending <= 1'b1;
                                ist_wr_idx     <= eff_cop_idx;
                                ist_wr_tag     <= eff_cop_taglo[31 -: TAG_W];
                                valid[eff_cop_idx] <= (eff_cop_taglo[7:6] != 2'b00);
                                dirty[eff_cop_idx] <= eff_cop_taglo[7];
                            end
                            3'd3: begin // Create Dirty Exclusive
                                if (valid[eff_cop_idx]) begin
                                    backinv_valid <= 1'b1;
                                    backinv_addr  <= {tags[eff_cop_idx], eff_cop_idx, 7'b0};
                                    backinv_cacheop <= 1'b1;
                                    backinv_icache <= 1'b1;
                                end
                                if (valid[eff_cop_idx] &&
                                    tags[eff_cop_idx] == eff_cop_tag) begin
                                    // A CDE hit retains secondary data; only
                                    // ownership/state and primary aliases
                                    // change.
                                    valid[eff_cop_idx] <= 1'b1;
                                    dirty[eff_cop_idx] <= 1'b1;
                                end else if (valid[eff_cop_idx] && dirty[eff_cop_idx] &&
                                    tags[eff_cop_idx] != eff_cop_tag) begin
                                    wb_addr <= {tags[eff_cop_idx], eff_cop_idx, 7'b0};
                                    for (i = 0; i < LINE_DWS; i = i + 1)
                                        wb_buf[i] <= data_array[{eff_cop_idx, i[3:0]}];
                                    fill_cnt <= 4'd0;
                                    fill_first <= 2'd2;
                                    cde_pending <= 1'b1;
                                    wb_keep_valid <= 1'b0;
                                    cde_tag <= eff_cop_tag;
                                    cde_idx <= eff_cop_idx;
                                    state <= S_CACHE_WB;
                                end else begin
                                    tags[eff_cop_idx]  <= eff_cop_tag;
                                    valid[eff_cop_idx] <= 1'b1;
                                    dirty[eff_cop_idx] <= 1'b1;
                                    for (i = 0; i < LINE_DWS; i = i + 1)
                                        data_array[{eff_cop_idx, i[3:0]}] <= 64'd0;
                                end
                            end
                            default: ;
                        endcase
                        end // cop_dispatch
                    end
                end

                S_DWB: if (ext_dmem_ready && fill_first == 2'd0) begin
                    // Writeback dirty victim, then fill
                    if (fill_cnt == 4'd15) begin
                        state      <= S_DFILL;
                        fill_cnt   <= 4'd0;
                        fill_first <= 2'd2; // guard fill start after WB
                    end else begin
                        fill_cnt <= fill_cnt + 4'd1;
                    end
                end

                S_DFILL: if (ext_dmem_ready && fill_first == 2'd0) begin
                    data_array[{fill_addr[IDX_W+6:7], fill_cnt}] <= ext_dmem_rdata;
                    if (fill_cnt == 4'd15)
                        state <= S_DDONE;
                    fill_cnt <= fill_cnt + 4'd1;
                end

                S_DDONE: begin
                    tags[fill_addr[IDX_W+6:7]]  <= fill_addr[31:IDX_W+7];
                    valid[fill_addr[IDX_W+6:7]] <= 1'b1;
                    dirty[fill_addr[IDX_W+6:7]] <= 1'b0;
                    state <= S_IDLE;
                end

                S_IWB: if (ext_dmem_ready && fill_first == 2'd0) begin
                    if (fill_cnt == 4'd15) begin
                        state      <= S_IFILL;
                        fill_cnt   <= 4'd0;
                        fill_first <= 2'd2; // guard fill start after WB
                    end else begin
                        fill_cnt <= fill_cnt + 4'd1;
                    end
                end

                S_IFILL: begin
                    if (ext_dmem_ready && fill_first == 2'd0) begin
                        data_array[{fill_addr[IDX_W+6:7], fill_cnt}] <= ext_dmem_rdata;
                        if (fill_cnt == 4'd15)
                            state <= S_IDONE;
                        fill_cnt <= fill_cnt + 4'd1;
                    end
                end

                S_IDONE: begin
                    tags[fill_addr[IDX_W+6:7]]  <= fill_addr[31:IDX_W+7];
                    valid[fill_addr[IDX_W+6:7]] <= 1'b1;
                    dirty[fill_addr[IDX_W+6:7]] <= 1'b0;
                    state <= S_IDLE;
                end

                S_CACHE_WB: if (ext_dmem_ready && fill_first == 2'd0) begin
                    if (fill_cnt == 4'd15) begin
                        if (cde_pending) begin
                            // CDE: install new tag + zero data after victim writeback
                            tags[cde_idx]  <= cde_tag;
                            valid[cde_idx] <= 1'b1;
                            dirty[cde_idx] <= 1'b1;
                            for (i = 0; i < LINE_DWS; i = i + 1)
                                data_array[{cde_idx, i[3:0]}] <= 64'd0;
                            cde_pending <= 1'b0;
                        end else if (wb_keep_valid) begin
                            // HWB (op 6): keep valid, clear dirty
                            dirty[wb_addr[IDX_W+6:7]] <= 1'b0;
                        end else begin
                            // HWBINV / IndexWBInv: invalidate
                            valid[wb_addr[IDX_W+6:7]] <= 1'b0;
                            dirty[wb_addr[IDX_W+6:7]] <= 1'b0;
                        end
                        state <= S_IDLE;
                    end else begin
                        fill_cnt <= fill_cnt + 4'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase

            // DMA snoop: merge bus-master writes into any matching S-cache
            // line. A dirty line cannot be invalidated on a partial DMA write:
            // bytes not overwritten by the DMA still exist only in S-cache.
            // In real hardware, the MC handles this via bus snooping.
            if (snoop_valid && snoop_wr_ready) begin
                if (valid[snoop_idx] && tags[snoop_idx] == snoop_tag) begin
                    if (snoop_wmask != 8'h00) begin
                        data_array[{snoop_idx, snoop_dw}] <= byte_merge(
                            data_array[{snoop_idx, snoop_dw}],
                            snoop_wdata, snoop_wmask);
                    end else begin
                        valid[snoop_idx] <= 1'b0;
                        dirty[snoop_idx] <= 1'b0;
                    end
                end
            end
        end
    end

`endif

endmodule
