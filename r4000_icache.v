// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_icache.v — 8KB direct-mapped instruction cache
// 512 lines × 16 bytes/line, 32-bit instruction read
// Fill: 4 cycles (4 × 32-bit word reads from memory)

`default_nettype none

`ifdef FPGA_L1_BRAM
/* verilator lint_off DECLFILENAME */
module r4000_icache_tag_ram (
    input  wire        clk,
    input  wire [8:0]  raddr,
    output wire [19:0] rdata,
    input  wire        we,
    input  wire [8:0]  waddr,
    input  wire [19:0] wdata
);
`ifdef VERILATOR
    reg [19:0] mem [0:511];
    reg [19:0] rdata_q;
    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`elsif FPGA_XILINX
    (* ram_style = "block" *) reg [19:0] mem [0:511];
    reg [19:0] rdata_q;
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
    defparam ram.width_a = 20;
    defparam ram.widthad_a = 9;
    defparam ram.numwords_a = 512;
    defparam ram.width_b = 20;
    defparam ram.widthad_b = 9;
    defparam ram.numwords_b = 512;
    defparam ram.address_aclr_a = "NONE";
    defparam ram.address_aclr_b = "NONE";
    defparam ram.address_reg_b = "CLOCK0";
    defparam ram.outdata_reg_b = "UNREGISTERED";
    defparam ram.outdata_aclr_b = "NONE";
    defparam ram.indata_aclr_a = "NONE";
    defparam ram.wrcontrol_aclr_a = "NONE";
    defparam ram.read_during_write_mode_mixed_ports = "DONT_CARE";
`ifdef FPGA_CYCLONE_IV_GX
    defparam ram.intended_device_family = "Cyclone IV GX";
`else
    defparam ram.intended_device_family = "Cyclone IV E";
`endif
    defparam ram.lpm_type = "altsyncram";
`endif
endmodule

module r4000_icache_ram32 (
    input  wire        clk,
    input  wire [10:0] raddr,
    output wire [31:0] rdata,
    input  wire        we,
    input  wire [10:0] waddr,
    input  wire [31:0] wdata
);
`ifdef VERILATOR
    reg [31:0] mem [0:2047];
    reg [31:0] rdata_q;
    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`elsif FPGA_XILINX
    (* ram_style = "block" *) reg [31:0] mem [0:2047];
    reg [31:0] rdata_q;
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
    defparam ram.width_a = 32;
    defparam ram.widthad_a = 11;
    defparam ram.numwords_a = 2048;
    defparam ram.width_b = 32;
    defparam ram.widthad_b = 11;
    defparam ram.numwords_b = 2048;
    defparam ram.address_aclr_a = "NONE";
    defparam ram.address_aclr_b = "NONE";
    defparam ram.address_reg_b = "CLOCK0";
    defparam ram.outdata_reg_b = "UNREGISTERED";
    defparam ram.outdata_aclr_b = "NONE";
    defparam ram.indata_aclr_a = "NONE";
    defparam ram.wrcontrol_aclr_a = "NONE";
    defparam ram.read_during_write_mode_mixed_ports = "DONT_CARE";
`ifdef FPGA_CYCLONE_IV_GX
    defparam ram.intended_device_family = "Cyclone IV GX";
`else
    defparam ram.intended_device_family = "Cyclone IV E";
`endif
    defparam ram.lpm_type = "altsyncram";
`endif
endmodule
/* verilator lint_on DECLFILENAME */
`endif

module r4000_icache (
    input  wire        clk,
    input  wire        rst_n,

    // Core interface
    input  wire [31:0] core_addr,     // physical address
    input  wire        core_valid,    // address belongs to a non-faulting fetch
    // IF-stage address for a synchronous tag/data read.  core_addr remains
    // the identity of the IS-stage response.  When the core cannot advance,
    // read core_addr again so the registered response remains aligned.
    input  wire [31:0] fetch_addr,
    input  wire        fetch_advance,
    input  wire        uncached,      // bypass cache (kseg1)
    output wire [31:0] core_rdata,    // instruction word to core
    output wire        stall,         // pipeline stall

    // CACHE instruction (from WB commit)
    input  wire        cop_valid,
    input  wire [2:0]  cop_op,        // operation [20:18]
    input  wire [31:0] cop_paddr,     // physical address
    input  wire [31:0] cop_taglo,     // TagLo value (for Index Store Tag)

    // Freeze from secondary cache (hold state machine on S-cache miss)
    input  wire        freeze,

    // Back-invalidation from S-cache (maintain inclusion)
    input  wire        backinv_valid,
    input  wire [31:0] backinv_addr,

    // External write update from a bus-master DMA.  The R4000 secondary
    // cache merges this beat, and any matching primary I-cache line must be
    // invalidated so a later fetch observes the updated secondary line.
    input  wire        snoop_valid,
    input  wire [31:0] snoop_addr,

    // Memory-side interface (to S-cache or external memory)
    output wire [31:0] mem_addr,
    output wire        need_fill,     // active when needing data from memory side
    input  wire [31:0] mem_rdata,
    input  wire [31:0] fill_rdata,

    // Writeback interface (for CACHE PI|HWB — Hit Writeback to S-cache)
    output reg         mem_wen,
    output reg  [63:0] mem_wdata,

    // Debug capture for the post-banner if_ecintr instruction-data corruption.
    output wire [127:0] debug_capture,

    // Tag readback/completion for CACHE PI|ILT (Index Load Tag)
    output wire [31:0] cop_taglo_out,
    output wire        cop_taglo_valid
);

    // Address decomposition:
    // [3:0]   byte offset (16B line)
    // [12:4]  index (512 entries)
    // [31:12] physical tag (20 bits, including PA bit 12)
    localparam NLINES = 512;
    localparam TAG_W  = 20;

`ifdef FPGA_ICACHE_BYPASS
    // FPGA diagnostic mode: remove only the primary I-cache from the path.
    // The S-cache still handles cached instruction fetches and asserts freeze
    // until its synchronous data output belongs to core_addr.
`ifdef FPGA_CPU_50MHZ
    reg [31:0] uncached_addr_q;
    reg        uncached_addr_valid_q;
    wire       uncached_addr_ready =
        uncached_addr_valid_q && (uncached_addr_q == core_addr);
    wire       uncached_wait =
        core_valid && uncached && (freeze || !uncached_addr_ready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uncached_addr_q <= 32'd0;
            uncached_addr_valid_q <= 1'b0;
        end else if (!core_valid || !uncached) begin
            uncached_addr_valid_q <= 1'b0;
        end else if (!uncached_addr_ready) begin
            uncached_addr_q <= core_addr;
            uncached_addr_valid_q <= 1'b1;
        end
    end
`else
    wire       uncached_wait = core_valid && uncached && freeze;
`endif

    assign core_rdata = uncached ? (uncached_wait ? 32'd0 : mem_rdata) :
                        (!freeze ? mem_rdata : 32'd0);
    assign stall      = uncached_wait || (core_valid && !uncached && freeze);
    assign mem_addr   = core_addr;
    assign need_fill  = core_valid && !uncached;
    assign debug_capture = 128'd0;
    assign cop_taglo_out = 32'd0;
    assign cop_taglo_valid = 1'b0;

    always @(*) begin
        mem_wen   = 1'b0;
        mem_wdata = 64'd0;
    end
`elsif FPGA_L1_BYPASS
    // FPGA bring-up mode: remove the flop-based L1 I-cache arrays from the
    // netlist. The memory side still goes through S-cache, which supplies the
    // real cached FPGA storage and asserts freeze while it services misses.
`ifdef FPGA_CPU_50MHZ
    reg [31:0] uncached_addr_q;
    reg        uncached_addr_valid_q;
    wire       uncached_addr_ready =
        uncached_addr_valid_q && (uncached_addr_q == core_addr);
    wire       uncached_wait =
        core_valid && uncached && (freeze || !uncached_addr_ready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uncached_addr_q <= 32'd0;
            uncached_addr_valid_q <= 1'b0;
        end else if (!core_valid || !uncached) begin
            uncached_addr_valid_q <= 1'b0;
        end else if (!uncached_addr_ready) begin
            uncached_addr_q <= core_addr;
            uncached_addr_valid_q <= 1'b1;
        end
    end
`else
    wire       uncached_wait = core_valid && uncached && freeze;
`endif

    assign core_rdata = uncached_wait ? 32'd0 : mem_rdata;
    assign stall      = uncached_wait || (core_valid && !uncached && freeze);
    assign mem_addr   = core_addr;
    assign need_fill  = core_valid && !uncached;
    assign debug_capture = 128'd0;
    assign cop_taglo_out = 32'd0;
    assign cop_taglo_valid = 1'b0;

    always @(*) begin
        mem_wen   = 1'b0;
        mem_wdata = 64'd0;
    end
`elsif FPGA_L1_BRAM
    reg [NLINES-1:0] valid;

    wire [8:0]       idx      = core_addr[12:4];
    wire [TAG_W-1:0] addr_tag = core_addr[31:12];
    wire [1:0]       word_sel = core_addr[3:2];
    wire [31:0]      lookup_addr = fetch_advance ? fetch_addr : core_addr;
    wire [8:0]       lookup_idx = lookup_addr[12:4];
    wire [TAG_W-1:0] lookup_tag = lookup_addr[31:12];
    wire [8:0]       backinv_idx0 = {backinv_addr[12:7], 3'b000};
    wire [8:0]       backinv_idx1 = {backinv_addr[12:7], 3'b001};
    wire [8:0]       backinv_idx2 = {backinv_addr[12:7], 3'b010};
    wire [8:0]       backinv_idx3 = {backinv_addr[12:7], 3'b011};
    wire [8:0]       backinv_idx4 = {backinv_addr[12:7], 3'b100};
    wire [8:0]       backinv_idx5 = {backinv_addr[12:7], 3'b101};
    wire [8:0]       backinv_idx6 = {backinv_addr[12:7], 3'b110};
    wire [8:0]       backinv_idx7 = {backinv_addr[12:7], 3'b111};
    wire [TAG_W-1:0] backinv_tag  = backinv_addr[31:12];
    wire [8:0]       snoop_idx     = snoop_addr[12:4];
    wire [TAG_W-1:0] snoop_tag     = snoop_addr[31:12];

    reg        cop_latched;
    reg [2:0]  cop_op_latched;
    reg [31:0] cop_paddr_latched;
    reg [31:0] cop_taglo_latched;

    wire        cop_req_valid = cop_latched || cop_valid;
    wire [2:0]  cop_req_op    = cop_latched ? cop_op_latched    : cop_op;
    wire [31:0] cop_req_paddr = cop_latched ? cop_paddr_latched : cop_paddr;
    wire [31:0] cop_req_taglo = cop_latched ? cop_taglo_latched : cop_taglo;
    wire [8:0]       cop_idx  = cop_req_paddr[12:4];
    wire [TAG_W-1:0] cop_tag  = cop_req_paddr[31:12];

    wire [TAG_W-1:0] core_tag_rdata;
    wire [TAG_W-1:0] cop_tag_rdata;
    wire [TAG_W-1:0] backinv_tag_rdata0;
    wire [TAG_W-1:0] backinv_tag_rdata1;
    wire [TAG_W-1:0] backinv_tag_rdata2;
    wire [TAG_W-1:0] backinv_tag_rdata3;
    wire [TAG_W-1:0] backinv_tag_rdata4;
    wire [TAG_W-1:0] backinv_tag_rdata5;
    wire [TAG_W-1:0] backinv_tag_rdata6;
    wire [TAG_W-1:0] backinv_tag_rdata7;
    wire [TAG_W-1:0] snoop_tag_rdata;

    reg              tag_we;
    reg [8:0]        tag_waddr;
    reg [TAG_W-1:0]  tag_wdata;

    r4000_icache_tag_ram u_tag_core (
        .clk   (clk),
        .raddr (lookup_idx),
        .rdata (core_tag_rdata),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_cop (
        .clk   (clk),
        .raddr (cop_idx),
        .rdata (cop_tag_rdata),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_backinv0 (
        .clk   (clk),
        .raddr (backinv_idx0),
        .rdata (backinv_tag_rdata0),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_backinv1 (
        .clk   (clk),
        .raddr (backinv_idx1),
        .rdata (backinv_tag_rdata1),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_backinv2 (
        .clk   (clk),
        .raddr (backinv_idx2),
        .rdata (backinv_tag_rdata2),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_backinv3 (
        .clk   (clk),
        .raddr (backinv_idx3),
        .rdata (backinv_tag_rdata3),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_backinv4 (
        .clk   (clk),
        .raddr (backinv_idx4),
        .rdata (backinv_tag_rdata4),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_backinv5 (
        .clk   (clk),
        .raddr (backinv_idx5),
        .rdata (backinv_tag_rdata5),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_backinv6 (
        .clk   (clk),
        .raddr (backinv_idx6),
        .rdata (backinv_tag_rdata6),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_icache_tag_ram u_tag_backinv7 (
        .clk   (clk),
        .raddr (backinv_idx7),
        .rdata (backinv_tag_rdata7),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    // A dedicated replica keeps a DMA-beat invalidation independent of
    // the S-cache's separate 128-byte inclusion back-invalidation port.
    r4000_icache_tag_ram u_tag_snoop (
        .clk   (clk),
        .raddr (snoop_idx),
        .rdata (snoop_tag_rdata),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    reg              meta_sample_q;
    reg [8:0]        meta_idx_q;
    reg [TAG_W-1:0]  meta_addr_tag_q;
    reg              meta_line_valid_q;

    reg              cop_meta_sample_q;
    reg [8:0]        cop_meta_idx_q;
    reg [TAG_W-1:0]  cop_meta_addr_tag_q;
    reg              cop_meta_line_valid_q;
    reg              backinv_meta_req_q;
    reg [8:0]        backinv_meta_idx0_q;
    reg [8:0]        backinv_meta_idx1_q;
    reg [8:0]        backinv_meta_idx2_q;
    reg [8:0]        backinv_meta_idx3_q;
    reg [8:0]        backinv_meta_idx4_q;
    reg [8:0]        backinv_meta_idx5_q;
    reg [8:0]        backinv_meta_idx6_q;
    reg [8:0]        backinv_meta_idx7_q;
    reg [TAG_W-1:0]  backinv_meta_addr_tag_q;
    reg              backinv_meta_line_valid0_q;
    reg              backinv_meta_line_valid1_q;
    reg              backinv_meta_line_valid2_q;
    reg              backinv_meta_line_valid3_q;
    reg              backinv_meta_line_valid4_q;
    reg              backinv_meta_line_valid5_q;
    reg              backinv_meta_line_valid6_q;
    reg              backinv_meta_line_valid7_q;
    reg              snoop_meta_req_q;
    reg [8:0]        snoop_meta_idx_q;
    reg [TAG_W-1:0]  snoop_meta_addr_tag_q;
    reg              snoop_meta_line_valid_q;

    wire meta_ready = !uncached && meta_sample_q &&
                      (meta_idx_q == idx) &&
                      (meta_addr_tag_q == addr_tag);
    wire cop_meta_ready = cop_req_valid && cop_meta_sample_q &&
                          (cop_meta_idx_q == cop_idx) &&
                          (cop_meta_addr_tag_q == cop_tag);
    wire cop_meta_wait = cop_req_valid && !cop_meta_ready;

    wire meta_wait = core_valid && !uncached && !meta_ready;
    wire snoop_live_core_match = snoop_valid &&
                                 (snoop_idx == idx) &&
                                 (snoop_tag == addr_tag);
    wire snoop_meta_core_match = snoop_meta_req_q &&
                                 (snoop_meta_idx_q == idx) &&
                                 (snoop_meta_addr_tag_q == addr_tag);
    wire tag_hit = meta_ready && meta_line_valid_q &&
                   (core_tag_rdata == addr_tag) &&
                   !snoop_live_core_match && !snoop_meta_core_match;
    wire miss    = core_valid && !uncached && meta_ready && !tag_hit;
`ifdef FPGA_CPU_50MHZ
    reg [31:0] uncached_addr_q;
    reg        uncached_addr_valid_q;
    wire       uncached_addr_ready =
        uncached_addr_valid_q && (uncached_addr_q == core_addr);
    wire       uncached_wait =
        core_valid && uncached && (freeze || !uncached_addr_ready);
`else
    wire       uncached_wait = core_valid && uncached && freeze;
`endif

    localparam [2:0]
        S_IDLE     = 3'd0,
        S_FILL     = 3'd1,
        S_REFETCH  = 3'd2,
        S_WB_RD0   = 3'd3,
        S_WB_RD1   = 3'd4,
        S_WB_SEND  = 3'd5;

    reg [2:0]  state;
    reg [1:0]  fill_cnt;
    reg [31:0] fill_addr;
    reg        fill_backinv_pending;
    reg        fill_snoop_pending;
    reg        wb_cnt;
    reg [31:0] wb_addr;
    reg [31:0] wb_hi;
    reg [10:0] ram_raddr_q;
    reg        ram_raddr_valid;

    wire [10:0] core_word_addr = core_addr[12:2];
    wire [10:0] lookup_word_addr = lookup_addr[12:2];
    wire [10:0] wb_word0_addr  = {wb_addr[12:4], wb_cnt, 1'b0};
    wire [10:0] wb_word1_addr  = {wb_addr[12:4], wb_cnt, 1'b1};
    wire [10:0] ram_raddr =
        (state == S_WB_RD0)  ? wb_word0_addr :
        (state == S_WB_RD1 ||
         state == S_WB_SEND) ? wb_word1_addr :
                               lookup_word_addr;
    wire [31:0] ram_rdata;
    wire        ram_read_hit = ram_raddr_valid && (ram_raddr_q == core_word_addr);

    wire        ram_we    = (state == S_FILL) && !freeze;
    wire [10:0] ram_waddr = {fill_addr[12:4], fill_cnt};
    wire        fill_backinv_active = (state == S_FILL) || (state == S_REFETCH);
    wire        fill_backinv_hit0 = fill_backinv_active &&
                                    (fill_addr[12:4] == backinv_meta_idx0_q) &&
                                    (fill_addr[31:12] == backinv_meta_addr_tag_q);
    wire        fill_backinv_hit1 = fill_backinv_active &&
                                    (fill_addr[12:4] == backinv_meta_idx1_q) &&
                                    (fill_addr[31:12] == backinv_meta_addr_tag_q);
    wire        fill_backinv_hit2 = fill_backinv_active &&
                                    (fill_addr[12:4] == backinv_meta_idx2_q) &&
                                    (fill_addr[31:12] == backinv_meta_addr_tag_q);
    wire        fill_backinv_hit3 = fill_backinv_active &&
                                    (fill_addr[12:4] == backinv_meta_idx3_q) &&
                                    (fill_addr[31:12] == backinv_meta_addr_tag_q);
    wire        fill_backinv_hit4 = fill_backinv_active &&
                                    (fill_addr[12:4] == backinv_meta_idx4_q) &&
                                    (fill_addr[31:12] == backinv_meta_addr_tag_q);
    wire        fill_backinv_hit5 = fill_backinv_active &&
                                    (fill_addr[12:4] == backinv_meta_idx5_q) &&
                                    (fill_addr[31:12] == backinv_meta_addr_tag_q);
    wire        fill_backinv_hit6 = fill_backinv_active &&
                                    (fill_addr[12:4] == backinv_meta_idx6_q) &&
                                    (fill_addr[31:12] == backinv_meta_addr_tag_q);
    wire        fill_backinv_hit7 = fill_backinv_active &&
                                    (fill_addr[12:4] == backinv_meta_idx7_q) &&
                                    (fill_addr[31:12] == backinv_meta_addr_tag_q);
    wire        fill_backinv_hit = backinv_meta_req_q &&
                                   (fill_backinv_hit0 || fill_backinv_hit1 ||
                                    fill_backinv_hit2 || fill_backinv_hit3 ||
                                    fill_backinv_hit4 || fill_backinv_hit5 ||
                                    fill_backinv_hit6 || fill_backinv_hit7);
    wire        fill_snoop_active = (state == S_FILL) || (state == S_REFETCH);
    wire        fill_snoop_hit = snoop_meta_req_q && fill_snoop_active &&
                                 (fill_addr[12:4] == snoop_meta_idx_q) &&
                                 (fill_addr[31:12] == snoop_meta_addr_tag_q);

    localparam [31:0] DEBUG_LOWVEC_WATCH_ADDR = 32'h0000_0184;
    localparam [31:0] DEBUG_LOWVEC_WATCH_WORD = 32'h8D08_9CE0;
    localparam [31:0] DEBUG_LEGACY_WATCH_ADDR = 32'h0809_D200;
    localparam [31:0] DEBUG_LEGACY_WATCH_WORD = 32'h000F_C900;

    reg [31:0] debug_word0;
    reg [31:0] debug_word1;
    reg [31:0] debug_word2;
    reg [31:0] debug_word3;
    reg        debug_bad_seen;

    wire [31:0] debug_fill_paddr = {fill_addr[31:4], fill_cnt, 2'b00};
    wire debug_legacy_watch_fill = ram_we &&
                                   (fill_addr[31:4] == DEBUG_LEGACY_WATCH_ADDR[31:4]);
    wire debug_legacy_watch_word = debug_legacy_watch_fill &&
                                   (fill_cnt == DEBUG_LEGACY_WATCH_ADDR[3:2]);
    wire debug_legacy_watch_bad = debug_legacy_watch_word &&
                                  (fill_rdata != DEBUG_LEGACY_WATCH_WORD);
    wire debug_lowvec_watch_fill = ram_we &&
                                   (fill_addr[31:4] == DEBUG_LOWVEC_WATCH_ADDR[31:4]);
    wire debug_lowvec_watch_word = debug_lowvec_watch_fill &&
                                   (fill_cnt == DEBUG_LOWVEC_WATCH_ADDR[3:2]);
    wire debug_lowvec_watch_bad = debug_lowvec_watch_word &&
                                  (fill_rdata != DEBUG_LOWVEC_WATCH_WORD);
    wire debug_watch_bad = debug_lowvec_watch_bad ||
                           debug_legacy_watch_bad;
    wire [31:0] debug_flags = {
        debug_lowvec_watch_word ? 8'h4B : 8'h49, // "K" low vector / "I" legacy
        state,
        fill_cnt,
        freeze,
        uncached,
        tag_hit,
        miss,
        meta_ready,
        ram_read_hit,
        ram_we,
        debug_watch_bad,
        11'd0
    };
    assign debug_capture = {debug_word0, debug_word1, debug_word2, debug_word3};

    r4000_icache_ram32 u_data (
        .clk   (clk),
        .raddr (ram_raddr),
        .rdata (ram_rdata),
        .we    (ram_we),
        .waddr (ram_waddr),
        .wdata (fill_rdata)
    );

    // Cross into S-cache only after the miss edge has captured fill_addr.
    // The core still stalls combinationally on miss, while S_FILL provides a
    // registered request/address boundary for the secondary transaction.
    assign need_fill = (state == S_FILL);
    assign core_rdata = uncached ? (uncached_wait ? 32'd0 : mem_rdata) :
                        (tag_hit && ram_read_hit) ? ram_rdata :
                        32'd0;
    assign stall = uncached_wait ||
                   (state != S_IDLE) || meta_wait || miss ||
                   cop_meta_wait ||
                   (!uncached && tag_hit && !ram_read_hit);
    assign mem_addr = (state == S_FILL)    ? {fill_addr[31:4], fill_cnt, 2'b00} :
                      (state == S_WB_SEND) ? {wb_addr[31:4], wb_cnt, 3'b000} :
                                             core_addr;

    wire [1:0] cop_ic_pstate = (cop_meta_ready && cop_meta_line_valid_q) ? 2'b10 : 2'b00;
    assign cop_taglo_out = {4'b0, cop_tag_rdata, cop_ic_pstate, 6'b0};

    wire cop_process = (state == S_IDLE) && cop_req_valid && cop_meta_ready;
    assign cop_taglo_valid = cop_process && (cop_req_op == 3'd1);

    always @(*) begin
        tag_we    = 1'b0;
        tag_waddr = 9'd0;
        tag_wdata = {TAG_W{1'b0}};

        if (rst_n) begin
            if (state == S_FILL && !freeze && fill_cnt == 2'd3) begin
                tag_we    = 1'b1;
                tag_waddr = fill_addr[12:4];
                tag_wdata = fill_addr[31:12];
            end else if (cop_process && cop_req_op == 3'd2) begin
                tag_we    = 1'b1;
                tag_waddr = cop_idx;
                tag_wdata = cop_req_taglo[27:8];
            end
        end
    end

    always @(*) begin
        if (state == S_WB_SEND) begin
            mem_wen   = 1'b1;
            mem_wdata = {wb_hi, ram_rdata};
        end else begin
            mem_wen   = 1'b0;
            mem_wdata = 64'd0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            fill_cnt <= 2'd0;
            fill_backinv_pending <= 1'b0;
            fill_snoop_pending <= 1'b0;
            wb_cnt <= 1'b0;
            wb_hi <= 32'd0;
            valid <= {NLINES{1'b0}};
            cop_latched <= 1'b0;
            cop_op_latched <= 3'd0;
            cop_paddr_latched <= 32'd0;
            cop_taglo_latched <= 32'd0;
            ram_raddr_q <= 11'd0;
            ram_raddr_valid <= 1'b0;
            meta_sample_q <= 1'b0;
            meta_idx_q <= 9'd0;
            meta_addr_tag_q <= {TAG_W{1'b0}};
            meta_line_valid_q <= 1'b0;
            cop_meta_sample_q <= 1'b0;
            cop_meta_idx_q <= 9'd0;
            cop_meta_addr_tag_q <= {TAG_W{1'b0}};
            cop_meta_line_valid_q <= 1'b0;
            backinv_meta_req_q <= 1'b0;
            backinv_meta_idx0_q <= 9'd0;
            backinv_meta_idx1_q <= 9'd0;
            backinv_meta_idx2_q <= 9'd0;
            backinv_meta_idx3_q <= 9'd0;
            backinv_meta_idx4_q <= 9'd0;
            backinv_meta_idx5_q <= 9'd0;
            backinv_meta_idx6_q <= 9'd0;
            backinv_meta_idx7_q <= 9'd0;
            backinv_meta_addr_tag_q <= {TAG_W{1'b0}};
            backinv_meta_line_valid0_q <= 1'b0;
            backinv_meta_line_valid1_q <= 1'b0;
            backinv_meta_line_valid2_q <= 1'b0;
            backinv_meta_line_valid3_q <= 1'b0;
            backinv_meta_line_valid4_q <= 1'b0;
            backinv_meta_line_valid5_q <= 1'b0;
            backinv_meta_line_valid6_q <= 1'b0;
            backinv_meta_line_valid7_q <= 1'b0;
            snoop_meta_req_q <= 1'b0;
            snoop_meta_idx_q <= 9'd0;
            snoop_meta_addr_tag_q <= {TAG_W{1'b0}};
            snoop_meta_line_valid_q <= 1'b0;
            debug_word0 <= 32'd0;
            debug_word1 <= 32'd0;
            debug_word2 <= 32'd0;
            debug_word3 <= 32'd0;
            debug_bad_seen <= 1'b0;
`ifdef FPGA_CPU_50MHZ
            uncached_addr_q <= 32'd0;
            uncached_addr_valid_q <= 1'b0;
`endif
        end else begin
`ifdef FPGA_CPU_50MHZ
            if (!core_valid || !uncached) begin
                uncached_addr_valid_q <= 1'b0;
            end else if (!uncached_addr_ready) begin
                uncached_addr_q <= core_addr;
                uncached_addr_valid_q <= 1'b1;
            end
`endif
            ram_raddr_q <= ram_raddr;
            ram_raddr_valid <= 1'b1;
            meta_sample_q <= 1'b1;
            meta_idx_q <= lookup_idx;
            meta_addr_tag_q <= lookup_tag;
            meta_line_valid_q <= valid[lookup_idx];
            cop_meta_sample_q <= 1'b1;
            cop_meta_idx_q <= cop_idx;
            cop_meta_addr_tag_q <= cop_tag;
            cop_meta_line_valid_q <= valid[cop_idx];
            backinv_meta_req_q <= backinv_valid;
            backinv_meta_idx0_q <= backinv_idx0;
            backinv_meta_idx1_q <= backinv_idx1;
            backinv_meta_idx2_q <= backinv_idx2;
            backinv_meta_idx3_q <= backinv_idx3;
            backinv_meta_idx4_q <= backinv_idx4;
            backinv_meta_idx5_q <= backinv_idx5;
            backinv_meta_idx6_q <= backinv_idx6;
            backinv_meta_idx7_q <= backinv_idx7;
            backinv_meta_addr_tag_q <= backinv_tag;
            backinv_meta_line_valid0_q <= valid[backinv_idx0];
            backinv_meta_line_valid1_q <= valid[backinv_idx1];
            backinv_meta_line_valid2_q <= valid[backinv_idx2];
            backinv_meta_line_valid3_q <= valid[backinv_idx3];
            backinv_meta_line_valid4_q <= valid[backinv_idx4];
            backinv_meta_line_valid5_q <= valid[backinv_idx5];
            backinv_meta_line_valid6_q <= valid[backinv_idx6];
            backinv_meta_line_valid7_q <= valid[backinv_idx7];
            snoop_meta_req_q <= snoop_valid;
            snoop_meta_idx_q <= snoop_idx;
            snoop_meta_addr_tag_q <= snoop_tag;
            snoop_meta_line_valid_q <= valid[snoop_idx];

            if (cop_process) begin
                cop_latched <= 1'b0;
            end else if (cop_valid && !cop_latched) begin
                cop_latched <= 1'b1;
                cop_op_latched <= cop_op;
                cop_paddr_latched <= cop_paddr;
                cop_taglo_latched <= cop_taglo;
            end

            if (debug_watch_bad ||
                (!debug_bad_seen && debug_lowvec_watch_word)) begin
                debug_word0 <= debug_flags;
                debug_word1 <= fill_addr;
                debug_word2 <= debug_fill_paddr;
                debug_word3 <= fill_rdata;
                if (debug_watch_bad)
                    debug_bad_seen <= 1'b1;
            end

            case (state)
                S_IDLE: begin
                    if (miss && !freeze) begin
                        state <= S_FILL;
                        fill_cnt <= 2'd0;
                        fill_addr <= core_addr;
                        fill_backinv_pending <= 1'b0;
                        fill_snoop_pending <= 1'b0;
                    end

                    if (cop_process) begin
                        case (cop_req_op)
                            3'd0:
                                valid[cop_idx] <= 1'b0;
                            3'd2: begin
                                valid[cop_idx] <= (cop_req_taglo[7:6] != 2'b00);
                            end
                            3'd4: begin
                                if (cop_meta_line_valid_q && cop_tag_rdata == cop_tag)
                                    valid[cop_idx] <= 1'b0;
                            end
                            3'd5: begin
                                if (!freeze) begin
                                    state <= S_FILL;
                                    fill_cnt <= 2'd0;
                                    fill_addr <= cop_req_paddr;
                                    fill_backinv_pending <= 1'b0;
                                    fill_snoop_pending <= 1'b0;
                                end
                            end
                            3'd6: begin
                                if (cop_meta_line_valid_q && cop_tag_rdata == cop_tag && !freeze) begin
                                    wb_addr <= cop_req_paddr;
                                    wb_cnt  <= 1'b0;
                                    state   <= S_WB_RD0;
                                end
                            end
                            default: ;
                        endcase
                    end
                end

                S_FILL: if (!freeze) begin
                    if (fill_cnt == 2'd3) begin
                        valid[fill_addr[12:4]] <= !(fill_backinv_pending || fill_backinv_hit ||
                                                   fill_snoop_pending || fill_snoop_hit);
                        state <= S_REFETCH;
                    end
                    fill_cnt <= fill_cnt + 2'd1;
                end

                S_REFETCH: begin
                    state <= S_IDLE;
                end

                S_WB_RD0: begin
                    state <= S_WB_RD1;
                end

                S_WB_RD1: begin
                    wb_hi <= ram_rdata;
                    state <= S_WB_SEND;
                end

                S_WB_SEND: if (!freeze) begin
                    if (wb_cnt == 1'b1) begin
                        state <= S_IDLE;
                    end else begin
                        wb_cnt <= wb_cnt + 1'b1;
                        state <= S_WB_RD0;
                    end
                end

                default: state <= S_IDLE;
            endcase

            if (backinv_meta_req_q) begin
                if (fill_backinv_hit) begin
                    fill_backinv_pending <= 1'b1;
                    if (state == S_REFETCH)
                        valid[fill_addr[12:4]] <= 1'b0;
                end
                if (backinv_meta_line_valid0_q &&
                    backinv_tag_rdata0 == backinv_meta_addr_tag_q &&
                    !fill_backinv_hit0) begin
                    valid[backinv_meta_idx0_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid1_q &&
                    backinv_tag_rdata1 == backinv_meta_addr_tag_q &&
                    !fill_backinv_hit1) begin
                    valid[backinv_meta_idx1_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid2_q &&
                    backinv_tag_rdata2 == backinv_meta_addr_tag_q &&
                    !fill_backinv_hit2) begin
                    valid[backinv_meta_idx2_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid3_q &&
                    backinv_tag_rdata3 == backinv_meta_addr_tag_q &&
                    !fill_backinv_hit3) begin
                    valid[backinv_meta_idx3_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid4_q &&
                    backinv_tag_rdata4 == backinv_meta_addr_tag_q &&
                    !fill_backinv_hit4) begin
                    valid[backinv_meta_idx4_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid5_q &&
                    backinv_tag_rdata5 == backinv_meta_addr_tag_q &&
                    !fill_backinv_hit5) begin
                    valid[backinv_meta_idx5_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid6_q &&
                    backinv_tag_rdata6 == backinv_meta_addr_tag_q &&
                    !fill_backinv_hit6) begin
                    valid[backinv_meta_idx6_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid7_q &&
                    backinv_tag_rdata7 == backinv_meta_addr_tag_q &&
                    !fill_backinv_hit7) begin
                    valid[backinv_meta_idx7_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
            end

            if (snoop_meta_req_q) begin
                if (fill_snoop_hit) begin
                    fill_snoop_pending <= 1'b1;
                    if (state == S_REFETCH)
                        valid[fill_addr[12:4]] <= 1'b0;
                end
                if (snoop_meta_line_valid_q &&
                    snoop_tag_rdata == snoop_meta_addr_tag_q &&
                    !fill_snoop_hit) begin
                    valid[snoop_meta_idx_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
            end
        end
    end
`else
    // Packed (was `reg valid [0:NLINES-1]`) — Verilator 5.x rejects delayed
    // assignment to unpacked arrays inside for-loops, and packing lets reset
    // be a single bulk assign.  Quartus infers the same flops either way.
    reg [NLINES-1:0] valid;
    reg [TAG_W-1:0]  tags  [0:NLINES-1];
    // Data stored as 4 words per line (big-endian: word 0 at lowest address)
    reg [31:0] data0 [0:NLINES-1];
    reg [31:0] data1 [0:NLINES-1];
    reg [31:0] data2 [0:NLINES-1];
    reg [31:0] data3 [0:NLINES-1];

    wire [8:0]       idx      = core_addr[12:4];
    wire [TAG_W-1:0] addr_tag = core_addr[31:12];
    wire [1:0]       word_sel = core_addr[3:2];

    wire snoop_core_match = snoop_valid &&
                            (snoop_addr[12:4] == idx) &&
                            (snoop_addr[31:12] == addr_tag);
    wire tag_hit = valid[idx] && (tags[idx] == addr_tag) && !snoop_core_match;
    wire miss    = core_valid && !uncached && !tag_hit;

    // need_fill: indicates this cache needs data from the memory side
    // Suppress during S_WB (writeback uses mem_addr for write, not fill)
    // Cross into S-cache only after the miss edge has captured fill_addr.
    // The core still stalls combinationally on miss, while S_FILL provides a
    // registered request/address boundary for the secondary transaction.
    assign need_fill = (state == S_FILL);

    // State machine
    localparam [1:0] S_IDLE = 2'd0, S_FILL = 2'd1, S_DONE = 2'd2, S_WB = 2'd3;
    reg [1:0] state;
    reg [1:0] fill_cnt;
    reg [31:0] fill_addr;
    reg [31:0] fill_w [0:3];
    reg        fill_snoop_pending;

    // Writeback registers (for CACHE PI|HWB)
    reg        wb_cnt;
    reg [31:0] wb_addr;

    // Select word from cached line
    reg [31:0] cached_word;
    always @(*) begin
        case (word_sel)
            2'd0: cached_word = data0[idx];
            2'd1: cached_word = data1[idx];
            2'd2: cached_word = data2[idx];
            2'd3: cached_word = data3[idx];
        endcase
    end

    // Tag readback: PState (00=invalid, 10=valid)
    wire [1:0] cop_ic_pstate = valid[cop_paddr[12:4]] ? 2'b10 : 2'b00;
    assign cop_taglo_out =
        {4'b0, tags[cop_paddr[12:4]], cop_ic_pstate, 6'b0};
    assign cop_taglo_valid = cop_valid && (cop_op == 3'd1);

    // Doubleword read for writeback (pairs of 32-bit words, big-endian)
    wire [8:0] wb_idx = wb_addr[12:4];
    reg [63:0] wb_dw;
    always @(*) begin
        case (wb_cnt)
            1'b0: wb_dw = {data0[wb_idx], data1[wb_idx]};
            1'b1: wb_dw = {data2[wb_idx], data3[wb_idx]};
        endcase
    end

    // Outputs
    assign core_rdata = (!uncached && tag_hit) ? cached_word : mem_rdata;
    assign stall      = (state != S_IDLE) || miss;
    assign mem_addr   = (state == S_FILL) ? {fill_addr[31:4], fill_cnt, 2'b00} :
                        (state == S_WB)   ? {wb_addr[31:4], wb_cnt, 3'b000} :
                        core_addr;
    assign debug_capture = 128'd0;

    // Writeback write interface
    always @(*) begin
        if (state == S_WB) begin
            mem_wen   = 1'b1;
            mem_wdata = wb_dw;
        end else begin
            mem_wen   = 1'b0;
            mem_wdata = 64'd0;
        end
    end

    // Pipelined backinv: register inputs, process next cycle
    reg        backinv_req_q;
    reg [5:0]  backinv_idx_hi_q;  // backinv_addr[12:7]
    reg [TAG_W-1:0] backinv_tag_q;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            fill_cnt <= 2'd0;
            valid <= {NLINES{1'b0}};
            fill_snoop_pending <= 1'b0;
            backinv_req_q <= 1'b0;
            backinv_idx_hi_q <= 6'd0;
            backinv_tag_q <= {TAG_W{1'b0}};
        end else begin
            backinv_req_q <= backinv_valid;
            backinv_idx_hi_q <= backinv_addr[12:7];
            backinv_tag_q <= backinv_addr[31:12];

            case (state)
                S_IDLE: begin
                    if (miss && !freeze) begin
                        state <= S_FILL;
                        fill_cnt <= 2'd0;
                        fill_addr <= core_addr;
                        fill_snoop_pending <= 1'b0;
                    end
                    // CACHE operations
                    if (cop_valid) begin
                        case (cop_op)
                            3'd0: // Index Invalidate
                                valid[cop_paddr[12:4]] <= 1'b0;
                            3'd2: begin // Index Store Tag (write TagLo to cache)
                                // TagLo: [27:8]=PTag, [7:6]=PState (00=inv)
                                tags[cop_paddr[12:4]]  <= cop_taglo[27:8];
                                valid[cop_paddr[12:4]] <= (cop_taglo[7:6] != 2'b00);
                            end
                            3'd4: begin // Hit Invalidate
                                if (valid[cop_paddr[12:4]] &&
                                    tags[cop_paddr[12:4]] == cop_paddr[31:12])
                                    valid[cop_paddr[12:4]] <= 1'b0;
                            end
                            3'd5: begin // Fill — force I-cache line fill from S-cache/RAM
                                if (!freeze) begin
                                    state <= S_FILL;
                                    fill_cnt <= 2'd0;
                                    fill_addr <= cop_paddr;
                                    fill_snoop_pending <= 1'b0;
                                end
                            end
                            3'd6: begin // Hit Writeback — write I-cache line to S-cache
                                if (valid[cop_paddr[12:4]] &&
                                    tags[cop_paddr[12:4]] == cop_paddr[31:12]) begin
                                    if (!freeze) begin
                                        wb_addr <= cop_paddr;
                                        wb_cnt  <= 1'b0;
                                        state   <= S_WB;
                                    end
                                end
                            end
                            default: ;
                        endcase
                    end
                end

                S_FILL: if (!freeze) begin
                    fill_w[fill_cnt] <= fill_rdata;
                    if (fill_cnt == 2'd3)
                        state <= S_DONE;
                    fill_cnt <= fill_cnt + 2'd1;
                end

                S_WB: if (!freeze) begin
                    if (wb_cnt == 1'b1)
                        state <= S_IDLE;
                    wb_cnt <= wb_cnt + 1'b1;
                end

                S_DONE: begin
                    // Commit filled line
                    data0[fill_addr[12:4]] <= fill_w[0];
                    data1[fill_addr[12:4]] <= fill_w[1];
                    data2[fill_addr[12:4]] <= fill_w[2];
                    data3[fill_addr[12:4]] <= fill_w[3];
                    tags[fill_addr[12:4]]  <= fill_addr[31:12];
                    valid[fill_addr[12:4]] <= !(fill_snoop_pending ||
                                               (snoop_valid &&
                                                snoop_addr[12:4] == fill_addr[12:4] &&
                                                snoop_addr[31:12] == fill_addr[31:12]));
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            // S-cache back-invalidation (pipelined: sample on cycle N, process on N+1).
            // Suppress for the line currently being filled to prevent livelock.
            if (backinv_req_q) begin
                if (valid[{backinv_idx_hi_q, 3'b000}] &&
                    tags[{backinv_idx_hi_q, 3'b000}] == backinv_tag_q &&
                    !(state != S_IDLE && fill_addr[12:4] == {backinv_idx_hi_q, 3'b000}))
                    valid[{backinv_idx_hi_q, 3'b000}] <= 1'b0;
                if (valid[{backinv_idx_hi_q, 3'b001}] &&
                    tags[{backinv_idx_hi_q, 3'b001}] == backinv_tag_q &&
                    !(state != S_IDLE && fill_addr[12:4] == {backinv_idx_hi_q, 3'b001}))
                    valid[{backinv_idx_hi_q, 3'b001}] <= 1'b0;
                if (valid[{backinv_idx_hi_q, 3'b010}] &&
                    tags[{backinv_idx_hi_q, 3'b010}] == backinv_tag_q &&
                    !(state != S_IDLE && fill_addr[12:4] == {backinv_idx_hi_q, 3'b010}))
                    valid[{backinv_idx_hi_q, 3'b010}] <= 1'b0;
                if (valid[{backinv_idx_hi_q, 3'b011}] &&
                    tags[{backinv_idx_hi_q, 3'b011}] == backinv_tag_q &&
                    !(state != S_IDLE && fill_addr[12:4] == {backinv_idx_hi_q, 3'b011}))
                    valid[{backinv_idx_hi_q, 3'b011}] <= 1'b0;
                if (valid[{backinv_idx_hi_q, 3'b100}] &&
                    tags[{backinv_idx_hi_q, 3'b100}] == backinv_tag_q &&
                    !(state != S_IDLE && fill_addr[12:4] == {backinv_idx_hi_q, 3'b100}))
                    valid[{backinv_idx_hi_q, 3'b100}] <= 1'b0;
                if (valid[{backinv_idx_hi_q, 3'b101}] &&
                    tags[{backinv_idx_hi_q, 3'b101}] == backinv_tag_q &&
                    !(state != S_IDLE && fill_addr[12:4] == {backinv_idx_hi_q, 3'b101}))
                    valid[{backinv_idx_hi_q, 3'b101}] <= 1'b0;
                if (valid[{backinv_idx_hi_q, 3'b110}] &&
                    tags[{backinv_idx_hi_q, 3'b110}] == backinv_tag_q &&
                    !(state != S_IDLE && fill_addr[12:4] == {backinv_idx_hi_q, 3'b110}))
                    valid[{backinv_idx_hi_q, 3'b110}] <= 1'b0;
                if (valid[{backinv_idx_hi_q, 3'b111}] &&
                    tags[{backinv_idx_hi_q, 3'b111}] == backinv_tag_q &&
                    !(state != S_IDLE && fill_addr[12:4] == {backinv_idx_hi_q, 3'b111}))
                    valid[{backinv_idx_hi_q, 3'b111}] <= 1'b0;
            end


            // Each accepted external beat invalidates its matching 16-byte
            // primary line. Multi-beat DMA updates naturally cover both lines
            // of an updated 32-byte region.
            // Remember a collision with an in-flight fill so S_DONE cannot
            // resurrect stale instructions after the snoop has passed.
            if (snoop_valid) begin
                if (state != S_IDLE &&
                    fill_addr[12:4] == snoop_addr[12:4] &&
                    fill_addr[31:12] == snoop_addr[31:12]) begin
                    fill_snoop_pending <= 1'b1;
                end else if (valid[snoop_addr[12:4]] &&
                             tags[snoop_addr[12:4]] == snoop_addr[31:12]) begin
                    valid[snoop_addr[12:4]] <= 1'b0;
                end
            end
        end
    end

`endif
endmodule
