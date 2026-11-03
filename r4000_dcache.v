// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_dcache.v — 8KB direct-mapped data cache
// 256 lines × 32 bytes/line, 64-bit data path
// Write-through to S-cache on store hits, write-allocate on misses
// Fill: 4 cycles (4 × 64-bit doubleword reads)
// Writeback: 4 cycles (4 × 64-bit doubleword writes on eviction)

`default_nettype none

`ifdef FPGA_DCACHE_BRAM
/* verilator lint_off DECLFILENAME */
module r4000_dcache_tag_ram (
    input  wire        clk,
    input  wire [7:0]  raddr,
    output wire [18:0] rdata,
    input  wire        we,
    input  wire [7:0]  waddr,
    input  wire [18:0] wdata
);
`ifdef VERILATOR
    reg [18:0] mem [0:255];
    reg [18:0] rdata_q;
    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we)
            mem[waddr] <= wdata;
    end
`elsif FPGA_XILINX
    (* ram_style = "block" *) reg [18:0] mem [0:255];
    reg [18:0] rdata_q;
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
    defparam ram.width_a = 19;
    defparam ram.widthad_a = 8;
    defparam ram.numwords_a = 256;
    defparam ram.width_b = 19;
    defparam ram.widthad_b = 8;
    defparam ram.numwords_b = 256;
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

module r4000_dcache_ram64 (
    input  wire        clk,
    input  wire [9:0]  raddr,
    output wire [63:0] rdata,
    input  wire        we,
    input  wire [9:0]  waddr,
    input  wire [63:0] wdata,
    input  wire [7:0]  byteena
);
`ifdef VERILATOR
    reg [63:0] mem [0:1023];
    reg [63:0] rdata_q;
    integer b;

    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we) begin
            for (b = 0; b < 8; b = b + 1) begin
                if (byteena[b])
                    mem[waddr][b*8 +: 8] <= wdata[b*8 +: 8];
            end
        end
    end
`elsif FPGA_XILINX
    (* ram_style = "block" *) reg [63:0] mem [0:1023];
    reg [63:0] rdata_q;
    integer b;

    assign rdata = rdata_q;

    always @(posedge clk) begin
        rdata_q <= mem[raddr];
        if (we) begin
            for (b = 0; b < 8; b = b + 1) begin
                if (byteena[b])
                    mem[waddr][b*8 +: 8] <= wdata[b*8 +: 8];
            end
        end
    end
`else
    wire [7:0] q0, q1, q2, q3, q4, q5, q6, q7;

    assign rdata = {q7, q6, q5, q4, q3, q2, q1, q0};

    altsyncram ram0 (
        .clock0     (clk),
        .address_a  (waddr),
        .data_a     (wdata[7:0]),
        .wren_a     (we && byteena[0]),
        .address_b  (raddr),
        .q_b        (q0)
    );
    altsyncram ram1 (
        .clock0     (clk),
        .address_a  (waddr),
        .data_a     (wdata[15:8]),
        .wren_a     (we && byteena[1]),
        .address_b  (raddr),
        .q_b        (q1)
    );
    altsyncram ram2 (
        .clock0     (clk),
        .address_a  (waddr),
        .data_a     (wdata[23:16]),
        .wren_a     (we && byteena[2]),
        .address_b  (raddr),
        .q_b        (q2)
    );
    altsyncram ram3 (
        .clock0     (clk),
        .address_a  (waddr),
        .data_a     (wdata[31:24]),
        .wren_a     (we && byteena[3]),
        .address_b  (raddr),
        .q_b        (q3)
    );
    altsyncram ram4 (
        .clock0     (clk),
        .address_a  (waddr),
        .data_a     (wdata[39:32]),
        .wren_a     (we && byteena[4]),
        .address_b  (raddr),
        .q_b        (q4)
    );
    altsyncram ram5 (
        .clock0     (clk),
        .address_a  (waddr),
        .data_a     (wdata[47:40]),
        .wren_a     (we && byteena[5]),
        .address_b  (raddr),
        .q_b        (q5)
    );
    altsyncram ram6 (
        .clock0     (clk),
        .address_a  (waddr),
        .data_a     (wdata[55:48]),
        .wren_a     (we && byteena[6]),
        .address_b  (raddr),
        .q_b        (q6)
    );
    altsyncram ram7 (
        .clock0     (clk),
        .address_a  (waddr),
        .data_a     (wdata[63:56]),
        .wren_a     (we && byteena[7]),
        .address_b  (raddr),
        .q_b        (q7)
    );

    defparam ram0.operation_mode = "DUAL_PORT";
    defparam ram0.width_a = 8;
    defparam ram0.widthad_a = 10;
    defparam ram0.numwords_a = 1024;
    defparam ram0.width_b = 8;
    defparam ram0.widthad_b = 10;
    defparam ram0.numwords_b = 1024;
    defparam ram0.address_aclr_a = "NONE";
    defparam ram0.address_aclr_b = "NONE";
    defparam ram0.address_reg_b = "CLOCK0";
    defparam ram0.outdata_reg_b = "UNREGISTERED";
    defparam ram0.outdata_aclr_b = "NONE";
    defparam ram0.indata_aclr_a = "NONE";
    defparam ram0.wrcontrol_aclr_a = "NONE";
    defparam ram0.read_during_write_mode_mixed_ports = "DONT_CARE";
    defparam ram1.operation_mode = "DUAL_PORT";
    defparam ram1.width_a = 8;
    defparam ram1.widthad_a = 10;
    defparam ram1.numwords_a = 1024;
    defparam ram1.width_b = 8;
    defparam ram1.widthad_b = 10;
    defparam ram1.numwords_b = 1024;
    defparam ram1.address_aclr_a = "NONE";
    defparam ram1.address_aclr_b = "NONE";
    defparam ram1.address_reg_b = "CLOCK0";
    defparam ram1.outdata_reg_b = "UNREGISTERED";
    defparam ram1.outdata_aclr_b = "NONE";
    defparam ram1.indata_aclr_a = "NONE";
    defparam ram1.wrcontrol_aclr_a = "NONE";
    defparam ram1.read_during_write_mode_mixed_ports = "DONT_CARE";
    defparam ram2.operation_mode = "DUAL_PORT";
    defparam ram2.width_a = 8;
    defparam ram2.widthad_a = 10;
    defparam ram2.numwords_a = 1024;
    defparam ram2.width_b = 8;
    defparam ram2.widthad_b = 10;
    defparam ram2.numwords_b = 1024;
    defparam ram2.address_aclr_a = "NONE";
    defparam ram2.address_aclr_b = "NONE";
    defparam ram2.address_reg_b = "CLOCK0";
    defparam ram2.outdata_reg_b = "UNREGISTERED";
    defparam ram2.outdata_aclr_b = "NONE";
    defparam ram2.indata_aclr_a = "NONE";
    defparam ram2.wrcontrol_aclr_a = "NONE";
    defparam ram2.read_during_write_mode_mixed_ports = "DONT_CARE";
    defparam ram3.operation_mode = "DUAL_PORT";
    defparam ram3.width_a = 8;
    defparam ram3.widthad_a = 10;
    defparam ram3.numwords_a = 1024;
    defparam ram3.width_b = 8;
    defparam ram3.widthad_b = 10;
    defparam ram3.numwords_b = 1024;
    defparam ram3.address_aclr_a = "NONE";
    defparam ram3.address_aclr_b = "NONE";
    defparam ram3.address_reg_b = "CLOCK0";
    defparam ram3.outdata_reg_b = "UNREGISTERED";
    defparam ram3.outdata_aclr_b = "NONE";
    defparam ram3.indata_aclr_a = "NONE";
    defparam ram3.wrcontrol_aclr_a = "NONE";
    defparam ram3.read_during_write_mode_mixed_ports = "DONT_CARE";
    defparam ram4.operation_mode = "DUAL_PORT";
    defparam ram4.width_a = 8;
    defparam ram4.widthad_a = 10;
    defparam ram4.numwords_a = 1024;
    defparam ram4.width_b = 8;
    defparam ram4.widthad_b = 10;
    defparam ram4.numwords_b = 1024;
    defparam ram4.address_aclr_a = "NONE";
    defparam ram4.address_aclr_b = "NONE";
    defparam ram4.address_reg_b = "CLOCK0";
    defparam ram4.outdata_reg_b = "UNREGISTERED";
    defparam ram4.outdata_aclr_b = "NONE";
    defparam ram4.indata_aclr_a = "NONE";
    defparam ram4.wrcontrol_aclr_a = "NONE";
    defparam ram4.read_during_write_mode_mixed_ports = "DONT_CARE";
    defparam ram5.operation_mode = "DUAL_PORT";
    defparam ram5.width_a = 8;
    defparam ram5.widthad_a = 10;
    defparam ram5.numwords_a = 1024;
    defparam ram5.width_b = 8;
    defparam ram5.widthad_b = 10;
    defparam ram5.numwords_b = 1024;
    defparam ram5.address_aclr_a = "NONE";
    defparam ram5.address_aclr_b = "NONE";
    defparam ram5.address_reg_b = "CLOCK0";
    defparam ram5.outdata_reg_b = "UNREGISTERED";
    defparam ram5.outdata_aclr_b = "NONE";
    defparam ram5.indata_aclr_a = "NONE";
    defparam ram5.wrcontrol_aclr_a = "NONE";
    defparam ram5.read_during_write_mode_mixed_ports = "DONT_CARE";
    defparam ram6.operation_mode = "DUAL_PORT";
    defparam ram6.width_a = 8;
    defparam ram6.widthad_a = 10;
    defparam ram6.numwords_a = 1024;
    defparam ram6.width_b = 8;
    defparam ram6.widthad_b = 10;
    defparam ram6.numwords_b = 1024;
    defparam ram6.address_aclr_a = "NONE";
    defparam ram6.address_aclr_b = "NONE";
    defparam ram6.address_reg_b = "CLOCK0";
    defparam ram6.outdata_reg_b = "UNREGISTERED";
    defparam ram6.outdata_aclr_b = "NONE";
    defparam ram6.indata_aclr_a = "NONE";
    defparam ram6.wrcontrol_aclr_a = "NONE";
    defparam ram6.read_during_write_mode_mixed_ports = "DONT_CARE";
    defparam ram7.operation_mode = "DUAL_PORT";
    defparam ram7.width_a = 8;
    defparam ram7.widthad_a = 10;
    defparam ram7.numwords_a = 1024;
    defparam ram7.width_b = 8;
    defparam ram7.widthad_b = 10;
    defparam ram7.numwords_b = 1024;
    defparam ram7.address_aclr_a = "NONE";
    defparam ram7.address_aclr_b = "NONE";
    defparam ram7.address_reg_b = "CLOCK0";
    defparam ram7.outdata_reg_b = "UNREGISTERED";
    defparam ram7.outdata_aclr_b = "NONE";
    defparam ram7.indata_aclr_a = "NONE";
    defparam ram7.wrcontrol_aclr_a = "NONE";
    defparam ram7.read_during_write_mode_mixed_ports = "DONT_CARE";
`ifdef FPGA_CYCLONE_IV_GX
    defparam ram0.intended_device_family = "Cyclone IV GX";
    defparam ram1.intended_device_family = "Cyclone IV GX";
    defparam ram2.intended_device_family = "Cyclone IV GX";
    defparam ram3.intended_device_family = "Cyclone IV GX";
    defparam ram4.intended_device_family = "Cyclone IV GX";
    defparam ram5.intended_device_family = "Cyclone IV GX";
    defparam ram6.intended_device_family = "Cyclone IV GX";
    defparam ram7.intended_device_family = "Cyclone IV GX";
`else
    defparam ram0.intended_device_family = "Cyclone IV E";
    defparam ram1.intended_device_family = "Cyclone IV E";
    defparam ram2.intended_device_family = "Cyclone IV E";
    defparam ram3.intended_device_family = "Cyclone IV E";
    defparam ram4.intended_device_family = "Cyclone IV E";
    defparam ram5.intended_device_family = "Cyclone IV E";
    defparam ram6.intended_device_family = "Cyclone IV E";
    defparam ram7.intended_device_family = "Cyclone IV E";
`endif
    defparam ram0.lpm_type = "altsyncram";
    defparam ram1.lpm_type = "altsyncram";
    defparam ram2.lpm_type = "altsyncram";
    defparam ram3.lpm_type = "altsyncram";
    defparam ram4.lpm_type = "altsyncram";
    defparam ram5.lpm_type = "altsyncram";
    defparam ram6.lpm_type = "altsyncram";
    defparam ram7.lpm_type = "altsyncram";
`endif
endmodule
/* verilator lint_on DECLFILENAME */
`endif

module r4000_dcache (
    input  wire        clk,
    input  wire        rst_n,

    // Core interface
    input  wire [31:0] core_addr,     // physical address (doubleword-aligned)
    input  wire [31:0] lookup_addr,   // DF-stage registered-RAM lookahead
    input  wire        lookup_advance,
    input  wire        core_ren,      // read enable
    input  wire        core_wen,      // write enable
    input  wire [ 7:0] core_wmask,    // byte write mask
    input  wire [63:0] core_wdata,    // write data
    input  wire        uncached,      // bypass cache (kseg1)
    output wire [63:0] core_rdata,    // read data to core
    output wire        stall,         // pipeline stall
    output wire        core_store_visible, // cached store mutated D$ this edge

    // CACHE instruction (from WB commit)
    input  wire        cop_valid,
    input  wire [2:0]  cop_op,        // operation [20:18]
    input  wire [31:0] cop_paddr,     // physical address

    // Freeze from secondary cache (hold state machine on S-cache miss)
    input  wire        freeze,

    // Memory-side interface (to S-cache or external memory, 64-bit data path)
    output reg  [31:0] mem_addr,
    output reg         mem_ren,
    output reg         mem_wen,
    output reg  [ 7:0] mem_wmask,
    output reg  [63:0] mem_wdata,
    input  wire [63:0] mem_rdata,

    // DMA snoop port: invalidate D-cache lines on DMA writes (cache coherency)
    input  wire        snoop_valid,
    input  wire [31:0] snoop_addr,

    // S-cache back-invalidation port (inclusion maintenance)
    // When S-cache evicts a valid line, D-cache must invalidate matching entries.
    // One S-cache line (128B) covers up to 4 D-cache lines (32B each).
    input  wire        backinv_valid,
    input  wire [31:0] backinv_addr,
    input  wire        backinv_writeback,
    input  wire        backinv_ready,
    output wire        backinv_done,

    // Debug capture for one-shot FPGA cache-watch snapshots.
    output wire [127:0] debug_capture,

    // Tag readback for CACHE PD|ILT (Index Load Tag)
    output wire [31:0] cop_taglo_out
);

    // Address decomposition:
    // [4:0]   byte offset (32B line)
    // [12:5]  index (256 entries)
    // [31:13] tag (19 bits)
    localparam NLINES = 256;
    localparam TAG_W  = 19;

`ifdef FPGA_L1_BYPASS
    // FPGA bring-up mode: bypass the flop-based L1 D-cache arrays. Cached and
    // uncached requests are forwarded to the S-cache/memory side; the core is
    // held while S-cache asserts freeze for misses or busy cycles.
    assign core_rdata = mem_rdata;
    assign stall = (core_ren || core_wen) && freeze;
    assign core_store_visible = core_wen && !uncached && !freeze;
    assign debug_capture = 128'd0;
    assign cop_taglo_out = 32'd0;
    assign backinv_done = backinv_valid;

    always @(*) begin
        mem_addr  = core_addr;
        mem_ren   = core_ren;
        mem_wen   = core_wen;
        mem_wmask = core_wen ? core_wmask : 8'h00;
        mem_wdata = core_wdata;
    end
`elsif FPGA_DCACHE_BRAM
    reg [NLINES-1:0] valid;
    reg [NLINES-1:0] dirty;

    wire [7:0]       idx      = core_addr[12:5];
    wire [TAG_W-1:0] addr_tag = core_addr[31:13];
    wire [1:0]       dw_sel   = core_addr[4:3];
    wire             core_req = core_ren || core_wen;
    wire [31:0]      ram_lookup_addr =
        lookup_advance ? lookup_addr : core_addr;
    wire [7:0]       lookup_idx = ram_lookup_addr[12:5];
    wire [TAG_W-1:0] lookup_tag = ram_lookup_addr[31:13];

    // WB emits CACHE as a commit pulse, while the FPGA tag RAM returns the
    // selected line one cycle later. Retain the operation until that lookup
    // is ready, as the primary-I and secondary-cache paths already do.
    reg              cop_latched;
    reg [2:0]        cop_op_latched;
    reg [31:0]       cop_paddr_latched;

    wire             cop_req_valid = cop_latched || cop_valid;
    wire [2:0]       cop_req_op = cop_latched ? cop_op_latched : cop_op;
    wire [31:0]      cop_req_paddr =
        cop_latched ? cop_paddr_latched : cop_paddr;
    wire [7:0]       cop_idx  = cop_req_paddr[12:5];
    wire [TAG_W-1:0] cop_tag  = cop_req_paddr[31:13];

    wire [7:0]       snoop_idx = snoop_addr[12:5];
    wire [TAG_W-1:0] snoop_tag = snoop_addr[31:13];

    wire [7:0]       backinv_idx0 = {backinv_addr[12:7], 2'b00};
    wire [7:0]       backinv_idx1 = {backinv_addr[12:7], 2'b01};
    wire [7:0]       backinv_idx2 = {backinv_addr[12:7], 2'b10};
    wire [7:0]       backinv_idx3 = {backinv_addr[12:7], 2'b11};
    wire [TAG_W-1:0] backinv_tag  = backinv_addr[31:13];

    wire [TAG_W-1:0] core_tag_rdata;
    wire [TAG_W-1:0] cop_tag_rdata;
    wire [TAG_W-1:0] snoop_tag_rdata;
    wire [TAG_W-1:0] backinv_tag_rdata0;
    wire [TAG_W-1:0] backinv_tag_rdata1;
    wire [TAG_W-1:0] backinv_tag_rdata2;
    wire [TAG_W-1:0] backinv_tag_rdata3;

    reg              tag_we;
    reg [7:0]        tag_waddr;
    reg [TAG_W-1:0]  tag_wdata;

    r4000_dcache_tag_ram u_tag_core (
        .clk   (clk),
        .raddr (lookup_idx),
        .rdata (core_tag_rdata),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_dcache_tag_ram u_tag_cop (
        .clk   (clk),
        .raddr (cop_idx),
        .rdata (cop_tag_rdata),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_dcache_tag_ram u_tag_snoop (
        .clk   (clk),
        .raddr (snoop_idx),
        .rdata (snoop_tag_rdata),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_dcache_tag_ram u_tag_backinv0 (
        .clk   (clk),
        .raddr (backinv_idx0),
        .rdata (backinv_tag_rdata0),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_dcache_tag_ram u_tag_backinv1 (
        .clk   (clk),
        .raddr (backinv_idx1),
        .rdata (backinv_tag_rdata1),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_dcache_tag_ram u_tag_backinv2 (
        .clk   (clk),
        .raddr (backinv_idx2),
        .rdata (backinv_tag_rdata2),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    r4000_dcache_tag_ram u_tag_backinv3 (
        .clk   (clk),
        .raddr (backinv_idx3),
        .rdata (backinv_tag_rdata3),
        .we    (tag_we),
        .waddr (tag_waddr),
        .wdata (tag_wdata)
    );

    reg              meta_sample_q;
    reg [7:0]        meta_idx_q;
    reg [TAG_W-1:0]  meta_addr_tag_q;
    reg              meta_line_valid_q;
    reg              meta_line_dirty_q;

    reg              cop_meta_sample_q;
    reg [7:0]        cop_meta_idx_q;
    reg [TAG_W-1:0]  cop_meta_addr_tag_q;
    reg              cop_meta_line_valid_q;
    reg              cop_meta_line_dirty_q;

    reg              snoop_meta_req_q;
    reg [7:0]        snoop_meta_idx_q;
    reg [TAG_W-1:0]  snoop_meta_addr_tag_q;
    reg              snoop_meta_line_valid_q;

    reg              backinv_meta_req_q;
    reg [7:0]        backinv_meta_idx0_q;
    reg [7:0]        backinv_meta_idx1_q;
    reg [7:0]        backinv_meta_idx2_q;
    reg [7:0]        backinv_meta_idx3_q;
    reg [TAG_W-1:0]  backinv_meta_addr_tag_q;
    reg              backinv_meta_line_valid0_q;
    reg              backinv_meta_line_valid1_q;
    reg              backinv_meta_line_valid2_q;
    reg              backinv_meta_line_valid3_q;
    reg              backinv_meta_line_dirty0_q;
    reg              backinv_meta_line_dirty1_q;
    reg              backinv_meta_line_dirty2_q;
    reg              backinv_meta_line_dirty3_q;
    reg              backinv_writeback_q;

    wire [3:0] backinv_dirty_hit = {
        backinv_meta_line_valid3_q && backinv_meta_line_dirty3_q &&
            backinv_tag_rdata3 == backinv_meta_addr_tag_q,
        backinv_meta_line_valid2_q && backinv_meta_line_dirty2_q &&
            backinv_tag_rdata2 == backinv_meta_addr_tag_q,
        backinv_meta_line_valid1_q && backinv_meta_line_dirty1_q &&
            backinv_tag_rdata1 == backinv_meta_addr_tag_q,
        backinv_meta_line_valid0_q && backinv_meta_line_dirty0_q &&
            backinv_tag_rdata0 == backinv_meta_addr_tag_q
    };

    wire meta_ready = core_req && !uncached && meta_sample_q &&
                      (meta_idx_q == idx) &&
                      (meta_addr_tag_q == addr_tag);
    wire meta_wait /* verilator public_flat_rd */ =
        core_req && !uncached && !meta_ready;
    wire tag_hit = meta_ready && meta_line_valid_q && (core_tag_rdata == addr_tag);
    wire miss = core_req && !uncached && meta_ready && !tag_hit;
    wire need_wb = miss && meta_line_valid_q && meta_line_dirty_q;

    wire cop_meta_ready = cop_req_valid && cop_meta_sample_q &&
                          (cop_meta_idx_q == cop_idx) &&
                          (cop_meta_addr_tag_q == cop_tag);
    wire cop_meta_wait = cop_req_valid && !cop_meta_ready;

    wire [1:0] cop_dc_pstate = (cop_meta_ready && cop_meta_line_valid_q) ?
        (cop_meta_line_dirty_q ? 2'b11 : 2'b01) : 2'b00;
    assign cop_taglo_out = {cop_tag_rdata, 5'b0, cop_dc_pstate, 6'b0};

    localparam [3:0] S_IDLE             = 4'd0,
                     S_WB_RD            = 4'd1,
                     S_WB_SEND          = 4'd2,
                     S_FILL             = 4'd3,
                     S_CACHE_WB_RD      = 4'd4,
                     S_CACHE_WB_SEND    = 4'd5,
                     S_REFETCH          = 4'd6,
                     S_BACKINV_WB_RD    = 4'd7,
                     S_BACKINV_WB_SEND  = 4'd8;

    reg [3:0]  state;
    reg [1:0]  fill_cnt;
    reg [31:0] fill_addr;
    reg        fill_snoop_pending;
    reg [31:0] wb_addr;
    reg [9:0]  ram_raddr_q;
    reg        ram_raddr_valid;
    reg [3:0]  backinv_wb_mask;
    reg [1:0]  backinv_wb_line;
    reg [1:0]  backinv_wb_dw;
    reg [31:0] backinv_wb_base;
    reg [3:0]  backinv_resume_state;
    reg        backinv_done_q;
    assign backinv_done = backinv_done_q;
    wire       cop_process = (state == S_IDLE) && cop_req_valid &&
                             cop_meta_ready && !miss;

    wire [9:0] core_dw_addr = core_addr[12:3];
    wire [9:0] lookup_dw_addr = ram_lookup_addr[12:3];
    wire [9:0] wb_dw_addr   = {wb_addr[12:5], fill_cnt};
    wire       wb_read_state = (state == S_WB_RD) || (state == S_WB_SEND) ||
                               (state == S_CACHE_WB_RD) || (state == S_CACHE_WB_SEND);
    wire       backinv_wb_read_state = (state == S_BACKINV_WB_RD) ||
                                       (state == S_BACKINV_WB_SEND);
    wire [9:0] backinv_wb_dw_addr = {
        backinv_wb_base[12:7], backinv_wb_line, backinv_wb_dw
    };
    wire [9:0] ram_raddr = backinv_wb_read_state ? backinv_wb_dw_addr :
                           wb_read_state ? wb_dw_addr : lookup_dw_addr;
    wire [63:0] ram_rdata;
    wire        ram_read_hit = ram_raddr_valid && (ram_raddr_q == core_dw_addr);

    wire        fill_write = (state == S_FILL) && !freeze;
    wire        fill_snoop_live_hit = snoop_valid &&
                                      (state == S_FILL || state == S_REFETCH) &&
                                      (snoop_addr[31:5] == fill_addr[31:5]);
    wire        store_hit_write = (state == S_IDLE) && core_wen && !uncached && tag_hit && !freeze;
    assign core_store_visible = store_hit_write;
    wire        ram_we = fill_write || store_hit_write;
    wire [9:0]  ram_waddr = fill_write ? {fill_addr[12:5], fill_cnt} : core_dw_addr;
    wire [63:0] ram_wdata = fill_write ? mem_rdata : core_wdata;
    wire [7:0]  ram_byteena = fill_write ? 8'hFF : core_wmask;

`ifdef VERILATOR
    reg [63:0] data_ram [0:1023];
    reg [63:0] ram_rdata_q;
    integer bram_b;

    assign ram_rdata = ram_rdata_q;

    always @(posedge clk) begin
        ram_rdata_q <= data_ram[ram_raddr];
        if (ram_we) begin
            for (bram_b = 0; bram_b < 8; bram_b = bram_b + 1) begin
                if (ram_byteena[bram_b])
                    data_ram[ram_waddr][bram_b*8 +: 8] <= ram_wdata[bram_b*8 +: 8];
            end
        end
    end
`else
    r4000_dcache_ram64 u_data (
        .clk     (clk),
        .raddr   (ram_raddr),
        .rdata   (ram_rdata),
        .we      (ram_we),
        .waddr   (ram_waddr),
        .wdata   (ram_wdata),
        .byteena (ram_byteena)
    );
`endif

    assign core_rdata = (!uncached && tag_hit && ram_read_hit) ? ram_rdata : mem_rdata;
    assign stall = (state != S_IDLE) || meta_wait || miss || cop_meta_wait
                 || (core_ren && !uncached && tag_hit && !ram_read_hit)
                 || (core_wen && !uncached && tag_hit && freeze);

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
        DBG_SRC_NONE    = 4'h0,
        DBG_SRC_FILL    = 4'h1,
        DBG_SRC_STORE   = 4'h2,
        DBG_SRC_SNOOP   = 4'h3,
        DBG_SRC_BACKINV = 4'h4,
        DBG_SRC_COP     = 4'h5;

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

    wire [31:0] debug_supply_word =
        core_addr[2] ? ram_rdata[31:0] : ram_rdata[63:32];
    wire debug_head_supply =
        core_ren && !uncached && tag_hit && ram_read_hit &&
        kmem_head_word_addr(core_addr);
    wire debug_head_supply_bad =
        debug_head_supply &&
        (debug_supply_word[31:16] == 16'd0) &&
        (debug_supply_word != 32'd0);
    wire debug_head_fill =
        fill_write && kmem_head_dw_addr({fill_addr[31:5], fill_cnt, 3'b000});
    wire debug_head_store =
        store_hit_write && kmem_head_dw_addr(core_addr) &&
        ((core_wmask & kmem_head_mask_for_dw(core_addr)) != 8'h00);
    wire debug_head_snoop =
        snoop_valid && kmem_head_dw_addr(snoop_addr);
    wire debug_head_backinv =
        backinv_valid &&
        (((backinv_addr & 32'hFFFF_FF80) == 32'h0815_F180) ||
         ((backinv_addr & 32'hFFFF_FF80) == 32'h0815_F200));
    wire debug_head_cop =
        cop_process && kmem_head_dw_addr(cop_req_paddr);
    wire debug_watch_event =
        debug_head_fill || debug_head_store || debug_head_snoop ||
        debug_head_backinv || debug_head_cop;
    wire [3:0] debug_watch_src_code =
        debug_head_fill   ? DBG_SRC_FILL :
        debug_head_store  ? DBG_SRC_STORE :
        debug_head_snoop  ? DBG_SRC_SNOOP :
        debug_head_backinv ? DBG_SRC_BACKINV :
        debug_head_cop    ? DBG_SRC_COP :
                             DBG_SRC_NONE;
    wire [31:0] debug_watch_src_addr =
        debug_head_fill   ? {fill_addr[31:5], fill_cnt, 3'b000} :
        debug_head_store  ? core_addr :
        debug_head_snoop  ? snoop_addr :
        debug_head_backinv ? backinv_addr :
        debug_head_cop    ? cop_req_paddr :
                             32'd0;
    wire [63:0] debug_watch_src_data =
        debug_head_fill   ? ram_wdata :
        debug_head_store  ? core_wdata :
        debug_head_snoop  ? {snoop_addr, kmem_head_word_from_dw(snoop_addr, 64'd0)} :
        debug_head_backinv ? {backinv_addr, 32'h0815_F1C0} :
        debug_head_cop    ? {cop_taglo_out, cop_paddr} :
                             64'd0;

    reg [3:0]  debug_src_code_q;
    reg [2:0]  debug_bad_state_q;
    reg [31:0] debug_src_addr_q;
    reg [63:0] debug_src_data_q;
    reg        debug_bad_seen;
    reg [31:0] debug_bad_word_q;

    assign debug_capture = {
        8'h4B,
        debug_src_code_q,
        debug_bad_seen,
        debug_bad_state_q,
        debug_bad_word_q[15:0],
        debug_src_addr_q,
        debug_src_data_q[63:32],
        debug_src_data_q[31:0]
    };

    always @(*) begin
        tag_we    = 1'b0;
        tag_waddr = 8'd0;
        tag_wdata = {TAG_W{1'b0}};

        if (rst_n && state == S_FILL && !freeze && fill_cnt == 2'd3) begin
            tag_we    = 1'b1;
            tag_waddr = fill_addr[12:5];
            tag_wdata = fill_addr[31:13];
        end
    end

    always @(*) begin
        mem_addr  = core_addr;
        mem_ren   = 1'b0;
        mem_wen   = 1'b0;
        mem_wmask = 8'h00;
        mem_wdata = 64'd0;

        case (state)
            S_WB_SEND, S_CACHE_WB_SEND: begin
                mem_addr  = {wb_addr[31:5], fill_cnt, 3'b000};
                mem_ren   = 1'b0;
                mem_wen   = 1'b1;
                mem_wmask = 8'hFF;
                mem_wdata = ram_rdata;
            end
            S_BACKINV_WB_SEND: begin
                mem_addr  = {
                    backinv_wb_base[31:7], backinv_wb_line,
                    backinv_wb_dw, 3'b000
                };
                mem_ren   = 1'b0;
                mem_wen   = backinv_ready;
                mem_wmask = backinv_ready ? 8'hFF : 8'h00;
                mem_wdata = ram_rdata;
            end
            S_FILL: begin
                mem_addr  = {fill_addr[31:5], fill_cnt, 3'b000};
                mem_ren   = 1'b1;
                mem_wen   = 1'b0;
                mem_wmask = 8'h00;
                mem_wdata = 64'd0;
            end
            default: begin
                if (uncached) begin
                    mem_addr  = core_addr;
                    mem_ren   = core_ren;
                    mem_wen   = core_wen;
                    mem_wmask = core_wen ? core_wmask : 8'h00;
                    mem_wdata = core_wdata;
                end
                // Cached stores update the dirty Primary-D BRAM only. The
                // existing victim/CACHE writeback states publish those bytes
                // to S-cache. Forwarding a store here as well destroys the
                // old secondary image that Primary-I Fill is defined to see
                // in a cross-primary cache-maintenance sequence.
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            fill_cnt <= 2'd0;
            fill_snoop_pending <= 1'b0;
            valid <= {NLINES{1'b0}};
            dirty <= {NLINES{1'b0}};
            cop_latched <= 1'b0;
            cop_op_latched <= 3'd0;
            cop_paddr_latched <= 32'd0;
            ram_raddr_q <= 10'd0;
            ram_raddr_valid <= 1'b0;
            meta_sample_q <= 1'b0;
            meta_idx_q <= 8'd0;
            meta_addr_tag_q <= {TAG_W{1'b0}};
            meta_line_valid_q <= 1'b0;
            meta_line_dirty_q <= 1'b0;
            cop_meta_sample_q <= 1'b0;
            cop_meta_idx_q <= 8'd0;
            cop_meta_addr_tag_q <= {TAG_W{1'b0}};
            cop_meta_line_valid_q <= 1'b0;
            cop_meta_line_dirty_q <= 1'b0;
            snoop_meta_req_q <= 1'b0;
            snoop_meta_idx_q <= 8'd0;
            snoop_meta_addr_tag_q <= {TAG_W{1'b0}};
            snoop_meta_line_valid_q <= 1'b0;
            backinv_meta_req_q <= 1'b0;
            backinv_meta_idx0_q <= 8'd0;
            backinv_meta_idx1_q <= 8'd0;
            backinv_meta_idx2_q <= 8'd0;
            backinv_meta_idx3_q <= 8'd0;
            backinv_meta_addr_tag_q <= {TAG_W{1'b0}};
            backinv_meta_line_valid0_q <= 1'b0;
            backinv_meta_line_valid1_q <= 1'b0;
            backinv_meta_line_valid2_q <= 1'b0;
            backinv_meta_line_valid3_q <= 1'b0;
            backinv_meta_line_dirty0_q <= 1'b0;
            backinv_meta_line_dirty1_q <= 1'b0;
            backinv_meta_line_dirty2_q <= 1'b0;
            backinv_meta_line_dirty3_q <= 1'b0;
            backinv_writeback_q <= 1'b0;
            backinv_wb_mask <= 4'd0;
            backinv_wb_line <= 2'd0;
            backinv_wb_dw <= 2'd0;
            backinv_wb_base <= 32'd0;
            backinv_resume_state <= S_IDLE;
            backinv_done_q <= 1'b0;
            debug_src_code_q <= DBG_SRC_NONE;
            debug_bad_state_q <= 3'd0;
            debug_src_addr_q <= 32'd0;
            debug_src_data_q <= 64'd0;
            debug_bad_seen <= 1'b0;
            debug_bad_word_q <= 32'd0;
        end else begin
            backinv_done_q <= 1'b0;
            ram_raddr_q <= ram_raddr;
            ram_raddr_valid <= !ram_we;
            meta_sample_q <= 1'b1;
            meta_idx_q <= lookup_idx;
            meta_addr_tag_q <= lookup_tag;
            meta_line_valid_q <= valid[lookup_idx];
            // A resident store and the next DF lookup may share an edge.  If
            // that lookup aliases the just-dirtied index with a different
            // tag, its following miss must still write back the current
            // victim rather than observing the pre-store dirty bit.
            meta_line_dirty_q <= dirty[lookup_idx] ||
                (store_hit_write && (lookup_idx == idx));
            cop_meta_sample_q <= 1'b1;
            cop_meta_idx_q <= cop_idx;
            cop_meta_addr_tag_q <= cop_tag;
            cop_meta_line_valid_q <= valid[cop_idx];
            cop_meta_line_dirty_q <= dirty[cop_idx];
            snoop_meta_req_q <= snoop_valid;
            snoop_meta_idx_q <= snoop_idx;
            snoop_meta_addr_tag_q <= snoop_tag;
            snoop_meta_line_valid_q <= valid[snoop_idx];
            backinv_meta_req_q <= backinv_valid;
            backinv_meta_idx0_q <= backinv_idx0;
            backinv_meta_idx1_q <= backinv_idx1;
            backinv_meta_idx2_q <= backinv_idx2;
            backinv_meta_idx3_q <= backinv_idx3;
            backinv_meta_addr_tag_q <= backinv_tag;
            backinv_meta_line_valid0_q <= valid[backinv_idx0];
            backinv_meta_line_valid1_q <= valid[backinv_idx1];
            backinv_meta_line_valid2_q <= valid[backinv_idx2];
            backinv_meta_line_valid3_q <= valid[backinv_idx3];
            backinv_meta_line_dirty0_q <= dirty[backinv_idx0];
            backinv_meta_line_dirty1_q <= dirty[backinv_idx1];
            backinv_meta_line_dirty2_q <= dirty[backinv_idx2];
            backinv_meta_line_dirty3_q <= dirty[backinv_idx3];
            if (backinv_valid)
                backinv_writeback_q <= backinv_writeback;

            if (cop_process) begin
                cop_latched <= 1'b0;
            end else if (cop_valid && !cop_latched) begin
                cop_latched <= 1'b1;
                cop_op_latched <= cop_op;
                cop_paddr_latched <= cop_paddr;
            end

            if (!debug_bad_seen && debug_watch_event) begin
                debug_src_code_q <= debug_watch_src_code;
                debug_src_addr_q <= debug_watch_src_addr;
                debug_src_data_q <= debug_watch_src_data;
            end
            if (!debug_bad_seen && debug_head_supply_bad) begin
                debug_bad_state_q <= state[2:0];
                debug_bad_word_q <= debug_supply_word;
                debug_bad_seen <= 1'b1;
            end

            case (state)
                S_IDLE: begin
                    if (store_hit_write)
                        dirty[idx] <= 1'b1;

                    if (miss && !freeze) begin
                        fill_addr <= core_addr;
                        fill_snoop_pending <= 1'b0;
                        if (need_wb) begin
                            wb_addr <= {core_tag_rdata, idx, 5'b0};
                            state <= S_WB_RD;
                            fill_cnt <= 2'd0;
                        end else begin
                            state <= S_FILL;
                            fill_cnt <= 2'd0;
                        end
                    end

                    if (cop_process) begin
                        case (cop_req_op)
                            3'd0: begin
                                if (cop_meta_line_valid_q && cop_meta_line_dirty_q) begin
                                    wb_addr <= {cop_tag_rdata, cop_idx, 5'b0};
                                    state <= S_CACHE_WB_RD;
                                    fill_cnt <= 2'd0;
                                end else begin
                                    valid[cop_idx] <= 1'b0;
                                    dirty[cop_idx] <= 1'b0;
                                    meta_sample_q <= 1'b0;
                                    cop_meta_sample_q <= 1'b0;
                                end
                            end
                            3'd4: begin
                                if (cop_meta_line_valid_q && cop_tag_rdata == cop_tag) begin
                                    // Hit Invalidate discards the primary
                                    // line even when dirty.  Operation 5 is
                                    // the distinct Hit Writeback Invalidate.
                                    valid[cop_idx] <= 1'b0;
                                    dirty[cop_idx] <= 1'b0;
                                    meta_sample_q <= 1'b0;
                                    cop_meta_sample_q <= 1'b0;
                                end
                            end
                            3'd5: begin
                                if (cop_meta_line_valid_q && cop_tag_rdata == cop_tag) begin
                                    if (cop_meta_line_dirty_q) begin
                                        wb_addr <= {cop_tag_rdata, cop_idx, 5'b0};
                                        state <= S_CACHE_WB_RD;
                                        fill_cnt <= 2'd0;
                                    end else begin
                                        valid[cop_idx] <= 1'b0;
                                        meta_sample_q <= 1'b0;
                                        cop_meta_sample_q <= 1'b0;
                                    end
                                end
                            end
                            default: ;
                        endcase
                    end
                end

                S_WB_RD: begin
                    state <= S_WB_SEND;
                end

                S_WB_SEND: if (!freeze) begin
                    if (fill_cnt == 2'd3) begin
                        state <= S_FILL;
                        fill_cnt <= 2'd0;
                    end else begin
                        fill_cnt <= fill_cnt + 2'd1;
                        state <= S_WB_RD;
                    end
                end

                S_FILL: if (!freeze) begin
                    if (fill_cnt == 2'd3) begin
                        // A DMA write can race an uninstalled fill.  The tag
                        // lookup sees the old invalid/victim metadata, so a
                        // normal snoop hit cannot invalidate it.  Do not
                        // install data assembled before that external write.
                        valid[fill_addr[12:5]] <=
                            !(fill_snoop_pending || fill_snoop_live_hit);
                        dirty[fill_addr[12:5]] <= 1'b0;
                        state <= S_REFETCH;
                    end
                    fill_cnt <= fill_cnt + 2'd1;
                end

                S_REFETCH: begin
                    state <= S_IDLE;
                end

                S_CACHE_WB_RD: begin
                    state <= S_CACHE_WB_SEND;
                end

                S_CACHE_WB_SEND: if (!freeze) begin
                    if (fill_cnt == 2'd3) begin
                        valid[wb_addr[12:5]] <= 1'b0;
                        dirty[wb_addr[12:5]] <= 1'b0;
                        state <= S_REFETCH;
                    end else begin
                        fill_cnt <= fill_cnt + 2'd1;
                        state <= S_CACHE_WB_RD;
                    end
                end

                S_BACKINV_WB_RD: begin
                    state <= S_BACKINV_WB_SEND;
                end

                S_BACKINV_WB_SEND: if (backinv_ready) begin
                    if (backinv_wb_dw != 2'd3) begin
                        backinv_wb_dw <= backinv_wb_dw + 2'd1;
                        state <= S_BACKINV_WB_RD;
                    end else begin
                        backinv_wb_dw <= 2'd0;
                        case (backinv_wb_line)
                            2'd0: begin
                                if (backinv_wb_mask[1]) begin
                                    backinv_wb_line <= 2'd1;
                                    state <= S_BACKINV_WB_RD;
                                end else if (backinv_wb_mask[2]) begin
                                    backinv_wb_line <= 2'd2;
                                    state <= S_BACKINV_WB_RD;
                                end else if (backinv_wb_mask[3]) begin
                                    backinv_wb_line <= 2'd3;
                                    state <= S_BACKINV_WB_RD;
                                end else begin
                                    state <= backinv_resume_state;
                                    backinv_done_q <= 1'b1;
                                end
                            end
                            2'd1: begin
                                if (backinv_wb_mask[2]) begin
                                    backinv_wb_line <= 2'd2;
                                    state <= S_BACKINV_WB_RD;
                                end else if (backinv_wb_mask[3]) begin
                                    backinv_wb_line <= 2'd3;
                                    state <= S_BACKINV_WB_RD;
                                end else begin
                                    state <= backinv_resume_state;
                                    backinv_done_q <= 1'b1;
                                end
                            end
                            2'd2: begin
                                if (backinv_wb_mask[3]) begin
                                    backinv_wb_line <= 2'd3;
                                    state <= S_BACKINV_WB_RD;
                                end else begin
                                    state <= backinv_resume_state;
                                    backinv_done_q <= 1'b1;
                                end
                            end
                            default: begin
                                state <= backinv_resume_state;
                                backinv_done_q <= 1'b1;
                            end
                        endcase
                    end
                end

                default: state <= S_IDLE;
            endcase

            if (fill_snoop_live_hit) begin
                fill_snoop_pending <= 1'b1;
                if (state == S_REFETCH) begin
                    valid[fill_addr[12:5]] <= 1'b0;
                    dirty[fill_addr[12:5]] <= 1'b0;
                end
            end

            if (snoop_meta_req_q &&
                snoop_meta_line_valid_q &&
                snoop_tag_rdata == snoop_meta_addr_tag_q) begin
                valid[snoop_meta_idx_q] <= 1'b0;
                dirty[snoop_meta_idx_q] <= 1'b0;
                meta_sample_q <= 1'b0;
                cop_meta_sample_q <= 1'b0;
            end

            if (backinv_meta_req_q) begin
                if (backinv_meta_line_valid0_q &&
                    backinv_tag_rdata0 == backinv_meta_addr_tag_q) begin
                    valid[backinv_meta_idx0_q] <= 1'b0;
                    dirty[backinv_meta_idx0_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid1_q &&
                    backinv_tag_rdata1 == backinv_meta_addr_tag_q) begin
                    valid[backinv_meta_idx1_q] <= 1'b0;
                    dirty[backinv_meta_idx1_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid2_q &&
                    backinv_tag_rdata2 == backinv_meta_addr_tag_q) begin
                    valid[backinv_meta_idx2_q] <= 1'b0;
                    dirty[backinv_meta_idx2_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_meta_line_valid3_q &&
                    backinv_tag_rdata3 == backinv_meta_addr_tag_q) begin
                    valid[backinv_meta_idx3_q] <= 1'b0;
                    dirty[backinv_meta_idx3_q] <= 1'b0;
                    meta_sample_q <= 1'b0;
                    cop_meta_sample_q <= 1'b0;
                end
                if (backinv_writeback_q && |backinv_dirty_hit) begin
                    backinv_wb_mask <= backinv_dirty_hit;
                    backinv_wb_dw <= 2'd0;
                    backinv_wb_base <= {
                        backinv_meta_addr_tag_q,
                        backinv_meta_idx0_q[7:2], 7'b0
                    };
                    backinv_resume_state <= state;
                    if (backinv_dirty_hit[0])
                        backinv_wb_line <= 2'd0;
                    else if (backinv_dirty_hit[1])
                        backinv_wb_line <= 2'd1;
                    else if (backinv_dirty_hit[2])
                        backinv_wb_line <= 2'd2;
                    else
                        backinv_wb_line <= 2'd3;
                    state <= S_BACKINV_WB_RD;
                end else begin
                    backinv_done_q <= 1'b1;
                end
            end
        end
    end
`else
    assign backinv_done = backinv_valid;

    // Packed (was unpacked) — see icache.v note.
    reg [NLINES-1:0] valid;
    reg [NLINES-1:0] dirty;
    reg [TAG_W-1:0]  tags  [0:NLINES-1];
    // Data stored as 4 doublewords per line (big-endian)
    // dw0 = bytes at offset 0x00-0x07 (lowest address)
    // dw3 = bytes at offset 0x18-0x1F (highest address)
    reg [63:0] data0 [0:NLINES-1];
    reg [63:0] data1 [0:NLINES-1];
    reg [63:0] data2 [0:NLINES-1];
    reg [63:0] data3 [0:NLINES-1];

    wire [7:0]       idx      = core_addr[12:5];
    wire [TAG_W-1:0] addr_tag = core_addr[31:13];
    wire [1:0]       dw_sel   = core_addr[4:3];

    // DMA snoop address decomposition
    wire [7:0]       snoop_idx = snoop_addr[12:5];
    wire [TAG_W-1:0] snoop_tag = snoop_addr[31:13];

    wire tag_hit = valid[idx] && (tags[idx] == addr_tag);
    wire core_req = core_ren || core_wen;
    wire miss = core_req && !uncached && !tag_hit;
    wire need_wb = miss && valid[idx] && dirty[idx]; // dirty victim needs writeback

    // Tag readback: PState (00=invalid, 01=clean, 11=dirty)
    wire [1:0] cop_dc_pstate = valid[cop_paddr[12:5]] ?
        (dirty[cop_paddr[12:5]] ? 2'b11 : 2'b01) : 2'b00;
    assign cop_taglo_out = {tags[cop_paddr[12:5]], 5'b0, cop_dc_pstate, 6'b0};

    // Byte merge function: selectively update bytes
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

    // State machine
    localparam [2:0] S_IDLE     = 3'd0,
                     S_WB       = 3'd1,  // writeback dirty victim
                     S_FILL     = 3'd2,  // fill from memory
                     S_DONE     = 3'd3,  // commit filled line
                     S_CACHE_WB = 3'd4;  // CACHE writeback
    reg [2:0] state;
    reg [1:0] fill_cnt;    // doubleword counter (0-3) for fill/wb
    reg [31:0] fill_addr;  // address of line being filled
    reg [31:0] wb_addr;    // address of line being written back
    reg [63:0] fill_dw [0:3]; // fill buffer
    reg [63:0] wb_dw [0:3];   // writeback data buffer
    reg        fill_need_wb;  // did we need writeback before fill?

    // Select doubleword from cached line
    reg [63:0] cached_dw;
    always @(*) begin
        case (dw_sel)
            2'd0: cached_dw = data0[idx];
            2'd1: cached_dw = data1[idx];
            2'd2: cached_dw = data2[idx];
            2'd3: cached_dw = data3[idx];
        endcase
    end

    // Output read data
    assign core_rdata = (uncached || !tag_hit) ? mem_rdata : cached_dw;
    assign debug_capture = 128'd0;
    // Stall pipeline when:
    // 1. D-cache state machine is busy (miss handling, writeback)
    // 2. Cache miss detected
    // 3. Cached store hit but S-cache can't accept write-through (freeze=1)
    //    Without this, the pipeline advances past the store while D-cache is
    //    blocked by freeze, silently dropping the store. This happens when the
    //    S-cache evicts a line without back-invalidating D-cache (inclusion
    //    violation), causing the D-cache write-through to miss the S-cache.
    assign stall = (state != S_IDLE) || miss
                 || (core_wen && !uncached && tag_hit && freeze);
    assign core_store_visible = (state == S_IDLE) && core_wen &&
                                !uncached && tag_hit && !freeze;

    // Memory interface multiplexing
    always @(*) begin
        mem_addr  = core_addr;
        mem_ren   = 1'b0;
        mem_wen   = 1'b0;
        mem_wmask = 8'h00;
        mem_wdata = 64'd0;

        case (state)
            S_WB, S_CACHE_WB: begin
                mem_addr  = {wb_addr[31:5], fill_cnt, 3'b000};
                mem_ren   = 1'b0;
                mem_wen   = 1'b1;
                mem_wmask = 8'hFF;
                mem_wdata = wb_dw[fill_cnt];
            end
            S_FILL: begin
                mem_addr  = {fill_addr[31:5], fill_cnt, 3'b000};
                mem_ren   = 1'b1;
                mem_wen   = 1'b0;
                mem_wmask = 8'h00;
                mem_wdata = 64'd0;
            end
            default: begin // S_IDLE, S_DONE
                if (uncached) begin
                    mem_addr  = core_addr;
                    mem_ren   = core_ren;
                    mem_wen   = core_wen;
                    mem_wmask = core_wen ? core_wmask : 8'h00;
                    mem_wdata = core_wdata;
                end else if (core_wen) begin
                    // Write-through to S-cache on ALL cached stores (hit or miss).
                    // Real R4000 D-cache is write-through to L2 — even store misses
                    // must propagate to S-cache to maintain coherency.
                    mem_addr  = core_addr;
                    mem_wen   = 1'b1;
                    mem_wmask = core_wmask;
                    mem_wdata = core_wdata;
                end
            end
        endcase
    end

    // Main state machine
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            fill_cnt <= 2'd0;
            valid <= {NLINES{1'b0}};
            dirty <= {NLINES{1'b0}};
        end else begin
            case (state)
                S_IDLE: begin
                    // Store hit: update cache line, set dirty
                    if (core_wen && !uncached && tag_hit && !freeze) begin
                        case (dw_sel)
                            2'd0: data0[idx] <= byte_merge(data0[idx], core_wdata, core_wmask);
                            2'd1: data1[idx] <= byte_merge(data1[idx], core_wdata, core_wmask);
                            2'd2: data2[idx] <= byte_merge(data2[idx], core_wdata, core_wmask);
                            2'd3: data3[idx] <= byte_merge(data3[idx], core_wdata, core_wmask);
                        endcase
                        dirty[idx] <= 1'b1;
                    end

                    // Miss: start fill (with optional writeback)
                    if (miss && !freeze) begin
                        fill_addr <= core_addr;
                        if (need_wb) begin
                            // Save dirty line for writeback
                            wb_addr <= {tags[idx], idx, 5'b0};
                            wb_dw[0] <= data0[idx];
                            wb_dw[1] <= data1[idx];
                            wb_dw[2] <= data2[idx];
                            wb_dw[3] <= data3[idx];
                            state <= S_WB;
                            fill_cnt <= 2'd0;
                        end else begin
                            state <= S_FILL;
                            fill_cnt <= 2'd0;
                        end
                    end

                    // CACHE operations
                    if (cop_valid && !miss) begin
                        case (cop_op)
                            3'd0: begin // Index Writeback Invalidate
                                if (valid[cop_paddr[12:5]] && dirty[cop_paddr[12:5]]) begin
                                    wb_addr <= {tags[cop_paddr[12:5]], cop_paddr[12:5], 5'b0};
                                    wb_dw[0] <= data0[cop_paddr[12:5]];
                                    wb_dw[1] <= data1[cop_paddr[12:5]];
                                    wb_dw[2] <= data2[cop_paddr[12:5]];
                                    wb_dw[3] <= data3[cop_paddr[12:5]];
                                    state <= S_CACHE_WB;
                                    fill_cnt <= 2'd0;
                                end else begin
                                    valid[cop_paddr[12:5]] <= 1'b0;
                                    dirty[cop_paddr[12:5]] <= 1'b0;
                                end
                            end
                            3'd4: begin // Hit Invalidate
                                // On real R4000, D-cache is write-through so no dirty data.
                                // Our D-cache can have dirty data from inclusion violations
                                // (S-cache evicts without back-invalidating D-cache).
                                // Must writeback dirty data to prevent silent data loss.
                                if (valid[cop_paddr[12:5]] &&
                                    tags[cop_paddr[12:5]] == cop_paddr[31:13]) begin
                                    if (dirty[cop_paddr[12:5]]) begin
                                        wb_addr <= {tags[cop_paddr[12:5]], cop_paddr[12:5], 5'b0};
                                        wb_dw[0] <= data0[cop_paddr[12:5]];
                                        wb_dw[1] <= data1[cop_paddr[12:5]];
                                        wb_dw[2] <= data2[cop_paddr[12:5]];
                                        wb_dw[3] <= data3[cop_paddr[12:5]];
                                        state <= S_CACHE_WB;
                                        fill_cnt <= 2'd0;
                                    end else begin
                                        valid[cop_paddr[12:5]] <= 1'b0;
                                    end
                                end
                            end
                            3'd5: begin // Hit Writeback Invalidate
                                if (valid[cop_paddr[12:5]] &&
                                    tags[cop_paddr[12:5]] == cop_paddr[31:13]) begin
                                    if (dirty[cop_paddr[12:5]]) begin
                                        wb_addr <= {tags[cop_paddr[12:5]], cop_paddr[12:5], 5'b0};
                                        wb_dw[0] <= data0[cop_paddr[12:5]];
                                        wb_dw[1] <= data1[cop_paddr[12:5]];
                                        wb_dw[2] <= data2[cop_paddr[12:5]];
                                        wb_dw[3] <= data3[cop_paddr[12:5]];
                                        state <= S_CACHE_WB;
                                        fill_cnt <= 2'd0;
                                    end else begin
                                        valid[cop_paddr[12:5]] <= 1'b0;
                                    end
                                end
                            end
                            default: ;
                        endcase
                    end
                end

                S_WB: if (!freeze) begin
                    // Write dirty line to memory (4 doublewords)
                    if (fill_cnt == 2'd3) begin
                        state <= S_FILL;
                        fill_cnt <= 2'd0;
                    end else begin
                        fill_cnt <= fill_cnt + 2'd1;
                    end
                end

                S_FILL: if (!freeze) begin
                    fill_dw[fill_cnt] <= mem_rdata;
                    if (fill_cnt == 2'd3) begin
                        state <= S_DONE;
                    end
                    fill_cnt <= fill_cnt + 2'd1;
                end

                S_DONE: begin
                    // Commit filled line to cache
                    data0[fill_addr[12:5]] <= fill_dw[0];
                    data1[fill_addr[12:5]] <= fill_dw[1];
                    data2[fill_addr[12:5]] <= fill_dw[2];
                    data3[fill_addr[12:5]] <= fill_dw[3];
                    tags[fill_addr[12:5]]  <= fill_addr[31:13];
                    valid[fill_addr[12:5]] <= 1'b1;
                    dirty[fill_addr[12:5]] <= 1'b0;
                    state <= S_IDLE;
                end

                S_CACHE_WB: if (!freeze) begin
                    // CACHE-triggered writeback (4 doublewords)
                    if (fill_cnt == 2'd3) begin
                        // Invalidate after writeback
                        valid[wb_addr[12:5]] <= 1'b0;
                        dirty[wb_addr[12:5]] <= 1'b0;
                        state <= S_IDLE;
                    end else begin
                        fill_cnt <= fill_cnt + 2'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase

            // DMA snoop: invalidate D-cache line when DMA writes to a cached address
            if (snoop_valid) begin
                if (valid[snoop_idx] && tags[snoop_idx] == snoop_tag) begin
                    if (dirty[snoop_idx])
                        $display("DCACHE_SNOOP: DIRTY line invalidated! addr=0x%08x idx=%0d tag=0x%03x",
                            snoop_addr, snoop_idx, snoop_tag);
                    valid[snoop_idx] <= 1'b0;
                    dirty[snoop_idx] <= 1'b0;
                end
            end

            // S-cache back-invalidation: maintain inclusion property.
            // One S-cache line (128B) covers 4 D-cache lines (32B each).
            // Check all 4 possible D-cache indices in parallel.
            if (backinv_valid) begin
                if (valid[{backinv_addr[12:7], 2'b00}] &&
                    tags[{backinv_addr[12:7], 2'b00}] == backinv_addr[31:13]) begin
                    valid[{backinv_addr[12:7], 2'b00}] <= 1'b0;
                    dirty[{backinv_addr[12:7], 2'b00}] <= 1'b0;
                end
                if (valid[{backinv_addr[12:7], 2'b01}] &&
                    tags[{backinv_addr[12:7], 2'b01}] == backinv_addr[31:13]) begin
                    valid[{backinv_addr[12:7], 2'b01}] <= 1'b0;
                    dirty[{backinv_addr[12:7], 2'b01}] <= 1'b0;
                end
                if (valid[{backinv_addr[12:7], 2'b10}] &&
                    tags[{backinv_addr[12:7], 2'b10}] == backinv_addr[31:13]) begin
                    valid[{backinv_addr[12:7], 2'b10}] <= 1'b0;
                    dirty[{backinv_addr[12:7], 2'b10}] <= 1'b0;
                end
                if (valid[{backinv_addr[12:7], 2'b11}] &&
                    tags[{backinv_addr[12:7], 2'b11}] == backinv_addr[31:13]) begin
                    valid[{backinv_addr[12:7], 2'b11}] <= 1'b0;
                    dirty[{backinv_addr[12:7], 2'b11}] <= 1'b0;
                end
            end
        end
    end

`endif
endmodule
