// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
`default_nettype none

// R4000 FIR contract: FCR0[31:16] are reserved zero and [15:8] is the
// implementation identifier 0x05.
// Revision [7:0] is deliberately not inferred from the CPU PRId.
module r4000_fir_tb;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0;
    reg fr_mode = 0;
    reg fcsr_wen = 0;
    wire [31:0] fir, fcsr;
    r4000_fpu dut (
        .clk(clk), .rst_n(rst_n), .wen(1'b0), .fcsr_wen(fcsr_wen),
        .wen_early(1'b0), .fd_wr_early(5'd0), .fpr_wdata_early(64'd0),
        .fpr_wdata_early_word(1'b0), .flags_wen_early(1'b0),
        .fcsr_wen_early(1'b0), .fcsr_wdata_early(32'd0), .exc_trap_early(1'b0),
        .fr_mode(fr_mode), .fr_mode_wr(fr_mode),
        .fs(5'd0), .ft(5'd0), .fd(5'd0), .fr(5'd0),
        .gpr_wdata(64'd0), .fd_wr(5'd0), .fpr_wdata(64'd0),
        .fpr_wdata_word(1'b0), .fcsr_wdata(32'h0000000c),
        .fir_rdata(fir), .fcsr_rdata(fcsr),
        .fmt(5'd0), .funct(6'd0), .do_fpop(1'b0),
        .gpr_rt(5'd0), .gpr_rt_zero(1'b1), .madd_op(3'd0), .fpu_cancel(1'b0)
    );
    task check_fir;
        begin
            if ((fir & 32'hffffff00) !== 32'h00000500) begin
                $display("FAIL R4000 FIR=%08h expected reserved-zero/implementation-05", fir);
                $fatal(1);
            end
        end
    endtask
    initial begin
        #1; check_fir();
        repeat (2) @(negedge clk);
        rst_n = 1;
        @(negedge clk); check_fir();
        fr_mode = 1;
        fcsr_wen = 1;
        @(negedge clk);
        fcsr_wen = 0;
        #1; check_fir();
        if (fcsr !== 32'h0000000c) $fatal(1, "FCSR write control failed");
        rst_n = 0;
        @(negedge clk); check_fir();
        $display("PASS R4000 FIR implementation/reserved fields across reset, FR, FCSR write");
        $finish;
    end
    initial begin
        #1000; $fatal(1, "FIR test timeout");
    end
endmodule
`default_nettype wire
