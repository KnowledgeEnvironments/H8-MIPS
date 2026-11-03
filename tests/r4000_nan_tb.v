// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
`default_nettype none

// Exact R4000 legacy-NaN contract. Unlike the broad TestFloat adapter, this
// test compares sign, payload, operand precedence, quieting, and Invalid flags.
module r4000_nan_tb;
    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n = 0;
    reg wen_early = 0;
    reg [4:0] fd_wr_early = 0;
    reg [63:0] fpr_wdata_early = 0;
    reg fpr_wdata_early_word = 0;
    reg [4:0] fs = 0;
    reg [4:0] ft = 1;
    reg [4:0] fmt = 0;
    reg [5:0] funct = 0;
    reg do_fpop = 0;

    wire [63:0] fp_result;
    wire fp_exc;
    wire fp_exc_unimpl;
    wire [4:0] fp_exc_flags;
    wire fpu_busy;
    wire fpu_result_valid;

    localparam [4:0] FMT_S = 5'h10;
    localparam [4:0] FMT_D = 5'h11;
    localparam [5:0] F_ADD = 6'h00;
    localparam [5:0] F_ABS = 6'h05;
    localparam [5:0] F_MOV = 6'h06;
    localparam [5:0] F_NEG = 6'h07;
    localparam [5:0] F_CVT_D = 6'h21;

    r4000_fpu dut (
        .clk(clk), .rst_n(rst_n),
        .wen(1'b0), .fcsr_wen(1'b0),
        .wen_early(wen_early), .fd_wr_early(fd_wr_early),
        .fpr_wdata_early(fpr_wdata_early),
        .fpr_wdata_early_word(fpr_wdata_early_word),
        .flags_wen_early(1'b0), .fcsr_wen_early(1'b0),
        .fcsr_wdata_early(32'd0), .exc_trap_early(1'b0),
        .fr_mode(1'b1), .fr_mode_wr(1'b1),
        .fs(fs), .ft(ft), .fd(5'd2), .fr(5'd3),
        .gpr_wdata(64'd0), .fd_wr(5'd0), .fpr_wdata(64'd0),
        .fpr_wdata_word(1'b0), .fcsr_wdata(32'd0),
        .fmt(fmt), .funct(funct), .do_fpop(do_fpop),
        .gpr_rt(5'd0), .gpr_rt_zero(1'b1), .madd_op(3'd0),
        .fp_result(fp_result), .fp_exc(fp_exc),
        .fp_exc_unimpl(fp_exc_unimpl), .fp_exc_flags(fp_exc_flags),
        .fpu_busy(fpu_busy), .fpu_result_valid(fpu_result_valid),
        .fpu_cancel(1'b0)
    );

    task write_fpr;
        input [4:0] index;
        input [63:0] value;
        input word_write;
        begin
            @(negedge clk);
            fd_wr_early = index;
            fpr_wdata_early = value;
            fpr_wdata_early_word = word_write;
            wen_early = 1;
            @(posedge clk);
            #1;
            wen_early = 0;
            fpr_wdata_early_word = 0;
        end
    endtask

    task launch_and_wait;
        input [4:0] op_fmt;
        input [5:0] op_funct;
        integer cycles;
        begin
            @(negedge clk);
            fmt = op_fmt;
            funct = op_funct;
            do_fpop = 1;
            cycles = 0;
            while (!fpu_result_valid && cycles < 200) begin
                @(negedge clk);
                cycles = cycles + 1;
            end
            if (!fpu_result_valid)
                $fatal(1, "FPU operation timed out fmt=%h funct=%h", op_fmt, op_funct);
            #1;
        end
    endtask

    task finish_multicycle;
        begin
            do_fpop = 0;
            @(negedge clk);
        end
    endtask

    task expect_s;
        input [31:0] expected;
        input [4:0] expected_flags;
        input [255:0] name;
        begin
            if (fp_result[31:0] !== expected || fp_exc_flags !== expected_flags ||
                fp_exc_unimpl !== 1'b0) begin
                $display("FAIL %0s got=%08h flags=%02h unimpl=%b expected=%08h flags=%02h",
                         name, fp_result[31:0], fp_exc_flags, fp_exc_unimpl,
                         expected, expected_flags);
                $fatal(1);
            end
        end
    endtask

    task expect_d;
        input [63:0] expected;
        input [4:0] expected_flags;
        input [255:0] name;
        begin
            if (fp_result !== expected || fp_exc_flags !== expected_flags ||
                fp_exc_unimpl !== 1'b0) begin
                $display("FAIL %0s got=%016h flags=%02h unimpl=%b expected=%016h flags=%02h",
                         name, fp_result, fp_exc_flags, fp_exc_unimpl,
                         expected, expected_flags);
                $fatal(1);
            end
        end
    endtask

    initial begin
        repeat (2) @(negedge clk);
        rst_n = 1;

        write_fpr(0, 64'h00000000ff812345, 1'b1);
        write_fpr(1, 64'h000000007f823456, 1'b1);
        launch_and_wait(FMT_S, F_ADD);
        expect_s(32'hff812345, 5'b00000, "ADD.S first quiet NaN payload");
        finish_multicycle();

        write_fpr(0, 64'h000000007fc12345, 1'b1);
        write_fpr(1, 64'h000000003f800000, 1'b1);
        launch_and_wait(FMT_S, F_ADD);
        expect_s(32'h7f812345, 5'b10000, "ADD.S signaling NaN quieting");
        finish_multicycle();

        // Clearing the sole signaling bit must still leave a NaN, never Inf.
        write_fpr(0, 64'h000000007fc00000, 1'b1);
        launch_and_wait(FMT_S, F_ADD);
        expect_s(32'h7fbfffff, 5'b10000, "ADD.S minimal signaling NaN");
        finish_multicycle();

        write_fpr(0, 64'h7ff8000000000000, 1'b0);
        write_fpr(1, 64'h3ff0000000000000, 1'b0);
        launch_and_wait(FMT_D, F_ADD);
        expect_d(64'h7ff7ffffffffffff, 5'b10000,
                 "ADD.D minimal signaling NaN");
        finish_multicycle();

        write_fpr(0, 64'h000000007f800000, 1'b1);
        write_fpr(1, 64'h00000000ff800000, 1'b1);
        launch_and_wait(FMT_S, F_ADD);
        expect_s(32'h7fbfffff, 5'b10000, "ADD.S infinity cancellation");
        finish_multicycle();

        write_fpr(0, 64'h000000007fc00000, 1'b1);
        launch_and_wait(FMT_S, F_CVT_D);
        expect_d(64'h7ff7ffffffffffff, 5'b10000,
                 "CVT.D.S minimal signaling NaN");
        finish_multicycle();

        // ABS/NEG are arithmetic and any NaN raises Invalid. MOV is not.
        write_fpr(0, 64'h00000000ff812345, 1'b1);
        fmt = FMT_S;
        funct = F_ABS;
        do_fpop = 1;
        #1;
        expect_s(32'h7fbfffff, 5'b10000, "ABS.S quiet NaN invalid");
        funct = F_NEG;
        #1;
        expect_s(32'h7fbfffff, 5'b10000, "NEG.S quiet NaN invalid");
        funct = F_MOV;
        #1;
        expect_s(32'hff812345, 5'b00000, "MOV.S preserves NaN");
        do_fpop = 0;

        $display("PASS exact R4000 legacy NaN contract");
        $finish;
    end

    initial begin
        #20000;
        $fatal(1, "NaN contract test timeout");
    end
endmodule

`default_nettype wire
