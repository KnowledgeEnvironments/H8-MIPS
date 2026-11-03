// HaterMIPS Project <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// r4000_defs.vh — MIPS III ISA definitions for R4000SC
// Reference: MIPS IV ISA Rev 3.2 (R4000 implements MIPS III subset)

`ifndef R4000_DEFS_VH
`define R4000_DEFS_VH

// Primary opcodes [31:26]
`define OP_SPECIAL  6'h00
`define OP_REGIMM   6'h01
`define OP_J        6'h02
`define OP_JAL      6'h03
`define OP_BEQ      6'h04
`define OP_BNE      6'h05
`define OP_BLEZ     6'h06
`define OP_BGTZ     6'h07
`define OP_ADDI     6'h08
`define OP_ADDIU    6'h09
`define OP_SLTI     6'h0A
`define OP_SLTIU    6'h0B
`define OP_ANDI     6'h0C
`define OP_ORI      6'h0D
`define OP_XORI     6'h0E
`define OP_LUI      6'h0F
`define OP_COP0     6'h10
`define OP_COP1     6'h11
`define OP_COP1X    6'h13
`define OP_BEQL     6'h14
`define OP_BNEL     6'h15
`define OP_BLEZL    6'h16
`define OP_BGTZL    6'h17
`define OP_DADDI    6'h18
`define OP_DADDIU   6'h19
`define OP_LDL      6'h1A
`define OP_LDR      6'h1B
`define OP_LB       6'h20
`define OP_LH       6'h21
`define OP_LWL      6'h22
`define OP_LW       6'h23
`define OP_LBU      6'h24
`define OP_LHU      6'h25
`define OP_LWR      6'h26
`define OP_LWU      6'h27
`define OP_SB       6'h28
`define OP_SH       6'h29
`define OP_SWL      6'h2A
`define OP_SW       6'h2B
`define OP_SDL      6'h2C
`define OP_SDR      6'h2D
`define OP_SWR      6'h2E
`define OP_SPECIAL2 6'h1C
`define OP_CACHE    6'h2F
`define OP_LL       6'h30
`define OP_LWC1     6'h31
`define OP_LLD      6'h34
`define OP_LDC1     6'h35
`define OP_LD       6'h37
`define OP_SC       6'h38
`define OP_SWC1     6'h39
`define OP_SCD      6'h3C
`define OP_SDC1     6'h3D
`define OP_SD       6'h3F

// SPECIAL function codes [5:0] (when opcode = 6'h00)
`define FN_SLL      6'h00
`define FN_MOVCI    6'h01
`define FN_SRL      6'h02
`define FN_SRA      6'h03
`define FN_SLLV     6'h04
`define FN_SRLV     6'h06
`define FN_SRAV     6'h07
`define FN_JR       6'h08
`define FN_JALR     6'h09
`define FN_SYSCALL  6'h0C
`define FN_MOVZ     6'h0A
`define FN_MOVN     6'h0B
`define FN_BREAK    6'h0D
`define FN_SYNC     6'h0F
`define FN_MFHI     6'h10
`define FN_MTHI     6'h11
`define FN_MFLO     6'h12
`define FN_MTLO     6'h13
`define FN_DSLLV    6'h14
`define FN_DSRLV    6'h16
`define FN_DSRAV    6'h17
`define FN_MULT     6'h18
`define FN_MULTU    6'h19
`define FN_DIV      6'h1A
`define FN_DIVU     6'h1B
`define FN_DMULT    6'h1C
`define FN_DMULTU   6'h1D
`define FN_DDIV     6'h1E
`define FN_DDIVU    6'h1F
`define FN_ADD      6'h20
`define FN_ADDU     6'h21
`define FN_SUB      6'h22
`define FN_SUBU     6'h23
`define FN_AND      6'h24
`define FN_OR       6'h25
`define FN_XOR      6'h26
`define FN_NOR      6'h27
`define FN_SLT      6'h2A
`define FN_SLTU     6'h2B
`define FN_DADD     6'h2C
`define FN_DADDU    6'h2D
`define FN_DSUB     6'h2E
`define FN_DSUBU    6'h2F
`define FN_TGE      6'h30
`define FN_TGEU     6'h31
`define FN_TLT      6'h32
`define FN_TLTU     6'h33
`define FN_TEQ      6'h34
`define FN_TNE      6'h36
`define FN_DSLL     6'h38
`define FN_DSRL     6'h3A
`define FN_DSRA     6'h3B
`define FN_DSLL32   6'h3C
`define FN_DSRL32   6'h3E
`define FN_DSRA32   6'h3F

// SPECIAL2 function codes (MIPS32 extensions)
`define FN2_MADD    6'h00
`define FN2_MADDU   6'h01
`define FN2_MUL     6'h02
`define FN2_MSUB    6'h04
`define FN2_MSUBU   6'h05

// REGIMM rt codes [20:16] (when opcode = 6'h01)
`define RT_BLTZ     5'h00
`define RT_BGEZ     5'h01
`define RT_BLTZL    5'h02
`define RT_BGEZL    5'h03
`define RT_TGEI     5'h08
`define RT_TGEIU    5'h09
`define RT_TLTI     5'h0A
`define RT_TLTIU    5'h0B
`define RT_TEQI     5'h0C
`define RT_TNEI     5'h0E
`define RT_BLTZAL   5'h10
`define RT_BGEZAL   5'h11
`define RT_BLTZALL  5'h12
`define RT_BGEZALL  5'h13

// COP0 rs codes [25:21] (when opcode = 6'h10)
`define COP0_MFC0   5'h00
`define COP0_DMFC0  5'h01
`define COP0_MTC0   5'h04
`define COP0_DMTC0  5'h05
`define COP0_CO     5'h10

// COP0 CO function codes [5:0] (when opcode = 6'h10, rs[4] = 1)
`define C0FN_TLBR   6'h01
`define C0FN_TLBWI  6'h02
`define C0FN_TLBWR  6'h06
`define C0FN_TLBP   6'h08
`define C0FN_ERET   6'h18

// ALU operation encoding (internal)
`define ALU_ADD     4'd0
`define ALU_SUB     4'd1
`define ALU_AND     4'd2
`define ALU_OR      4'd3
`define ALU_XOR     4'd4
`define ALU_NOR     4'd5
`define ALU_SLT     4'd6
`define ALU_SLTU    4'd7
`define ALU_SLL     4'd8
`define ALU_SRL     4'd9
`define ALU_SRA     4'd10
`define ALU_PASSB   4'd11

// Writeback source select
`define WB_ALU      3'd0
`define WB_MEM      3'd1
`define WB_LINK     3'd2
`define WB_HI       3'd3
`define WB_LO       3'd4
`define WB_CP0      3'd5
`define WB_LLSC     3'd6
`define WB_MDLO     3'd7

// Memory operation size
`define MEM_BYTE    3'd0
`define MEM_HALF    3'd1
`define MEM_WORD    3'd2
`define MEM_DWORD   3'd3
`define MEM_WORDL   3'd4
`define MEM_WORDR   3'd5
`define MEM_DWORDL  3'd6
`define MEM_DWORDR  3'd7

// Exception codes (Cause register ExcCode field)
`define EXC_INT     5'd0    // Interrupt
`define EXC_MOD     5'd1    // TLB Modified
`define EXC_TLBL    5'd2    // TLB Refill (load/fetch)
`define EXC_TLBS    5'd3    // TLB Refill (store)
`define EXC_ADEL    5'd4    // Address Error (load/fetch)
`define EXC_ADES    5'd5    // Address Error (store)
`define EXC_IBE     5'd6    // Bus Error (instruction)
`define EXC_DBE     5'd7    // Bus Error (data)
`define EXC_SYS     5'd8    // Syscall
`define EXC_BP      5'd9    // Breakpoint
`define EXC_RI      5'd10   // Reserved Instruction
`define EXC_CPU     5'd11   // Coprocessor Unusable
`define EXC_OV      5'd12   // Integer Overflow
`define EXC_TR      5'd13   // Trap
`define EXC_FPE     5'd15   // Floating Point Exception
`define EXC_MCHECK  5'd24   // Machine Check
`define EXC_WATCH   5'd23   // Watch

// CP0 register numbers
`define CP0_INDEX       5'd0
`define CP0_RANDOM      5'd1
`define CP0_ENTRYLO0    5'd2
`define CP0_ENTRYLO1    5'd3
`define CP0_CONTEXT     5'd4
`define CP0_PAGEMASK    5'd5
`define CP0_WIRED       5'd6
`define CP0_BADVADDR    5'd8
`define CP0_COUNT       5'd9
`define CP0_ENTRYHI     5'd10
`define CP0_COMPARE     5'd11
`define CP0_STATUS      5'd12
`define CP0_CAUSE       5'd13
`define CP0_EPC         5'd14
`define CP0_PRID        5'd15
`define CP0_CONFIG      5'd16
`define CP0_LLADDR      5'd17
`define CP0_WATCHLO     5'd18
`define CP0_WATCHHI     5'd19
`define CP0_XCONTEXT    5'd20
`define CP0_ECC         5'd26
`define CP0_CACHEERR    5'd27
`define CP0_TAGLO       5'd28
`define CP0_TAGHI       5'd29
`define CP0_ERROREPC    5'd30

// COP1 rs/fmt codes [25:21] (when opcode = 6'h11)
`define COP1_MFC1   5'h00
`define COP1_DMFC1  5'h01
`define COP1_CFC1   5'h02
`define COP1_MTC1   5'h04
`define COP1_DMTC1  5'h05
`define COP1_CTC1   5'h06
`define COP1_BC1    5'h08   // Branch on FP condition
`define COP1_FMT_S  5'h10   // Single precision
`define COP1_FMT_D  5'h11   // Double precision
`define COP1_FMT_W  5'h14   // Word fixed-point
`define COP1_FMT_L  5'h15   // Long fixed-point

// BC1 rt codes [20:16] (when rs = COP1_BC1)
`define BC1_BCF     5'h00   // BC1F
`define BC1_BCT     5'h01   // BC1T
`define BC1_BCFL    5'h02   // BC1FL (likely)
`define BC1_BCTL    5'h03   // BC1TL (likely)

// COP1 function codes [5:0] (when fmt = S/D)
`define FPU_ADD     6'h00
`define FPU_SUB     6'h01
`define FPU_MUL     6'h02
`define FPU_DIV     6'h03
`define FPU_SQRT    6'h04
`define FPU_ABS     6'h05
`define FPU_MOV     6'h06
`define FPU_NEG     6'h07
`define FPU_ROUND_L 6'h08
`define FPU_TRUNC_L 6'h09
`define FPU_CEIL_L  6'h0A
`define FPU_FLOOR_L 6'h0B
`define FPU_ROUND_W 6'h0C
`define FPU_TRUNC_W 6'h0D
`define FPU_CEIL_W  6'h0E
`define FPU_FLOOR_W 6'h0F
`define FPU_MOVCF   6'h11
`define FPU_MOVZ    6'h12
`define FPU_MOVN    6'h13
`define FPU_CVT_S   6'h20
`define FPU_CVT_D   6'h21
`define FPU_CVT_W   6'h24
`define FPU_CVT_L   6'h25
// C.cond: function codes 6'h30-6'h3F (cond = funct[3:0])
// Condition codes: 0=F, 1=UN, 2=EQ, 3=UEQ, 4=OLT, 5=ULT, 6=OLE, 7=ULE
//                  8=SF, 9=NGLE, 10=SEQ, 11=NGL, 12=LT, 13=NGE, 14=LE, 15=NGT

// FCSR register fields
`define FCSR_RM     1:0     // Rounding mode
`define FCSR_FLAGS  6:2     // Sticky flags (I,U,O,Z,V)
`define FCSR_ENABLE 11:7    // Exception enables
`define FCSR_CAUSE  16:12   // Exception cause
`define FCSR_E      17      // Unimplemented operation
`define FCSR_FCC0   23      // FP condition code 0
`define FCSR_FS     24      // Flush denorms to zero
`define FCSR_FCC7_1 31:25   // FP condition codes 7-1

// Rounding modes
`define RM_RN       2'b00   // Round to Nearest
`define RM_RZ       2'b01   // Round toward Zero
`define RM_RP       2'b10   // Round toward +Infinity
`define RM_RM       2'b11   // Round toward -Infinity

// R4000 FCR0: reserved[31:16]=0, implementation[15:8]=0x05.
// This is an implementation identifier, not a MIPS32 capability map.
// Revision 0 is the model's revision, not a claim about physical silicon.
// Legacy R4000 software uses the implementation field when selecting
// floating-point exception and workaround paths.
`define FIR_R4000   32'h00000500

// PRId value for the R4000SC revision implemented by this core.
// Imp = 0x04 (R4000), Rev = 0x22 (major 2, minor 2).
// Some R4000 software keys errata paths on this revision; a mismatch can
// select incompatible workarounds.
`define PRID_R4000SC    32'h00000422

`endif
