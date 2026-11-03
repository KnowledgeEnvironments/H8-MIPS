# init.s — Minimal startup for running cheritest framework tests on R4000SC
# Entry at reset vector, sets up stack, installs exception handler,
# calls test(), then halts with infinite loop.

.set mips64
.set noreorder
.set nobopt
.set noat

.text
.global _start
.global start
.ent _start
_start:
start:
	# Set up stack in kseg0 (physical 0x00100000 = 1MB)
	dli	$sp, 0xFFFFFFFF80100000
	move	$fp, $sp

	# Clear BEV bit so exceptions go to 0x80000180
	mfc0	$t0, $12		# Status
	li	$t1, ~(1 << 22)	# Clear BEV (bit 22)
	and	$t0, $t0, $t1
	# Set IE (interrupts enabled), clear EXL/ERL
	li	$t1, ~((1 << 1) | (1 << 2))  # Clear EXL, ERL
	and	$t0, $t0, $t1
	ori	$t0, $t0, 1		# Set IE
	# Set KX, SX, UX (bits 7, 6, 5) for 64-bit addressing
	ori	$t0, $t0, 0xe0
	# Enable CP1 (CU1 = bit 29)
	lui	$t1, 0x2000
	or	$t0, $t0, $t1
	mtc0	$t0, $12		# Status

	# Install exception handler at 0x80000180 (via kseg1 to bypass D-cache)
	# Copy the default_trap_handler to the exception vector
	dla	$t0, default_trap_handler
	dli	$t1, 0xFFFFFFFFA0000180
	# Copy 32 instructions (128 bytes) — enough for our handler
	li	$t2, 32
1:
	lw	$t3, 0($t0)
	sw	$t3, 0($t1)
	daddiu	$t0, $t0, 4
	daddiu	$t1, $t1, 4
	addiu	$t2, $t2, -1
	bnez	$t2, 1b
	nop

	# Initialize registers to known patterns (like BERI init.s)
	dli	$1,  0x0101010101010101
	# $2 ($v0) and $3 ($v1) zeroed — $v0 used as trap counter by test framework
	move	$2,  $zero
	move	$3,  $zero
	# $4-$15 ($a0-$t3) are test outputs, leave at 0
	move	$4,  $zero
	move	$5,  $zero
	move	$6,  $zero
	move	$7,  $zero
	move	$8,  $zero
	move	$9,  $zero
	move	$10, $zero
	move	$11, $zero
	move	$12, $zero
	move	$13, $zero
	move	$14, $zero
	move	$15, $zero
	# $16-$23 ($s0-$s7) are test outputs, leave at 0
	move	$16, $zero
	move	$17, $zero
	move	$18, $zero
	move	$19, $zero
	move	$20, $zero
	move	$21, $zero
	move	$22, $zero
	move	$23, $zero
	# $24-$25 ($t8,$t9) leave at 0
	move	$24, $zero
	move	$25, $zero
	# $26-$27 ($k0,$k1) reserved for exception handler
	# $28 ($gp), $29 ($sp) already set, $30 ($fp) already set
	# $31 ($ra) set by jal

	# Call test function
	dla	$25, test
	jalr	$25
	nop

	# Halt — infinite loop
.global _halt
_halt:
	b	_halt
	nop

.end _start

# ---------------------------------------------------------------
# bev_clear — Clear BEV bit in Status register
# Called by tests that install their own exception handler
# ---------------------------------------------------------------
.global bev_clear
.ent bev_clear
bev_clear:
	mfc0	$t0, $12		# Status
	li	$t1, ~(1 << 22)		# Clear BEV
	and	$t0, $t0, $t1
	mtc0	$t0, $12
	jr	$ra
	nop
.end bev_clear

# ---------------------------------------------------------------
# bev0_handler_install — Install exception handler at 0x80000180
# $a0 = pointer to handler function to install
# Writes a trampoline (LUI/ORI/JR/NOP) that jumps to the handler
# at its original link address. This avoids broken PC-relative
# branches when handlers reference external labels.
# ---------------------------------------------------------------
.global bev0_handler_install
.ent bev0_handler_install
bev0_handler_install:
	dli	$t1, 0xFFFFFFFFA0000180
	# LUI $k0, upper16(handler)  — opcode 0x3C1A
	srl	$t2, $a0, 16
	andi	$t2, $t2, 0xFFFF
	lui	$t3, 0x3C1A
	or	$t3, $t3, $t2
	sw	$t3, 0($t1)
	# ORI $k0, $k0, lower16(handler)  — opcode 0x375A
	andi	$t2, $a0, 0xFFFF
	lui	$t3, 0x375A
	or	$t3, $t3, $t2
	sw	$t3, 4($t1)
	# JR $k0  — 0x03400008
	lui	$t3, 0x0340
	ori	$t3, $t3, 0x0008
	sw	$t3, 8($t1)
	# NOP (delay slot)
	sw	$0, 12($t1)
	jr	$ra
	nop
.end bev0_handler_install

# ---------------------------------------------------------------
# bev0_handler_install_tlbrefill — Install TLB Refill handler at 0x80000000
# $a0 = pointer to handler function to install
# Writes a trampoline that jumps to the handler at its link address
# ---------------------------------------------------------------
.global bev0_handler_install_tlbrefill
.ent bev0_handler_install_tlbrefill
bev0_handler_install_tlbrefill:
	dli	$t1, 0xFFFFFFFFA0000000
	srl	$t2, $a0, 16
	andi	$t2, $t2, 0xFFFF
	lui	$t3, 0x3C1A
	or	$t3, $t3, $t2
	sw	$t3, 0($t1)
	andi	$t2, $a0, 0xFFFF
	lui	$t3, 0x375A
	or	$t3, $t3, $t2
	sw	$t3, 4($t1)
	lui	$t3, 0x0340
	ori	$t3, $t3, 0x0008
	sw	$t3, 8($t1)
	sw	$0, 12($t1)
	jr	$ra
	nop
.end bev0_handler_install_tlbrefill

# ---------------------------------------------------------------
# install_tlb_entry — Write a single TLB entry
# $a0 = TLB index
# $a1 = physical base address
# $a2 = virtual base address (written to EntryHi)
# $a3 = page mask
# Sets up both EntryLo0 and EntryLo1 with sequential physical pages,
# V=1, G=1, D=0, C=010 (uncached).
# ---------------------------------------------------------------
.global install_tlb_entry
.ent install_tlb_entry
install_tlb_entry:
	dmtc0	$a3, $5			# PageMask
	dmtc0	$a0, $0			# Index
	dmtc0	$a2, $10		# EntryHi
	dli	$at, 0xfffffff000	# PFN mask
	and	$t1, $a1, $at
	dsrl	$t2, $t1, 6		# PFN in EntryLo position
	ori	$t2, $t2, 0x13		# V=1, G=1, C=010 (uncached)
	dmtc0	$t2, $2			# EntryLo0
	daddu	$t3, $t2, 0x40		# Next page PFN for EntryLo1
	dmtc0	$t3, $3			# EntryLo1
	ssnop
	ssnop
	ssnop
	ssnop
	tlbwi
	ssnop
	ssnop
	ssnop
	ssnop
	jr	$ra
	nop
.end install_tlb_entry

# ---------------------------------------------------------------
# set_bev0_xtlb_handler — Install XTLB Miss handler at 0x80000080
# $a0 = pointer to handler function to install
# Writes a trampoline that jumps to the handler at its link address
# ---------------------------------------------------------------
.global set_bev0_xtlb_handler
.ent set_bev0_xtlb_handler
set_bev0_xtlb_handler:
	dli	$t1, 0xFFFFFFFFA0000080
	srl	$t2, $a0, 16
	andi	$t2, $t2, 0xFFFF
	lui	$t3, 0x3C1A
	or	$t3, $t3, $t2
	sw	$t3, 0($t1)
	andi	$t2, $a0, 0xFFFF
	lui	$t3, 0x375A
	or	$t3, $t3, $t2
	sw	$t3, 4($t1)
	lui	$t3, 0x0340
	ori	$t3, $t3, 0x0008
	sw	$t3, 8($t1)
	sw	$0, 12($t1)
	jr	$ra
	nop
.end set_bev0_xtlb_handler

# ---------------------------------------------------------------
# bev1_handler_install — Install exception handler at BEV=1 vector
# $a0 = pointer to handler function to install
# BEV=1 general exception vector: 0xBFC00380
# Writes a trampoline that jumps to the handler at its link address
# ---------------------------------------------------------------
.global bev1_handler_install
.ent bev1_handler_install
bev1_handler_install:
	dli	$t1, 0xFFFFFFFFBFC00380
	srl	$t2, $a0, 16
	andi	$t2, $t2, 0xFFFF
	lui	$t3, 0x3C1A
	or	$t3, $t3, $t2
	sw	$t3, 0($t1)
	andi	$t2, $a0, 0xFFFF
	lui	$t3, 0x375A
	or	$t3, $t3, $t2
	sw	$t3, 4($t1)
	lui	$t3, 0x0340
	ori	$t3, $t3, 0x0008
	sw	$t3, 8($t1)
	sw	$0, 12($t1)
	jr	$ra
	nop
.end bev1_handler_install
