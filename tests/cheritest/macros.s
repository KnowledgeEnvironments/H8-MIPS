# macros.s — Minimal stub for running cheritest raw tests on R4000SC
# Replaces the full BERI cheritest macros.s framework.
# Only provides what the "test_raw_*" tests actually need.

# Most raw tests just include macros.s but don't use any macros.
# Some non-raw tests use BEGIN_TEST / END_TEST — provide stubs.

# The cheritest framework expects a `test` function called by init.s.
# For raw tests, `start:` is the entry point — we alias _start to it.

.macro mips_function_entry extra_stack_space=0
	daddiu	$sp, $sp, -(\extra_stack_space + 16)
	sd	$ra, (\extra_stack_space + 8)($sp)
	sd	$fp, (\extra_stack_space)($sp)
	daddiu	$fp, $sp, (\extra_stack_space + 16)
.endm

.macro mips_function_return extra_stack_space=0
	ld	$ra, (\extra_stack_space + 8)($sp)
	ld	$fp, (\extra_stack_space)($sp)
	jr	$ra
	daddiu	$sp, $sp, (\extra_stack_space + 16)
.endm

.macro __SET_DEFAULT_TEST_ASM_OPTS
	.set mips64
	.set noreorder
	.set nobopt
.endm

.macro BEGIN_TEST_WITH_CUSTOM_TRAP_HANDLER extra_stack_space=0
	__SET_DEFAULT_TEST_ASM_OPTS
	.text
	.global test
	.ent test
	test:
		mips_function_entry \extra_stack_space
.endm

.macro END_TEST extra_stack_space=0
	mips_function_return \extra_stack_space
	.end test
.endm

# Counting trap handler stub — for framework tests that use BEGIN_TEST.
# Our exception handler in RTL will skip the faulting instruction and increment $v0.
.macro DEFINE_COUNTING_CHERI_TRAP_HANDLER name=counting_trap_handler, trap_count_reg=$v0
.text
.global \name
.ent \name
\name:
	.set push
	.set noat
	.set noreorder
	# Increment exception count
	daddiu	\trap_count_reg, \trap_count_reg, 1
	# Read EPC
	dmfc0	$k0, $14
	# Check if in branch delay slot
	mfc0	$k1, $13		# Cause
	bgez	$k1, 1f			# BD bit is sign bit (bit 31)
	nop
	# In delay slot: EPC points to branch, skip branch + delay slot
	daddiu	$k0, $k0, 8
	b	2f
	nop
1:
	# Not in delay slot: skip one instruction
	daddiu	$k0, $k0, 4
2:
	# Store compressed trap info in $k1
	# Bits 16-31: low 16 bits of Cause
	mfc0	$k1, $13
	andi	$k1, $k1, 0xffff
	dsll	$k1, $k1, 16
	# Bits 32-63: exception count
	dsll	$at, \trap_count_reg, 32
	or	$k1, $k1, $at
	# Return
	dmtc0	$k0, $14		# Update EPC
	.set pop
	eret
.end \name
.endm

.macro BEGIN_TEST extra_stack_space=0, trap_count_reg=$v0
.text
	__SET_DEFAULT_TEST_ASM_OPTS
	DEFINE_COUNTING_CHERI_TRAP_HANDLER default_trap_handler, trap_count_reg=\trap_count_reg

	BEGIN_TEST_WITH_CUSTOM_TRAP_HANDLER \extra_stack_space
		dli $v0, 0
		dli $k1, 0
.endm

.macro BEGIN_TEST_WITH_OLD_EXCEPTION_HANDLER extra_stack_space=0
	BEGIN_TEST \extra_stack_space
.endm

.macro clear_counting_exception_handler_regs
	dli $k0, 0
	dli $k1, 0
.endm

.macro check_instruction_traps compressed_info, insn:vararg
	dli \compressed_info, 0
	clear_counting_exception_handler_regs
	\insn
	move \compressed_info, $k1
	clear_counting_exception_handler_regs
.endm

# ssnop = SLL $0, $0, 1 (super-scalar NOP)
.macro ssnop
	sll $0, $0, 1
.endm

.macro DO_ERET
	ssnop
	ssnop
	ssnop
	ssnop
	eret
.endm

# init_tlb — Initialize all 48 TLB entries to invalid
# Clobbers t0, t1, t2
.macro init_tlb
	dmtc0	$zero, $5		# PageMask = 0
	dmtc0	$zero, $2		# EntryLo0 = 0 (V=0)
	dmtc0	$zero, $3		# EntryLo1 = 0 (V=0)

	dli	$t0, 47			# R4000 has 48 TLB entries (0-47)
	dli	$t1, 0x3fffffff80000000	# KSEG0 start with region bits clear
1:
	dsll	$t2, $t0, 13		# TLB index -> unique VPN2
	dadd	$t2, $t1, $t2
	dmtc0	$t2, $10		# EntryHi = unique address
	mtc0	$t0, $0			# Index
	ssnop
	ssnop
	ssnop
	ssnop
	tlbwi

	daddi	$t0, $t0, -1
	bgez	$t0, 1b
	nop

	ssnop
	ssnop
	ssnop
	ssnop
.endm

# install_tlb_entry — Write a single TLB entry
# Assumes EntryHi, EntryLo0, EntryLo1, PageMask, and Index already set
# collect_compressed_trap_info — Simplified version for R4000 (no CHERI)
# Sets compressed_info_reg to Cause bits (shifted) and increments trap_count_reg
.macro collect_compressed_trap_info compressed_info_reg=$k1, tmp_reg=$k0, trap_count_reg=$v0
	daddiu	\trap_count_reg, \trap_count_reg, 1
	dmfc0	\tmp_reg, $13			# Cause
	dsll	\tmp_reg, \tmp_reg, 32
	dsrl	\compressed_info_reg, \tmp_reg, 16	# Bits 16-47 = low 32 Cause bits
	# Bits 32-63: exception count
	dsll	\tmp_reg, \trap_count_reg, 32
	or	\compressed_info_reg, \compressed_info_reg, \tmp_reg
.endm

# BEGIN_CUSTOM_TRAP_HANDLER / END_CUSTOM_TRAP_HANDLER
.macro BEGIN_CUSTOM_TRAP_HANDLER
	.global default_trap_handler
	.ent default_trap_handler
	default_trap_handler:
.endm

.macro END_CUSTOM_TRAP_HANDLER
	.end default_trap_handler
.endm

.macro install_tlb_entry
	ssnop
	ssnop
	ssnop
	ssnop
	tlbwi
	ssnop
	ssnop
	ssnop
	ssnop
.endm

# jump_to_usermode — Switch from kernel to user mode and jump to label
#
# Sets up a TLB entry mapping the physical page of the label to kuseg
# virtual address 0, then enters user mode via ERET.
#
# The label must be in kseg0/kseg1 (so we can derive the physical address).
# Uses $k0/$k1 as scratch (standard for exception/privilege code).
# Preserves all other registers.
.macro jump_to_usermode label
	.set push
	.set noat
	.set noreorder

	# Get physical address of label: phys = vaddr & 0x1FFFFFFF
	dla	$k0, \label
	dli	$k1, 0x1FFFFFFF
	and	$k0, $k0, $k1		# k0 = physical address

	# Save physical address on stack for later use
	daddiu	$sp, $sp, -16
	sd	$k0, 0($sp)

	# Set up TLB entry 0: map physical page pair to kuseg virtual address 0
	dmtc0	$zero, $5		# PageMask = 0 (4KB pages)
	dmtc0	$zero, $10		# EntryHi = VPN2=0, ASID=0

	# Compute EntryLo0: even page of the pair
	# PFN = phys_page_pair_base >> 6 (bits [29:6] of EntryLo)
	dli	$k1, ~0x1FFF
	and	$k0, $k0, $k1		# page pair base (clear low 13 bits)
	dsrl	$k0, $k0, 6		# PFN in EntryLo position
	ori	$k0, $k0, 0x17		# C=2(uncached), D=1(writable), V=1, G=1
	dmtc0	$k0, $2			# EntryLo0 = even page
	daddiu	$k1, $k0, 0x40		# next PFN (+1 page)
	dmtc0	$k1, $3			# EntryLo1 = odd page
	mtc0	$zero, $0		# Index = 0
	ssnop
	ssnop
	ssnop
	ssnop
	tlbwi
	ssnop
	ssnop

	# Compute kuseg virtual address = offset within page pair
	ld	$k0, 0($sp)		# restore physical address
	daddiu	$sp, $sp, 16
	andi	$k0, $k0, 0x1FFF	# offset within 8KB pair = kuseg vaddr

	# Set EPC to kuseg virtual address
	dmtc0	$k0, $14		# EPC = user-mode address

	# Set Status: KSU=2 (user mode), EXL=1 (so we stay kernel until ERET)
	# ERET will clear EXL and jump to EPC in user mode
	mfc0	$k0, $12		# Read Status
	# Clear KSU[4:3], EXL[1], ERL[2]
	li	$k1, ~0x1E
	and	$k0, $k0, $k1
	ori	$k0, $k0, 0x12		# KSU=2 (bit4=1,bit3=0) | EXL=1 (bit1=1)
	mtc0	$k0, $12		# Write Status
	ssnop
	ssnop
	ssnop
	ssnop
	eret				# Clears EXL, jumps to EPC in user mode

	.set pop
.endm

# __set_counting_trap_handler_count — Reset the exception counter
.macro __set_counting_trap_handler_count reg
	move	$v0, \reg
.endm

# __get_counting_trap_handler_count — Read the exception counter
.macro __get_counting_trap_handler_count reg
	move	\reg, $v0
.endm
