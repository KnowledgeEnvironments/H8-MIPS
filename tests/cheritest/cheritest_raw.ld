/* Linker script for cheritest raw tests on R4000SC
   Code at reset vector: kseg1 0xBFC00000 (phys 0x1FC00000)
   Raw tests define start: directly — no init.s needed */

OUTPUT_FORMAT("elf32-tradbigmips")
OUTPUT_ARCH(mips)
ENTRY(start)

SECTIONS
{
    . = 0xBFC00000;

    .text : SUBALIGN(4) {
        *(.text)
        *(.text.*)
    }

    .rodata : {
        *(.rodata)
        *(.rodata.*)
    }

    .data : {
        *(.data)
        *(.data.*)
    }

    /DISCARD/ : {
        *(.reginfo)
        *(.MIPS.abiflags)
        *(.comment)
        *(.note.*)
    }
}
