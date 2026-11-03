/* Linker script for cheritest raw tests on R4000SC (elf64 / n64 ABI)
   Code at reset vector: kseg1 0xBFC00000 (phys 0x1FC00000) */

OUTPUT_FORMAT("elf64-tradbigmips")
OUTPUT_ARCH(mips)
ENTRY(start)

SECTIONS
{
    . = 0xFFFFFFFFBFC00000;

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
        *(.MIPS.options)
        *(.comment)
        *(.note.*)
    }
}
