/* Linker script for R4000SC Phase 1 tests
   Code placed at reset vector: kseg1 0xBFC00000 (phys 0x1FC00000)
   Results stored at kseg0 0x80000000 (phys 0x00000000) */

OUTPUT_FORMAT("elf32-bigmips")
OUTPUT_ARCH(mips)
ENTRY(_start)

SECTIONS
{
    . = 0xBFC00000;

    .text : {
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
