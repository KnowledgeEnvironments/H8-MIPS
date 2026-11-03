/* Linker script for cheritest framework tests on R4000SC (elf64 / n64 ABI)
   Code at reset vector: kseg1 0xBFC00000 (phys 0x1FC00000)
   init.s must come first (provides _start at reset vector) */

OUTPUT_FORMAT("elf64-tradbigmips")
OUTPUT_ARCH(mips)
ENTRY(_start)

SECTIONS
{
    . = 0xFFFFFFFFBFC00000;

    .text : SUBALIGN(4) {
        *init.o(.text)
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

    .bss : {
        *(.bss)
        *(.bss.*)
    }

    end = .;
    PROVIDE(__kernel_load_offset__ = 0x0);

    /DISCARD/ : {
        *(.reginfo)
        *(.MIPS.abiflags)
        *(.MIPS.options)
        *(.comment)
        *(.note.*)
    }
}
