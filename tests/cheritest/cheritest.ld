/* Linker script for cheritest tests on R4000SC
   Code at reset vector: kseg1 0xBFC00000 (phys 0x1FC00000)
   init.s must come first (provides _start at reset vector)
   Exception vector area: 0x80000000-0x80000400 (writable via kseg0)
   Stack: grows down from 0x80100000 */

OUTPUT_FORMAT("elf32-tradbigmips")
OUTPUT_ARCH(mips)
ENTRY(_start)

SECTIONS
{
    . = 0xBFC00000;

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
        *(.comment)
        *(.note.*)
    }
}
