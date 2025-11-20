/* Specifying the firmware's entry point */
ENTRY(Reset_Handler)

/* Available Memory */
MEMORY {
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 1024K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* Specifying heap and stack sizes */

__max_heap_size  = 0x200;
__max_stack_size = 0x400;

/* Defining output sections */

SECTIONS {
    .isr_vector_tbl :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector_tbl))
        KEEP(*(.isr_vector_tbl.*))
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.rodata)
        . = ALIGN(4);
     _etext = .;
    } >FLASH

    .data :
    {
        . = ALIGN(4);
     _sdata = .;  
        *(.data)
        . = ALIGN(4);
     _edata = .;
    } >SRAM AT>FLASH

    .bss : 
    {
        . = ALIGN(4);
     _sbss = .;
        *(.bss)
        . = ALIGN(4);
     _ebss = .;
    } > SRAM
}