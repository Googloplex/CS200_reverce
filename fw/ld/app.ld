/* LCM32F037: Flash 32 КБ @0x08000000, SRAM 4 КБ @0x20000000.
 * Приложение живёт с 0x08001000 (после загрузчика Kpower) и до 0x08007A00 (дальше конфигурация v1.5, с 0x08007C00 журнал). */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08001000, LENGTH = 0x6A00
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x1000
}

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);
_min_stack = 0x300;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        KEEP(*(.noinit*))
        . = ALIGN(4);
    } > RAM

    ._stack_check :
    {
        . = ALIGN(4);
        . = . + _min_stack;
        . = ALIGN(4);
    } > RAM

    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.eh_frame*) }
}
