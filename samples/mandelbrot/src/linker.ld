/* https://timmy.moe/blog/barebones-os-zig/ */
OUTPUT_ARCH("riscv")

/* Use _enter as our entry point to our program */
ENTRY(_enter)

MEMORY
{
  /* Source of ORIGIN: https://github.com/qemu/qemu/blob/master/hw/riscv/virt.c#L97
   * Source of LENGTH: QEMU manual has -m default to be 128MiB
   */
  ram (rwxa): ORIGIN = 0x80000000, LENGTH = 128M
}


/* We are only loading from the file using PT_LOAD, we are not using dynamic linking */
PHDRS
{
  ram_fakerom PT_LOAD;
  ram_init PT_LOAD;
  ram PT_LOAD;
}

/* We are putting everything into RAM */
SECTIONS
{
  .text : ALIGN(4K) {
    *(.text.init);
    *(.text);
  } >ram :ram_fakerom

  .rodata : ALIGN(4K) {
    PROVIDE( _global_pointer = . );
    *(.rodata);
  } >ram :ram_fakerom

  .data : ALIGN(4K) {
    *(.data);
  } >ram :ram_init

  .bss : ALIGN(4K) {
    *(.bss);
  } >ram :ram

  PROVIDE( _bss_start = ADDR(.bss) );
  PROVIDE( _bss_end = ADDR(.bss) + SIZEOF(.bss) );

  PROVIDE( _stack_start = _bss_end );
  /* Stack size of 4kB */
  PROVIDE( _stack_end = _stack_start + 0x4000 );
  PROVIDE( _heap_start = _stack_end );
  PROVIDE( _PHYSTOP = ORIGIN(ram) + LENGTH(ram) );
}
