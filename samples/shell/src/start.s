# https://timmy.moe/blog/barebones-os-zig/
.section .text.init

.global _start
_start:
    # Only make the guaranteed hardware thread (hart) of id 0 do bootstrapping
    # while the rest just wait for interrupts
    csrr        t0, mhartid
    bnez        t0, wait_for_interrupt

    # Don't do any address translation or protection
    csrw        satp, zero

# https://www.sifive.com/blog/all-aboard-part-3-linker-relaxation-in-riscv-toolchain
.option push
.option norelax
    la      gp, _global_pointer
.option pop
    
    # Set up the stack
    la      sp, _stack_end

    # zero-init bss section
    la a0, _bss_start
    la a1, _bss_end
    bge a0, a1, end_init_bss
loop_init_bss:
    sw zero, 0(a0)
    addi a0, a0, 4
    blt a0, a1, loop_init_bss
end_init_bss:

    tail kmain

wait_for_interrupt:
    wfi
    j wait_for_interrupt

