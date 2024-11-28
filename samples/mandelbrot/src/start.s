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

    # Clear the bss section; it is expected to be zero
    la      t5, _bss_start
    la      t6, _bss_end
bss_clear:
    #sd      zero, (t5)
    #addi    t5, t5, 8
    #bltu    t5, t6, bss_clear
3:
    # interrupts later
    # la      t1, kmain
    # csrw    mepc, t1
    tail kmain

wait_for_interrupt:
    wfi
    j wait_for_interrupt

