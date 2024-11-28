# https://github.com/cnlohr/mini-rv32ima/tree/master/baremetal
.align 4
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

	la	sp, _sstack
	addi	sp,sp,-16
	sw	ra,12(sp)
	jal	ra, kmain

wait_for_interrupt:
    wfi
    j wait_for_interrupt

