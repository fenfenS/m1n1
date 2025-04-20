/* spDx-License-Identifier: MIT */

#include "exception.h"

.extern xnu_vbar_el1

.global _vt_m1n1_mmu
.global _vt_phy2virtOff
.global _vt_xnu_init
.global _vt_double_panic
//called after kernel's high mem got mapped in ttbr1_el1
//pc register at low addr identity mapping with ttbr0_el1
//spsel ==0
//sp register at ttbr1_el1's kernel mapping
//Plan:
//set pan,spsel
//save regular registers to current kernel stack
//continue to save ttbr0_el1 tcr_el1 mair_el1 to kernel stack
//set pc to high addr mapping with ttbr1_el1
//set tcr to middle state(which is appropriate for both m1n1's ttbr0_el1 and kernel's ttbr1_el1)
//recover m1n1's ttbr0 
//recover m1n1's mair
//dsb ...
//set pc to lowaddr with m1n1's mapping
//TBD: (set sp to m1n1's stack)
//now we can call any m1n1's function
_vt_double_panic:
    adr x11, xnu_double_panic
    b _vt_m1n1_call
_vt_xnu_sync:
    adr x11, xnu_sync
    b _vt_m1n1_call
_vt_xnu_init:
    adr x11, xnu_init
_vt_m1n1_call: //switch to m1n1 context and call x11
    msr pan, #0
    msr spsel, #1
    sub sp, sp, #(SIZEOF_EXC_INFO - 32 * 8)
    str x30, [sp, #-16]!
    bl _vt_entry
    mrs x1, TTBR0_EL1
    str x1, [sp, #0x1e8]
    mrs x2, TCR_EL1
    str x2, [sp, #0x1e0]
    mrs x3, MAIR_EL1
    str x3, [sp, #0x1d8]
    // the return function is in low mem or high mem, depend on the orginal pc
    // save it for call return later
    adr x3, _vt_ereturn
    adr x6, _vt_return
    str x3, [sp, #0x1f8] //extra void*
    //let pc register go high address
    adr x3, next_insn_in_highmem
    adr x5, _vt_phy2virtOff
    ldr x5, [x5]
    tbnz    x3, #63, next_insn_in_highmem
    //jump if pc is already in high addr
    add x3, x3, x5
    add x11, x11, x5
    str x6, [sp, #0x1f8] //extra void*
    br x3
    
next_insn_in_highmem:
    orr x2, x2, #(1 << 15) 
    and x2, x2, #(~(1 << 14))
    and x2, x2, #(~0xf)
    msr TCR_EL1, x2
    adr x1, _vt_phy2virtOff
    ldr x1, [x1, 8]
    msr TTBR0_EL1, x1

    adr x1, _vt_phy2virtOff
    ldr x1, [x1, 16]
    msr MAIR_EL1, x1

    dsb ishst
    tlbi vmalle1is
    dsb ish
    isb

    adr x1, _vt_m1n1_mmu
    sub x1, x1, x5
    sub x11, x11, x5
    br x1
_vt_m1n1_mmu:
//    b .
    adr lr, after_call
    br x11
after_call:
    ldr x2, [sp, #0x1e0]
    msr TCR_EL1, x2
    ldr x1, [sp, #0x1e8]
    msr TTBR0_EL1, x1
    ldr x3, [sp, #0x1d8]
    msr MAIR_EL1, x3
    ldr x3, [sp, #0x1f8]
    br x3
    .align 4


_vt_phy2virtOff:
    .align 5


.align 11

.globl _vt_vectors_start
.globl _vt_vectors_el1_sp1_sync
_vt_vectors_start:

    /* EL1 with SP_EL0 */
    b _vt_xnu_sync
    b .
    .align 7
    b .
    .align 7
    b .
    .align 7
    b .
    .align 7

    /* EL1 with SP_EL1 */
    b _vt_double_panic
_vt_vectors_el1_sp1_sync:
    b .
    .align 7
    b .
    .align 7
    b .
    .align 7
    b .
    .align 7

    /* EL1/0 64-bit */
    b .
    .align 7
    b .
    .align 7
    b .
    .align 7
    b .
    .align 7

    /* EL1/0 32-bit */
    b .
    .align 7
    b .
    .align 7
    b .
    .align 7
    b .
    .align 7

.globl _vt_entry
.type _vt_entry, @function
_vt_entry:
    stp x28, x29, [sp, #-16]!
    stp x26, x27, [sp, #-16]!
    stp x24, x25, [sp, #-16]!
    stp x22, x23, [sp, #-16]!
    stp x20, x21, [sp, #-16]!
    stp x18, x19, [sp, #-16]!
    stp x16, x17, [sp, #-16]!
    stp x14, x15, [sp, #-16]!
    stp x12, x13, [sp, #-16]!
    stp x10, x11, [sp, #-16]!
    stp x8, x9, [sp, #-16]!
    stp x6, x7, [sp, #-16]!
    stp x4, x5, [sp, #-16]!
    stp x2, x3, [sp, #-16]!
    stp x0, x1, [sp, #-16]!

    dsb sy
    isb

    mov x0, sp
    ret

.globl _vt_return
.type _vt_return, @function
_vt_return:
    ldp x0, x1, [sp], #16
    ldp x2, x3, [sp], #16
    ldp x4, x5, [sp], #16
    ldp x6, x7, [sp], #16
    ldp x8, x9, [sp], #16
    ldp x10, x11, [sp], #16
    ldp x12, x13, [sp], #16
    ldp x14, x15, [sp], #16
    ldp x16, x17, [sp], #16
    ldp x18, x19, [sp], #16
    ldp x20, x21, [sp], #16
    ldp x22, x23, [sp], #16
    ldp x24, x25, [sp], #16
    ldp x26, x27, [sp], #16
    ldp x28, x29, [sp], #16
    ldr x30, [sp], #16

    add sp, sp, #(SIZEOF_EXC_INFO - 32 * 8)

    msr spsel, #0
    ret

.globl _vt_ereturn
.type _vt_ereturn, @function
_vt_ereturn:
    ldp x0, x1, [sp], #16
    ldp x2, x3, [sp], #16
    ldp x4, x5, [sp], #16
    ldp x6, x7, [sp], #16
    ldp x8, x9, [sp], #16
    ldp x10, x11, [sp], #16
    ldp x12, x13, [sp], #16
    ldp x14, x15, [sp], #16
    ldp x16, x17, [sp], #16
    ldp x18, x19, [sp], #16
    ldp x20, x21, [sp], #16
    ldp x22, x23, [sp], #16
    ldp x24, x25, [sp], #16
    ldp x26, x27, [sp], #16
    ldp x28, x29, [sp], #16
    ldr x30, [sp], #16

    add sp, sp, #(SIZEOF_EXC_INFO - 32 * 8)

    msr spsel, #0
    eret

//what a mess

.globl _vt_redirect_sync
.type _vt_redirect_sync, @function
_vt_redirect_sync:
    b .



.globl vt_exc_sync
.type vt_exc_sync, @function
vt_exc_sync:
//should save ttbr0_el1; switch ttbr0_el1; and call xnu_sync
//and can't use bss and x0 here
    b .
//    cbz x1, call_xnu_sync
// forced mmu already
    mrs x1, TTBR0_EL1
//    str x1, [sp, #0x1f8]//extra void*
    str x1, [sp, #0x1e8]
    mrs x2, TCR_EL1
    str x2, [sp, #0x1e0]

    mrs x3, SCTLR_EL1

    adr x1, 0x18
    ldr x1, [x1]
    ldr x2, [x1, #8]
    msr TCR_EL1, x2
    msr TTBR0_EL1, x1
    bl xnu_sync

.globl _vt_redirect
.type _vt_redirect, @function
_vt_redirect:
    stp x0, x1, [sp, #-16]!
    ldr x0, =xnu_vbar_el1
    ldp x0, x1, [sp], #16
//not in used now


.globl _vt_sync
.type _vt_sync, @function
_vt_sync:
    b .
    msr pan, #0
    mrs x2, SCTLR_EL1
    tbnz x2, #0, after_enable_mmu
    orr x2, x2, 1 //SCTLR_M and x2 is safe to corrupt
    msr SCTLR_EL1, x2
after_enable_mmu:
    sub sp, sp, #(SIZEOF_EXC_INFO - 32 * 8)
    str x30, [sp, #-16]!
    bl _vt_entry
    bl vt_exc_sync

    b _vt_return

.globl _v_vt_irq
.type _v_vt_irq, @function
_v_vt_irq:
    msr pan, #0
    sub sp, sp, #(SIZEOF_EXC_INFO - 32 * 8)
    str x30, [sp, #-16]!
    bl _vt_entry
    bl hv_exc_irq

    b _vt_return

.globl _v_vt_fiq
.type _v_vt_fiq, @function
_v_vt_fiq:
    msr pan, #0
    sub sp, sp, #(SIZEOF_EXC_INFO - 32 * 8)
    str x30, [sp, #-16]!
    bl _vt_entry
    bl hv_exc_fiq

    b _vt_return

.globl _v_vt_serr
.type _v_vt_serr, @function
_v_vt_serr:
    msr pan, #0
    sub sp, sp, #(SIZEOF_EXC_INFO - 32 * 8)
    str x30, [sp, #-16]!
    bl _vt_entry
    bl hv_exc_serr

    b _vt_return


