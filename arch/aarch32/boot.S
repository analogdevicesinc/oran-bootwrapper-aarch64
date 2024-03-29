/*
 * arch/aarch32/boot.S - simple register setup code for stand-alone Linux booting
 *
 * Copyright (C) 2015 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */

#include <cpu.h>
#include <linkage.h>

#include "common.S"

	.arch_extension sec
	.arch_extension virt

	.section .init

	/*
	 * The boot-wrapper must be entered from the reset vector at the
	 * highest implemented exception level. The boot-wrapper only supports
	 * being entered in the following modes:
	 *
	 * - PL1 / EL3 (Secure) Supervisor mode
	 *   Entering in this mode is strongly recommended.
	 *   PL2 must be implemented.
	 *
	 * - PL2 / EL2 (Non-secure) Hypervisor mode
	 *   Entering in this mode is partially supported.
	 *   PSCI is not supported when entered in this mode.
	 */
ASM_FUNC(_start)
	/* Stack initialisation */
	cpuid	r0, r1
	bl	find_logical_id
	cmp	r0, #MPIDR_INVALID
	beq	err_invalid_id

	bl	setup_stack

	mrs	r0, cpsr
	and	r0, #PSR_MODE_MASK

	cmp	r0, #PSR_HYP
	bne	_switch_monitor

	mov	r0, #1
	ldr	r1, =flag_no_el3
	str	r0, [r1]

	bl	cpu_init_bootwrapper

	b	start_bootmethod

_switch_monitor:
	adr	lr, _monitor
	ldr	r0, =(PSR_A | PSR_I | PSR_F | PSR_MON)
	msr	spsr, r0
	movs	pc, lr

_monitor:
	/* Move the stack to Monitor mode*/
	mrs	sp, sp_svc

	bl	cpu_init_secure_pl1

	bl	cpu_init_bootwrapper

	bl	gic_secure_init

	b	start_bootmethod

err_invalid_id:
	b	.

	.text

	/*
	 * r0: kernel address
	 * r1-r3, sp[0]: kernel arguments
	 */
ASM_FUNC(jump_kernel)
	sub	sp, #4				@ Ignore fourth argument
	push	{r0 - r3}
	mov	r5, sp

	ldr	r0, =HSCTLR_KERNEL
	mcr	p15, 4, r0, c1, c0, 0		@ HSCTLR

	ldr	r0, =SCTLR_KERNEL
	mcr	p15, 0, r0, c1, c0, 0		@ SCTLR

	/* Reset our stack pointer */
	cpuid	r0, r1
	bl	find_logical_id
	bl	setup_stack

	ldr	lr, [r5], #4
	ldm	r5, {r0 - r2}

	ldr	r4, =flag_no_el3
	ldr	r4, [r4]
	cmp	r4, #1
	bxeq	lr				@ no EL3

	ldr	r4, =SPSR_KERNEL
	/* Return in thumb2 mode when bit 0 of address is 1 */
	tst	lr, #1
	orrne	r4, #PSR_T

	msr	spsr_cxf, r4
	movs	pc, lr

	.section .data
	.align 2
flag_no_el3:
	.long 0
