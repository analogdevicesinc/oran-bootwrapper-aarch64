/*
 * arch/aarch64/spin.S - spin-table boot protocol implementation
 *
 * Copyright (C) 2013 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */
#include <linkage.h>

#include "common.S"

	.text

ASM_FUNC(cpu_init_bootmethod)
	ret

ASM_FUNC(start_bootmethod)
	cpuid	x0, x1
	bl	find_logical_id

	/*
	 * Primary CPU (x0 = 0) jumps to kernel, the other ones wait for an
	 * address to appear in mbox
	 */
	adr	x3, mbox
	adr	x4, kernel_address
	cmp	x0, #0
	csel	x1, x3, x4, ne
	mov	x2, #0
	bl	first_spin

	.align 3
kernel_address:
	.long 0

	.ltorg
