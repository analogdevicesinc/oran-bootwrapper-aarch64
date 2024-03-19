/*
 * model.lds.S - simple linker script for stand-alone Linux booting
 *
 * Copyright (C) 2012 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */

#ifdef ADI_PLATFORM
#include "include/adrv906x_def.h"
#endif

#ifdef BOOTWRAPPER_32
OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
#else
OUTPUT_FORMAT("elf64-littleaarch64")
OUTPUT_ARCH(aarch64)
#endif
TARGET(binary)

ENTRY(_start)

SECTIONS
{
	bootargs_start = BOOT_ARGS_BASE;

	.boot (BOOT_BASE): {
		text__start = .;
		*(.init)
		*(.text*)
		*(.data* .rodata* .bss* COMMON)
		*(.vectors)
		*(.stack)
		PROVIDE(etext = .);
		text__end = .;
	}

	.mbox (BOOT_BASE + MBOX_OFFSET): {
		mbox__start = .;
		mbox = .;
		QUAD(0x0)
		mbox__end = .;
	}

	ASSERT(etext <= (BOOT_BASE + TEXT_LIMIT), ".text overflow!")
}
