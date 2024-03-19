/*
 * init.c - common boot-wrapper initialization
 *
 * Copyright (C) 2021 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */
#include <boot.h>
#include <cpu.h>
#include <platform.h>

static void announce_bootwrapper(void)
{
	print_string("Boot-wrapper v0.2\r\n");
}

#define announce_object(object, desc)				\
do {								\
	extern char object##__start[];				\
	extern char object##__end[];				\
	print_string("[");					\
	print_ulong_hex((unsigned long)object##__start);	\
	print_string("..");					\
	print_ulong_hex((unsigned long)object##__end);		\
	print_string("] => " desc "\r\n");			\
} while (0)

static void announce_objects(void)
{
	print_string("Memory layout:\r\n");
	announce_object(text, "boot-wrapper");
	announce_object(mbox, "mbox");
#ifndef DYNAMIC_CONFIG
	announce_object(kernel, "kernel");
#endif
#ifdef XEN
	announce_object(xen, "xen");
#endif
#ifndef DYNAMIC_CONFIG
	announce_object(dtb, "dtb");
#endif
#ifdef USE_INITRD
	announce_object(filesystem, "initrd");
#endif
}

void announce_arch(void);

void cpu_init_bootwrapper(void)
{
	unsigned int cpu = this_cpu_logical_id();

	if (cpu == 0) {
		init_uart();
		announce_bootwrapper();
		announce_arch();
		announce_objects();
		print_string("\r\n");
		init_platform();
	} else {
#if ADI_PLATFORM
		init_platform_secondary();
#endif
	}

	cpu_init_bootmethod(cpu);
}
