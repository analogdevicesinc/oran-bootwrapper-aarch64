/*
 * include/cpu.h - Generic CPU features
 *
 * Copyright (C) 2015 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */
#ifndef __CPU_H
#define __CPU_H

#include <asm/cpu.h>

#define MPIDR_INVALID	(-1)

#ifndef __ASSEMBLY__

#define dsb(arg)	asm volatile ("dsb " #arg "\n" : : : "memory")
#define sev()		asm volatile ("sev\n" : : : "memory")
#define wfe()		asm volatile ("wfe\n" : : : "memory")

unsigned int find_logical_id(unsigned long mpidr);

#endif /* !__ASSEMBLY__ */
#endif
