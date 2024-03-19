/*
 * include/platform.h - Platform initialization and I/O.
 *
 * Copyright (C) 2021 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

void print_char(char c);
void print_string(const char *str);
void print_ulong_hex(unsigned long val);

void init_uart(void);

void init_platform(void);

#if ADI_PLATFORM
void init_platform_secondary(void);
#endif

#endif /* __PLATFORM_H */
