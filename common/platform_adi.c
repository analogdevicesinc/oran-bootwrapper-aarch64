/*
 * platform_adi.c - ADI Platform initialization and I/O.
 *
 * Copyright (c) 2023, Analog Devices Incorporated - All Rights Reserved
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */

#include <cpu.h>
#include <stdint.h>

#include <asm/io.h>

#include <adrv906x_def.h>

#define PL011_UARTDR		0x00
#define PL011_UARTFR		0x18
#define PL011_UARTIBRD		0x24
#define PL011_UARTFBRD		0x28
#define PL011_UART_LCR_H	0x2c
#define PL011_UARTCR		0x30

#define PL011_UARTFR_BUSY	(1 << 3)
#define PL011_UARTFR_FIFO_FULL	(1 << 5)

#define PL011(reg)	((void *)UART_BASE + PL011_##reg)

#ifdef SYSREGS_BASE
#define V2M_SYS_CFGDATA		0xa0
#define V2M_SYS_CFGCTRL		0xa4

#define V2M_SYS(reg)	((void *)SYSREGS_BASE + V2M_SYS_##reg)
#endif

/* Boot args offsets */
#define BOOTARGS_OFFSET_MAGIC           0
#define BOOTARGS_OFFSET_KERNEL          1
#define BOOTARGS_OFFSET_COUNTER_FREQ    2
#define BOOTARGS_OFFSET_UART_FREQ       3

/* Kernel args offsets */
#define KERNELARGS_OFFSET_MAGIC         0
#define KERNELARGS_OFFSET_ENTRYPOINT    1
#define KERNELARGS_OFFSET_DTB           2

#define DEFAULT_UART_CLK_FREQ 491520000
#define DEFAULT_CNT_FREQ 30720000
#define BOOT_ARG_MAGIC_NUM 0xAD12B007
extern unsigned long bootargs_start;
unsigned long dtb;
unsigned long entrypoint;

void print_char(char c)
{
	uint32_t flags;

	do {
		flags = raw_readl(PL011(UARTFR));
	} while (flags & PL011_UARTFR_FIFO_FULL);

	raw_writel(c, PL011(UARTDR));

	do {
		flags = raw_readl(PL011(UARTFR));
	} while (flags & PL011_UARTFR_BUSY);
}

void print_string(const char *str)
{
	while (*str)
		print_char(*str++);
}

#define HEX_CHARS_PER_LONG	(2 * sizeof(long))
#define HEX_CHARS		"0123456789abcdef"

void print_ulong_hex(unsigned long val)
{
	int i;

	for (i = HEX_CHARS_PER_LONG - 1; i >= 0; i--) {
		int v = (val >> (4 * i)) & 0xf;
		print_char(HEX_CHARS[v]);
	}
}

void init_uart(void)
{
	unsigned long uart_clk_freq = DEFAULT_UART_CLK_FREQ;
	uint32_t reg;

#ifdef DYNAMIC_CONFIG
	unsigned long* boot_args;
	unsigned long boot_args_valid = 0;
	unsigned long magic;

	/* Get a pointer to the boot args */
	boot_args = &bootargs_start;
	/* First arg is magic number */
	magic = *(boot_args);
	if (magic == BOOT_ARG_MAGIC_NUM) {
		/* Fourth arg is the UART clock frequency */
		uart_clk_freq = *(boot_args + BOOTARGS_OFFSET_UART_FREQ);
		boot_args_valid = 1;
	}
#endif /* DYNAMIC_CONFIG */

	/* Enable UART */
	/* Set baudrate to 115200
	* divisor_int = ((uartclk * 4)/baudrate) >> 6
	* divisor_frac = ((uartclk * 4)/baudrate) & 0x3F
	*/
	reg = ((uart_clk_freq * 4)/115200);
	raw_writel(reg >> 6,	PL011(UARTIBRD));
	raw_writel(reg & 0x3F,	PL011(UARTFBRD));
	/* Set parameters to 8N1 and enable the FIFOs */
	raw_writel(0x70,	PL011(UART_LCR_H));
	/* Enable the UART, TXen and RXen */
	raw_writel(0x301,	PL011(UARTCR));

	if (!boot_args_valid) {
		print_string("ERROR: Invalid boot args. Cannot read UART frequency, using default value.\r\n");
	}
}

void init_platform(void)
{
#ifdef DYNAMIC_CONFIG
	uint32_t reg;
	unsigned long* boot_args;
	unsigned long boot_args_valid;
	unsigned long* kernel_args;
	unsigned long cnt_freq;
	unsigned long magic;

	boot_args_valid = 0;

	/* Get a pointer to the boot args */
	boot_args = &bootargs_start;

	/* First arg is magic number */
	magic = *(boot_args + BOOTARGS_OFFSET_MAGIC);

	if (magic == BOOT_ARG_MAGIC_NUM) {
		/* Second arg is a pointer to the kernel args */
		kernel_args = (unsigned long*)*(boot_args + BOOTARGS_OFFSET_KERNEL);

		/* Third arg is the counter frequency */
		cnt_freq = *(boot_args + BOOTARGS_OFFSET_COUNTER_FREQ);

		/* First kernel arg is magic number */
		magic = *(kernel_args + KERNELARGS_OFFSET_MAGIC);

		if (magic == BOOT_ARG_MAGIC_NUM) {
			/* Second kernel arg is the entrypoint */
			entrypoint = *(kernel_args + KERNELARGS_OFFSET_ENTRYPOINT);

			/* Third kernel arg is the device tree addr */
			dtb = *(kernel_args + KERNELARGS_OFFSET_DTB);

			boot_args_valid = 1;
		}
		else {
			/* Kernel args magic number doesn't match.
			 * Args are invalid.
			 */
			boot_args_valid = 0;
		}
	}
	else {
		/* If magic number doesn't match, boot args aren't valid.
		 * Use default values for UART clock freq and cnt freq.
		 */
		boot_args_valid = 0;
		cnt_freq = DEFAULT_CNT_FREQ;
	}

	/* Set cntfrq with the boot arg value, overriding the value set in boot.S */
	__asm__ volatile("msr cntfrq_el0, %x0" : : "r"(cnt_freq));
#endif /* DYNAMIC_CONFIG */

	/* Enable TSGEN */
	reg = raw_readl((void*)TSGEN_BASE);
	reg |= 0x1;
	raw_writel(reg, (void*)TSGEN_BASE);

#if DYNAMIC_CONFIG
	/* If boot args are not valid, we can't boot. Halt here. */
	if (boot_args_valid == 0) {
		print_string("ERROR: Invalid boot args. Boot halted.\r\n");
		while(1) {};
	}
#endif
	print_string("Booting Linux...\r\n\r\n");
}

void init_platform_secondary(void)
{
#ifdef DYNAMIC_CONFIG
	unsigned long* boot_args;
	unsigned long cnt_freq;

	/* Get a pointer to the boot args */
	boot_args = &bootargs_start;

	/* Third arg is the counter frequency */
	cnt_freq = *(boot_args + BOOTARGS_OFFSET_COUNTER_FREQ);

	/* Set cntfrq with the boot arg value, overriding the value set in boot.S */
	__asm__ volatile("msr cntfrq_el0, %x0" : : "r"(cnt_freq));
#endif /* DYNAMIC_CONFIG */
}

