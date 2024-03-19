/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __ADRV906X_DEF_H__
#define __ADRV906X_DEF_H__

/* Yoda defines */
#define SEC_TSGEN_BASE  0x24044000


/* Hardcoded Secondary BL1 addresses */
#define SEC_BL1_CFG_ADDR  0x04500000
#define SEC_BL1_LOAD_ADDR 0x045E0000

/* Variables for boot-wrapper-aarch64 */
#define BOOT_ARGS_BASE    SEC_BL1_CFG_ADDR
#define BOOT_BASE         SEC_BL1_LOAD_ADDR
#define TSGEN_BASE        SEC_TSGEN_BASE

#endif /* __ADRV906X_DEF_H__ */
