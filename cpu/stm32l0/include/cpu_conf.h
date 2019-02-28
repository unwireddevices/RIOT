/*
 * Copyright (C) 2016 Freie Universität Berlin
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_stm32l0 STM32L0
 * @brief           STM32L0 specific code
 * @ingroup         cpu
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *
 * @author          Hauke Petersen <hauke.pertersen@fu-berlin.de>
 * @author          Alexandre Abadie <alexandre.abadie@inria.fr>
*/

#ifndef CPU_CONF_H
#define CPU_CONF_H

#include "cpu_conf_common.h"

#include "vendor/stm32l0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32L0_DEV_ID_CAT1             0x457
#define STM32L0_DEV_ID_CAT2             0x425
#define STM32L0_DEV_ID_CAT3             0x417
#define STM32L0_DEV_ID_CAT5             0x447

#define ST_DEV_ID                       ((DBGMCU->IDCODE) & DBGMCU_IDCODE_DEV_ID)
#define STM32L0_CPUID_ADDR              (0x1ff80050)

/**
 * @brief   ARM Cortex-M specific CPU configuration
 * @{
 */
#define CPU_DEFAULT_IRQ_PRIO            (1U)
#if defined(CPU_LINE_STM32L031xx)
#define CPU_IRQ_NUMOF                   (30U)
#else
#define CPU_IRQ_NUMOF                   (32U)
#endif
/** @} */

/**
 * @brief Switch to MSI clock
 * @param[in] msi_range MSI frequency range
 * @param[in] ahb_divider AHB bus divider
 */
void switch_to_msi(uint32_t msi_range, uint32_t ahb_divider);

/**
 * @brief   Initizliaze clocks (switch to default clock)
 */
void clk_init(void);

/**
 * @name    Flash page configuration
 * @{
 */
#define FLASHPAGE_SIZE      (128U)

#define FLASHPAGE_NUMOF     (STM32_FLASHSIZE / FLASHPAGE_SIZE)

/* The minimum block size which can be written is 4B. However, the erase
 * block is always FLASHPAGE_SIZE.
 */
#define FLASHPAGE_RAW_BLOCKSIZE    (4U)
/* Writing should be always 4 byte aligned */
#define FLASHPAGE_RAW_ALIGNMENT    (4U)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
