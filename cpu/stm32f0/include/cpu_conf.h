/*
 * Copyright (C) 2016 Freie Universität Berlin
 *               2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_stm32f0 STM32F0
 * @brief           STM32F0 specific code
 * @ingroup         cpu
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *
 * @author          Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author          Alexandre Abadie <alexandre.abadie@inria.fr>
*/

#ifndef CPU_CONF_H
#define CPU_CONF_H

#include "cpu_conf_common.h"

#include "vendor/stm32f0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   ARM Cortex-M specific CPU configuration
 * @{
 */
#define CPU_DEFAULT_IRQ_PRIO            (1U)
#if defined(CPU_LINE_STM32F030x8)
#define CPU_IRQ_NUMOF                   (29U)
#elif defined(CPU_LINE_STM32F031x6)
#define CPU_IRQ_NUMOF                   (28U)
#elif defined(CPU_LINE_STM32F051x8) || defined(CPU_LINE_STM32F091xC)
#define CPU_IRQ_NUMOF                   (31U)
#else
#define CPU_IRQ_NUMOF                   (32U)
#endif
/** @} */

#define STM32F0_DEV_ID_CAT3     0x444
#define STM32F0_DEV_ID_CAT4     0x445
#define STM32F0_DEV_ID_CAT5     0x440
#define STM32F0_DEV_ID_CAT7     0x448
#define STM32F0_DEV_ID_CAT9     0x442
#define ST_DEV_ID           ((DBGMCU->IDCODE) & DBGMCU_IDCODE_DEV_ID)

/**
 * @brief   Initizliaze clocks (switch to default clock)
 */
void clk_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
