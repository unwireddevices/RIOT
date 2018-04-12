/*
 * Copyright (C) 2017 Freie Universität Berlin
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_stm32l4 STM32L4
 * @brief           STM32L4 specific code
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

#if defined(CPU_MODEL_STM32L496ZG)
#include "vendor/stm32l496xx.h"
#elif defined(CPU_MODEL_STM32L476RG) || defined(CPU_MODEL_STM32L476VG)
#include "vendor/stm32l476xx.h"
#elif defined(CPU_MODEL_STM32L475VG)
#include "vendor/stm32l475xx.h"
#elif defined(CPU_MODEL_STM32L452RE)
#include "vendor/stm32l452xx.h"
#elif defined(CPU_MODEL_STM32L451CC)
#include "vendor/stm32l451xx.h"
#elif defined(CPU_MODEL_STM32L433RC)
#include "vendor/stm32l433xx.h"
#elif defined(CPU_MODEL_STM32L432KC)
#include "vendor/stm32l432xx.h"
#elif defined(CPU_MODEL_STM32L4R5ZI)
#include "vendor/stm32l4r5xx.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   ARM Cortex-M specific CPU configuration
 * @{
 */
#define CPU_DEFAULT_IRQ_PRIO            (1U)
#if defined(CPU_MODEL_STM32L432KC) || defined(CPU_MODEL_STM32L433RC)
#define CPU_IRQ_NUMOF                   (83U)
#elif defined(CPU_MODEL_STM32L451CC) || defined(CPU_MODEL_STM32L452RE)
#define CPU_IRQ_NUMOF                   (85U)
#elif defined(CPU_MODEL_STM32L496ZG)
#define CPU_IRQ_NUMOF                   (91U)
#elif defined(CPU_MODEL_STM32L4R5ZI)
#define CPU_IRQ_NUMOF                   (95U)
#else
#define CPU_IRQ_NUMOF                   (82U)
#endif
/** @} */

#define STM32L4_DEV_ID_CAT1     0x464 /* STM32L41xxx/42xxx */
#define STM32L4_DEV_ID_CAT3     0x435 /* STM32L43xxx/44xxx */
#define STM32L4_DEV_ID_CAT5     0x462 /* STM32L45xxx/46xxx */
#define STM32L4_DEV_ID_CAT7     0x415 /* STM32L47xxx/48xxx */
#define STM32L4_DEV_ID_CATR     0x470 /* STM32L4Rxxx/4Sxxx */

#define ST_DEV_ID           ((DBGMCU->IDCODE) & DBGMCU_IDCODE_DEV_ID)

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
 * @name   Flash page configuration
 * @{
 */
#define FLASHPAGE_SIZE      (2048U)

#if defined(CPU_MODEL_STM32L432KC) || defined(CPU_MODEL_STM32L433RC)
#define FLASHPAGE_NUMOF            (128U)
#elif defined(CPU_MODEL_STM32L452RE)
#define FLASHPAGE_NUMOF            (256U)
#else
#define FLASHPAGE_NUMOF            (512U)
#endif
/* The minimum block size which can be written is 8B. However, the erase
 * block is always FLASHPAGE_SIZE.
 */
#define FLASHPAGE_RAW_BLOCKSIZE    (8U)
/* Writing should be always 8 bytes aligned */
#define FLASHPAGE_RAW_ALIGNMENT    (8U)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
