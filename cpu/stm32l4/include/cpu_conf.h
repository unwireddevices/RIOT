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
#endif

/**
 * @name   Flash page configuration
 * @{
 */
#define FLASHPAGE_SIZE      (2048U)

#if defined(CPU_MODEL_STM32L432KC) || defined(CPU_MODEL_STM32L433RC)
#define FLASHPAGE_NUMOF            (128U)
#elif defined(CPU_MODEL_STM32L452RE) || defined(CPU_MODEL_STM32L451CC)
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
#else
#define CPU_IRQ_NUMOF                   (82U)
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
 * @brief   Determine CPU RAM size
 * @return  RAM size in bytes
 */
uint32_t get_cpu_ram_size(void);

/**
 * @brief   Determine CPU flash size
 * @return  Flash size in bytes
 */
uint32_t get_cpu_flash_size(void);

/**
 * @brief   Determine CPU EEPROM size
 * @return  EEPROM size in bytes
 */
uint32_t get_cpu_eeprom_size(void);

/**
 * @brief   Determine CPU Category size
 * @return  CPU category according to RM0038 Reference Manual
 */
uint32_t get_cpu_category(void);

/**
 * @brief   Determine CPU RAM size
 * @param[out]  name Pointer to char array to store CPU name
 * @return  0 on success
 */
uint32_t get_cpu_name(char *name);

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
