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

#ifdef CPU_MODEL_STM32L073RZ
#include "vendor/stm32l073xx.h"
#endif
#ifdef CPU_MODEL_STM32L072CZ
#include "vendor/stm32l072xx.h"
#endif
#ifdef CPU_MODEL_STM32L053R8
#include "vendor/stm32l053xx.h"
#endif
#ifdef CPU_MODEL_STM32L031K6
#include "vendor/stm32l031xx.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define STM32L0_DEV_ID_CAT3             0x417
#define STM32L0_DEV_ID_CAT5             0x447

#define ST_DEV_ID                       ((DBGMCU->IDCODE) & DBGMCU_IDCODE_DEV_ID)
#define STM32L0_CPUID_ADDR              (0x1ff80050)

/**
 * @brief   ARM Cortex-M specific CPU configuration
 * @{
 */
#define CPU_DEFAULT_IRQ_PRIO            (1U)
#if defined(CPU_MODEL_STM32L031K6)
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
 * @brief   Determine CPU RAM size
 * @return	RAM size in bytes
 */
uint32_t get_cpu_ram_size(void);

/**
 * @brief   Determine CPU flash size
 * @return	Flash size in bytes
 */
uint32_t get_cpu_flash_size(void);

/**
 * @brief   Determine CPU EEPROM size
 * @return	EEPROM size in bytes
 */
uint32_t get_cpu_eeprom_size(void);

/**
 * @brief   Determine CPU Category size
 * @return	CPU category according to RM0038 Reference Manual
 */
uint32_t get_cpu_category(void);

/**
 * @brief   Determine CPU RAM size
 * @param[out]	name Pointer to char array to store CPU name
 * @return	0 on success
 */
uint32_t get_cpu_name(char *name);

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
