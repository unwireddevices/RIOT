/*
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_stm32f1 STM32F1
 * @ingroup         cpu
 * @brief           CPU specific implementations for the STM32F1
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *
 * @author          Alaeddine Weslati <alaeddine.weslati@intia.fr>
 * @author          Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef CPU_CONF_H
#define CPU_CONF_H

#include "cpu_conf_common.h"

#include "vendor/stm32f1xx.h"

#include "vendor/stm32f1xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   ARM Cortex-M specific CPU configuration
 * @{
 */
#define CPU_DEFAULT_IRQ_PRIO            (1U)
#if defined(CPU_LINE_STM32F103xE) || defined(CPU_LINE_STM32F107xC)
#define CPU_IRQ_NUMOF                   (60U)
#else
#define CPU_IRQ_NUMOF                   (43U)
#endif
#define CPU_FLASH_BASE                  FLASH_BASE
/** @} */

#define STM32F1_DEV_ID_LD     0x412
#define STM32F1_DEV_ID_MD     0x410
#define STM32F1_DEV_ID_MDVL   0x420
#define STM32F1_DEV_ID_HD     0x414
#define STM32F1_DEV_ID_HDVL   0x428
#define STM32F1_DEV_ID_XL     0x430
#define STM32F1_DEV_ID_CD     0x418
#define ST_DEV_ID           ((DBGMCU->IDCODE) & DBGMCU_IDCODE_DEV_ID)

/**
 * @brief   Flash page configuration
 * @{
 */
#if defined(CPU_LINE_STM32F103xB)
#define FLASHPAGE_SIZE      (1024U)
#elif defined(CPU_LINE_STM32F103xE)
#define FLASHPAGE_SIZE      (2048U)
#endif

#define FLASHPAGE_NUMOF     (STM32_FLASHSIZE / FLASHPAGE_SIZE)

/* The minimum block size which can be written is 2B. However, the erase
 * block is always FLASHPAGE_SIZE.
 */
#define FLASHPAGE_RAW_BLOCKSIZE    (2U)
/* Writing should be always 4 bytes aligned */
#define FLASHPAGE_RAW_ALIGNMENT    (4U)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
