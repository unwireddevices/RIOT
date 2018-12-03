/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_stm32f3 STM32F3
 * @ingroup         cpu
 * @brief           CPU specific implementations for the STM32F3
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *
 * @author          Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author          Katja Kirstein <katja.kirstein@haw-hamburg.de>
 */

#ifndef CPU_CONF_H
#define CPU_CONF_H

#include "cpu_conf_common.h"

#include "vendor/stm32f3xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   ARM Cortex-M specific CPU configuration
 * @{
 */
#define CPU_DEFAULT_IRQ_PRIO            (1U)
#if defined(CPU_LINE_STM32F303xE)
#define CPU_IRQ_NUMOF                   (85U)
#else
#define CPU_IRQ_NUMOF                   (82U)
#endif
#define CPU_FLASH_BASE                  FLASH_BASE
/** @} */

#define STM32F3_DEV_ID_CAT1     0x432 /* STM32F373xx/F378xx */
#define STM32F3_DEV_ID_CAT2     0x422 /* STM32F302xB(C)/F303xB(C)/F358xx */
#define STM32F3_DEV_ID_CAT3     0x439 /* STM32F301xx/F302x4(6/8)/F318xx */
#define STM32F3_DEV_ID_CAT4     0x438 /* STM32F303x4(6/8)/F334xx/F328xx */
#define STM32F3_DEV_ID_CAT5     0x446 /* STM32F302xD(E)/F303xD(E)/F398xx */
#define ST_DEV_ID           ((DBGMCU->IDCODE) & DBGMCU_IDCODE_DEV_ID)


#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */
