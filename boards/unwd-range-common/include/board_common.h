/*
 * Copyright (C) 2016 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_unwd_range STM32-based Unwired Range module
 * @ingroup     drivers_periph
 * @brief       Common files for STM32-based Unwired Range module
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Cr0s
 */

#ifndef BOARD_COMMON__H
#define BOARD_COMMON__H

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   LED pin definitions and handlers
 * @{
 */
#define LED0_PIN            GPIO_PIN(PORT_B, 0)

#define LED0_MASK           (1 << 0)

#if defined(CPU_FAM_STM32F4)
#define LED_CREG            BSRRH
#else
#define LED_CREG            BRR
#endif
#if defined(CPU_FAM_STM32F3) || defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32L1)
#define LED_SREG            BSRRL
#else
#define LED_SREG            BSRR
#endif

#define LED0_ON             (GPIOB->LED_SREG = LED0_MASK)
#define LED0_OFF            (GPIOB->LED_CREG = LED0_MASK)
#define LED0_TOGGLE         (GPIOB->ODR     ^= LED0_MASK)


/** @} */

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_COMMON__H */
/** @} */
