/*
 * Copyright (C) 2019 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_unwd-nrf UNWD-NRF
 * @ingroup     boards
 * @brief       Support for the UNWD-NRF board
 * @{
 *
 * @file
 * @brief       Board specific configuration for the UNWD-NRF
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    LED pin configuration
 * @{
 */
#define LED0_PIN            GPIO_PIN(0, 27)
#define LED1_PIN            GPIO_UNDEF
/** @} */

/** GPIO Ports */
#define UNWD_GPIO_1 		GPIO_PIN(0, 8)
#define UNWD_GPIO_2 		GPIO_PIN(0, 7)
#define UNWD_GPIO_3 		GPIO_PIN(0, 6)
#define UNWD_GPIO_4 		GPIO_PIN(0, 3)
#define UNWD_GPIO_5 		GPIO_PIN(0, 2)
#define UNWD_GPIO_6 		GPIO_PIN(0, 12)
#define UNWD_GPIO_7 		GPIO_PIN(0, 13)

#define UNWD_GPIO_30 		GPIO_PIN(0, 20)
#define UNWD_GPIO_29 		GPIO_PIN(0, 19)
#define UNWD_GPIO_28 		GPIO_PIN(0, 31)
#define UNWD_GPIO_27 		GPIO_PIN(0, 30)
#define UNWD_GPIO_26 		GPIO_PIN(0, 29)
#define UNWD_GPIO_25 		GPIO_PIN(0, 5)
#define UNWD_GPIO_24 		GPIO_PIN(0, 4)
/** @} */

/**
 * @brief   Initialize the platform
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
