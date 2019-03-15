/*
 * Copyright (C) 2016-2017 Feie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_nrf52dk nRF52 DK
 * @ingroup     boards
 * @brief       Support for the nRF52 DK
 * @{
 *
 * @file
 * @brief       Board specific configuration for the nRF52 DK
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
 * @name    Button pin configuration
 * @{
 */
#define BTN0_PIN            GPIO_PIN(0, 13)
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
