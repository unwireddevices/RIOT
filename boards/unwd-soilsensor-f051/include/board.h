/*
 * Copyright (C) 2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_nucleo-f030 Nucleo-F030
 * @ingroup     boards
 * @brief       Board specific files for the nucleo-f030 board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the nucleo-f030 board
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Mohmmad Ayman <mohmmad.khzrag@gmail.com>
 * @author      José Alamos <jialamos@uc.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdint.h>

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   xtimer configuration
 * @{
 */
#define XTIMER_WIDTH        (16)
/** @} */

#define LED0_PIN            GPIO_PIN(PORT_A, 5)

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
