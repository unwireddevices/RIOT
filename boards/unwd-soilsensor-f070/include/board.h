/*
 * Copyright (C) 2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    unwd-soilsensor-f070
 * @ingroup     boards
 * @brief       Soil moisture sensor board, based on STM32F070F6P6
 * @{
 *
 * @file
 * @brief       Soil moisture sensor board, based on STM32F070F6P6
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
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

#define UART_STDIO_DEV              UART_DEV(0)
#define UART_STDIO_BAUDRATE         (115200U)
#define UART_STDIO_RX_BUFSIZE       (64U)

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
