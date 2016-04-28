/*
 * Copyright (C) 2015 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_unwired-range-f401
 * @{
 *
 * @file
 * @brief       Board specific implementations for the unwired-range-f401 board
 *
 * @author      Cr0s
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"

void board_init(void)
{
    /* initialize the CPU */
    cpu_init();

    /* initialize the boards LEDs */
    gpio_init(LED0_PIN, GPIO_OUT);

    // TODO: initialize the SX1276 on-board radio device
    // sx1276_init();
}
