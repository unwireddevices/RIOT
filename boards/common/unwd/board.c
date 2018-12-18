/*
 * Copyright (C) 2016-2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_unwd-range
 * @{
 *
 * @file
 * @brief       Board specific implementations for the Unwired Range boards
 *
 * @author      Mihail Churikov
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include "board.h"
#include "periph/gpio.h"
#include "periph/pm.h"

void board_init(void)
{
    /* initialize the CPU */
    cpu_init();

    /* initialize the boards LEDs */
    if (LED0_PIN != GPIO_UNDEF) {
        gpio_init(LED0_PIN, GPIO_OUT);
    }
    
    if (LED1_PIN != GPIO_UNDEF) {
        gpio_init(LED1_PIN, GPIO_OUT);
    }
}
