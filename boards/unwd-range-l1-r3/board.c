/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_unwd-range-l1-r3
 * @{
 *
 * @file
 * @brief       Board specific implementations for the Unwired Range boards
 *
 * @author      Mihail Churikov
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
    if (LED_GREEN != GPIO_UNDEF) {
        gpio_init(LED_GREEN, GPIO_OUT);
        pm_add_gpio_exclusion(LED_GREEN);
    }
    
    if (LED_RED != GPIO_UNDEF) {
        gpio_init(LED_RED, GPIO_OUT);
        pm_add_gpio_exclusion(LED_RED);
    }
}
