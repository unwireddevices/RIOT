/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "periph/gpio.h"
#include "rtctimers-millis.h"

uint32_t milliseconds_last_press = 0;

static void btn_led_toggle(void* arg) {
    (void)arg;
    
    if ((rtctimers_millis_now() - milliseconds_last_press) > 100) {        
        gpio_toggle(GPIO_PIN(PORT_B, 0));
        milliseconds_last_press = rtctimers_millis_now();
    }
}

int main(void)
{
    rtctimers_millis_init();
    
    puts("Hello World!");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);
    
    gpio_init(GPIO_PIN(PORT_B, 0), GPIO_OUT);
    gpio_set(GPIO_PIN(PORT_B, 0));
    
    gpio_init_int(GPIO_PIN(PORT_B, 1), GPIO_IN_PU, GPIO_FALLING, btn_led_toggle, NULL);

    return 0;
}
