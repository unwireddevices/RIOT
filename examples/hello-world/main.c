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

#include "rtctimers-millis.h"
#include "board.h"
#include "periph/gpio.h"
#include "thread.h"
#include "lpm.h"

#include "xtimer.h"

static rtctimers_millis_t led_timer;
static msg_t timer_msg = {};
static kernel_pid_t led_pid;


void *blink_a_led(void *arg) {
    msg_t msg;
    
    while (1) {
        msg_receive(&msg);
        
        gpio_toggle(LED_GREEN);
        
        rtctimers_millis_set_msg(&led_timer, 500, &timer_msg, led_pid);
    }
}

volatile uint32_t last_button_press;

void rgb_toggle(void *arg) {
    if (xtimer_now_usec() < last_button_press + 200000) {
        puts("Bounce");
        return;
    }
    puts("Press");
    last_button_press = xtimer_now_usec();
        
    gpio_toggle(UNWD_GPIO_26);
    
    return;
}

int main(void)
{
    lpm_prevent_sleep = 1;
    
    rtctimers_millis_init();
    xtimer_init();
    
    puts("Hello World!");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

    char stack[2048];
    led_pid = thread_create(stack, 2048, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, blink_a_led, NULL, "led thread");
    
    rtctimers_millis_set_msg(&led_timer, 500, &timer_msg, led_pid);
    
    gpio_init(UNWD_GPIO_26, GPIO_OUT);
    
    gpio_init_int(UNWD_GPIO_4, GPIO_IN_PU, GPIO_FALLING, rgb_toggle, NULL);

    return 0;
}
