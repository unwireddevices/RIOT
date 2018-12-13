/*
 * Copyright (C) 2018 Freie Universität Berlin
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
 * @brief       Test
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include "stdio.h"
#include "board.h"

// #include "periph/pm.h"
#include "periph/pwm.h"
#include "periph/gpio.h"
#include "xtimer.h"

#define ENABLE_DEBUG        (1)
#include "debug.h"

int main(void)
{
    puts("Test gpio");

    gpio_init(GPIO_PIN(0,13), GPIO_IN_PU);

    // while(1)
    // {
    //     gpio_set(GPIO_PIN(0,13));
    //     xtimer_usleep(1000);
    //     gpio_clear(GPIO_PIN(0,13));
    //     xtimer_usleep(1000);
    // }

    // pwm_t dev = 0;
    // pwm_mode_t mode = PWM_RIGHT;
    // uint32_t freq = 550;
    // uint16_t res = 16000;
    // uint8_t channel = 0;
    // uint16_t value = res / 4;

    // pwm_init(dev, mode, freq, res);
    // pwm_set(dev, channel, value); 
    // pwm_start(dev); 

    // pwm_stop(dev);
    
    // value = res / 2;
    // pwm_set(dev, channel, value); 
    // pwm_start(dev); 
    // pwm_poweroff(dev); 

    while(1)
    {
        // pm_set(PM_SLEEP);
        __WFI();
    }

    return 0;
}
