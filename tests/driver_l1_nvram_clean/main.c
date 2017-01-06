/*
 * Copyright (C) 2016 Morsview
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     test
 * @{
 *
 * @file
 * @brief       NVRAM cleaning firmware
 *
 * @author      Morsview
 *
 * @}
 */

#include <stdio.h>

#include "lpm.h"
#include "arch/lpm_arch.h"
#include "periph/gpio.h"
#include "thread.h"
#include "xtimer.h"

#include "board.h"
#include "nvram.h"
#include "eeprom.h"

#include "stm32l1xx.h"

/**
 * @brief Time delay before RTC alarm
 */

#define SLEEP_TIME_SEC 1

static nvram_t nvram;

void gcb(void *arg)
{
}

int main(void)
{
    lpm_prevent_sleep = 1;
    puts("= STM32L1 NVRAM Cleaning test =");

    nvram_eeprom_init(&nvram);

    gpio_init_int(GPIO_PIN(PORT_C, 1), GPIO_IN, GPIO_RISING, gcb, NULL);

    volatile int i = 0;
    for(i = 0; i < 10000000; i++) ;

    puts("Start cleaning NVRAM");
    LED0_TOGGLE;
    if (nvram.clear(&nvram) == 0x1FFF) {
      puts("Cleaned successfully");
    }
    else {
      puts("Cleaning ERROR!");
    }

    LED0_TOGGLE;
    puts("Going to infinite loop");

    while (1) ;

    return 0;
}
