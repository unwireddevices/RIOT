/*
 * Copyright (C) 2016 Cr0s
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
 * @brief       minimal RIOT application to test entrance into LPM and waking-up by RTC alarm
 *
 * @author      Cr0s
 *
 * @}
 */

#include <stdio.h>

#include "periph/gpio.h"
#include "thread.h"
#include "xtimer.h"

/**
 * @brief Time delay before RTC alarm
 */
#define SLEEP_TIME_SEC 1
void gcb(void *arg)
{
  LED0_TOGGLE;
  puts("CB!!!");
}

int main(void)
{
    puts("= STM32 Low Power mode test =");

    puts("Entering LPM...");

	/* Enter low power mode */
	xtimer_sleep(SLEEP_TIME_SEC);

	/* This code is supposed to execute after wake-up */
	LED0_TOGGLE;
	puts("Normally running");

	xtimer_sleep(10);
	puts("After 10 seconds");

    while (1) ;

    return 0;
}
