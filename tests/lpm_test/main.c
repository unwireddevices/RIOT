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
#include "lpm.h"

#include "periph/rtc.h"
#include "thread.h"

/**
 * @brief Time delay before RTC alarm
 */
#define SLEEP_TIME_SEC 3

void rtc_alarm_cb(void *arg) {
	(void) arg;
}

int main(void)
{
    rtc_init();

    puts("= STM32 Low Power mode test =");

    /* Setup RTC alarm time to wake up on */
    struct tm time;
    rtc_get_time(&time);
    time.tm_sec  += SLEEP_TIME_SEC;
    rtc_set_alarm(&time, rtc_alarm_cb, NULL);

    puts("Entering LPM...");

    /* Enter low power mode */
    lpm_set(LPM_SLEEP);

    /* This code is supposed to execute after wake-up on RTC alarm */
    puts("Normally running");

    while (1) ;

    return 0;
}
