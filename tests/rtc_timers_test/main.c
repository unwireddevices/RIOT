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
 * @brief       minimal RIOT application to test RTC timers
 *
 * @author      Cr0s
 *
 * @}
 */

#include <stdio.h>

#include "lpm.h"
#include "periph/rtc.h"
#include "arch/lpm_arch.h"
#include "periph/gpio.h"
#include "thread.h"
#include "xtimer.h"
#include "board.h"
#include "mutex.h"

/**
 * @brief Time delay before RTC alarm
 */

#define SLEEP_TIME_SEC 1

static void cb(void *arg) {
	mutex_unlock((mutex_t *) arg);
}

int main(void)
{
	lpm_prevent_sleep = 1;

    xtimer_init();
    rtc_init();

    struct tm time;
    rtc_get_time(&time);

    mutex_t mutex = MUTEX_INIT;

    puts("Sleeping 5 seconds");
    time.tm_sec += 5;
    mutex_lock(&mutex);
    rtc_set_alarm(&time, cb, &mutex);
    mutex_lock(&mutex);
    puts("Woken up");

    while (1) ;

    return 0;
}
