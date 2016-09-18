/*
 * Copyright (C) 2016 cr0s
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file
 * @brief       
 * @author      cr0s
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>

#include "xtimer.h"
#include "mutex.h"
#include "thread.h"
#include "irq.h"
#include "div.h"
#include "rtc-timers.h"

static void _callback_unlock_mutex(void* arg);
static void _callback_unlock_mutex(void* arg)
{
    //mutex_t *mutex = (mutex_t *) arg;
    //mutex_unlock(mutex);
}

void rtctimers_sleep(uint32_t sleep_sec) {
    if (irq_is_in()) {
    	puts("[rtctimers] Unable to sleep in IRQ"); // FIXME
        return;
    }

    rtctimer_t timer;
    mutex_t mutex = MUTEX_INIT;

    timer.callback = _callback_unlock_mutex;
    timer.arg = (void*) &mutex;
    timer.target = 0;

    mutex_lock(&mutex);
    rtctimers_set(&timer, sleep_sec);
    mutex_lock(&mutex);
}

void rtctimers_set_msg(rtctimer_t *timer, uint32_t offset, msg_t *msg, kernel_pid_t target_pid) {

}

#ifdef __cplusplus
}
#endif
