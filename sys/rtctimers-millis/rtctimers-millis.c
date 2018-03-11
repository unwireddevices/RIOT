/*
 * Copyright (C) 2017 Unwired Devices <info@unwds.com>
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
 * @brief       RTC subseconds timer (not every device supports it)
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include "mutex.h"
#include "irq.h"
#include "sched.h"

#include "rtctimers-millis.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

static void _callback_unlock_mutex(void* arg)
{
    mutex_t *mutex = (mutex_t *) arg;
    mutex_unlock(mutex);
}

static void _callback_msg(void* arg)
{
    msg_t *msg = (msg_t*)arg;
    DEBUG("Callback: sending message to %d\n", msg->sender_pid);
    msg_send_int(msg, msg->sender_pid);
}

void rtctimers_millis_sleep(uint32_t sleep_millis) {
    if (irq_is_in()) {
    	DEBUG("[rtctimers] Unable to sleep in IRQ"); // FIXME
        return;
    }

    rtctimers_millis_t timer;
    mutex_t mutex = MUTEX_INIT;

    timer.callback = _callback_unlock_mutex;
    timer.arg = (void*) &mutex;
    timer.target = 0;

    mutex_lock(&mutex);
    rtctimers_millis_set(&timer, sleep_millis);
    mutex_lock(&mutex);
}

void rtctimers_millis_set_msg(rtctimers_millis_t *timer, uint32_t offset, msg_t *msg, kernel_pid_t target_pid) {
	timer->callback = _callback_msg;
	timer->arg = (void *) msg;

	msg->sender_pid = target_pid;
    DEBUG("Calling rtctimers_millis_set, target PID %d\n", target_pid);
	rtctimers_millis_set(timer, offset);
}

void rtctimers_millis_set_msg_absolute(rtctimers_millis_t *timer, msg_t *msg, kernel_pid_t target_pid,
                                       uint8_t wday, uint8_t hour, uint8_t min, uint8_t sec) {
	timer->callback = _callback_msg;
	timer->arg = (void *) msg;

	msg->sender_pid = target_pid;
	rtctimers_millis_set_absolute(timer, wday, hour, min, sec);
}

#ifdef __cplusplus
}
#endif
