/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
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
#ifndef RTC_TIMERS_MILLIS_H_
#define RTC_TIMERS_MILLIS_H_

#include <stdint.h>

#include "thread.h"

typedef void (*rtctimers_millis_cb_t)(void*);

typedef struct rtctimers_millis {
	struct rtctimers_millis *next;

	uint32_t target;

	rtctimers_millis_cb_t callback;
	void *arg;
} rtctimers_millis_t;

#define RTCTIMERS_MILLIS_OVERHEAD 0
#define RTCTIMERS_MILLIS_BACKOFF 0
#define RTCTIMERS_MILLIS_ISR_BACKOFF 5

void rtctimers_millis_init(void);

void rtctimers_millis_set(rtctimers_millis_t *timer, uint32_t offset);
void rtctimers_millis_remove(rtctimers_millis_t *timer);

void rtctimers_millis_sleep(uint32_t sleep_millis);
void rtctimers_millis_set_msg(rtctimers_millis_t *timer, uint32_t offset, msg_t *msg, kernel_pid_t target_pid);


#endif /* RTC_TIMERS_MILLIS_H_ */
