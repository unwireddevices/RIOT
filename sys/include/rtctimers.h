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
 * @brief       
 * @author      EP <ep@unwds.com>
 */
#ifndef RTC_TIMERS_H_
#define RTC_TIMERS_H_

#include <stdint.h>

#include "thread.h"

typedef void (*rtctimer_cb_t)(void*);

typedef struct rtctimer {
	struct rtctimer *next;

	uint32_t target;
    uint32_t long_target;

	rtctimer_cb_t callback;
	void *arg;
} rtctimers_t;

#define RTCTIMERS_OVERHEAD 0
#define RTCTIMERS_BACKOFF 0
#define RTCTIMERS_ISR_BACKOFF 1

void rtctimers_init(void);

void rtctimers_set(rtctimers_t *timer, uint32_t offset);
void rtctimers_remove(rtctimers_t *timer);

void rtctimers_sleep(uint32_t sleep_sec);
void rtctimers_set_msg(rtctimers_t *timer, uint32_t offset, msg_t *msg, kernel_pid_t target_pid);


#endif /* RTC_TIMERS_H_ */
