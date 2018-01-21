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
#include "time.h"

#ifndef RTCTIMERS_MILLIS_OVERHEAD
    #define RTCTIMERS_MILLIS_OVERHEAD 0
#endif
#ifndef RTCTIMERS_MILLIS_BACKOFF
    #define RTCTIMERS_MILLIS_BACKOFF 4
#endif
#ifndef RTCTIMERS_MILLIS_ISR_BACKOFF
    #define RTCTIMERS_MILLIS_ISR_BACKOFF 4
#endif

typedef void (*rtctimers_millis_cb_t)(void*);

typedef struct rtctimers_millis {
	struct rtctimers_millis *next;

	uint32_t target;
    uint32_t long_target;

	rtctimers_millis_cb_t callback;
	void *arg;
} rtctimers_millis_t;

void rtctimers_millis_init(void);
void rtctimers_millis_set(rtctimers_millis_t *timer, uint32_t offset);
void rtctimers_millis_remove(rtctimers_millis_t *timer);
void rtctimers_millis_sleep(uint32_t sleep_millis);
void rtctimers_millis_set_msg(rtctimers_millis_t *timer, uint32_t offset, msg_t *msg, kernel_pid_t target_pid);

/**
 * @brief Sets timers to an absolute time defined by day of week, h:m:s just like a regular alarm clock
 *
 * @note Days of week starting from Sunday (0), thus, Monday is (1), ..., and Saturday is (6)
 */
void rtctimers_millis_set_absolute(rtctimers_millis_t *timer, uint8_t wday, uint8_t hour, uint8_t min, uint8_t sec);
void rtctimers_millis_set_msg_absolute(rtctimers_millis_t *timer, msg_t *msg, kernel_pid_t target_pid, uint8_t wday, uint8_t hour, uint8_t min, uint8_t sec);

/**
 * @brief Changes current time without affecting currently running timers (shifts them accordingly)
 */
void rtctimers_millis_set_timebase(struct tm *new_time);

#endif /* RTC_TIMERS_MILLIS_H_ */
