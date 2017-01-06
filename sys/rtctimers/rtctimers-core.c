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

#ifdef __cplusplus
extern "C" {
#endif

#include <xtimer.h>
#include <stdio.h>
#include <stdlib.h>

#include "rtctimers.h"
#include "periph/rtc.h"

#include "debug.h"
//#define DEBUG printf


static rtctimer_t *timer_list_head = NULL;

static void _add_timer_to_list(rtctimer_t **list_head, rtctimer_t *timer);
static void _remove(rtctimer_t *timer);
static void _rtc_callback(void *arg);
static void _timer_callback(void);

static inline int _is_set(rtctimer_t *timer)
{
    return timer->target;
}

static void _lltimer_set(uint32_t sec) {
    struct tm time;

	rtc_get_time(&time);

	DEBUG("[rtctimers] %d %d:%d:%d -> ", time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

	int days = sec / (3600 * 24);
	sec -= days * (3600 * 24);

	int hours = sec / 3600;
	sec -= hours * 3600;

	int mins = sec / 60;
	sec -= mins * 60;

	time.tm_mday = days;
	time.tm_hour = hours;
	time.tm_min = mins;
	time.tm_sec = sec;

	rtc_clear_alarm();
	rtc_set_alarm(&time, _rtc_callback, NULL);

	rtc_get_alarm(&time);
	DEBUG("%d %d:%d:%d\n", time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
}

static void _rtc_callback(void *arg) {
	_timer_callback();
}

void rtctimers_init(void) {
	rtc_init();
}

uint32_t rtctimers_now(void) {
	struct tm time;
	rtc_get_time(&time);

	return (time.tm_mday * 3600 * 24)
			+ (time.tm_hour * 60 * 60)
			+ (time.tm_min * 60) + time.tm_sec;
}

int _rtctimers_set_absolute(rtctimer_t *timer, uint32_t target)
{
    uint32_t now = rtctimers_now();
    int res = 0;

    //DEBUG("timer_set_absolute(): now=%u target=%u\n", (unsigned) now, (unsigned) target);

    timer->next = NULL;

    unsigned state = irq_disable();
    if (_is_set(timer)) {
        _remove(timer);
    }

    timer->target = target;

	if (now >= target) {
		DEBUG("[rtctimers] now >= target!\n");
	}
	else {
		_add_timer_to_list(&timer_list_head, timer);

		if (timer_list_head == timer) {
			_lltimer_set(target - RTCTIMERS_OVERHEAD);
		}
	}

    irq_restore(state);

    return res;
}

void rtctimers_set(rtctimer_t *timer, uint32_t offset) {
    if (!timer->callback) {
        DEBUG("timer_set(): timer has no callback.\n");
        return;
    }

    rtctimers_remove(timer);

	uint32_t target = rtctimers_now() + offset;
	_rtctimers_set_absolute(timer, target);
}

static void _add_timer_to_list(rtctimer_t **list_head, rtctimer_t *timer)
{
    while (*list_head && (*list_head)->target <= timer->target) {
        list_head = &((*list_head)->next);
    }

    timer->next = *list_head;
    *list_head = timer;
}

static int _remove_timer_from_list(rtctimer_t **list_head, rtctimer_t *timer)
{
    while (*list_head) {
        if (*list_head == timer) {
            *list_head = timer->next;
            return 1;
        }

        list_head = &((*list_head)->next);
    }

    return 0;
}

static void _remove(rtctimer_t *timer)
{
    if (timer_list_head == timer) {
        uint32_t next;
        timer_list_head = timer->next;
        if (timer_list_head) {
            /* schedule callback on next timer target time */
            next = timer_list_head->target - RTCTIMERS_OVERHEAD;
        }
        else {
            next = 0xFFFFFFFF;
        }

        _lltimer_set(next);
    }
    else {
        if (!_remove_timer_from_list(&timer_list_head, timer)) {
        	//DEBUG("[rtctimers] Unable to remove timer from list!\n");
        }
    }
}

void rtctimers_remove(rtctimer_t *timer) {
    int state = irq_disable();

    if (_is_set(timer)) {
        _remove(timer);
    }

    irq_restore(state);
}

static inline void _shoot(rtctimer_t *timer) {
	timer->callback(timer->arg);
}

static void _timer_callback(void)
{
    uint32_t next_target = 0;
    uint32_t reference;

    //_in_handler = 1;
	reference = rtctimers_now();

    /* check if next timers are close to expiring */
    while (timer_list_head && (timer_list_head->target - reference <= 0/* RTCTIMERS_ISR_BACKOFF*/)) {
        /* pick first timer in list */
        rtctimer_t *timer = timer_list_head;

        /* advance list */
        timer_list_head = timer->next;

        /* make sure timer is recognized as being already fired */
        timer->target = 0;

        /* fire timer */
        _shoot(timer);
    }

    if (timer_list_head) {
        /* schedule callback on next timer target time */
        next_target = timer_list_head->target - RTCTIMERS_OVERHEAD;

        /* make sure we're not setting a time in the past */
        if (next_target < (rtctimers_now() + RTCTIMERS_ISR_BACKOFF)) {
            DEBUG("[rtctimers] next_target < now + isr\n");
        }
    }

    /* set low level timer */
    if (next_target)
    	_lltimer_set(next_target);
}

#ifdef __cplusplus
}
#endif
