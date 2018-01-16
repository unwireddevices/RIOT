/**
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2016 Eistec AB
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * @ingroup rtctimers_millis
 * @{
 * @file   rtctimers-core.c
 * @brief  rtctimers core functionality
 * @author Kaspar Schleiser <kaspar@schleiser.de>
 * @author Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author Oleg Artamonov <oleg@unwds.com>
 * @}
 */

#include <stdint.h>
#include <string.h>
#include "board.h"
#include "periph/rtc.h"
#include "periph_conf.h"

#include "rtctimers.h"
#include "irq.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/* 7*24*60*60 seconds = 1 week */
#define RTCTIMERS_OVERFLOW_VALUE 604800

static volatile int _in_handler = 0;

static volatile uint32_t _long_cnt = 0;

static rtctimers_t *timer_list_head = NULL;
static rtctimers_t *overflow_list_head = NULL;
static rtctimers_t *long_list_head = NULL;

static int _rtctimers_set_absolute(rtctimers_t *timer, uint32_t target);
static void _add_timer_to_list(rtctimers_t **list_head, rtctimers_t *timer);
static void _add_timer_to_long_list(rtctimers_t **list_head, rtctimers_t *timer);
static uint32_t _rtctimers_lltimer_maximum(uint32_t target);
static void _shoot(rtctimers_t *timer);
static void _remove(rtctimers_t *timer);
static uint32_t rtctimers_now(void);
static void _lltimer_set(uint32_t target);
static uint32_t _time_left(uint32_t target, uint32_t reference);

static void _timer_callback(void);
static void _periph_timer_callback(void *arg);

static inline int _this_high_period(uint32_t target);

static uint32_t rtctimers_now(void) {
    struct tm time;
    rtc_get_time(&time);

    /* returns seconds passed since 00:00:00 Sunday */
    return (time.tm_wday * 3600 * 24) +
           (time.tm_hour * 60 * 60) +
           (time.tm_min * 60) +
            time.tm_sec;
}

static uint32_t _rtctimers_lltimer_maximum(uint32_t target) {
    return (target % RTCTIMERS_OVERFLOW_VALUE);
}

static inline int _is_set(rtctimers_t *timer)
{
    return (timer->target || timer->long_target);
}

void rtctimers_init(void)
{
    /* initialize low-level timer */
    rtc_init();
    
    /* register initial overflow 1-week tick */
    _lltimer_set(RTCTIMERS_OVERFLOW_VALUE);
}

void rtctimers_set(rtctimers_t *timer, uint32_t offset)
{
    DEBUG("timer_set(): offset=%" PRIu32 " now=%" PRIu32 "\n", offset, rtctimers_now());
    if (!timer->callback) {
        DEBUG("timer_set(): timer has no callback.\n");
        return;
    }

    rtctimers_remove(timer);

    if (offset < RTCTIMERS_BACKOFF) {
        _shoot(timer);
    }
    else {
        uint32_t target = rtctimers_now() + offset;
        if (target >= RTCTIMERS_OVERFLOW_VALUE - RTCTIMERS_ISR_BACKOFF) {
            target -= (RTCTIMERS_OVERFLOW_VALUE - RTCTIMERS_ISR_BACKOFF);
        }
        _rtctimers_set_absolute(timer, target);
    }
}

static void _periph_timer_callback(void *arg)
{
    (void)arg;
    _timer_callback();
}

static void _shoot(rtctimers_t *timer)
{
    timer->callback(timer->arg);
}

static void _lltimer_set(uint32_t target)
{
    if (_in_handler) {
        return;
    }
    DEBUG("_lltimer_set(): setting %" PRIu32 " s, now is %" PRIu32 " s\n", _rtctimers_lltimer_maximum(target), rtctimers_now());
    
    uint32_t sec = _rtctimers_lltimer_maximum(target);
    
    struct tm time;
    
#if ENABLE_DEBUG
    rtc_get_time(&time);
    DEBUG("[rtctimers] %02d %02d:%02d:%02d -> ", time.tm_wday, time.tm_hour, time.tm_min, time.tm_sec);
#endif
    
    int days = sec / (3600 * 24);
    sec -= days * (3600 * 24);
    
    int hours = sec / 3600;
    sec -= hours * 3600;

	int mins = sec / 60;
	sec -= mins * 60;

    time.tm_mday = 0;
	time.tm_wday = days;
	time.tm_hour = hours;
	time.tm_min = mins;
	time.tm_sec = sec;
    
    DEBUG("%02d %02d:%02d:%02d\n", time.tm_wday, time.tm_hour, time.tm_min, time.tm_sec);
    
    rtc_set_alarm(&time, _periph_timer_callback, NULL);
}

int _rtctimers_set_absolute(rtctimers_t *timer, uint32_t target)
{
    uint32_t now = rtctimers_now();
    int res = 0;

    DEBUG("timer_set_absolute(): now=%" PRIu32 " target=%" PRIu32 "\n", now, target);

    timer->next = NULL;
    if ((target >= now) && ((target - RTCTIMERS_BACKOFF) < now)) {
        _shoot(timer);
        return 0;
    }

    unsigned state = irq_disable();
    if (_is_set(timer)) {
        _remove(timer);
    }

    timer->target = target;
    timer->long_target = _long_cnt;
    if (target < now) {
        timer->long_target++;
    }

    if ( (timer->long_target > _long_cnt) || !_this_high_period(target) ) {
        DEBUG("rtctimers_set_absolute(): the timer doesn't fit into the low-level timer's mask.\n");
        _add_timer_to_long_list(&long_list_head, timer);
    }
    else {
        if (_rtctimers_lltimer_maximum(now) >= target) {
            DEBUG("rtctimers_set_absolute(): the timer will expire in the next timer period\n");
            _add_timer_to_list(&overflow_list_head, timer);
        }
        else {
            DEBUG("timer_set_absolute(): timer will expire in this timer period.\n");
            _add_timer_to_list(&timer_list_head, timer);

            if (timer_list_head == timer) {
                DEBUG("timer_set_absolute(): timer is new list head. updating lltimer.\n");
                _lltimer_set(target - RTCTIMERS_OVERHEAD);
            }
        }
    }

    irq_restore(state);

    return res;
}

static void _add_timer_to_list(rtctimers_t **list_head, rtctimers_t *timer)
{
    while (*list_head && (*list_head)->target <= timer->target) {
        list_head = &((*list_head)->next);
    }

    timer->next = *list_head;
    *list_head = timer;
}

static void _add_timer_to_long_list(rtctimers_t **list_head, rtctimers_t *timer)
{
    while (*list_head
        && (((*list_head)->long_target < timer->long_target)
        || (((*list_head)->long_target == timer->long_target) && ((*list_head)->target <= timer->target)))) {
        list_head = &((*list_head)->next);
    }

    timer->next = *list_head;
    *list_head = timer;
}

static int _remove_timer_from_list(rtctimers_t **list_head, rtctimers_t *timer)
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

static void _remove(rtctimers_t *timer)
{
    if (timer_list_head == timer) {
        uint32_t next;
        timer_list_head = timer->next;
        if (timer_list_head) {
            /* schedule callback on next timer target time */
            next = timer_list_head->target - RTCTIMERS_OVERHEAD;
        }
        else {
            next = RTCTIMERS_OVERFLOW_VALUE;
        }
        _lltimer_set(next);
    }
    else {
        if (!_remove_timer_from_list(&timer_list_head, timer)) {
            if (!_remove_timer_from_list(&overflow_list_head, timer)) {
                _remove_timer_from_list(&long_list_head, timer);
            }
        }
    }
}

void rtctimers_remove(rtctimers_t *timer)
{
    int state = irq_disable();
    if (_is_set(timer)) {
        _remove(timer);
    }
    irq_restore(state);
}

static uint32_t _time_left(uint32_t target, uint32_t reference)
{
    uint32_t now = rtctimers_now();

    if (now < reference) {
        return 0;
    }

    if (target > now) {
        return target - now;
    }
    else {
        return 0;
    }
}

static inline int _this_high_period(uint32_t target) {
    (void)target;
    return 1;
}

/**
 * @brief compare two timers' target values, return the one with lower value.
 *
 * if either is NULL, return the other.
 * if both are NULL, return NULL.
 */
static inline rtctimers_t *_compare(rtctimers_t *a, rtctimers_t *b)
{
    if (a && b) {
        return ((a->target <= b->target) ? a : b);
    }
    else {
        return (a ? a : b);
    }
}

/**
 * @brief merge two timer lists, return head of new list
 */
static rtctimers_t *_merge_lists(rtctimers_t *head_a, rtctimers_t *head_b)
{
    rtctimers_t *result_head = _compare(head_a, head_b);
    rtctimers_t *pos = result_head;

    while(1) {
        head_a = head_a->next;
        head_b = head_b->next;
        if (!head_a) {
            pos->next = head_b;
            break;
        }
        if (!head_b) {
            pos->next = head_a;
            break;
        }

        pos->next = _compare(head_a, head_b);
        pos = pos->next;
    }

    return result_head;
}

/**
 * @brief parse long timers list and copy those that will expire in the current
 *        short timer period
 */
static void _select_long_timers(void)
{
    rtctimers_t *select_list_start = long_list_head;
    rtctimers_t *select_list_last = NULL;

    /* advance long_list head so it points to the first timer of the next (not
     * just started) "long timer period" */
    while (long_list_head) {
        if ((long_list_head->long_target <= _long_cnt) && _this_high_period(long_list_head->target)) {
            select_list_last = long_list_head;
            long_list_head = long_list_head->next;
        }
        else {
            /* remaining long_list timers belong to later long periods */
            break;
        }
    }

    /* cut the "selected long timer list" at the end */
    if (select_list_last) {
        select_list_last->next = NULL;
    }

    /* merge "current timer list" and "selected long timer list" */
    if (timer_list_head) {
        if (select_list_last) {
            /* both lists are non-empty. merge. */
            timer_list_head = _merge_lists(timer_list_head, select_list_start);
        }
        else {
            /* "selected long timer list" is empty, nothing to do */
        }
    }
    else { /* current timer list is empty */
        if (select_list_last) {
            /* there's no current timer list, but a non-empty "selected long
             * timer list".  So just use that list as the new current timer
             * list.*/
            timer_list_head = select_list_start;
        }
    }
}

/**
 * @brief handle low-level timer overflow, advance to next short timer period
 */
static void _next_period(void)
{
    /* advance >32bit counter */
    _long_cnt++;

    /* swap overflow list to current timer list */
    timer_list_head = overflow_list_head;
    overflow_list_head = NULL;

    _select_long_timers();
}

/**
 * @brief main rtctimers_millis callback function
 */
static void _timer_callback(void)
{
    uint32_t next_target;
    uint32_t reference;

    _in_handler = 1;

    DEBUG("_timer_callback() now=%" PRIu32 " pleft=%" PRIu32 "\n", 
            rtctimers_now(), RTCTIMERS_OVERFLOW_VALUE - rtctimers_now());

    if (!timer_list_head) {
        DEBUG("_timer_callback(): tick\n");
        /* there's no timer for this timer period,
         * so this was a timer overflow callback.
         *
         * In this case, we advance to the next timer period.
         */
        _next_period();

        reference = 0;
    }
    else {
        /* we ended up in _timer_callback and there is
         * a timer waiting.
         */
        /* set our period reference to the current time. */
        reference = rtctimers_now();
        DEBUG("[rtctimers] timer_callback reference %" PRIu32 " s\n", reference);
    }

overflow:
    /* check if next timers are close to expiring */
    while (timer_list_head &&
          (_time_left(_rtctimers_lltimer_maximum(timer_list_head->target), reference) <= RTCTIMERS_ISR_BACKOFF)) {
        /* make sure we don't fire too early */
        while (_time_left(_rtctimers_lltimer_maximum(timer_list_head->target), reference));

        /* pick first timer in list */
        rtctimers_t *timer = timer_list_head;

        /* advance list */
        timer_list_head = timer->next;

        /* make sure timer is recognized as being already fired */
        timer->target = 0;
        timer->long_target = 0;

        /* fire timer */
        _shoot(timer);
    }

    /* possibly executing all callbacks took enough
     * time to overflow.  In that case we advance to
     * next timer period and check again for expired
     * timers.*/
    if (reference > rtctimers_now()) {
        DEBUG("_timer_callback: overflowed while executing callbacks. %i\n", timer_list_head != 0);
        _next_period();
        reference = 0;
        goto overflow;
    }

    if (timer_list_head) {
        /* schedule callback on next timer target time */
        next_target = timer_list_head->target - RTCTIMERS_OVERHEAD;

        /* make sure we're not setting a time in the past */
        if (next_target < (rtctimers_now() + RTCTIMERS_ISR_BACKOFF)) {
            goto overflow;
        }
    }
    else {
        /* there's no timer planned for this timer period */
        /* schedule callback on next overflow */
        next_target = RTCTIMERS_OVERFLOW_VALUE;
        uint32_t now = rtctimers_now();

        /* check for overflow again */
        if (now < reference) {
            _next_period();
            reference = 0;
            goto overflow;
        }
        else {
            /* check if the end of this period is very soon */
            if (_rtctimers_lltimer_maximum(now + RTCTIMERS_ISR_BACKOFF) < now) {
                /* advance to the next period */
                _next_period();
                reference = 0;
                goto overflow;
            }
        }
    }

    _in_handler = 0;

    /* set low level timer */
    _lltimer_set(next_target);
}
