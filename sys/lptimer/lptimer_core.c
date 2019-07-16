/**
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *               2016 Eistec AB
 *               2018 Josua Arndt
 *               2019 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup sys_lptimer
 *
 * @{
 * @file
 * @brief lptimer core functionality
 * @author Kaspar Schleiser <kaspar@schleiser.de>
 * @author Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author Josua Arndt <jarndt@ias.rwth-aachen.de>
 * @author  Oleg Artamonov <oleg@unwds.com>
 * @}
 */

#include <stdint.h>
#include <string.h>
#include "board.h"
#include "periph/timer.h"
#include "periph_conf.h"

#include "lptimer.h"
#include "irq.h"

/* WARNING! enabling this will have side effects and can lead to timer underflows. */
#define ENABLE_DEBUG 0
#include "debug.h"

static volatile int _in_handler = 0;

static volatile uint32_t _long_cnt = 0;
#if LPTIMER_MASK
volatile uint32_t _lptimer_high_cnt = 0;
#endif

static inline void lptimer_spin_until(uint32_t value);

static lptimer_t *timer_list_head = NULL;
static lptimer_t *overflow_list_head = NULL;
static lptimer_t *long_list_head = NULL;

static void _add_timer_to_list(lptimer_t **list_head, lptimer_t *timer);
static void _add_timer_to_long_list(lptimer_t **list_head, lptimer_t *timer);
static void _shoot(lptimer_t *timer);
static void _remove(lptimer_t *timer);
static inline void _lltimer_set(uint32_t target);
static uint32_t _time_left(uint32_t target, uint32_t reference);

static void _timer_callback(void);
static void _periph_timer_callback(void *arg, int chan);

static inline int _this_high_period(uint32_t target);

static inline int _is_set(lptimer_t *timer)
{
    return (timer->target || timer->long_target);
}

static inline void lptimer_spin_until(uint32_t target)
{
#if LPTIMER_MASK
    target = _lptimer_lltimer_mask(target);
#endif
    while (_lptimer_lltimer_now() > target) {}
    while (_lptimer_lltimer_now() < target) {}
}

void lptimer_init(void)
{
    /* initialize low-level timer */
    timer_init(LPTIMER_DEV, LPTIMER_HZ, _periph_timer_callback, NULL);

    /* register initial overflow tick */
    _lltimer_set(0xFFFFFFFF);
}

static void _lptimer_now_internal(uint32_t *short_term, uint32_t *long_term)
{
    uint32_t before, after, long_value;

    /* loop to cope with possible overflow of _lptimer_now() */
    do {
        before = _lptimer_now();
        long_value = _long_cnt;
        after = _lptimer_now();

    } while (before > after);

    *short_term = after;
    *long_term = long_value;
}

uint64_t _lptimer_now64(void)
{
    uint32_t short_term, long_term;

    _lptimer_now_internal(&short_term, &long_term);

    return ((uint64_t)long_term << 32) + short_term;
}

void _lptimer_set64(lptimer_t *timer, uint32_t offset, uint32_t long_offset)
{
    DEBUG(" _lptimer_set64() offset=%" PRIu32 " long_offset=%" PRIu32 "\n", offset, long_offset);
    if (!long_offset) {
        /* timer fits into the short timer */
        _lptimer_set(timer, (uint32_t)offset);
    }
    else {
        int state = irq_disable();
        if (_is_set(timer)) {
            _remove(timer);
        }

        _lptimer_now_internal(&timer->target, &timer->long_target);
        timer->target += offset;
        timer->long_target += long_offset;
        if (timer->target < offset) {
            timer->long_target++;
        }

        _add_timer_to_long_list(&long_list_head, timer);
        irq_restore(state);
        DEBUG("lptimer_set64(): added longterm timer (long_target=%" PRIu32 " target=%" PRIu32 ")\n",
              timer->long_target, timer->target);
    }
}

void _lptimer_set(lptimer_t *timer, uint32_t offset)
{
    DEBUG("timer_set(): offset=%" PRIu32 " now=%" PRIu32 " (%" PRIu32 ")\n",
          offset, lptimer_now().ticks32, _lptimer_lltimer_now());
    if (!timer->callback) {
        DEBUG("timer_set(): timer has no callback.\n");
        return;
    }

    lptimer_remove(timer);

    if (offset < LPTIMER_BACKOFF) {
        _lptimer_spin(offset);
        _shoot(timer);
    }
    else {
        uint32_t target = _lptimer_now() + offset;
        _lptimer_set_absolute(timer, target);
    }
}

static void _periph_timer_callback(void *arg, int chan)
{
    (void)arg;
    (void)chan;
    _timer_callback();
}

static void _shoot(lptimer_t *timer)
{
    timer->callback(timer->arg);
}

static inline void _lltimer_set(uint32_t target)
{
    if (_in_handler) {
        return;
    }
    DEBUG("_lltimer_set(): setting %" PRIu32 "\n", _lptimer_lltimer_mask(target));
    timer_set_absolute(LPTIMER_DEV, LPTIMER_CHAN, _lptimer_lltimer_mask(target));
}

int _lptimer_set_absolute(lptimer_t *timer, uint32_t target)
{
    uint32_t now = _lptimer_now();
    int res = 0;

    timer->next = NULL;

    /* Ensure that offset is bigger than 'LPTIMER_BACKOFF',
     * 'target - now' will allways be the offset no matter if target < or > now.
     *
     * This expects that target was not set too close to now and overrun now, so
     * from setting target up until the call of '_lptimer_now()' above now has not
     * become equal or bigger than target.
     * This is crucial when using low CPU frequencies so reaching the '_lptimer_now()'
     * call needs multiple lptimer ticks.
     *
     * '_lptimer_set()' and `_lptimer_periodic_wakeup()` ensure this by already
     * backing off for small values. */
    uint32_t offset = (target - now);

    DEBUG("timer_set_absolute(): now=%" PRIu32 " target=%" PRIu32 " offset=%" PRIu32 "\n",
          now, target, offset);

    if (offset <= LPTIMER_BACKOFF) {
        /* backoff */
        lptimer_spin_until(target);
        _shoot(timer);
        return 0;
    }

    unsigned state = irq_disable();
    if (_is_set(timer)) {
        _remove(timer);
    }

    timer->target = target;
    timer->long_target = _long_cnt;

    /* Ensure timer is fired in right timer period.
     * Backoff condition above ensures that 'target - LPTIMER_OVERHEAD` is later
     * than 'now', also for values when now will overflow and the value of target
     * is smaller then now.
     * If `target < LPTIMER_OVERHEAD` the new target will be at the end of this
     * 32bit period, as `target - LPTIMER_OVERHEAD` is a big number instead of a
     * small at the beginning of the next period. */
    target = target - LPTIMER_OVERHEAD;

    /* 32 bit target overflow, target is in next 32bit period */
    if (target < now) {
        timer->long_target++;
    }

    if ((timer->long_target > _long_cnt) || !_this_high_period(target)) {
        DEBUG("lptimer_set_absolute(): the timer doesn't fit into the low-level timer's mask.\n");
        _add_timer_to_long_list(&long_list_head, timer);
    }
    else {
        if (_lptimer_lltimer_mask(now) >= target) {
            DEBUG("lptimer_set_absolute(): the timer will expire in the next timer period\n");
            _add_timer_to_list(&overflow_list_head, timer);
        }
        else {
            DEBUG("timer_set_absolute(): timer will expire in this timer period.\n");
            _add_timer_to_list(&timer_list_head, timer);

            if (timer_list_head == timer) {
                DEBUG("timer_set_absolute(): timer is new list head. updating lltimer.\n");
                _lltimer_set(target);
            }
        }
    }

    irq_restore(state);

    return res;
}

static void _add_timer_to_list(lptimer_t **list_head, lptimer_t *timer)
{
    while (*list_head && (*list_head)->target <= timer->target) {
        list_head = &((*list_head)->next);
    }

    timer->next = *list_head;
    *list_head = timer;
}

static void _add_timer_to_long_list(lptimer_t **list_head, lptimer_t *timer)
{
    while (*list_head
           && (((*list_head)->long_target < timer->long_target)
           || (((*list_head)->long_target == timer->long_target) && ((*list_head)->target <= timer->target)))) {
        list_head = &((*list_head)->next);
    }

    timer->next = *list_head;
    *list_head = timer;
}

static int _remove_timer_from_list(lptimer_t **list_head, lptimer_t *timer)
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

static void _remove(lptimer_t *timer)
{
    if (timer_list_head == timer) {
        uint32_t next;
        timer_list_head = timer->next;
        if (timer_list_head) {
            /* schedule callback on next timer target time */
            next = timer_list_head->target - LPTIMER_OVERHEAD;
        }
        else {
            next = _lptimer_lltimer_mask(0xFFFFFFFF);
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

void lptimer_remove(lptimer_t *timer)
{
    int state = irq_disable();

    if (_is_set(timer)) {
        _remove(timer);
    }
    irq_restore(state);
}

static uint32_t _time_left(uint32_t target, uint32_t reference)
{
    uint32_t now = _lptimer_lltimer_now();

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

static inline int _this_high_period(uint32_t target)
{
#if LPTIMER_MASK
    return (target & LPTIMER_MASK) == _lptimer_high_cnt;
#else
    (void)target;
    return 1;
#endif
}

/**
 * @brief compare two timers' target values, return the one with lower value.
 *
 * if either is NULL, return the other.
 * if both are NULL, return NULL.
 */
static inline lptimer_t *_compare(lptimer_t *a, lptimer_t *b)
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
static lptimer_t *_merge_lists(lptimer_t *head_a, lptimer_t *head_b)
{
    lptimer_t *result_head = _compare(head_a, head_b);
    lptimer_t *pos = result_head;

    while (1) {
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
    lptimer_t *select_list_start = long_list_head;
    lptimer_t *select_list_last = NULL;

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
#if LPTIMER_MASK
    /* advance <32bit mask register */
    _lptimer_high_cnt += ~LPTIMER_MASK + 1;
    if (_lptimer_high_cnt == 0) {
        /* high_cnt overflowed, so advance >32bit counter */
        _long_cnt++;
    }
#else
    /* advance >32bit counter */
    _long_cnt++;
#endif

    /* swap overflow list to current timer list */
    timer_list_head = overflow_list_head;
    overflow_list_head = NULL;

    _select_long_timers();
}

/**
 * @brief main lptimer callback function
 */
static void _timer_callback(void)
{
    uint32_t next_target;
    uint32_t reference;

    _in_handler = 1;

    DEBUG("_timer_callback() now=%" PRIu32 " (%" PRIu32 ")pleft=%" PRIu32 "\n",
          lptimer_now().ticks32, _lptimer_lltimer_mask(lptimer_now().ticks32),
          _lptimer_lltimer_mask(0xffffffff - lptimer_now().ticks32));

    if (!timer_list_head) {
        DEBUG("_timer_callback(): tick\n");
        /* there's no timer for this timer period,
         * so this was a timer overflow callback.
         *
         * In this case, we advance to the next timer period.
         */
        _next_period();

        reference = 0;

        /* make sure the timer counter also arrived
         * in the next timer period */
        while (_lptimer_lltimer_now() == _lptimer_lltimer_mask(0xFFFFFFFF)) {}
    }
    else {
        /* we ended up in _timer_callback and there is
         * a timer waiting.
         */
        /* set our period reference to the current time. */
        reference = _lptimer_lltimer_now();
    }

overflow:
    /* check if next timers are close to expiring */
    while (timer_list_head && (_time_left(_lptimer_lltimer_mask(timer_list_head->target), reference) < LPTIMER_ISR_BACKOFF)) {
        /* make sure we don't fire too early */
        while (_time_left(_lptimer_lltimer_mask(timer_list_head->target), reference)) {}

        /* pick first timer in list */
        lptimer_t *timer = timer_list_head;

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
    /* check if the end of this period is very soon */
    uint32_t now = _lptimer_lltimer_now() + LPTIMER_ISR_BACKOFF;
    if (now < reference) {
        DEBUG("_timer_callback: overflowed while executing callbacks. %i\n",
              timer_list_head != NULL);
        _next_period();
        /* wait till overflow */
        while( reference < _lptimer_lltimer_now()){}
        reference = 0;
        goto overflow;
    }

    if (timer_list_head) {
        /* schedule callback on next timer target time */
        next_target = timer_list_head->target - LPTIMER_OVERHEAD;

        /* make sure we're not setting a time in the past */
        if (next_target < (_lptimer_now() + LPTIMER_ISR_BACKOFF)) {
            goto overflow;
        }
    }
    else {
        /* there's no timer planned for this timer period */
        /* schedule callback on next overflow */
        next_target = _lptimer_lltimer_mask(0xFFFFFFFF);
        uint32_t now = _lptimer_lltimer_now();

        /* check for overflow again */
        if (now < reference) {
            _next_period();
            reference = 0;
            goto overflow;
        }
        else {
            /* check if the end of this period is very soon */
            if (_lptimer_lltimer_mask(now + LPTIMER_ISR_BACKOFF) < now) {
                /* spin until next period, then advance */
                while (_lptimer_lltimer_now() >= now) {}
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
