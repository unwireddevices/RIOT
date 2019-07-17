/*
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
 * @ingroup   sys_lptimer

 * @{
 * @file
 * @brief   lptimer implementation
 *
 * @author  Kaspar Schleiser <kaspar@schleiser.de>
 * @author  Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author  Josua Arndt <jarndt@ias.rwth-aachen.de>
 * @author  Oleg Artamonov <oleg@unwds.com>
 *
 */
#ifndef LPTIMER_IMPLEMENTATION_H
#define LPTIMER_IMPLEMENTATION_H

#ifndef LPTIMER_H
#error "Do not include this file directly! Use lptimer.h instead"
#endif

#include "periph/rtt.h"

#ifdef __cplusplus
extern "C" {
#endif

#if LPTIMER_MASK
extern volatile uint32_t _lptimer_high_cnt;
#endif

/**
 * @brief IPC message type for lptimer msg callback
 */
#define MSG_LPTIMER 12345

/**
 * @brief returns the (masked) low-level timer counter value.
 */
static inline uint32_t _lptimer_lltimer_now(void)
{
    return rtt_get_counter();
}

/**
 * @brief drop bits of a value that don't fit into the low-level timer.
 */
static inline uint32_t _lptimer_lltimer_mask(uint32_t val)
{
    /* cppcheck-suppress shiftTooManyBits
     * (reason: cppcheck bug. `LPTIMER_MASK` is zero when `LPTIMER_WIDTH` is 32) */
    return val & ~LPTIMER_MASK;
}

/**
 * @{
 * @brief lptimer internal stuff
 * @internal
 */

uint64_t _lptimer_now64(void);

/**
 * @brief Sets the timer to the appropriate timer_list or list_head.
 *
 * @note    The target to set the timer to has to be at least bigger then the
 *          ticks needed to jump into the function and calculate '_lptimer_now()'.
 *          So that 'now' did not pass the target.
 *          This is crucial when using low CPU frequencies and/or when the
 *          '_lptimer_now()' call needs multiple lptimer ticks to evaluate.
 *
 * @param[in] timer   pointer to lptimer_t which is added to the list.
 * @param[in] target  Absolute target value in ticks.
 */
int _lptimer_set_absolute(lptimer_t *timer, uint32_t target);
void _lptimer_set(lptimer_t *timer, uint32_t offset);
void _lptimer_set64(lptimer_t *timer, uint32_t offset, uint32_t long_offset);
void _lptimer_periodic_wakeup(uint32_t *last_wakeup, uint32_t period);
void _lptimer_set_msg(lptimer_t *timer, uint32_t offset, msg_t *msg, kernel_pid_t target_pid);
void _lptimer_set_msg64(lptimer_t *timer, uint64_t offset, msg_t *msg, kernel_pid_t target_pid);
void _lptimer_set_wakeup(lptimer_t *timer, uint32_t offset, kernel_pid_t pid);
void _lptimer_set_wakeup64(lptimer_t *timer, uint64_t offset, kernel_pid_t pid);
int _lptimer_msg_receive_timeout(msg_t *msg, uint32_t ticks);
int _lptimer_msg_receive_timeout64(msg_t *msg, uint64_t ticks);

/**
 * @brief  Sleep for the given number of ticks
 */
void _lptimer_tsleep(uint32_t offset, uint32_t long_offset);
/** @} */

#ifndef LPTIMER_MIN_SPIN
/**
 * @brief Minimal value lptimer_spin() can spin
 */
#define LPTIMER_MIN_SPIN _lptimer_usec_from_ticks(1)
#endif

#ifndef DOXYGEN
/* Doxygen warns that these are undocumented, but the documentation can be found in lptimer.h */

static inline uint32_t _lptimer_now(void)
{
#if LPTIMER_MASK
    uint32_t latched_high_cnt, now;

    /* _high_cnt can change at any time, so check the value before
     * and after reading the low-level timer. If it hasn't changed,
     * then it can be safely applied to the timer count. */

    do {
        latched_high_cnt = _lptimer_high_cnt;
        now = _lptimer_lltimer_now();
    } while (_lptimer_high_cnt != latched_high_cnt);

    return latched_high_cnt | now;
#else
    return _lptimer_lltimer_now();
#endif
}

static inline lptimer_ticks32_t lptimer_now(void)
{
    lptimer_ticks32_t ret;
    ret.ticks32 = _lptimer_now();
    return ret;
}

static inline lptimer_ticks64_t lptimer_now64(void)
{
    lptimer_ticks64_t ret;
    ret.ticks64 = _lptimer_now64();
    return ret;
}

static inline uint32_t lptimer_now_usec(void)
{
    return lptimer_usec_from_ticks(lptimer_now());
}

static inline uint64_t lptimer_now_usec64(void)
{
    return lptimer_usec_from_ticks64(lptimer_now64());
}

static inline void _lptimer_spin(uint32_t offset) {
    uint32_t start = _lptimer_lltimer_now();
#if LPTIMER_MASK
    offset = _lptimer_lltimer_mask(offset);
    while (_lptimer_lltimer_mask(_lptimer_lltimer_now() - start) < offset);
#else
    while ((_lptimer_lltimer_now() - start) < offset);
#endif
}

static inline void _lptimer_tsleep32(uint32_t ticks)
{
    _lptimer_tsleep(ticks, 0);
}

static inline void _lptimer_tsleep64(uint64_t ticks)
{
    _lptimer_tsleep((uint32_t)ticks, (uint32_t)(ticks >> 32));
}

static inline void lptimer_spin(lptimer_ticks32_t ticks) {
    _lptimer_spin(ticks.ticks32);
}

static inline void lptimer_usleep(uint32_t microseconds)
{
    _lptimer_tsleep32(_lptimer_ticks_from_usec(microseconds));
}

static inline void lptimer_usleep64(uint64_t microseconds)
{
    _lptimer_tsleep64(_lptimer_ticks_from_usec64(microseconds));
}

static inline void lptimer_sleep(uint32_t seconds)
{
    _lptimer_tsleep64(_lptimer_ticks_from_usec64((uint64_t)seconds * US_PER_SEC));
}

static inline void lptimer_nanosleep(uint32_t nanoseconds)
{
    _lptimer_tsleep32(_lptimer_ticks_from_usec(nanoseconds / NS_PER_US));
}

static inline void lptimer_tsleep32(lptimer_ticks32_t ticks)
{
    _lptimer_tsleep32(ticks.ticks32);
}

static inline void lptimer_tsleep64(lptimer_ticks64_t ticks)
{
    _lptimer_tsleep64(ticks.ticks64);
}

static inline void lptimer_periodic_wakeup(lptimer_ticks32_t *last_wakeup, uint32_t period)
{
    _lptimer_periodic_wakeup(&last_wakeup->ticks32, _lptimer_ticks_from_usec(period));
}

static inline void lptimer_set_msg(lptimer_t *timer, uint32_t offset, msg_t *msg, kernel_pid_t target_pid)
{
    _lptimer_set_msg(timer, _lptimer_ticks_from_usec(offset), msg, target_pid);
}

static inline void lptimer_set_msg64(lptimer_t *timer, uint64_t offset, msg_t *msg, kernel_pid_t target_pid)
{
    _lptimer_set_msg64(timer, _lptimer_ticks_from_usec64(offset), msg, target_pid);
}

static inline void lptimer_set_wakeup(lptimer_t *timer, uint32_t offset, kernel_pid_t pid)
{
    _lptimer_set_wakeup(timer, _lptimer_ticks_from_usec(offset), pid);
}

static inline void lptimer_set_wakeup64(lptimer_t *timer, uint64_t offset, kernel_pid_t pid)
{
    _lptimer_set_wakeup64(timer, _lptimer_ticks_from_usec64(offset), pid);
}

static inline void lptimer_set(lptimer_t *timer, uint32_t offset)
{
    _lptimer_set(timer, _lptimer_ticks_from_usec(offset));
}

static inline void lptimer_set64(lptimer_t *timer, uint64_t period_us)
{
    uint64_t ticks = _lptimer_ticks_from_usec64(period_us);
    _lptimer_set64(timer, ticks, ticks >> 32);
}

static inline int lptimer_msg_receive_timeout(msg_t *msg, uint32_t timeout)
{
    return _lptimer_msg_receive_timeout(msg, _lptimer_ticks_from_usec(timeout));
}

static inline int lptimer_msg_receive_timeout64(msg_t *msg, uint64_t timeout)
{
    return _lptimer_msg_receive_timeout64(msg, _lptimer_ticks_from_usec64(timeout));
}

static inline lptimer_ticks32_t lptimer_ticks_from_usec(uint32_t usec)
{
    lptimer_ticks32_t ticks;
    ticks.ticks32 = _lptimer_ticks_from_usec(usec);
    return ticks;
}

static inline lptimer_ticks64_t lptimer_ticks_from_usec64(uint64_t usec)
{
    lptimer_ticks64_t ticks;
    ticks.ticks64 = _lptimer_ticks_from_usec64(usec);
    return ticks;
}

static inline uint32_t lptimer_usec_from_ticks(lptimer_ticks32_t ticks)
{
    return _lptimer_usec_from_ticks(ticks.ticks32);
}

static inline uint64_t lptimer_usec_from_ticks64(lptimer_ticks64_t ticks)
{
    return _lptimer_usec_from_ticks64(ticks.ticks64);
}

static inline lptimer_ticks32_t lptimer_ticks(uint32_t ticks)
{
    lptimer_ticks32_t ret;
    ret.ticks32 = ticks;
    return ret;
}

static inline lptimer_ticks64_t lptimer_ticks64(uint64_t ticks)
{
    lptimer_ticks64_t ret;
    ret.ticks64 = ticks;
    return ret;
}

static inline lptimer_ticks32_t lptimer_diff(lptimer_ticks32_t a, lptimer_ticks32_t b)
{
    lptimer_ticks32_t ret;
    ret.ticks32 = a.ticks32 - b.ticks32;
    return ret;
}

static inline lptimer_ticks64_t lptimer_diff64(lptimer_ticks64_t a, lptimer_ticks64_t b)
{
    lptimer_ticks64_t ret;
    ret.ticks64 = a.ticks64 - b.ticks64;
    return ret;
}

static inline lptimer_ticks32_t lptimer_diff32_64(lptimer_ticks64_t a, lptimer_ticks64_t b)
{
    uint64_t diff = a.ticks64 - b.ticks64;
    lptimer_ticks32_t ret;
    ret.ticks32 = diff;
    return ret;
}

static inline bool lptimer_less(lptimer_ticks32_t a, lptimer_ticks32_t b)
{
    return (a.ticks32 < b.ticks32);
}

static inline bool lptimer_less64(lptimer_ticks64_t a, lptimer_ticks64_t b)
{
    return (a.ticks64 < b.ticks64);
}

#endif /* !defined(DOXYGEN) */

#ifdef __cplusplus
}
#endif

#endif /* LPTIMER_IMPLEMENTATION_H */
