/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2016 Eistec AB
 *               2019 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup  sys_lptimer Timers
 * @ingroup   sys
 * @brief     Provides a high level timer module to register
 *            timers, get current system time, and let a thread sleep for
 *            a certain amount of time.
 *
 * The implementation takes one low-level timer and multiplexes it.
 *
 * Insertion and removal of timers has O(n) complexity with (n) being the
 * number of active timers.  The reason for this is that multiplexing is
 * realized by next-first singly linked lists.
 *
 * @{
 * @file
 * @brief   lptimer interface definitions
 * @author  Kaspar Schleiser <kaspar@schleiser.de>
 * @author  Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author  Oleg Artamonov <oleg@unwds.com>
 */
#ifndef LPTIMER_H
#define LPTIMER_H

#include <stdint.h>
#include "timex.h"
#include "msg.h"
#include "mutex.h"

#include "board.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief lptimer timestamp (64 bit)
 *
 * @note This is a struct in order to make the lptimer API type strict
 */
typedef struct {
    uint64_t ticks64;       /**< Tick count */
} lptimer_ticks64_t;

/**
 * @brief lptimer timestamp (32 bit)
 *
 * @note This is a struct in order to make the lptimer API type strict
 */
typedef struct {
    uint32_t ticks32;       /**< Tick count */
} lptimer_ticks32_t;

/**
 * @brief lptimer callback type
 */
typedef void (*lptimer_callback_t)(void*);

/**
 * @brief lptimer timer structure
 */
typedef struct lptimer {
    struct lptimer *next;         /**< reference to next timer in timer lists */
    uint32_t target;             /**< lower 32bit absolute target time */
    uint32_t long_target;        /**< upper 32bit absolute target time */
    lptimer_callback_t callback;  /**< callback function to call when timer
                                     expires */
    void *arg;                   /**< argument to pass to callback function */
} lptimer_t;

/**
 * @brief get the current system time as 32bit time stamp value
 *
 * @note    Overflows 2**32 ticks, thus returns lptimer_now64() % 32,
 *          but is cheaper.
 *
 * @return  current time as 32bit time stamp
 */
static inline lptimer_ticks32_t lptimer_now(void);

/**
 * @brief get the current system time as 64bit time stamp
 *
 * @return  current time as 64bit time stamp
 */
static inline lptimer_ticks64_t lptimer_now64(void);

/**
 * @brief get the current system time into a timex_t
 *
 * @param[out] out  pointer to timex_t the time will be written to
 */
void lptimer_now_timex(timex_t *out);

/**
 * @brief get the current system time in milliseconds since start
 *
 * This is a convenience function for @c lptimer_msec_from_ticks(lptimer_now())
 */
static inline uint32_t lptimer_now_msec(void);

/**
 * @brief get the current system time in milliseconds since start
 *
 * This is a convenience function for @c lptimer_msec_from_ticks64(lptimer_now64())
 */
static inline uint64_t lptimer_now_msec64(void);

/**
 * @brief lptimer initialization function
 *
 * This sets up lptimer. Has to be called once at system boot.
 * If @ref auto_init is enabled, it will call this for you.
 */
void lptimer_init(void);

/**
 * @brief Pause the execution of a thread for some seconds
 *
 * When called from an ISR, this function will spin and thus block the MCU in
 * interrupt context for the specified amount in *seconds*, so don't *ever* use
 * it there.
 *
 * @param[in] seconds   the amount of seconds the thread should sleep
 */
static inline void lptimer_sleep(uint32_t seconds);

/**
 * @brief Pause the execution of a thread for some microseconds
 *
 * When called from an ISR, this function will spin and thus block the MCU for
 * the specified amount in microseconds, so only use it there for *very* short
 * periods, e.g., less than LPTIMER_BACKOFF.
 *
 * @param[in] microseconds  the amount of microseconds the thread should sleep
 */
static inline void lptimer_usleep(uint32_t microseconds);

/**
 * @brief Stop execution of a thread for some time, 32bit version
 *
 * When called from an ISR, this function will spin and thus block the MCU for
 * the specified amount, so only use it there for *very* short periods,
 * e.g. less than LPTIMER_BACKOFF.
 *
 * @param[in] ticks  number of ticks the thread should sleep
 */
static inline void lptimer_tsleep32(lptimer_ticks32_t ticks);

/**
 * @brief Stop execution of a thread for some time, 64bit version
 *
 * When called from an ISR, this function will spin and thus block the MCU for
 * the specified amount, so only use it there for *very* short periods,
 * e.g. less than LPTIMER_BACKOFF.
 *
 * @param[in] ticks  number of ticks the thread should sleep
 */
static inline void lptimer_tsleep64(lptimer_ticks64_t ticks);

/**
 * @brief Stop execution of a thread for some time, blocking
 *
 * This function will spin-block, so only use it *very* short periods.
 *
 * @param[in] ticks  the number of lptimer ticks the thread should spin for
 */
static inline void lptimer_spin(lptimer_ticks32_t ticks);

/**
 * @brief will cause the calling thread to be suspended until the absolute
 * time (@p last_wakeup + @p period).
 *
 * When the function returns, @p last_wakeup is set to
 * (@p last_wakeup + @p period).
 *
 * This function can be used to create periodic wakeups.
 * @c last_wakeup should be set to lptimer_now() before first call of the
 * function.
 *
 * If the result of (@p last_wakeup + @p period) would be in the past, the function
 * sets @p last_wakeup to @p last_wakeup + @p period and returns immediately.
 *
 * @param[in] last_wakeup   base time stamp for the wakeup
 * @param[in] period        time in microseconds that will be added to last_wakeup
 */
static inline void lptimer_periodic_wakeup(lptimer_ticks32_t *last_wakeup, uint32_t period);

/**
 * @brief Set a timer that sends a message
 *
 * This function sets a timer that will send a message @p offset ticks
 * from now.
 *
 * The mesage struct specified by msg parameter will not be copied, e.g., it
 * needs to point to valid memory until the message has been delivered.
 *
 * @param[in] timer         timer struct to work with.
 *                          Its lptimer_t::target and lptimer_t::long_target
 *                          fields need to be initialized with 0 on first use.
 * @param[in] offset        microseconds from now
 * @param[in] msg           ptr to msg that will be sent
 * @param[in] target_pid    pid the message will be sent to
 */
static inline void lptimer_set_msg(lptimer_t *timer, uint32_t offset, msg_t *msg, kernel_pid_t target_pid);

/**
 * @brief Set a timer that sends a message, 64bit version
 *
 * This function sets a timer that will send a message @p offset microseconds
 * from now.
 *
 * The mesage struct specified by msg parameter will not be copied, e.g., it
 * needs to point to valid memory until the message has been delivered.
 *
 * @param[in] timer         timer struct to work with.
 *                          Its lptimer_t::target and lptimer_t::long_target
 *                          fields need to be initialized with 0 on first use.
 * @param[in] offset        microseconds from now
 * @param[in] msg           ptr to msg that will be sent
 * @param[in] target_pid    pid the message will be sent to
 */
static inline void lptimer_set_msg64(lptimer_t *timer, uint64_t offset, msg_t *msg, kernel_pid_t target_pid);

/**
 * @brief Set a timer that wakes up a thread
 *
 * This function sets a timer that will wake up a thread when the timer has
 * expired.
 *
 * @param[in] timer         timer struct to work with.
 *                          Its lptimer_t::target and lptimer_t::long_target
 *                          fields need to be initialized with 0 on first use
 * @param[in] offset        microseconds from now
 * @param[in] pid           pid of the thread that will be woken up
 */
static inline void lptimer_set_wakeup(lptimer_t *timer, uint32_t offset, kernel_pid_t pid);

/**
 * @brief Set a timer that wakes up a thread, 64bit version
 *
 * This function sets a timer that will wake up a thread when the timer has
 * expired.
 *
 * @param[in] timer         timer struct to work with.
 *                          Its lptimer_t::target and lptimer_t::long_target
 *                          fields need to be initialized with 0 on first use
 * @param[in] offset        microseconds from now
 * @param[in] pid           pid of the thread that will be woken up
 */
static inline void lptimer_set_wakeup64(lptimer_t *timer, uint64_t offset, kernel_pid_t pid);

/**
 * @brief Set a timer to execute a callback at some time in the future
 *
 * Expects timer->callback to be set.
 *
 * The callback specified in the timer struct will be executed @p offset
 * ticks in the future.
 *
 * @warning BEWARE! Callbacks from lptimer_set() are being executed in interrupt
 * context (unless offset < LPTIMER_BACKOFF). DON'T USE THIS FUNCTION unless you
 * know *exactly* what that means.
 *
 * @param[in] timer     the timer structure to use.
 *                      Its lptimer_t::target and lptimer_t::long_target
 *                      fields need to be initialized with 0 on first use
 * @param[in] offset    time in microseconds from now specifying that timer's
 *                      callback's execution time
 */
static inline void lptimer_set(lptimer_t *timer, uint32_t offset);

/**
 * @brief Set a timer to execute a callback at some time in the future, 64bit
 * version
 *
 * Expects timer->callback to be set.
 *
 * The callback specified in the timer struct will be executed @p offset_ms
 * milliseconds in the future.
 *
 * @warning BEWARE! Callbacks from lptimer_set() are being executed in interrupt
 * context (unless offset < LPTIMER_BACKOFF). DON'T USE THIS FUNCTION unless you
 * know *exactly* what that means.
 *
 * @param[in] timer       the timer structure to use.
 *                        Its lptimer_t::target and lptimer_t::long_target
 *                        fields need to be initialized with 0 on first use
 * @param[in] offset_us   time in milliseconds from now specifying that timer's
 *                        callback's execution time
 */
static inline void lptimer_set64(lptimer_t *timer, uint64_t offset_ms);

/**
 * @brief remove a timer
 *
 * @note this function runs in O(n) with n being the number of active timers
 *
 * @param[in] timer ptr to timer structure that will be removed
 */
void lptimer_remove(lptimer_t *timer);

/**
 * @brief receive a message blocking but with timeout
 *
 * @param[out] msg      pointer to a msg_t which will be filled in case of
 *                      no timeout
 * @param[in]  timeout  timeout in microseconds relative
 *
 * @return     < 0 on error, other value otherwise
 */
static inline int lptimer_msg_receive_timeout(msg_t *msg, uint32_t timeout);

/**
 * @brief receive a message blocking but with timeout, 64bit version
 *
 * @param[out] msg      pointer to a msg_t which will be filled in case of no
 *                      timeout
 * @param[in]  timeout  timeout in microseconds relative
 *
 * @return     < 0 on error, other value otherwise
 */
static inline int lptimer_msg_receive_timeout64(msg_t *msg, uint64_t timeout);

/**
 * @brief Convert milliseconds to lptimer ticks
 *
 * @param[in] msec  microseconds
 *
 * @return lptimer time stamp
 */
static inline lptimer_ticks32_t lptimer_ticks_from_msec(uint32_t msec);

/**
 * @brief Convert milliseconds to lptimer ticks, 64 bit version
 *
 * @param[in] msec  microseconds
 *
 * @return lptimer time stamp
 */
static inline lptimer_ticks64_t lptimer_ticks_from_msec64(uint64_t msec);

/**
 * @brief Convert lptimer ticks to milliseconds
 *
 * @param[in] ticks  lptimer time stamp
 *
 * @return microseconds
 */
static inline uint32_t lptimer_msec_from_ticks(lptimer_ticks32_t ticks);

/**
 * @brief Convert lptimer ticks to milliseconds, 64 bit version
 *
 * @param[in] ticks  lptimer time stamp
 *
 * @return microseconds
 */
static inline uint64_t lptimer_msec_from_ticks64(lptimer_ticks64_t ticks);

/**
 * @brief Create an lptimer time stamp
 *
 * @param[in] ticks  number of lptimer ticks
 *
 * @return lptimer time stamp
 */
static inline lptimer_ticks32_t lptimer_ticks(uint32_t ticks);

/**
 * @brief Create an lptimer time stamp, 64 bit version
 *
 * @param[in] ticks  number of lptimer ticks
 *
 * @return lptimer time stamp
 */
static inline lptimer_ticks64_t lptimer_ticks64(uint64_t ticks);

/**
 * @brief Compute difference between two lptimer time stamps
 *
 * @param[in] a  left operand
 * @param[in] b  right operand
 *
 * @return @p a - @p b
 */
static inline lptimer_ticks32_t lptimer_diff(lptimer_ticks32_t a, lptimer_ticks32_t b);

/**
 * @brief Compute difference between two lptimer time stamps, 64 bit version
 *
 * @param[in] a  left operand
 * @param[in] b  right operand
 *
 * @return @p a - @p b
 */
static inline lptimer_ticks64_t lptimer_diff64(lptimer_ticks64_t a, lptimer_ticks64_t b);

/**
 * @brief Compute 32 bit difference between two 64 bit lptimer time stamps
 *
 * @param[in] a  left operand
 * @param[in] b  right operand
 *
 * @return @p a - @p b cast truncated to 32 bit
 */
static inline lptimer_ticks32_t lptimer_diff32_64(lptimer_ticks64_t a, lptimer_ticks64_t b);

/**
 * @brief Compare two lptimer time stamps
 *
 * @param[in] a  left operand
 * @param[in] b  right operand
 *
 * @return @p a < @p b
 */
static inline bool lptimer_less(lptimer_ticks32_t a, lptimer_ticks32_t b);

/**
 * @brief Compare two lptimer time stamps, 64 bit version
 *
 * @param[in] a  left operand
 * @param[in] b  right operand
 *
 * @return @p a < @p b
 */
static inline bool lptimer_less64(lptimer_ticks64_t a, lptimer_ticks64_t b);

/**
 * @brief lock a mutex but with timeout
 *
 * @note this requires core_thread_flags to be enabled
 *
 * @param[in]    mutex  mutex to lock
 * @param[in]    us     timeout in microseconds relative
 *
 * @return       0, when returned after mutex was locked
 * @return       -1, when the timeout occcured
 */
int lptimer_mutex_lock_timeout(mutex_t *mutex, uint64_t us);

/**
 * @brief    Set timeout thread flag after @p timeout
 *
 * This function will set THREAD_FLAG_TIMEOUT on the current thread after @p
 * timeout msec have passed.
 *
 * @param[in]   t       timer struct to use
 * @param[in]   timeout timeout in msec
 */
void lptimer_set_timeout_flag(lptimer_t *t, uint32_t timeout);

/**
 * @brief Removes all timers
 */
void lptimer_remove_all(void);

/**
 * @brief lptimer backoff value
 *
 * All timers that are less than LPTIMER_BACKOFF milliseconds in the future will
 * just spin.
 *
 * This is supposed to be defined per-device in e.g., periph_conf.h.
 */
#ifndef LPTIMER_BACKOFF
#define LPTIMER_BACKOFF 1
#endif

/**
 * @brief lptimer overhead value, in hardware ticks
 *
 * This value specifies the time a timer will be late if uncorrected, e.g.,
 * the system-specific lptimer execution time from timer ISR to executing
 * a timer's callback's first instruction.
 *
 * E.g., with LPTIMER_OVERHEAD == 0
 * start=lptimer_now();
 * lptimer_set(&timer, X);
 * (in callback:)
 * overhead=lptimer_now()-start-X;
 *
 * lptimer automatically substracts LPTIMER_OVERHEAD from a timer's target time,
 * but when the timer triggers, lptimer will spin-lock until a timer's target
 * time is reached, so timers will never trigger early.
 *
 * This is supposed to be defined per-device in e.g., periph_conf.h.
 */
#ifndef LPTIMER_OVERHEAD
#define LPTIMER_OVERHEAD 0
#endif

#ifndef LPTIMER_ISR_BACKOFF
/**
 * @brief   lptimer IRQ backoff time, in hardware ticks
 *
 * When scheduling the next IRQ, if it is less than the backoff time
 * in the future, just spin.
 *
 * This is supposed to be defined per-device in e.g., periph_conf.h.
 */
#define LPTIMER_ISR_BACKOFF 1
#endif

#ifndef LPTIMER_PERIODIC_SPIN
/**
 * @brief   lptimer_periodic_wakeup spin cutoff
 *
 * If the difference between target time and now is less than this value, then
 * lptimer_periodic_wakeup will use lptimer_spin instead of setting a timer.
 */
#define LPTIMER_PERIODIC_SPIN (LPTIMER_BACKOFF * 2)
#endif

#ifndef LPTIMER_PERIODIC_RELATIVE
/**
 * @brief   lptimer_periodic_wakeup relative target cutoff
 *
 * If the difference between target time and now is less than this value, then
 * lptimer_periodic_wakeup will set a relative target time in the future instead
 * of the true target.
 *
 * This is done to prevent target time underflows.
 */
#define LPTIMER_PERIODIC_RELATIVE (10)
#endif

/*
 * Default lptimer configuration
 */
#ifndef LPTIMER_DEV
/**
 * @brief Underlying hardware timer device to assign to lptimer
 */
#define LPTIMER_DEV TIMER_DEV(0)
/**
 * @brief Underlying hardware timer channel to assign to lptimer
 */
#define LPTIMER_CHAN (0)

#if (LPTIMER_MAX_VALUE) == 0xfffffful
#define LPTIMER_WIDTH (24)
#elif (LPTIMER_MAX_VALUE) == 0xffff
#define LPTIMER_WIDTH (16)
#endif

#endif

#ifndef LPTIMER_WIDTH
/**
 * @brief lptimer timer width
 *
 * This value specifies the width (in bits) of the hardware timer used by lptimer.
 * Default is 32.
 */
#define LPTIMER_WIDTH (32)
#endif

#if (LPTIMER_WIDTH != 32) || DOXYGEN
/**
 * @brief lptimer timer mask
 *
 * This value specifies the mask relative to 0xffffffff that the used timer
 * counts to, e.g., 0xffffffff & ~TIMER_MAXVALUE.
 *
 * For a 16bit timer, the mask would be 0xFFFF0000, for a 24bit timer, the mask
 * would be 0xFF000000.
 */
#define LPTIMER_MASK ((0xffffffff >> LPTIMER_WIDTH) << LPTIMER_WIDTH)
#else
#define LPTIMER_MASK (0)
#endif

/**
 * @brief  Base frequency of lptimer is 1 MHz
 */
#define LPTIMER_HZ_BASE (1000000ul)

#ifndef LPTIMER_HZ
/**
 * @brief  Frequency of the underlying hardware timer
 */
#define LPTIMER_HZ LPTIMER_HZ_BASE
#endif

#include "lptimer/tick_conversion.h"

#include "lptimer/implementation.h"

#ifdef __cplusplus
}
#endif

/** @} */
#endif /* LPTIMER_H */
