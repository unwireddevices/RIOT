/*
 * Copyright (C) 2019 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_nrf5x_common
 * @{
 * @file
 * @brief       Low-level RTC driver implementation based on RTT hardware
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 * @}
 */

#include <time.h>
#include "cpu.h"
#include "mutex.h"
#include "periph/rtt.h"
#include "periph/rtc.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

static mutex_t rtc_mutex;

static struct {
    rtc_alarm_cb_t cb_a;        /**< callback called from RTC interrupt */
    rtc_alarm_cb_t cb_b;        /**< Subseconds alarm callback */
    rtc_wkup_cb_t  cb_wkup;     /**< Wake up timer callback */
    
    void *arg_a;                /**< argument passed to the callback */
    void *arg_b;                /**< argument passed to subseconds alarm callback */
    void *arg_wkup;             /**< argument passed to wakeup callback */
} isr_ctx;

void rtc_relese(void) {
    mutex_unlock(&rtc_mutex);
}

void rtc_acquire(void) {
    mutex_lock(&rtc_mutex);
}

void rtc_callback(void* arg)
{
}

void rtc_init(void)
{
    mutex_init(&rtc_mutex);
    rtc_acquire();
    
    /* enable low frequency clock */
    rtt_init();
    
    rtc_release();
}

int rtc_set_time(struct tm *time)
{
    rtc_acquire();
    
    rtc_release();

    return 0;
}

int rtc_get_time(struct tm *time)
{
    rtc_acquire();
    
    rtc_release();

    return 0;
}

int rtc_set_alarm(struct tm *time, rtc_alarm_cb_t cb, void *arg)
{
    rtc_acquire();
    
    rtc_release();

    return 0;
}

int rtc_get_alarm(struct tm *time)
{
    rtc_acquire();
    
    rtc_release();

    return 0;
}

void rtc_clear_alarm(void)
{
    rtc_acquire();

    isr_ctx.cb_a = NULL;
    isr_ctx.arg_a = NULL;
    
    rtc_release();
}

int rtc_millis_set_alarm(uint32_t milliseconds, rtc_alarm_cb_t cb, void *arg)
{   
    rtc_acquire();

    isr_ctx.cb_b = cb;
    isr_ctx.arg_b = arg;

    rtc_release();
    return 0;
}

void rtc_millis_clear_alarm(void)
{
    rtc_acquire();

    isr_ctx.cb_b = NULL;
    isr_ctx.arg_b = NULL;

    rtc_release();
}

int rtc_millis_get_time(uint32_t *millis)
{
    rtc_acquire();
    
    rtc_release();
    return 0;
}

int rtc_set_wakeup(uint32_t period_us, rtc_wkup_cb_t cb, void *arg)
{
    rtc_acquire();

    isr_ctx.cb_wkup = cb;
    isr_ctx.arg_wkup = arg;
    
    rtc_release();

    return 0;
}

void rtc_enable_wakeup(void) {
    rtc_acquire();

    rtc_release();
}

void rtc_disable_wakeup(void)
{
    rtc_acquire();

    rtc_release();
}

void rtc_poweron(void)
{
    rtt_poweron();
}

void rtc_poweroff(void)
{
    rtt_poweroff();
}

#endif /* RTC */
