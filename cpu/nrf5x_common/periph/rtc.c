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
 * @brief       Low-level RTC driver implementation based on Nordic RTT hardware
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 * @}
 */

#include <time.h>
#include <string.h>
#include "cpu.h"
#include "mutex.h"
#include "periph/rtt.h"
#include "periph/rtc.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/* 
 * With 1024 Hz RTT frequency one tick is 976.5625 us
 */

#define RTC_WEEK_MILLISECONDS    604800000

static mutex_t rtc_mutex;
static uint64_t time_epoch_us = 0;

static struct {
    uint64_t        alarm;
    rtc_alarm_cb_t  cb;
    void*           arg;
} rtc_next_timer_millis;

void rtc_release(void) {
    mutex_unlock(&rtc_mutex);
}

void rtc_acquire(void) {
    mutex_lock(&rtc_mutex);
}

void rtc_ovf_callback(void* arg)
{
    (void) arg;

    /* timer overflows every 2e24 * 976.5625 us = 16 384 000 000 us = ~4.55 hours */
    time_epoch_us += 16384000000;
    
    DEBUG("Timer overflow\n");
    
    if ((rtc_next_timer_millis.alarm != 0) &&
       ((rtc_next_timer_millis.alarm - time_epoch_us) <= RTT_TICKS_TO_US(RTT_MAX_VALUE))) {
           
        uint32_t target = RTT_US_TO_TICKS(rtc_next_timer_millis.alarm - time_epoch_us);
        
        rtc_next_timer_millis.alarm = 0;

        rtt_set_alarm(target, rtc_next_timer_millis.cb, rtc_next_timer_millis.arg);
    }
}

void rtc_init(void)
{
    mutex_init(&rtc_mutex);
    rtc_acquire();
    
    rtc_next_timer_millis.alarm = 0;
    
    /* enable low frequency clock */
    rtt_init();
    
    /* overflow callback */
    rtt_set_overflow_cb(&rtc_ovf_callback, NULL);
    
    rtc_release();
}

int rtc_set_time(struct tm *time)
{
    /* not implemented yet */
    rtc_acquire();

    (void) time;
    
    rtc_release();

    return 0;
}

int rtc_get_time(struct tm *time)
{
    rtc_acquire();
    
    uint32_t ticks = rtt_get_counter();
    time_t epoch = (time_epoch_us + RTT_TICKS_TO_US(ticks)) / 1000000;
    memcpy(&time, gmtime(&epoch), sizeof(struct tm));
    
    DEBUG("Epoch time: %" PRIu64 " ms\n", epoch);
    
    rtc_release();

    return 0;
}

int rtc_set_alarm(struct tm *time, rtc_alarm_cb_t cb, void *arg)
{
    /* not implemented yet */
    rtc_acquire();
    
    (void) time;
    (void) cb;
    (void) arg;
    
    rtc_release();

    return 0;
}

int rtc_get_alarm(struct tm *time)
{
    /* not implemented yet */
    rtc_acquire();
    
    (void) time;
    
    rtc_release();

    return 0;
}

void rtc_clear_alarm(void)
{
    /* not implemented yet */
    rtc_acquire();
    
    rtc_release();
}

int rtc_millis_set_alarm(uint32_t milliseconds, rtc_alarm_cb_t cb, void *arg)
{   
    rtc_acquire();

    uint32_t target = RTT_MS_TO_TICKS(milliseconds - (uint32_t)(time_epoch_us/1000));
    
    DEBUG("Alarm at %lu ms\n", milliseconds);

    if (target <= RTT_MAX_VALUE) {
        DEBUG("Alarm now at %lu ticks\n", target);
        rtt_set_alarm(target, cb, arg);
    } else {
        /* alarm absolute time */
        rtc_next_timer_millis.alarm = (uint64_t)(milliseconds) * 1000;
        rtc_next_timer_millis.arg = arg;
        rtc_next_timer_millis.cb = cb;
        DEBUG("Alarm next period: %lu ms\n", (uint32_t)(rtc_next_timer_millis.alarm/1000));
    }

    rtc_release();

    return 0;
}

void rtc_millis_clear_alarm(void)
{
    rtc_acquire();
    
    rtt_clear_alarm();
    rtc_next_timer_millis.alarm = 0;

    rtc_release();
}

int rtc_millis_get_time(uint32_t *millis)
{
    rtc_acquire();
    
    /* 1 week overflow */
    uint32_t now = rtt_get_counter();
    *millis = (RTT_TICKS_TO_MS(now) +  time_epoch_us/1000) % RTC_WEEK_MILLISECONDS;
    
    DEBUG("Now %lu ticks (%lu ms)\n", now, *millis);
    
    rtc_release();
    return 0;
}

int rtc_set_wakeup(uint32_t period_us, rtc_wkup_cb_t cb, void *arg)
{
    /* not implemented yet */
    rtc_acquire();
    
    (void) period_us;
    (void) cb;
    (void) arg;

    rtc_release();

    return 0;
}

void rtc_enable_wakeup(void) {
    /* not implemented yet */
    rtc_acquire();

    rtc_release();
}

void rtc_disable_wakeup(void)
{
    /* not implemented yet */
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
