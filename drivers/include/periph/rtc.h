/*
 * Copyright (C) 2014 Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    drivers_periph_rtc RTC
 * @ingroup     drivers_periph
 * @brief       Low-level RTC (Real Time Clock) peripheral driver
 *
 * @note
 * The values used for setting and getting the time/alarm should
 * conform to the `struct tm` specification.
 * Compare: http://pubs.opengroup.org/onlinepubs/7908799/xsh/time.h.html
 *
 * @{
 * @file
 * @brief       Low-level RTC peripheral driver interface definitions
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 */

#ifndef PERIPH_RTC_H
#define PERIPH_RTC_H

#include <time.h>
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Signature for alarm Callback
 *
 * @param[in] arg           optional argument to put the callback in the right context
 */
typedef void(*rtc_alarm_cb_t)(void *arg);

/**
 * @brief Signaure for wakeup callback
 */
typedef void(*rtc_wkup_cb_t)(void *arg);

/**
 * @brief Initialize RTC module
 */
void rtc_init(void);

/**
 * @brief Set RTC to given time.
 *
 * @param[in] time          Pointer to the struct holding the time to set.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_set_time(struct tm *time);

/**
 * @brief Get current RTC time.
 *
 * @param[out] time         Pointer to the struct to write the time to.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_time(struct tm *time);

/**
 * @brief Set an alarm for RTC to the specified value.
 *
 * @note Any already set alarm will be overwritten.
 *
 * @param[in] time          The value to trigger an alarm when hit.
 * @param[in] cb            Callback executed when alarm is hit.
 * @param[in] arg           Argument passed to callback when alarm is hit.
 *
 * @return  0 for success
 * @return -2 invalid `time` parameter
 * @return -1 other errors
 */
int rtc_set_alarm(struct tm *time, rtc_alarm_cb_t cb, void *arg);

/**
 * @brief Set RTC subseconds alarm
 *
 * @note Any already set alarm will be overwritten.
 *
 * @param[in] milliseconds  Millseconds from now to alarm (max 1000).
 * @param[in] cb            Callback executed when alarm is hit.
 * @param[in] arg           Argument passed to callback when alarm is hit.
 *
 * @return  0 for success
 * @return -2 invalid `time` parameter
 * @return -1 other errors
 */
int rtc_millis_set_alarm(int milliseconds, rtc_alarm_cb_t cb, void *arg);

/**
 * @brief Get current milliseconds
 *
 * @param[out] millis         Pointer to the variable to write the time to.
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_millis_get_time(uint32_t *millis);

/**
 * @brief Set an RTC wakeup timer
 *
 * @note Any already set alarm will be overwritten.
 *
 * @param[in] period_us     Wakeup period in microseconds
 * @param[in] cb            Callback executed when alarm is hit.
 * @param[in] arg           Argument passed to callback when alarm is hit.
 *
 * @return  0 for success
 * @return -2 invalid `time` parameter
 * @return -1 other errors
 */
int rtc_set_wakeup(uint32_t period_us, rtc_wkup_cb_t cb, void *arg);

/**
 * @brief Gets the current alarm setting
 *
 * @param[out]  time        Pointer to structure to receive alarm time
 *
 * @return  0 for success
 * @return -1 an error occurred
 */
int rtc_get_alarm(struct tm *time);

/**
 * @brief Clear any set alarm, do nothing if nothing set
 */
void rtc_clear_alarm(void);

/**
 * @brief Clear any set subsecond alarm, do nothing if nothing set
 */
void rtc_millis_clear_alarm(void);

/**
 * @brief Disable periodic wakeup
 */
void rtc_disable_wakeup(void);

/**
 * @brief Re-enable previously configured periodic wakeup
 */
void rtc_enable_wakeup(void);

/**
 * @brief Turns the RTC hardware module on
 */
void rtc_poweron(void);

/**
 * @brief Saves data to RTC backup registers
 */
int rtc_save_backup(uint32_t data, uint8_t reg_num);

/**
 * @brief Restores data from RTC backup registers
 */
uint32_t rtc_restore_backup(uint8_t reg_num);

/**
 * @brief Turns the RTC hardware module off
 */
void rtc_poweroff(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_RTC_H */
/** @} */
