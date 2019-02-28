/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @file        ultrasoundrange.h
 * @brief       driver for ultrasonic rangefinder
 * @author      Dmitry Golik [info@unwds.com]
 */
#ifndef ULTRASOUNDRANGE_H_
#define ULTRASOUNDRANGE_H_

#include "periph/pwm.h"
#include "periph/adc.h"
#include "periph/timer.h"

/**
 * @brief Structure that holds the USOUNDRANGE driver internal state and parameters
 */
typedef struct {
    pwm_t   pwm;
    uint8_t pwm_channel;
    adc_t   adc;
    uint8_t adc_channel;
    uint8_t pulses;
    timer_t timer;
    gpio_t  signal_pin;
    gpio_t  suppress_pin;
} ultrasoundrange_t;

int ultrasoundrange_init(ultrasoundrange_t *dev);

void ultrasoundrange_turn_on(ultrasoundrange_t *dev);
void ultrasoundrange_turn_off(ultrasoundrange_t *dev);
uint32_t ultrasoundrange_measure(ultrasoundrange_t *dev);

#endif /* ULTRASOUNDRANGE_H_ */

