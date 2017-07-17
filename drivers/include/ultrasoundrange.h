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
 * @file		ultrasoundrange.h
 * @brief       driver for ultrasound rangefinder
 * @author      Dmitry Golik [info@unwds.com]
 */
#ifndef ULTRASOUNDRANGE_H_
#define ULTRASOUNDRANGE_H_

#include "thread.h"
#include "periph/i2c.h"

#include <stdlib.h>
#include <math.h>

// ultrasound definitions!
// TODO: move definitions somewhere else

// divisor of sub-microsecond time setting in 1/us, e. g. if UZ_PRECISION 10, then 5 is 0.5 us
#define UZ_SUBUS_DIVISOR 10

// 1000000/40000 is 25
// #define UZ_PERIOD_US (1000000/40000)
// 22 is actually about 25.9 us
#define UZ_PERIOD_US 24
#define UZ_PERIOD_SUBUS 5
#define UZ_DUTY 50

#define UZ_IDLE_PERIOD_US 13
#define UZ_DUTY2 50

// maximum time for waiting for echo - 30 ms is about 10 m
// not in milliseconds now..
#define UZ_MAX_TIMEOUT_MS 30

#define UZ_TRANSMIT_PULSES 10
#define UZ_SILENCING_PULSES 3
#define UZ_MAX_PULSES 1024


// PWM definitions, TODO: use definitions from riot
#define UMDK_PWM_0 0
#define UMDK_PWM_CH_0 0
#define UMDK_PWM_CH_1 1
#define UMDK_PWM_CH_2 2
#define UMDK_PWM_CH_3 3

/**
 * @brief Structure that holds the USOUNDRANGE driver internal state and parameters
 */
typedef struct {
	// gpio_t threshold_pin;					/**< Sensor "enable" pin */
	gpio_t silencing_pin;					/**< Silencing pin when used with 1 transceiver */
	gpio_t sens_pin;				/**< GPIO pin on which sensor is attached */
	gpio_t t1_pin;				/**< GPIO pin on which sensor is attached */
	gpio_t t2_pin;				/**< GPIO pin on which sensor is attached */
    int transmit_pulses;
    int silencing_pulses;
    int period_us;
    // int period_subus;
    int chirp;
    int idle_period_us;
    int duty;
    int duty2;
    bool verbose;
} ultrasoundrange_t;


typedef struct {
    uint32_t range;        /**< */
} ultrasoundrange_measure_t;

// some math

typedef int fix16;

// fix16 _fast_sin(fix16 x);
// fix16 _fast_cos(fix16 x);
// fix16 _fast_cos_1(fix16 x);
fix16 fast_sin(unsigned int x); // full period is 2**32
fix16 fast_cos(unsigned int x); // full period is 2**32

// fix16 _atan_t7(fix16 d);
// fix16 _atan_p4(fix16 d);
// fix16 _atan2(int y, int x);
fix16 fast_atan2(int y, int x); // |y| or |x| must be less than 32768 (or 0.5 in fix16)

/**
 * @brief USOUNDRANGE driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param USOUNDRANGE driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int ultrasoundrange_init(ultrasoundrange_t *dev);

/**
 * @brief Get USOUNDRANGE measure
 *
 * @param[in] dev pointer to the initialized USOUNDRANGE device
 * @param[in] measure pointer to the allocated memory
 *
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * ultrasoundrange_measure_t measure;
 *
 * ultrasoundrange_init(usoundrange);
 * ultrasoundrange_measure(usoundrange, &measure)
 */
uint32_t ultrasoundrange_measure(ultrasoundrange_t *dev, ultrasoundrange_measure_t *measure);

#endif /* ULTRASOUNDRANGE_H_ */

