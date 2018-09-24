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

#include "thread.h"
#include "periph/i2c.h"

#include <stdlib.h>
#include <math.h>

// maximum quantity of registered echo peaks
#define UZ_MAX_PEAKS 32

// maximum adc output value
#define UZ_MAX_ADC 4096
#define UZ_HALF_ADC (UZ_MAX_ADC / 2)

// divisor for normalizing amplitudes (multiplying them by distance and dividing by this to get more uniform peak heights across distance)
#define UZ_NORMALIZING_DIVISOR 2048

// divisor of sub-microsecond time setting in 1/us, e. g. if UZ_SUBUS_DIVISOR is 8, then 5 is 0.625 us
// should be power of 2
#define UZ_SUBUS_DIVISOR 32
#define UZ_QUARTER_PERIOD_DIVISOR (UZ_SUBUS_DIVISOR * 4)

// number of periods to average echo signal over
#define UZ_AVERAGING_PERIODS 8

// size of arrays for analog measurement
// #define UZ_MAX_READS 32
#define UZ_MAX_READS 1024

// default parameters for ultrasonic rangefinder
#define UZ_DEFAULT_TRANSMIT_PULSES 10
#define UZ_DEFAULT_SILENCING_PULSES 4
#define UZ_DEFAULT_PERIOD_US 790
#define UZ_DEFAULT_SILENCING_PERIOD_US 1000
#define UZ_DEFAULT_IDLE_PERIOD_US 400
#define UZ_DEFAULT_DUTY 390
#define UZ_DEFAULT_DUTY2 250
#define UZ_DEFAULT_SENSITIVITY 50
#define UZ_DEFAULT_SENSITIVITY2 17
#define UZ_DEFAULT_MIN_DISTANCE 300
#define UZ_DEFAULT_MAX_DISTANCE 6000


// pins and channels
#define UZ_DISRUPTING_PIN UNWD_GPIO_4
#define UZ_SILENCING_PIN UNWD_GPIO_25
#define UZ_BEEPING_PIN UNWD_GPIO_26
#define UZ_ADC_PIN UNWD_GPIO_5
#define UZ_ADC_CHANNEL 5
#define UZ_PWREN_PIN GPIO_PIN(PORT_B, 1)

// delay for charging capacitors in power and reference
#define UZ_PWRON_DELAY_MS 10

// speed of sound at 288K is 0.34 mm/us, so half speed of sound is 1392 / 8192 mm/us
#define UZ_DEFAULT_TEMPERATURE 288
// #define UZ_DEFAULT_HALFSOUNDSPEED 1392
#define UZ_SOUNDSPEED_DIVISOR 8192
// speed of sound = 20.05 * sqrt(temperature) m/s = 164 / 8192 * sqrt(temperature) mm/s
#define UZ_SOUNDSPEED_CONSTANT 164

/**
 * @brief Structure that holds the USOUNDRANGE driver internal state and parameters
 */
typedef struct {
    gpio_t disrupting_pin;
    gpio_t silencing_pin;
    gpio_t beeping_pin;
    gpio_t adc_pin;
    gpio_t adc_channel;
    gpio_t pwren_pin;
    int transmit_pulses;        // number of pulses for beeping
    int silencing_pulses;       // number of pulses for counterphase beeping
    int period_us;              // period of oscillation
    int silencing_period_us;    // duration of antiresonance silencing
    int idle_period_us;         // time between end of beeping and beginning of counterphase beeping
    int duty;                   // duty cycle of beeping
    int duty2;                  // duty cycle of counterphase beeping
    uint16_t sensitivity;       // sensitivity of echo detection
    uint16_t sensitivity2;      // sensitivity of echo detection (distance dependent)
    uint16_t min_distance;      // minimum distance of echo detection
    uint16_t max_distance;      // maximum distance of echo detection
    int temperature;
    int half_speed_of_sound;
} ultrasoundrange_t;


typedef struct {
    uint32_t range;        /**< */
} ultrasoundrange_measure_t;

// some math

int ultrasoundrange_us_to_mm(ultrasoundrange_t *dev, int us);
int ultrasoundrange_mm_to_us(ultrasoundrange_t *dev, int mm);


// int adc_read(uint8_t ch);
// int * adc_read_multi(ultrasoundrange_t *dev, int n, int begin_time);
int * ultrasoundrange_measure_analog(ultrasoundrange_t *dev, int n);


/**
 * @brief USOUNDRANGE driver initialization routine
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

int ultrasoundrange_tune(ultrasoundrange_t *dev, int * period, int * idle, int * duty2, int * silencing, int point_n, int max_time, int tolerance, int alpha);
int ultrasoundrange_test(ultrasoundrange_t *dev);
int ultrasoundrange_transmit(ultrasoundrange_t *dev);
int ultrasoundrange_optimize_counterphase(ultrasoundrange_t *dev, int * idle, int * duty2, int point_n, int min_idle, int min_duty2, int max_idle, int max_duty2, int max_time, int tolerance, int alpha, bool verbose);
void ultrasoundrange_turn_off(ultrasoundrange_t *dev);
void ultrasoundrange_turn_on(ultrasoundrange_t *dev);

#endif /* ULTRASOUNDRANGE_H_ */

