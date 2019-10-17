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
 * @file        usonicrange.h
 * @brief       driver for ultrasonic rangefinder
 * @author      Dmitry Golik [info@unwds.com]
 * @author      Oleg Artamonov [info@unwds.com]
 */
#ifndef USONICRANGE_H_
#define USONICRANGE_H_

#include "periph/gpio.h"
#include "periph/pwm.h"
#include "periph/adc.h"
#include "periph/timer.h"

/* 2*250 samples ADC DMA buffer */
#define USONICRANGE_DMABUF_SIZE        500

/* 2 us ADC sampling period */
#define USONICRANGE_ADC_PERIOD_US      2

/* 20 ms total ADC acquisition time (~6 m range) */
#define USONICRANGE_TOTAL_ACQ_TIME_MS  20

/* 50 us window amplitude calculation, 25 samples per half of DMA buffer */
#define USONICRANGE_DMABUF_WINDOW_SIZE (50/USONICRANGE_ADC_PERIOD_US)

/* 2*20000/50 = 800 samples total for < 20 mm resolution */
#define USONICRANGE_SIGNALBUF_SIZE     ((1000*2*USONICRANGE_TOTAL_ACQ_TIME_MS)/(25*USONICRANGE_ADC_PERIOD_US))

/* minimum measureable distance */
#define USONICRANGE_MIN_DISTANCE_MM    300

/* ringing suppression period */
#define USONICRANGE_DAMPING_PERIOD_US  300

/* ultrasonic transducer default frequency */
#define USONICRANGE_DEFAULT_FREQ       40000

#define USONICRANGE_NOISE_THRESHOLD    125
#define USONICRANGE_SIGNAL_THRESHOLD   150
#define USONICRANGE_TEMPERATURE_NONE   (-127)

typedef enum {
    USONICRANGE_OK = 0,
    USONICRANGE_MINDISTANCE = 1,
    USONICRANGE_MAXDISTANCE = 2,
} usonicrange_errors_t;

typedef enum {
    USONICRANGE_MODE_FIRST_ECHO = 0,
    USONICRANGE_MODE_STRONGEST_ECHO = 1,
    USONICRANGE_MODE_MULTIPLE_ECHOES = 2, /* not implemented yet;
                                      * probably useless due to multiple false reverberation echoes
                                      * with close objects */
} usonicrange_mode_t;

/**
 * @brief Structure that holds the USONICRANGE driver internal state and parameters
 */
typedef struct {
    pwm_t       pwm;
    uint8_t     pwm_channel;
    adc_t       adc;
    uint32_t    frequency;
    timer_t     timer;
    gpio_t      rx_pin;
    gpio_t      damping_pin;
    uint32_t    damping_time;
    gpio_t      power_pin;
    uint8_t     power_active;
    int8_t      temperature;
    usonicrange_mode_t mode;

    uint16_t *dmabuffer;    /* USONICRANGE_DMABUF_SIZE elements */
    uint16_t *signalbuffer; /* USONICRANGE_SIGNALBUF_SIZE elements */
} usonicrange_t;

int  usonicrange_init(usonicrange_t *dev);
void usonicrange_calibrate(usonicrange_t *dev);
int  usonicrange_measure(usonicrange_t *dev);
void usonicrange_poweron(usonicrange_t *dev);
void usonicrange_poweroff(usonicrange_t *dev);

#endif /* USONICRANGE_H_ */

