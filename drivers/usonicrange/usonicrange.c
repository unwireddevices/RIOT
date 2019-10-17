/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
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
 * @file        usonicrange.c
 * @brief       basic driver for Ultrasonic Rangefinder sensor
 * @author      Dmitry Golik [info@unwds.com]
 * @author      Oleg Artamonov [info@unwds.com]
 */


#include "usonicrange.h"
#include "periph/gpio.h"
#include "periph/timer.h"
#include "periph/pwm.h"
#include "random.h"
#include "xtimer.h"
#include "lptimer.h"
#include "periph/adc.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/* speed of sound at normal pressure and 0 °C */
#define SOUND_SPEED_AT_0C_100HPA    331

typedef struct {
    uint16_t sample;
    uint16_t ampl;
} adc_amplitude_t;

typedef enum {
    ADC_MODE_NOISEFLOOR,
    ADC_MODE_CALIBRATE,
    ADC_MODE_MEASURE,
} usound_adc_mode_t;

static uint8_t           adc_mode;
static uint16_t          adc_avg_value = 0;
static volatile uint32_t ampl_index;
static volatile bool     adc_callback_finished = false;
static volatile uint32_t signal_max_samples;

static uint16_t *dmabuffer_local;
static uint16_t *signalbuffer_local;

/* rough calculation, linear coefficient 0.59 m/s/°C */
static int speed_of_sound(int temperature) {
    return (SOUND_SPEED_AT_0C_100HPA + (59*temperature + 50)/100);
}

/* convert microseconds to millimeters */
static int us_to_mm(usonicrange_t *dev, int useconds) {
    (void) dev;
    return (((speed_of_sound(dev->temperature))*useconds)/(2*1000));
}

/* convert millimeters to microseconds */
static int mm_to_us(usonicrange_t *dev, int distance) {
    return (2*1000*distance) / speed_of_sound(dev->temperature);
}

/* convert amplitude samples to millimeters */
static int sample_to_mm(usonicrange_t *dev, int sample) {
    int us = sample*USONICRANGE_DMABUF_WINDOW_SIZE*USONICRANGE_ADC_PERIOD_US + USONICRANGE_DAMPING_PERIOD_US;
    return us_to_mm(dev, us);
}

/* convert millimeters to amplitude samples */
static int mm_to_sample(usonicrange_t *dev, int distance) {
    int us = mm_to_us(dev, distance);
    return (us - USONICRANGE_DAMPING_PERIOD_US)/(USONICRANGE_DMABUF_WINDOW_SIZE*USONICRANGE_ADC_PERIOD_US);
}

void usonicrange_poweron(usonicrange_t *dev) {
    if (dev->power_active) {
        gpio_set(dev->power_pin);
    } else {
        gpio_clear(dev->power_pin);
    }
}

void usonicrange_poweroff(usonicrange_t *dev) {
    if (dev->power_active) {
        gpio_clear(dev->power_pin);
    } else {
        gpio_set(dev->power_pin);
    }
}

/* initialize hardware */
int usonicrange_init(usonicrange_t *dev)
{
    if (dev->frequency == 0) {
        dev->frequency = USONICRANGE_DEFAULT_FREQ;
    }
    
    if (dev->damping_time == 0) {
        dev->damping_time = USONICRANGE_DAMPING_PERIOD_US;
    }

    dmabuffer_local = dev->dmabuffer;
    signalbuffer_local = dev->signalbuffer;

    /* initialize GPIOs */
    gpio_init(dev->rx_pin, GPIO_OUT);
    gpio_set(dev->rx_pin); /* signal to be always on */

    gpio_init(dev->damping_pin, GPIO_OUT);
    gpio_clear(dev->damping_pin);
    
    gpio_init(dev->power_pin, GPIO_OUT);
    usonicrange_poweroff(dev);

    return 0;
}

/* DMA callback with ADC data */
void adc_callback(adc_dma_event_t event) {
    switch (adc_mode) {
        case ADC_MODE_NOISEFLOOR: {
            uint32_t avg = 0;
            for (int i = 0; i < USONICRANGE_DMABUF_SIZE; i++) {
                avg += dmabuffer_local[i];
            }
            adc_avg_value = (avg/USONICRANGE_DMABUF_SIZE);
            break;
        }
        case ADC_MODE_MEASURE: {
            uint16_t *start = dmabuffer_local;
            if (event == ADC_DMA_CALLBACK_COMPLETED) {
                start = &dmabuffer_local[USONICRANGE_DMABUF_SIZE/2];
            }

            uint16_t avg = adc_avg_value;
            uint16_t index = ampl_index;
            /* USONICRANGE_DMABUF_WINDOW_SIZE samples window to find signal amplitude */
            for (int i = 0; i < USONICRANGE_DMABUF_SIZE/2; i += USONICRANGE_DMABUF_WINDOW_SIZE) {
                uint16_t max = avg;
                uint16_t min = avg;
                for (int k = 0; k < USONICRANGE_DMABUF_WINDOW_SIZE; k++) {
                    if (min > start[i + k]) {
                        min = start[i + k];
                    }
                    if (max < start[i + k]) {
                        max = start[i + k];
                    }
                }
                if (index < signal_max_samples) {
                    signalbuffer_local[index++] = max - min;
                } else {
                    index = 0xFFFF;
                }
            }
            ampl_index = index;
            break;
        }
        default:
            break;
    }

    adc_callback_finished = true;
}

/* determine transducer resonance frequency */
void usonicrange_calibrate(usonicrange_t *dev)
{
    adc_mode = ADC_MODE_CALIBRATE;
    adc_callback_finished = false;

    /* setup ADC trigger */
    timer_init_periodic(dev->timer, USONICRANGE_ADC_PERIOD_US, NULL, NULL, true);

    adc_sampling_start(dev->adc, ADC_RES_12BIT, dev->dmabuffer, USONICRANGE_DMABUF_SIZE, adc_callback, ADC_CONTINUOUS_SINGLE);

    /* generate a few single pulses */
    pwm_init(dev->pwm, PWM_LEFT, dev->frequency, 10);
    pwm_set(dev->pwm, dev->pwm_channel, 5);
    pwm_pulses(dev->pwm, dev->pwm_channel, 5);

    /* start ADC acquisition */
    timer_start(dev->timer);

    /* wait for acquisition to finish */
    while (!adc_callback_finished) {};

    timer_stop(dev->timer);

#if ENABLE_DEBUG
    for (int i = 0; i < USONICRANGE_DMABUF_SIZE; i += 10) {
        for (int k = 0; k < 10; k++) {
            printf("%04d ", dev->dmabuffer[i + k]);
        }
        printf("\n");
        lptimer_sleep(10);
    }
#endif

    int i = 0, k = 0;
    int transitions[20];

    while (dev->dmabuffer[i] < 500) {
        i++;
    }

    while ((k < 20) && (i < USONICRANGE_DMABUF_SIZE - 1)) {
        while (dev->dmabuffer[i] > 2000) {
            i++;
        }
        transitions[k] = i;
        k++;

        while (dev->dmabuffer[i] <= 2000) {
            i++;
        }
    }

    if (k < 2) {
        DEBUG("No sensor response\n");
        return;
    }

    int period = 0;

    for (i = 0; i < k; i += 2) {
        period += (transitions[i+1] - transitions[i]);
    }
    period = (2*10*period)/k; /* cycles x10 */

    dev->frequency = 10000000 / period;

    DEBUG("Actual frequency %lu Hz\n", dev->frequency);
}

/* measure distance in millimeters */
static int usound_measure_distance(usonicrange_t *dev) {
    /* prepare ADC */
    adc_init(dev->adc);

    /* measure virtual zero value */
    adc_mode = ADC_MODE_NOISEFLOOR;
    adc_callback_finished = false;

    adc_sampling_start(dev->adc, ADC_RES_12BIT, dev->dmabuffer, USONICRANGE_DMABUF_SIZE, adc_callback, ADC_CONTINUOUS_SINGLE);
    /* 2 us sampling period for 500 kHz sampling rate */
    timer_init_periodic(dev->timer, USONICRANGE_ADC_PERIOD_US, NULL, NULL, true);

    /* start ADC acquisition */
    timer_start(dev->timer);

    /* wait for ADC to complete */
    while (!adc_callback_finished) {};

    DEBUG("Zero signal: %d\n", adc_avg_value);
    timer_stop(dev->timer);

    int result = -USONICRANGE_MAXDISTANCE;
    int start = 0;
    int stop = 0;
    int max = USONICRANGE_SIGNAL_THRESHOLD;
    int maxindex = 0;

    /* measure actual echo in 3 distance/power ranges */
    for (int range = 0; range < 3; range++) {
        switch (range) {
            case 0: /* USONICRANGE_MIN_DISTANCE_MM to 2000 mm range */
                start = mm_to_sample(dev, USONICRANGE_MIN_DISTANCE_MM);
                stop = mm_to_sample(dev, 2000);
                break;
            case 1: /* 2000 to 4000 mm range */
                start = stop + 1;
                stop = mm_to_sample(dev, 4000);
                break;
            case 2: /* 4000 to 6000 mm range */
                start = stop + 1;
                stop = USONICRANGE_SIGNALBUF_SIZE;
                break;
            default: /* should never happen */
                break;
        }

        DEBUG("Range %d, samples %d to %d\n", range, start, stop);

        signal_max_samples = stop;

        adc_mode = ADC_MODE_MEASURE;
        ampl_index = 0;
        adc_sampling_start(dev->adc, ADC_RES_12BIT, dev->dmabuffer, USONICRANGE_DMABUF_SIZE, adc_callback, ADC_CONTINUOUS_CIRCULAR);

        /* setup ADC trigger */
        timer_init_periodic(dev->timer, USONICRANGE_ADC_PERIOD_US, NULL, NULL, true);

        /* generate some 40 kHz pulses */
        /* 1 pulse is 25 us */
        pwm_init(dev->pwm, PWM_LEFT, dev->frequency, 10);
        pwm_set(dev->pwm, dev->pwm_channel, 5);

        /* generate ultrasonic pulses */
        pwm_pulses(dev->pwm, dev->pwm_channel, range*10 + 5);

        /* suppress transducer ringing */
        gpio_set(dev->damping_pin);
        xtimer_usleep(dev->damping_time);
        gpio_clear(dev->damping_pin);

        /* start ADC acquisition */
        timer_start(dev->timer);

        /* wait for acquisition to finish */
        while (ampl_index != 0xFFFF) {};

        timer_stop(dev->timer);
        adc_sampling_stop();

        /* process data */
    #if ENABLE_DEBUG
        for (int i = 0; i < USONICRANGE_SIGNALBUF_SIZE; i += 10) {
            for (int k = 0; k < 10; k++) {
                printf("%04d ", dev->signalbuffer[i + k]);
            }
            printf("\n");
            lptimer_sleep(10);
        }
    #endif

        int i = 0;

        /* on range 0, check if there are objects closer than USONICRANGE_MIN_DISTANCE_MM */
        if (range == 0) {
            /* need some 50 mm margin to be sure */
            int limit = mm_to_sample(dev, USONICRANGE_MIN_DISTANCE_MM - 50);

            while ((dev->signalbuffer[i] > 3500) && (i < limit)) {
                i++;
            }

            if (i == limit) {
                result = -USONICRANGE_MINDISTANCE;
                break;
            }

            /* on a distance below USONICRANGE_MIN_DISTANCE_MM signal should
             * not have any local maximums above USONICRANGE_SIGNAL_THRESHOLD
             * and finally fall below 750 pts
             */

            if (dev->signalbuffer[limit] > 750) {
                result = -USONICRANGE_MINDISTANCE;
                break;
            }

            for ( ; i < limit; i++) {
                if ((dev->signalbuffer[i + 1] > USONICRANGE_SIGNAL_THRESHOLD) &&
                    (dev->signalbuffer[i + 1] > dev->signalbuffer[i]) &&
                    (dev->signalbuffer[i + 2] > dev->signalbuffer[i + 1])) {

                    result = -USONICRANGE_MINDISTANCE;
                    break;
                }
            }
        }

        if (result == -USONICRANGE_MINDISTANCE) {
            DEBUG("Obstacle closer than %d mm\n", USONICRANGE_MIN_DISTANCE_MM);
            break;
        }

        for (i = start; i < stop; i++) {
            if (dev->signalbuffer[i] > max) {
                max = dev->signalbuffer[i];
                maxindex = i;
            }
        }

        if (!maxindex) {
            DEBUG("No reliable peaks detected in range %d\n", range);
            result = -USONICRANGE_MAXDISTANCE;
        } else {
            result = maxindex;
            DEBUG("Distance: %d mm at sample %d (strength %d)\n", sample_to_mm(dev, result), result, max);
            if (dev->mode == USONICRANGE_MODE_FIRST_ECHO) {
                break;
            }
        }

        /* timeout for all echoes to die */
        lptimer_sleep(50);
    }

    if (result > 0) {
        return sample_to_mm(dev, result);
    } else {
        return result;
    }
}

int usonicrange_measure(usonicrange_t *dev)
{
    assert(dev != NULL);
    
    if (dev->temperature == USONICRANGE_TEMPERATURE_NONE) {
        /* chip temperature in °C */
        if (adc_init(ADC_LINE(ADC_TEMPERATURE_INDEX)) == 0) {
            dev->temperature = adc_sample(ADC_LINE(ADC_TEMPERATURE_INDEX), ADC_RES_12BIT);
        }
    }
    DEBUG("Ambient temperature: %d C\n", dev->temperature);

    return usound_measure_distance(dev);
}

#ifdef __cplusplus
}
#endif
