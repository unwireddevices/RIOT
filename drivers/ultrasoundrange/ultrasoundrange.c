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
 * @file        ultrasoundrange.c
 * @brief       basic driver for Ultrasonic Rangefinder sensor
 * @author      Dmitry Golik [info@unwds.com]
 * @author      Oleg Artamonov [info@unwds.com]
 */


#include "ultrasoundrange.h"
#include "periph/gpio.h"
#include "periph/timer.h"
#include "periph/pwm.h"
#include "random.h"
#include "xtimer.h"
#include "rtctimers-millis.h"
#include "periph/adc.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USOUND_MIN_DISTANCE_MM      300
#define USOUND_NOISE_THRESHOLD      125
#define USOUND_SIGNAL_THRESHOLD     150
#define USOUND_DEFAULT_FREQ         40000

#define USOUND_SUPRESS_PERIOD_US    400

/* speed of sound at normal pressure and 0 °C */
#define SOUND_SPEED_AT_0C_100HPA    331

/* 2 us sampling period */
#define ADC_PERIOD_US       2

/* 2*250 samples ADC DMA buffer */
#define ADC_BUF_SIZE        500
uint16_t adc_buf[ADC_BUF_SIZE];

#define TOTAL_ACQ_TIME_MS   20

/* 50 us window amplitude calculation, 25 samples per half of DMA buffer */
#define ADC_BUF_WINDOW      (50/ADC_PERIOD_US)

/* 2*20000/50 = 800 samples total for < 20 mm resolution */
#define AMPL_BUF_SIZE       ((1000*2*TOTAL_ACQ_TIME_MS)/(25*ADC_PERIOD_US))
static uint16_t ampl_buf[AMPL_BUF_SIZE];

typedef struct {
    uint16_t sample;
    uint16_t ampl;
} adc_amplitude_t;

static volatile uint16_t adc_avg_value = 0;
#if 0
/* http://www.codecodex.com/wiki/Calculate_an_integer_square_root */
int isqrt(uint32_t x)
{
    register uint32_t op, res, one;  
  
    op = x;  
    res = 0;  
  
    /* "one" starts at the highest power of four <= than the argument. */  
    one = 1 << 30;  /* second-to-top bit set */  
    while (one > op) one >>= 2;  
  
    while (one != 0) {  
        if (op >= res + one) {  
            op -= res + one;  
            res += one << 1;
        }  
        res >>= 1;  
        one >>= 2;  
    }
    return res;  
}
#endif

/* rough calculation, delta = 0.59 m/s/°C */
static int speed_of_sound(int temperature) {
    return (SOUND_SPEED_AT_0C_100HPA + (59*temperature + 50)/100);
}

static int us_to_mm(ultrasoundrange_t *dev, int useconds) {
    (void) dev;
    return (((speed_of_sound(dev->temperature))*useconds)/(2*1000));
}

static int mm_to_us(ultrasoundrange_t *dev, int distance) {
    return (2*1000*distance) / speed_of_sound(dev->temperature);
}

static int sample_to_mm(ultrasoundrange_t *dev, int sample) {
    int us = sample*ADC_BUF_WINDOW*ADC_PERIOD_US + USOUND_SUPRESS_PERIOD_US + (dev->pulses*25)/2;
    return us_to_mm(dev, us);
}

static int mm_to_sample(ultrasoundrange_t *dev, int distance) {
    int us = mm_to_us(dev, distance);
    return (us - USOUND_SUPRESS_PERIOD_US - (dev->pulses*25)/2)/(ADC_BUF_WINDOW*ADC_PERIOD_US);
}

/**
 * @brief USOUNDRANGE driver initialization routine
 *
 * @param[out] dev device structure pointer
 * @param[in] param USOUNDRANGE driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of an error
 */
int ultrasoundrange_init(ultrasoundrange_t *dev)
{
    if (dev->frequency == 0) {
        dev->frequency = USOUND_DEFAULT_FREQ;
    }

    return 0;
}
/*
void ultrasoundrange_turn_off(ultrasoundrange_t *dev)
{
    gpio_init(dev->pwren_pin, GPIO_OUT);
    gpio_set(dev->pwren_pin);
}

void ultrasoundrange_turn_on(ultrasoundrange_t *dev)
{
    gpio_init(dev->pwren_pin, GPIO_OUT);
    gpio_clear(dev->pwren_pin);
    rtctimers_millis_sleep(UZ_PWRON_DELAY_MS);
}
*/

typedef enum {
    ADC_MODE_NOISEFLOOR,
    ADC_MODE_CALIBRATE,
    ADC_MODE_MEASURE,
} usound_adc_mode_t;

volatile uint16_t ampl_index;
volatile uint8_t  adc_mode;
volatile bool     adc_callback_finished = false;

void adc_callback(adc_dma_event_t event) {
    switch (adc_mode) {
        case ADC_MODE_NOISEFLOOR: {
            uint32_t avg = 0;
            for (int i = 0; i < ADC_BUF_SIZE; i++) {
                avg += adc_buf[i];
            }
            adc_avg_value = (avg/ADC_BUF_SIZE);
            break;
        }
        case ADC_MODE_MEASURE: {
            uint16_t *start = adc_buf;
            if (event == ADC_DMA_CALLBACK_COMPLETED) {
                start = &adc_buf[ADC_BUF_SIZE/2];
            }

            uint16_t avg = adc_avg_value;
            uint16_t index = ampl_index;
            /* ADC_BUF_WINDOW samples window to find signal amplitude */
            for (int i = 0; i < ADC_BUF_SIZE/2; i += ADC_BUF_WINDOW) {
                uint16_t max = avg;
                uint16_t min = avg;
                for (int k = 0; k < ADC_BUF_WINDOW; k++) {
                    if (min > start[i + k]) {
                        min = start[i + k];
                    }
                    if (max < start[i + k]) {
                        max = start[i + k];
                    }
                }
                if (index < AMPL_BUF_SIZE) {
                    ampl_buf[index++] = max - min;
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

void ultrasoundrange_calibrate(ultrasoundrange_t *dev)
{
    adc_mode = ADC_MODE_CALIBRATE;
    adc_callback_finished = false;
    
    /* setup ADC trigger */
    timer_init_periodic(dev->timer, ADC_PERIOD_US, NULL, NULL, true);

    adc_sampling_start(dev->adc, ADC_RES_12BIT, adc_buf, ADC_BUF_SIZE, adc_callback, ADC_CONTINUOUS_SINGLE);
    
    /* generate a few single pulses */
    pwm_init(dev->pwm, PWM_LEFT, dev->frequency, 10);
    pwm_set(dev->pwm, dev->pwm_channel, 5);
    pwm_pulses(dev->pwm, dev->pwm_channel, 5);
    
    /* enable signal pass-through */
    gpio_set(dev->signal_pin);
    
    /* start ADC acquisition */
    timer_start(dev->timer);

    /* wait for acquisition to finish */
    while (!adc_callback_finished) {};

    gpio_clear(dev->signal_pin);
    timer_stop(dev->timer);
    
#if ENABLE_DEBUG
    for (int i = 0; i < ADC_BUF_SIZE; i += 10) {
        for (int k = 0; k < 10; k++) {
            printf("%04d ", adc_buf[i + k]);
        }
        printf("\n");
        rtctimers_millis_sleep(10);
    }
#endif

    int i = 0, k = 0;
    int transitions[20];
    
    while (adc_buf[i] < 500) {
        i++;
    }
    
    while ((k < 20) && (i < ADC_BUF_SIZE - 1)) {
        while (adc_buf[i] > 2000) {
            i++;
        }
        transitions[k] = i;
        k++;
        
        while (adc_buf[i] <= 2000) {
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

int ultrasoundrange_measure(ultrasoundrange_t *dev)
{
    assert(dev != NULL);
    
    gpio_init(GPIO_PIN(PORT_B, 11), GPIO_OUT);
    gpio_clear(GPIO_PIN(PORT_B, 11));
    
    if (dev->temperature == ULTRASOUNDRANGE_TEMPERATURE_NONE) {
        /* chip temperature in °C */
        if (adc_init(ADC_LINE(ADC_TEMPERATURE_INDEX)) == 0) {
            dev->temperature = adc_sample(ADC_LINE(ADC_TEMPERATURE_INDEX), ADC_RES_12BIT);
        }
    }
    DEBUG("Ambient temperature: %d C\n", dev->temperature);
    
    gpio_init(dev->signal_pin, GPIO_OUT);
    gpio_clear(dev->signal_pin);
    
    gpio_init(dev->suppress_pin, GPIO_OUT);
    gpio_clear(dev->suppress_pin);
    
    /* prepare ADC */
    adc_init(dev->adc);
    
    /* measure virtual zero point */
    adc_mode = ADC_MODE_NOISEFLOOR;
    adc_callback_finished = false;

    adc_sampling_start(dev->adc, ADC_RES_12BIT, adc_buf, ADC_BUF_SIZE, adc_callback, ADC_CONTINUOUS_SINGLE);
    /* 2 us sampling period for 500 kHz sampling rate */
    timer_init_periodic(dev->timer, ADC_PERIOD_US, NULL, NULL, true);
    
    /* start ADC acquisition */
    gpio_set(dev->signal_pin);
    timer_start(dev->timer);
    
    /* wait for ADC to complete */
    while (!adc_callback_finished) {};
    
    DEBUG("Zero signal: %d\n", adc_avg_value);
    gpio_clear(dev->signal_pin);
    timer_stop(dev->timer);
    
    /* measure actual echo */
    adc_mode = ADC_MODE_MEASURE;
    ampl_index = 0;
    adc_sampling_start(dev->adc, ADC_RES_12BIT, adc_buf, ADC_BUF_SIZE, adc_callback, ADC_CONTINUOUS_CIRCULAR);
    
    /* setup ADC trigger */
    timer_init_periodic(dev->timer, ADC_PERIOD_US, NULL, NULL, true);
    
    /* generate some 40 kHz pulses */
    /* 1 pulse is 25 us */
    pwm_init(dev->pwm, PWM_LEFT, dev->frequency, 10);
    pwm_set(dev->pwm, dev->pwm_channel, 5);
    
    /* generate ultrasonic pulses */
    pwm_pulses(dev->pwm, dev->pwm_channel, dev->pulses);

    /* suppress transducer ringing */
    gpio_set(dev->suppress_pin);
    xtimer_usleep(USOUND_SUPRESS_PERIOD_US);
    gpio_clear(dev->suppress_pin);
    
    /* enable signal pass-through */
    gpio_set(dev->signal_pin);
    
    /* start ADC acquisition */
    timer_start(dev->timer);

    /* wait for acquisition to finish */
    while (ampl_index != 0xFFFF) {};

    gpio_clear(dev->signal_pin);
    timer_stop(dev->timer);
    adc_sampling_stop();
    
    /* process data */
#if ENABLE_DEBUG
    for (int i = 0; i < AMPL_BUF_SIZE; i += 10) {
        for (int k = 0; k < 10; k++) {
            printf("%04d ", ampl_buf[i + k]);
        }
        printf("\n");
        rtctimers_millis_sleep(10);
    }
#endif

    int i = 0;
    int start = mm_to_sample(dev, USOUND_MIN_DISTANCE_MM - 50);
    for (i = 10; i < start; i++) {
        /* without close obstacles function should be monotonycally declining here */
        if ((ampl_buf[i] < ampl_buf[i + 1]) &&
            (ampl_buf[i + 1] < ampl_buf[i + 2]) && 
            (ampl_buf[i + 2] < ampl_buf[i + 3])) {

            DEBUG("Obstacle closer than %d mm\n", USOUND_MIN_DISTANCE_MM);
            return -USOUND_MINDISTANCE;
        }
    }
    
    /*
    for (i = start; i < AMPL_BUF_SIZE; i++) {
        if (ampl_buf[i] < USOUND_NOISE_THRESHOLD) {
            start = i;
            break;
        }
    }
    
    if (i == (AMPL_BUF_SIZE - 1)) {
        DEBUG("Signal too noisy\n");
        return -USOUND_NOISY;
    }
    */
    int max = USOUND_SIGNAL_THRESHOLD;
    int maxindex = 0;
    for (i = start; i < AMPL_BUF_SIZE; i++) {
        if (ampl_buf[i] > max) {
            max = ampl_buf[i];
            maxindex = i;
        }
    }
    
    if (!maxindex) {
        DEBUG("No reliable peaks detected\n");
        return -USOUND_MAXDISTANCE;
    }
    
    DEBUG("Distance: %d mm at sample %d\n", sample_to_mm(dev, maxindex), maxindex);
    
    return sample_to_mm(dev, maxindex);
}

#ifdef __cplusplus
}
#endif
