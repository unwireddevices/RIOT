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

#define ADC_PERIOD_US       2

#define ADC_BUF_SIZE    500
uint16_t adc_buf[ADC_BUF_SIZE];

#define TOTAL_ACQ_TIME_MS   20

#define ADC_BUF_WINDOW      ((1000 * TOTAL_ACQ_TIME_MS) / AMPL_BUF_SIZE)/ADC_PERIOD_US
typedef struct {
    uint32_t time;
    uint16_t ampl;
} adc_amplitude_t;

#define AMPL_BUF_SIZE       1000
uint16_t ampl_buf[AMPL_BUF_SIZE];

volatile uint16_t adc_avg_value = 0;
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

/* calculate speed of sound at given temperature */
/* https://en.wikipedia.org/wiki/Speed_of_sound  */
int speed_of_sound(int temperature) {
    return UZ_SOUNDSPEED_CONSTANT * isqrt(temperature * 1024) / 32; // 0.02005 * sqrt(temperature)
}

/* speed of sound is 0.34 mm/us, so we multiply microseconds by 0.17 and get millimeters */
int ultrasoundrange_us_to_mm(ultrasoundrange_t *dev, int us) {
    return us * dev->half_speed_of_sound / UZ_SOUNDSPEED_DIVISOR;
}

/* speed of sound is 0.34 mm/us, so we divide millimeters by 0.17 and get microseconds */
int ultrasoundrange_mm_to_us(ultrasoundrange_t *dev, int mm) {
    return mm * UZ_SOUNDSPEED_DIVISOR / dev->half_speed_of_sound;
}

int calibrate_sound_speed(ultrasoundrange_t *dev, bool measure_temperature) {
    int t = UZ_DEFAULT_TEMPERATURE;
    if (measure_temperature)
        t = adc_measure_temperature();
    dev->temperature = t;
    int s = speed_of_sound(t) / 2;
    dev->half_speed_of_sound = s;
    DEBUG("half_speed_of_sound: %d / %d mm/s\n", s, UZ_SOUNDSPEED_DIVISOR);
    return s;
}
#endif

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
    (void)dev;

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

volatile uint16_t ampl_index;

void adc_callback(adc_dma_event_t event) {
    if (adc_avg_value == 0) {
        uint32_t avg = 0;
        for (int i = 0; i < ADC_BUF_SIZE; i++) {
            avg += adc_buf[i];
        }
        adc_avg_value = (avg/ADC_BUF_SIZE);
        return;
    }

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
}

uint32_t ultrasoundrange_measure(ultrasoundrange_t *dev)
{
    assert(dev != NULL);
    
    /* chip temperature in °C */
    int temp = -100;
    if (adc_init(ADC_LINE(ADC_TEMPERATURE_INDEX)) == 0) {
         temp = adc_sample(ADC_LINE(ADC_TEMPERATURE_INDEX), ADC_RES_12BIT);
    }
    printf("AMBIENT TEMP: %d C\n", temp);
    
    gpio_init(dev->signal_pin, GPIO_OUT);
    gpio_clear(dev->signal_pin);
    
    gpio_init(dev->suppress_pin, GPIO_OUT);
    gpio_clear(dev->suppress_pin);
    
    /* prepare ADC */
    adc_init(dev->adc);
    
    /* measure virtual zero point */
    adc_avg_value = 0;
    adc_sampling_start(dev->adc, ADC_RES_12BIT, adc_buf, ADC_BUF_SIZE, adc_callback, ADC_CONTINUOUS_SINGLE);
    /* 2 us sampling period for 500 kHz sampling rate */
    timer_init_periodic(dev->timer, ADC_PERIOD_US, NULL, NULL, true);
    
    /* wait for ADC to complete */
    while (!adc_avg_value) {};
    
    printf ("AVG: %d\n", adc_avg_value);
    timer_stop(dev->timer);
    
    /* measure actual echo */
    ampl_index = 0;
    adc_sampling_start(dev->adc, ADC_RES_12BIT, adc_buf, ADC_BUF_SIZE, adc_callback, ADC_CONTINUOUS_CIRCULAR);
    
    /* generate some 40 kHz pulses */
    /* 1 pulse is 25 us */
    pwm_init(dev->pwm, PWM_LEFT, 40000, 10);
    pwm_set(dev->pwm, dev->pwm_channel, 5);
    
    /* save time */
    //uint32_t start = xtimer_now_usec();
    
    /* generate ultrasonic pulses */
    pwm_pulses(dev->pwm, dev->pwm_channel, dev->pulses);
    
    /* suppress transducer ringing */
    gpio_set(dev->suppress_pin);
    xtimer_usleep(400);
    gpio_clear(dev->suppress_pin);
    
    /* enable signal pass-through */
    gpio_set(dev->signal_pin);
    
    /* start ADC acquisition */
    timer_init_periodic(dev->timer, ADC_PERIOD_US, NULL, NULL, true);

    /* wait for acquisition to finish */
    while (ampl_index != 0xFFFF) {};

    timer_stop(dev->timer);
    adc_sampling_stop();
    
    /* process data */
    for (int i = 0; i < AMPL_BUF_SIZE; i += 10) {
        for (int k = 0; k < 10; k++) {
            printf("%04d ", ampl_buf[i + k]);
        }
        printf("\n");
        rtctimers_millis_sleep(10);
    }
    
    return 0;
}

#ifdef __cplusplus
}
#endif
