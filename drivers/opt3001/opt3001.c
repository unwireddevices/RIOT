/* Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @file        opt3001.c
 * @brief       basic driver for OPT3001 sensor
 * @authoh      Oleg Artamonov [info@unwds.com]
 */


#include "opt3001.h"
// #include "periph/i2c.h"
#include "periph/gpio.h"

#include "xtimer.h"

#ifdef __cplusplus
extern "C" {
#endif




// TODO: move definitions somewhere else

// 1000000/40000 is 25
#define UZ_PERIOD_US (1000000/40000)
#define UZ_IDLE_PERIOD_US 1000

// maximum time for waiting for echo - 30 ms is about 10 m
#define UZ_MAX_TIMEOUT_MS 30

#define UZ_TRANSMIT_PULSES 8
#define UZ_MAX_PULSES 32

/**
 * @brief OPT3001 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param OPT3001 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of an error
 */
int opt3001_init(opt3001_t *dev)
{
    
    dev-> threshold_pin = UNWD_GPIO_30;                   /**< Sensor "enable" pin */
    dev-> sens_pin = UNWD_GPIO_29;                /**< GPIO pin on which sensor is attached */
    dev-> t1_pin = UNWD_GPIO_28;              /**< GPIO pin on which sensor is attached  - hi-z (GPIO_AIN) while measuring! */
    dev-> t2_pin = UNWD_GPIO_27;              /**< GPIO pin on which sensor is attached  -  ground while measuring! */
    dev-> transmit_pulses = UZ_TRANSMIT_PULSES;
    dev-> period_us = UZ_PERIOD_US / 2;
    dev-> idle_period_us = UZ_IDLE_PERIOD_US;

    gpio_init(dev->sens_pin, GPIO_IN);
    gpio_init(dev->threshold_pin, GPIO_OUT);
    gpio_init(dev->t1_pin, GPIO_OUT);
    gpio_init(dev->t2_pin, GPIO_OUT);

    return 0;
}


/*
static uint32_t read_sensor(opt3001_t *dev) {
    uint16_t data;
    data = OPT3001_CFG;
    i2c_write_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_CONFIG, (char *)&data, 2);
    
    // wait till measurement is finished 
    volatile uint16_t i = 0;
    do {
        for (i=0; i<1000; i++) {}
        i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_CONFIG, (char *)&data, 2);
    } while (!(data & OPT3001_CFG_CRF));

    // read result 
    i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_RESULT, (char *)&data, 2);
    
    // swap bytes, as OPT3001 sends MSB first and I2C driver expects LSB first 
    data = ((data >> 8) | (data << 8));

    // calculate lux per LSB 
    uint8_t exp = (data >> 12) & 0x0F;
    uint32_t lsb_size_x100 = (1 << exp);
    
    // remove 4-bit exponent and leave 12-bit mantissa only 
    data &= 0x0FFF;
    
    return (((uint32_t)data * lsb_size_x100) / 100);
}
*/



/*  examples

    gpio_set(lmt01->en_pin);
    gpio_clear(lmt01->en_pin);
    gpio_init(lmt01->sens_pin, GPIO_IN);
    gpio_curr_value = gpio_read(lmt01->sens_pin);
*/

static uint32_t count_pulses(opt3001_t *dev) {

    int pulse_count = 0;
    int times[UZ_MAX_PULSES] = {};
    uint8_t gpio_prev_value = 0;
    uint8_t gpio_curr_value = 0;
    int prev_time = xtimer_now_usec();
    int curr_time = 0;
    uint16_t timeout_us = 0;

    // gpio_set(dev->threshold_pin); // ?

    // gpio_prev_value = gpio_read(dev->sens_pin);

    // for (int i = 0; i < UZ_MAX_PULSES; i++){
 
    while (pulse_count < UZ_MAX_PULSES) {
        gpio_curr_value = gpio_read(dev->sens_pin);
        if (gpio_curr_value != gpio_prev_value) {
            curr_time = xtimer_now_usec();
            times[pulse_count] = curr_time - prev_time;
            pulse_count++;
            prev_time = curr_time;
            timeout_us = 0;
        } else {
            __asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
            timeout_us++;
        }

        if (pulse_count) { /* at least one pulse was detected */
            if (timeout_us > 1000 * UZ_MAX_TIMEOUT_MS) {
                break;
            }
        } else {
            if (timeout_us > 1000 * UZ_MAX_TIMEOUT_MS) { /* no sensor detected */
                break;
            }   
        }
    }

    gpio_clear(dev->threshold_pin); // ?

    printf("[umdk-opt3001-uz] Pulse number : %d\n[umdk-opt3001-uz] Pulse times:", pulse_count);
    for (int i = 0; i < pulse_count; i++){
        printf(" %d,", times[i]);
    }
    printf("\n");
    return pulse_count;
}

static void transmit(opt3001_t *dev) {
    gpio_init(dev->t1_pin, GPIO_OUT);
    gpio_init(dev->t2_pin, GPIO_OUT);
    for (int i = 0; i < dev->transmit_pulses; i++){
        gpio_clear(dev->t1_pin);
        gpio_set(dev->t2_pin);
        // xtimer_spin(xtimer_ticks_from_usec(dev->period_us * 0.5));
        xtimer_spin(xtimer_ticks_from_usec(dev->period_us));
        gpio_clear(dev->t2_pin);
        gpio_set(dev->t1_pin);
        // xtimer_spin(xtimer_ticks_from_usec(dev->period_us - (dev->period_us * 0.5)));
        xtimer_spin(xtimer_ticks_from_usec(dev->period_us));
    }
    gpio_clear(dev->t1_pin);
    xtimer_spin(xtimer_ticks_from_usec(dev->idle_period_us));
    gpio_init(dev->t1_pin, GPIO_AIN);
    gpio_init(dev->t2_pin, GPIO_OUT);
    
}


/**
 * @brief Gets OPT3001 measure.
 *
 * @param[in] dev pointer to the initialized OPT3001 device
 * @param[in] measure pointer to the allocated memory
 * for retval
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * opt3001_measure_t measure;
 *
 * opt3001_init(opt3001);
 * opt3001_measure(opt3001, &measure)
 */
uint32_t opt3001_measure(opt3001_t *dev, opt3001_measure_t *measure)
{
    assert(dev != NULL);

    // i2c_acquire(dev->i2c);
    // uint32_t lum = read_sensor(dev);
    // i2c_release(dev->i2c);

    transmit(dev);
    count_pulses(dev);

    measure->luminocity = 42;

    return 0;
}


#ifdef __cplusplus
}
#endif
