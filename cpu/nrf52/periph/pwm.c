/*
 * Copyright (C) 2018 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_nrf52
 * @ingroup     drivers_periph_pwm
 * @{
 *
 * @file
 * @brief       Low-level PWM driver implementation
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 * @}
 */

#include <stdint.h>
#include <string.h>

#include "board.h" 
#include "periph/gpio.h"
#include "periph/pwm.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

#define PWM_MAX_RESOLUTION  (0x7FFF)
#define PWM_CLK             (16000000UL)

typedef struct {
    uint16_t comp[PWM_CHAN]; 
} res_chan_t;

res_chan_t pwm_dev[PWM_NUMOF];

static inline NRF_PWM_Type *dev(pwm_t pwm)
{
    return pwm_config[pwm].dev;
}

/**
 * @brief   Initialize a PWM device
 *
 * The PWM module is based on virtual PWM devices, which can have one or more
 * channels. The PWM devices can be configured to run with a given frequency and
 * resolution, which are always identical for the complete device, hence for
 * every channel on a device.
 *
 * The desired frequency and resolution may not be possible on a given device
 * when chosen too large. In this case the PWM driver will always keep the
 * resolution and decrease the frequency if needed. To verify the correct
 * settings compare the returned value which is the actually set frequency.
 *
 * @param[in] dev           PWM device to initialize
 * @param[in] mode          PWM mode, left, right or center aligned
 * @param[in] freq          PWM frequency in Hz
 * @param[in] res           PWM resolution
 *
 * @return                  actual PWM frequency on success
 * @return                  0 on error
 */
uint32_t pwm_init(pwm_t pwm,
                  pwm_mode_t mode, 
                  uint32_t freq, 
                  uint16_t res)
{
    assert( (pwm < PWM_NUMOF) &&
            ((mode == PWM_RIGHT) || (mode == PWM_CENTER) ) &&
            (res < PWM_MAX_RESOLUTION));

    /* Calculate prescaler */
    uint8_t prescaler;

    if(mode == PWM_CENTER)
        prescaler = PWM_CLK / ((res * 2) * freq); 
    else
        prescaler = PWM_CLK / (res * freq); 

    uint8_t prescaler_reg; 

    uint32_t timer_freq;

    if(prescaler > 128)
        return 0;
    else if(prescaler >= 64)
    {
        prescaler_reg = PWM_PRESCALER_PRESCALER_DIV_128;
        prescaler = 128;
    }
    else if(prescaler >= 32)
    {
        prescaler_reg = PWM_PRESCALER_PRESCALER_DIV_64;
        prescaler = 64;
    }
    else if(prescaler >= 16)
    {
        prescaler_reg = PWM_PRESCALER_PRESCALER_DIV_32;
        prescaler = 32;
    }
    else if(prescaler >= 8)
    {
        prescaler_reg = PWM_PRESCALER_PRESCALER_DIV_16; 
        prescaler = 16;
    }
    else if(prescaler >= 4)
    {
        prescaler_reg = PWM_PRESCALER_PRESCALER_DIV_8; 
        prescaler = 8;
    }
    else if(prescaler >= 2)
    {
        prescaler_reg = PWM_PRESCALER_PRESCALER_DIV_4; 
        prescaler = 4;
    }
    else if(prescaler >= 1)
    {
        prescaler_reg = PWM_PRESCALER_PRESCALER_DIV_2; 
        prescaler = 2;
    }
    else
    {
        prescaler_reg = PWM_PRESCALER_PRESCALER_DIV_1; 
        prescaler = 1; 
    }

    if(mode == PWM_CENTER)
        timer_freq = PWM_CLK / ((res * 2) * prescaler);
    else
        timer_freq = PWM_CLK / (res * prescaler);

    /* Test frequency */
    if(timer_freq > freq)
    {
        DEBUG("Error timer_freq > freq\n");
        return 0;
    }

    /* Enable PWM module */
    dev(pwm)->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos); // Enable

    /* Selects operating mode of the wave counter */
    if(mode == PWM_CENTER)
        dev(pwm)->MODE = (PWM_MODE_UPDOWN_UpAndDown << PWM_MODE_UPDOWN_Pos); // Up and down counter - center aligned PWM duty cycle
    else
        dev(pwm)->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos); // Up counter - edge aligned PWM duty-cycle

    /* Configuration for PWM_CLK */
    dev(pwm)->PRESCALER = (prescaler_reg << PWM_PRESCALER_PRESCALER_Pos); // Divide by prescaler

    /* Value up to which the pulse generator counter counts */
    dev(pwm)->COUNTERTOP = (res << PWM_COUNTERTOP_COUNTERTOP_Pos); // res

    /* Value up to which the pulse generator counter counts */
    dev(pwm)->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos); // Looping disabled (stop at the end of the sequence)

    /* Configuration of the decoder */
    dev(pwm)->DECODER = (PWM_DECODER_LOAD_Individual    << PWM_DECODER_LOAD_Pos)|
                        (PWM_DECODER_MODE_RefreshCount  << PWM_DECODER_MODE_Pos);

    /* Beginning address in Data RAM of this sequence */
    dev(pwm)->SEQ[0].PTR = ((uint32_t)(pwm_dev[pwm].comp) << PWM_SEQ_PTR_PTR_Pos); 
    
    /* Amount of values (duty cycles) in this sequence */
    dev(pwm)->SEQ[0].CNT = ((sizeof(pwm_dev[pwm].comp) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    /* Amount of additional PWM periods between samples loaded into compare register */
    dev(pwm)->SEQ[0].REFRESH = 0; // Update every PWM period

    /* Time added after the sequence */
    dev(pwm)->SEQ[0].ENDDELAY = 0; // Time added after the sequence in PWM periods

    DEBUG("Timer frequency is set to %ld\n", timer_freq);

    return timer_freq; 
}

/**
 * @brief   Get the number of available channels
 *
 * @param[in] dev           PWM device
 *
 * @return                  Number of channels available for the given device
 */
uint8_t pwm_channels(pwm_t pwm)
{
    assert(pwm < PWM_NUMOF);

    uint8_t num_channel = 0;
    for(uint8_t i = 0; (i < PWM_CHAN); i++)
    {
        if(pwm_config[pwm].channel[i].pin != GPIO_UNDEF)
            num_channel++;
    }

    return num_channel;
}

/**
 * @brief   Set the duty-cycle for a given channel of the given PWM device
 *
 * The duty-cycle is set in relation to the chosen resolution of the given
 * device. If value > resolution, value is set to resolution.
 *
 * @param[in] dev           the PWM device to set
 * @param[in] channel       the channel of the given device to set
 * @param[in] value         the desired duty-cycle to set
 */
void pwm_set(pwm_t pwm, 
             uint8_t channel, 
             uint16_t value)
{
    assert( (pwm < PWM_NUMOF)    &&
            (channel < PWM_CHAN) &&
            (pwm_config[pwm].channel[channel].pin != GPIO_UNDEF));

    /* If value > resolution, value is set to resolution */
    if(value > dev(pwm)->COUNTERTOP)
        pwm_dev[pwm].comp[channel] = dev(pwm)->COUNTERTOP;
    else
        pwm_dev[pwm].comp[channel] = value;

    /* Configure GPIO */
    gpio_clear(pwm_config[pwm].channel[channel].pin);
    gpio_init(pwm_config[pwm].channel[channel].pin, GPIO_OUT);

    /* Output pin select for PWM channel 0 */
    dev(pwm)->PSEL.OUT[channel]  =  (pwm_config[pwm].channel[channel].pin   << PWM_PSEL_OUT_PIN_Pos)    |
                                    (PWM_PSEL_OUT_CONNECT_Connected         << PWM_PSEL_OUT_CONNECT_Pos);
}

/**
 * @brief   Start PWM generation on the given device
 *
 * @param[in] dev           device to start
 */
void pwm_start(pwm_t pwm, uint8_t channel)
{
    (void) channel;
    assert(pwm < PWM_NUMOF);
    dev(pwm)->TASKS_SEQSTART[0] = 1;
}

/**
 * @brief   Stop PWM generation on the given device
 *
 * @param[in] dev           device to stop
 */
void pwm_stop(pwm_t pwm, uint8_t channel)
{
    (void) channel;
    assert(pwm < PWM_NUMOF);
    dev(pwm)->TASKS_STOP = 1;
}

/**
 * @brief   Resume PWM generation on the given device
 *
 * When this function is called, the given PWM device is powered on and
 * continues its previously configured operation. The duty cycle of each channel
 * will be the value that was last set.
 *
 * This function must not be called before the PWM device was initialized.
 *
 * @param[in] dev           device to start
 */
void pwm_poweron(pwm_t pwm)
{
    assert(pwm < PWM_NUMOF);
    dev(pwm)->TASKS_SEQSTART[0] = 1;
}

/**
 * @brief   Stop PWM generation on the given device
 *
 * This function stops the PWM generation on all configured channels for the
 * given device and powers down the given PWM peripheral.
 *
 * @param[in] dev           device to stop
 */
void pwm_poweroff(pwm_t pwm)
{
    assert(pwm < PWM_NUMOF);
    dev(pwm)->TASKS_STOP = 1;
}
