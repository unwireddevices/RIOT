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
 * @author      Semjon Kerner <semjon.kerner@fu-berlin.de>
 * @}
 */

#include <stdint.h>
#include <string.h>

#include "board.h" //
#include "periph/gpio.h"
#include "periph/pwm.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

#define PWM_CH0_DUTY (0x1F40)
#define PWM_CH1_DUTY (0x1F40)
#define PWM_CH2_DUTY (0x1F40)
#define PWM_CH3_DUTY (0x1F40)

#define PWM_PIN (13)

#define PWM_MAX_RESOLUTION  (0x7FFF)
#define PWM_CLK             (16000000UL)

static inline NRF_PWM_Type *dev(pwm_t pwm)
{
    return pwm_config[pwm].dev;
}

uint32_t pwm_init(pwm_t dev, pwm_mode_t mode, uint32_t freq, uint16_t res)
{
    (void)dev;
    (void)mode;

    /* Check on max resolution */
    if(res > PWM_MAX_RESOLUTION)
        return 0;

    uint8_t prescaler = PWM_CLK / (res * freq); 
    uint8_t prescaler_reg; 

    printf("prescaler = %i\n", prescaler);

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

    timer_freq = PWM_CLK / (res * prescaler);

    printf("prescaler_new = %i\n", prescaler);
    printf("timer_freq = %lu\n", timer_freq);

    if(timer_freq > freq)
    {
        puts("Error timer_freq > freq");
        return 0;
    }

    uint16_t pwm_seq[4] = {PWM_CH0_DUTY, PWM_CH1_DUTY, PWM_CH2_DUTY, PWM_CH3_DUTY};

    gpio_clear(BTN0_PIN);
    gpio_init(BTN0_PIN, GPIO_OUT);

    /* Output pin select for PWM channel 0 */
    // NRF_PWM0->PSEL.OUT[0] = (PWM_PIN << PWM_PSEL_OUT_PIN_Pos) |
    //                         (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    // NRF_PWM0->PSEL.OUT[1] = (second_pin << PWM_PSEL_OUT_PIN_Pos) |
    //                         (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    /* Enable PWM module */
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos); // Enable

    /* Selects operating mode of the wave counter */
    NRF_PWM0->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos); // Up

    /* Configuration for PWM_CLK */
    NRF_PWM0->PRESCALER = (prescaler_reg << PWM_PRESCALER_PRESCALER_Pos); // Divide by prescaler

    /* Value up to which the pulse generator counter counts */
    NRF_PWM0->COUNTERTOP = (res << PWM_COUNTERTOP_COUNTERTOP_Pos); // res

    /* Value up to which the pulse generator counter counts */
    NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos); // Looping disabled (stop at the end of the sequence)

    /* Configuration of the decoder */
    NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);

    /* Beginning address in Data RAM of this sequence */
    NRF_PWM0->SEQ[0].PTR = ((uint32_t)(pwm_seq) << PWM_SEQ_PTR_PTR_Pos);
    
    /* Amount of values (duty cycles) in this sequence */
    NRF_PWM0->SEQ[0].CNT = ((sizeof(pwm_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);

    /* Amount of additional PWM periods between samples loaded into compare register */
    NRF_PWM0->SEQ[0].REFRESH = 0; // Update every PWM period

    /* Time added after the sequence */
    NRF_PWM0->SEQ[0].ENDDELAY = 0; // Time added after the sequence in PWM periods

    DEBUG("Timer frequency is set to %ld\n", timer_freq);
    return timer_freq; 





    // assert(dev == 0 && ((mode == PWM_LEFT) || (mode == PWM_RIGHT)));

    // /* reset and configure the timer */
    // PWM_TIMER->POWER = 1;
    // PWM_TIMER->TASKS_STOP = 1;
    // PWM_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    // PWM_TIMER->MODE = TIMER_MODE_MODE_Timer;
    // PWM_TIMER->TASKS_CLEAR = 1;

    // /* calculate and set prescaler */
    // uint32_t timer_freq = freq * res;
    // uint32_t lower = (timer_freq - (PWM_PERCENT_VAL * (timer_freq / 100)));
    // uint32_t upper = (timer_freq + (PWM_PERCENT_VAL * (timer_freq / 100)));
    // for (uint32_t ps = 0; ps <= (PWM_PS_MAX + 1); ps++) {
    //     if (ps == (PWM_PS_MAX + 1)) {
    //         DEBUG("[pwm] init error: resolution or frequency not supported\n");
    //         return 0;
    //     }
    //     if ((divtable[ps] < upper) && (divtable[ps] > lower)) {
    //         PWM_TIMER->PRESCALER = ps;
    //         timer_freq = divtable[ps];
    //         break;
    //     }
    // }

    // /* reset timer compare events */
    // PWM_TIMER->EVENTS_COMPARE[0] = 0;
    // PWM_TIMER->EVENTS_COMPARE[1] = 0;
    // /* init timer compare values */
    // PWM_TIMER->CC[0] = 1;
    // PWM_TIMER->CC[1] = res;

    // /* configure PPI Event (set compare values and pwm width) */
    // if (mode == PWM_LEFT) {
    //     NRF_GPIOTE->CONFIG[PWM_GPIOTE_CH] = (GPIOTE_CONFIG_MODE_Task     |
    //                                          (PWM_PIN << 8)              |
    //                                          GPIOTE_CONFIG_POLARITY_Msk  |
    //                                          GPIOTE_CONFIG_OUTINIT_Msk);
    // }
    // else if (mode == PWM_RIGHT) {
    //     NRF_GPIOTE->CONFIG[PWM_GPIOTE_CH] = (GPIOTE_CONFIG_MODE_Task     |
    //                                          (PWM_PIN << 8)              |
    //                                          GPIOTE_CONFIG_POLARITY_Msk);
    // }

    // /* configure PPI Channels (connect compare-event and gpiote-task) */
    // NRF_PPI->CH[PWM_PPI_A].EEP = (uint32_t)(&PWM_TIMER->EVENTS_COMPARE[0]);
    // NRF_PPI->CH[PWM_PPI_B].EEP = (uint32_t)(&PWM_TIMER->EVENTS_COMPARE[1]);

    // NRF_PPI->CH[PWM_PPI_A].TEP =
    //     (uint32_t)(&NRF_GPIOTE->TASKS_OUT[PWM_GPIOTE_CH]);
    // NRF_PPI->CH[PWM_PPI_B].TEP =
    //     (uint32_t)(&NRF_GPIOTE->TASKS_OUT[PWM_GPIOTE_CH]);

    // /* enable configured PPI Channels */
    // NRF_PPI->CHENSET = PWM_PPI_CHANNELS;

    // /* shortcut to reset Counter after CC[1] event */
    // PWM_TIMER->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Msk;

    // /* start pwm with value '0' */
    // pwm_set(dev, 0, 0);

    // DEBUG("Timer frequency is set to %ld\n", timer_freq);

    // return (uint32_t)(timer_freq / res);
}

void pwm_set(pwm_t dev, uint8_t channel, uint16_t value)
{
    (void)dev;
    (void)channel;
    (void)value;

    /* Output pin select for PWM channel 0 */
    NRF_PWM0->PSEL.OUT[0] = (PWM_PIN << PWM_PSEL_OUT_PIN_Pos) |
                            (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

    // NRF_PWM0->PSEL.OUT[1] = (second_pin << PWM_PSEL_OUT_PIN_Pos) |
    //                         (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);











    // assert((dev == 0) && (channel == 0));

    // /*
    //  * make sure duty cycle is set at the beggining of each period
    //  * ensure to stop the timer as soon as possible
    //  */
    // PWM_TIMER->TASKS_STOP = 1;
    // PWM_TIMER->EVENTS_COMPARE[1] = 0;
    // PWM_TIMER->SHORTS = TIMER_SHORTS_COMPARE1_STOP_Msk;
    // PWM_TIMER->TASKS_START = 1;

    // /*
    //  * waiting for the timer to stop
    //  * This loop generates heavy load. This is not optimal therefore a local
    //  * sleep function should be implemented.
    //  */
    // while (PWM_TIMER->EVENTS_COMPARE[1] == 0) {};

    // /*
    //  * checking pwm alignment first
    //  * and guarding if duty cycle is 0% / 100%
    //  */
    // if (NRF_GPIOTE->CONFIG[PWM_GPIOTE_CH] & GPIOTE_CONFIG_OUTINIT_Msk) {
    //     if (value == 0) {
    //         if (PWM_TIMER->CC[0] != 0) {
    //             NRF_GPIOTE->TASKS_OUT[PWM_GPIOTE_CH] = 1;
    //         }

    //         NRF_PPI->CHENCLR = PWM_PPI_CHANNELS;
    //         PWM_TIMER->CC[0] = 0;
    //     } else {
    //         if (PWM_TIMER->CC[0] == 0) {
    //             NRF_GPIOTE->TASKS_OUT[PWM_GPIOTE_CH] = 1;
    //         }
    //         if (value >= PWM_TIMER->CC[1]) {
    //             NRF_PPI->CHENCLR = PWM_PPI_CHANNELS;
    //             PWM_TIMER->CC[0] = PWM_TIMER->CC[1];
    //         } else {
    //             NRF_PPI->CHENSET = PWM_PPI_CHANNELS;
    //             PWM_TIMER->CC[0] = value;
    //         }
    //     }
    // } else {
    //     if (value >= PWM_TIMER->CC[1]) {
    //         if (PWM_TIMER->CC[0] != PWM_TIMER->CC[1]) {
    //             NRF_GPIOTE->TASKS_OUT[PWM_GPIOTE_CH] = 1;
    //         }
    //         NRF_PPI->CHENCLR = PWM_PPI_CHANNELS;
    //         PWM_TIMER->CC[0] = PWM_TIMER->CC[1];
    //     } else {
    //         if (PWM_TIMER->CC[0] == PWM_TIMER->CC[1]) {
    //             NRF_GPIOTE->TASKS_OUT[PWM_GPIOTE_CH] = 1;
    //         }
    //         if (value == 0) {
    //             NRF_PPI->CHENCLR = PWM_PPI_CHANNELS;
    //             PWM_TIMER->CC[0] = 0;
    //         } else {
    //             NRF_PPI->CHENSET = PWM_PPI_CHANNELS;
    //             PWM_TIMER->CC[0] = PWM_TIMER->CC[1] - value;
    //         }
    //     }
    // }

    // /* reconfigure pwm to standard mode */
    // PWM_TIMER->TASKS_CLEAR = 1;
    // PWM_TIMER->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Msk;
    // PWM_TIMER->TASKS_START = 1;
}

uint8_t pwm_channels(pwm_t dev)
{
    assert((dev == 0) || (dev == 1) || (dev == 2));
    return 4;
}

void pwm_start(pwm_t pwm)
{
    assert(pwm < PWM_NUMOF);
    dev(pwm)->TASKS_SEQSTART[0] = 1;
}

void pwm_stop(pwm_t pwm)
{
    assert(pwm < PWM_NUMOF);
    dev(pwm)->TASKS_STOP = 1;
}

void pwm_poweron(pwm_t dev)
{
    (void)dev;

    // assert(dev == 0);

    // /*
    //  * reinit pwm with correct alignment
    //  */
    // if (NRF_GPIOTE->CONFIG[PWM_GPIOTE_CH] & GPIOTE_CONFIG_OUTINIT_Msk) {
    //     pwm_init(dev, PWM_LEFT, init_data[1], (init_data[0] >> 16));
    // } else {
    //     pwm_init(dev, PWM_RIGHT, init_data[1], (init_data[0] >> 16));
    // }

    // /*
    //  * reset dutycycle
    //  */
    // pwm_set(dev, 0, (init_data[0] & 0xffff));

}

void pwm_poweroff(pwm_t dev)
{
    (void)dev;

    // assert(dev == 0);



    // PWM_TIMER->TASKS_STOP = 1;

    // /*
    //  * power off function ensures that the inverted CC[0] is cached correctly
    //  * when right aligned
    //  */
    // if (((NRF_GPIOTE->CONFIG[PWM_GPIOTE_CH] & GPIOTE_CONFIG_OUTINIT_Msk) == 0) &
    //     (PWM_TIMER->CC[1] != PWM_TIMER->CC[0]) &
    //     (PWM_TIMER->CC[0] != 0)) {
    //         init_data[0] = ((PWM_TIMER->CC[1] << 16) |
    //                         (PWM_TIMER->CC[1] - PWM_TIMER->CC[0]));
    // } else {
    //     init_data[0] = ((PWM_TIMER->CC[1] << 16) | PWM_TIMER->CC[0]);
    // }

    // init_data[1] = (divtable[PWM_TIMER->PRESCALER] / PWM_TIMER->CC[1]);

    // /*
    //  * make sure the gpio is set to '0' while power is off
    //  */
    // pwm_set(dev, 0, 0);

    // PWM_TIMER->POWER = 0;
}
