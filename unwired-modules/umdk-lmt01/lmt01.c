/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file		lmt01.c
 * @brief       LMT01 temperature sensor driver module definitions
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>

#include "stm32l1xx.h"

#include "periph/gpio.h"
#include "xtimer.h"
#include "assert.h"

#include "lmt01.h"

#define DONT_USE_TIMER

static void start_counter_timer(lmt01_t *lmt01) {
	/* Enable timer GPIO */
	gpio_init(lmt01->sens_pin, GPIO_IN_PU);
	
#ifndef DONT_USE_TIMER
	gpio_init_af(lmt01->sens_pin, 2);
	
	RCC->APB1ENR |= 0x02; /* Clock for timer 3 */
	TIM3->CNT = 0;				/* Reset counter value */

	/* Start timer in pulse clock mode */
	TIM3->CCMR1   |= 0x0100;	/* Ch. 2 as TI */
	TIM3->SMCR    |= 0x0007;    /* External clock mode 1 */
	TIM3->SMCR    |= 0x0060;    /* TI2FP2 as ext. clock */
	TIM3->CR1     |= 0x0001;    /* Enable counting */
#endif
}

static inline void lmt01_off(lmt01_t *lmt01) {
	gpio_clear(lmt01->en_pin);
	gpio_init(lmt01->sens_pin, GPIO_IN); /* disable pull-up */

#ifndef DONT_USE_TIMER
	TIM3->CR1 &= ~0x01;			/* Disable timer */
	RCC->APB1ENR &= ~0x02;		/* Disable timer clocking */
#endif

	lmt01->_internal.do_count = false;
	lmt01->_internal.pulse_count = 0;
}

static inline void lmt01_on(lmt01_t *lmt01) {
	/* Enable sensor */
	gpio_set(lmt01->en_pin);

	/* Start to count pulses */
	lmt01->_internal.do_count = true;
}

static inline float pulses_to_temp(uint16_t pc) {
	return (pc / 4096.0 * 256) - 50;	/* Datasheet 7.3.2, equation 1 */
}

int lmt01_init(lmt01_t *lmt01, gpio_t en_pin, gpio_t sens_pin) {
	assert(lmt01 != NULL);

	lmt01->en_pin = en_pin;
	lmt01->sens_pin = sens_pin;

	lmt01->_internal.pulse_count = 0;
	lmt01->_internal.state = LMT01_UNKNOWN;

	int res = gpio_init(en_pin, GPIO_OUT);
	if (res < 0)
		return res;

	return 0;
}

static int count_pulses(lmt01_t *lmt01) {
	lmt01_on(lmt01);

	/* Wait minimum time for sensor wake up and all transitions are done */
	xtimer_usleep(1e3 * LMT01_MIN_TIMEOUT_MS);
	
	/* Init timer in counter mode */
	start_counter_timer(lmt01);

#ifndef DONT_USE_TIMER
	int prev = TIM3->CNT;
	int ms_idle = 0;

	while (1) {
		int curr = TIM3->CNT;

		if (curr == prev) {
			xtimer_usleep(1e3 * 5);	/* Sleep some time */

			if (prev == TIM3->CNT && (ms_idle += 5) > LMT01_MAX_IDLE_TIME_MS)
				break;
		} else {
			ms_idle = 0;
		}

		prev = curr;
	}

	/* Get counted pulses */
	int pulse_count = TIM3->CNT;
#else
	uint16_t timeout_us = 0;
	uint16_t pulse_count = 0;
	uint8_t gpio_prev_value = 0;
	uint8_t gpio_curr_value = 0;

	while (1) {
		gpio_curr_value = gpio_read(lmt01->sens_pin);
		if (gpio_curr_value != gpio_prev_value) {
			/* count positive pulses only */
			if (gpio_curr_value) {
				pulse_count++;
			}
			gpio_prev_value = gpio_curr_value;
			timeout_us = 0;
		} else {
			/* xtimer_usleep(1); */ /* xtimer is sooooo sloooooow */
			__asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
			timeout_us++;
		}

		if (pulse_count) { /* sensor is alive, at least one pulse was detected */
			if (timeout_us > 1000) {
				break;
			}
		} else {
			if (timeout_us > 1000*LMT01_MAX_IDLE_TIME_MS) { /* no sensor detected */
				break;
			}	
		}
	}
#endif
	
	/* Disable sensor */
	lmt01_off(lmt01);

	return pulse_count;
}

int lmt01_get_temp(lmt01_t *lmt01, float *temp) {
	assert(lmt01 != NULL);
	assert(temp != NULL);

	lmt01->_internal.state = LMT01_MEASURING;

	/* Reset sensor and enable it and interrupt handler */
	int pulse_count = count_pulses(lmt01);

	if (!pulse_count) {/* Has to be at least one pulse */
		puts("[lmt01] no pulses detected");

		lmt01->_internal.state = LMT01_UNKNOWN;
		return 0;
	}

	lmt01->_internal.state = LMT01_VALID_DATA;

	/* Convert counted pulses into temperature */
	*temp = pulses_to_temp(pulse_count);

	return pulse_count;
}

#ifdef __cplusplus
}
#endif
