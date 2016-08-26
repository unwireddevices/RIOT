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

static void start_counter_timer(lmt01_t *lmt01) {
	RCC->APB1ENR |= 0x01; /* Clock for timer 2 */

	/* Enable timer GPIO */
	gpio_init(lmt01->sens_pin, GPIO_IN);
	gpio_init_af(lmt01->sens_pin, 1);

	TIM2->CNT = 0;				/* Reset counter value */

	/* Start timer in pulse clock mode */
	TIM2->CCMR1   |= 0x0100;	/* Ch. 2 as TI */
	TIM2->SMCR    |= 0x0007;    /* External clock mode 1 */
	TIM2->SMCR    |= 0x0060;    /* TI2FP2 as ext. clock */
	TIM2->CR1     |= 0x0001;    /* Enable counting */
}

static inline void lmt01_off(lmt01_t *lmt01) {
	gpio_clear(lmt01->en_pin);

	TIM2->CR1 &= ~0x01;			/* Disable timer */
	RCC->APB1ENR &= ~0x01;		/* Disable timer clocking */

	lmt01->_internal.do_count = false;
	lmt01->_internal.pulse_count = 0;
}

static inline void lmt01_on(lmt01_t *lmt01) {
	/* Init timer in counter mode */
	start_counter_timer(lmt01);

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

bool lmt01_detect(lmt01_t *lmt01, uint32_t timeout_ms) {
	assert(lmt01 != NULL);

	/* Wait for pulses either within given timeout or within at least two minimum datasheet timeouts */
	if (timeout_ms <= LMT01_MIN_TIMEOUT_MS)
		timeout_ms = 2 * LMT01_MIN_TIMEOUT_MS;

	/* Reset sensor and enable it and interrupt handler */
	lmt01_off(lmt01);
	lmt01_on(lmt01);

	xtimer_usleep(1e3 * 2 * LMT01_MIN_TIMEOUT_MS); /* Wait for the sensor start-up */
	xtimer_usleep(1e3 * 2 * LMT01_MAX_PULSES_TRAIN_TIMEOUT_MS); /* Wait for the sensor pulse train finishes */

	lmt01->_internal.pulse_count = TIM2->CNT;

	if (!lmt01->_internal.pulse_count) {
		lmt01->_internal.state = LMT01_UNKNOWN;
		lmt01_off(lmt01);

		return false;
	}

	lmt01->_internal.state = LMT01_DETECTED;

	lmt01_off(lmt01);

	/* Got impulses, sensor is seems to be present */
	return true;
}

int lmt01_get_temp(lmt01_t *lmt01, float *temp) {
	assert(lmt01 != NULL);
	assert(temp != NULL);

	/* Check presence of a sensor */
	if (lmt01->_internal.state != LMT01_DETECTED && !lmt01_detect(lmt01, 2 * LMT01_MIN_TIMEOUT_MS)) {
		lmt01->_internal.state = LMT01_UNKNOWN;
		return 0;
	}

	lmt01->_internal.state = LMT01_MEASURING;

	/* Reset sensor and enable it and interrupt handler */
	lmt01_off(lmt01);
	lmt01_on(lmt01);

	/*
	 * After 50ms from this moment the sensor starts a train of pulses that will be counted
	 * by the interrupt handler, so we have to wait at least 50ms as it is maximum time interval
	 * in which sensor is performing a pulse train
	 */
	xtimer_usleep(1e3 * 2 * LMT01_MIN_TIMEOUT_MS); /* Wait for the sensor start-up */
	xtimer_usleep(1e3 * 2 * LMT01_MAX_PULSES_TRAIN_TIMEOUT_MS); /* Wait for the sensor pulse train finishes */

	/* Get counted pulses */
	uint16_t pulse_count = lmt01->_internal.pulse_count = TIM2->CNT;
	if (!pulse_count) {/* Has to be at least one pulse */
		puts("[lmt01] no pulses detected");

		lmt01->_internal.state = LMT01_UNKNOWN;
		lmt01_off(lmt01);
		return 0;
	}

	lmt01->_internal.state = LMT01_VALID_DATA;

	/* Convert counted pulses into temperature */
	*temp = pulses_to_temp(pulse_count);

	/* Disable sensor */
	lmt01_off(lmt01);

	return pulse_count;
}

#ifdef __cplusplus
}
#endif
