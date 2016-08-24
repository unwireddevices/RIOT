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

#include "periph/gpio.h"
#include "xtimer.h"
#include "assert.h"

#include "lmt01.h"

/**
 * @brief Pulse counter interrupt
 */
static void interrupt_cb(void *arg) {
	assert(arg != NULL);

	lmt01_t *dev = (lmt01_t *) arg;
	dev->_internal.pulse_count++;
}

static inline void reset_pulse_count(lmt01_t *lmt01) {
	lmt01->_internal.pulse_count = 0;
}

static inline void lmt01_off(lmt01_t *lmt01) {
	gpio_clear(lmt01->en_pin);
}

static inline void lmt01_on(lmt01_t *lmt01) {
	/* Init interrupt handler */
	gpio_init_int(lmt01->sens_pin, GPIO_IN, GPIO_RISING, interrupt_cb, lmt01);

	/* Enable sensor */
	gpio_set(lmt01->en_pin);
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

static void test_pulse(int n) {
	int i = 0;

	gpio_init(GPIO_PIN(PORT_A, 7), GPIO_OUT);

	for (i = 0; i < n; i++) {
		gpio_set(GPIO_PIN(PORT_A, 7));

		xtimer_usleep(100);

		gpio_clear(GPIO_PIN(PORT_A, 7));

		xtimer_usleep(100);
	}
}

bool lmt01_detect(lmt01_t *lmt01, uint32_t timeout_ms) {
	assert(lmt01 != NULL);

	/* Wait for pulses either within given timeout or within at least two minimum datasheet timeouts */
	if (timeout_ms <= LMT01_MIN_TIMEOUT_MS)
		timeout_ms = 2 * LMT01_MIN_TIMEOUT_MS;

	int time = xtimer_now();

	/* Reset sensor and enable it and interrupt handler */
	reset_pulse_count(lmt01);
	lmt01_on(lmt01);

	/* Busy-waiting for the impulses within the timeout */
	while (!lmt01->_internal.pulse_count) {
		test_pulse(1);

		/* Check timeout expiration */
		int diff = xtimer_now() - time;

		if (diff >= timeout_ms * 1e3)
			return false;

		/* Sleep one millisecond, arriving pulses in counted by interrupt handler */
		xtimer_usleep(1000);
	}

	lmt01->_internal.state = LMT01_DETECTED;

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
	reset_pulse_count(lmt01);
	lmt01_on(lmt01);

	test_pulse(808);

	/*
	 * After 50ms from this moment the sensor starts a train of pulses that will be counted
	 * by the interrupt handler, so we have to wait at least 50ms as it is maximum time interval
	 * in which sensor is performing a pulse train
	 */
	xtimer_usleep(1e3 * LMT01_MIN_TIMEOUT_MS); /* Wait for the sensor start-up */
	xtimer_usleep(1e3 * LMT01_MAX_PULSES_TRAIN_TIMEOUT_MS); /* Wait for the sensor pulse train finishes */

	/* Get counted pulses */
	uint16_t pulse_count = lmt01->_internal.pulse_count;
	if (!pulse_count) {/* Has to be at least one pulse */
		lmt01->_internal.state = LMT01_UNKNOWN;
		return 0;
	}

	lmt01->_internal.state = LMT01_VALID_DATA;

	/* Convert counted pulses into temperature */
	*temp = pulses_to_temp(pulse_count);

	return lmt01->_internal.pulse_count;
}

#ifdef __cplusplus
}
#endif
