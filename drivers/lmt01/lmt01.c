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
#include "assert.h"
#include "xtimer.h"
#include "lpm.h"

#include "lmt01.h"

static inline void lmt01_off(lmt01_t *lmt01) {
    /* disable GPIOs */
    gpio_init(lmt01->en_pin, GPIO_AIN);
	gpio_init(lmt01->sens_pin, GPIO_AIN);

	lmt01->_internal.do_count = false;
	lmt01->_internal.pulse_count = 0;
}

static inline void lmt01_on(lmt01_t *lmt01) {
	/* Enable sensor */
    gpio_init(lmt01->sens_pin, GPIO_IN_PU);
    gpio_init(lmt01->en_pin, GPIO_OUT);
	gpio_set(lmt01->en_pin);

	/* Start to count pulses */
	lmt01->_internal.do_count = true;
}

static inline int pulses_to_temp(uint32_t pc) {   
    /* Datasheet 7.3.2, equation 1, 0.1 deg C accuracy is enough*/
    return ((2560 * pc) / 4096) - 500;
}

int lmt01_init(lmt01_t *lmt01, gpio_t en_pin, gpio_t sens_pin) {
	assert(lmt01 != NULL);

	lmt01->en_pin = en_pin;
	lmt01->sens_pin = sens_pin;

	lmt01->_internal.pulse_count = 0;
	lmt01->_internal.state = LMT01_UNKNOWN;

    /*
	int res = gpio_init(en_pin, GPIO_OUT);
	if (res < 0)
		return res;
    */

	return 0;
}

static int count_pulses(lmt01_t *lmt01) {
    /* enable LMT01 power */
	lmt01_on(lmt01);

	/* Wait minimum time for sensor wake up and all transitions to be done */
    xtimer_spin(xtimer_ticks_from_usec(1e3 * LMT01_MIN_TIMEOUT_MS));
    
    uint8_t powermode = lpm_get();
    if (powermode != LPM_ON) {
        lpm_set(LPM_ON);
    }

	int pulse_count = 0;
	uint8_t gpio_prev_value = 0;
	uint8_t gpio_curr_value = 0;
    
    uint32_t timestamp = xtimer_now_usec();

	while (1) {
		gpio_curr_value = gpio_read(lmt01->sens_pin);
		if (gpio_curr_value != gpio_prev_value) {
			/* count positive pulses only */
			if (gpio_curr_value) {
				pulse_count++;
			}
			gpio_prev_value = gpio_curr_value;
			timestamp = xtimer_now_usec();
		} else {
			__asm("nop; nop;");
		}

		if (pulse_count) { /* sensor is alive, at least one pulse was detected */
            if ((xtimer_now_usec() - timestamp) > 1000) {
                break;
            }
		} else {
			if ((xtimer_now_usec() - timestamp) > 1000*LMT01_MAX_IDLE_TIME_MS) { /* no sensor detected */
				break;
			}	
		}
	}
    
    if (powermode != LPM_ON) {
        lpm_set(powermode);
    }
	
	/* Disable sensor */
	lmt01_off(lmt01);

	return pulse_count;
}

int lmt01_get_temp(lmt01_t *lmt01, int *temp) {
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
