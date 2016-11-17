/*
 * Copyright (C) 2016 cr0s
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
 * @file
 * @brief       
 * @author      cr0s
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "periph/gpio.h"

#include "board.h"
#include "unwds-common.h"

#include "unwds-gpio.h"

static const gpio_t unwds_gpio_map[] = {
		0,
#ifdef UNWD_GPIO_1
		UNWD_GPIO_1,
#else
		0,
#endif
#ifdef UNWD_GPIO_2
		UNWD_GPIO_2,
#else
		0,
#endif
#ifdef UNWD_GPIO_3
		UNWD_GPIO_3,
#else
		0,
#endif
#ifdef UNWD_GPIO_4
		UNWD_GPIO_4,
#else
		0,
#endif
#ifdef UNWD_GPIO_5
		UNWD_GPIO_5,
#else
		0,
#endif
#ifdef UNWD_GPIO_6
		UNWD_GPIO_6,
#else
		0,
#endif
#ifdef UNWD_GPIO_7
		UNWD_GPIO_7,
#else
		0,
#endif
#ifdef UNWD_GPIO_8
		UNWD_GPIO_8,
#else
		0,
#endif
#ifdef UNWD_GPIO_9
		UNWD_GPIO_9,
#else
		0,
#endif
#ifdef UNWD_GPIO_10
		UNWD_GPIO_10,
#else
		0,
#endif
#ifdef UNWD_GPIO_11
		UNWD_GPIO_11,
#else
		0,
#endif
#ifdef UNWD_GPIO_12
		UNWD_GPIO_12,
#else
		0,
#endif
#ifdef UNWD_GPIO_13
		UNWD_GPIO_13,
#else
		0,
#endif
#ifdef UNWD_GPIO_14
		UNWD_GPIO_14,
#else
		0,
#endif
#ifdef UNWD_GPIO_15
		UNWD_GPIO_15,
#else
		0,
#endif
#ifdef UNWD_GPIO_16
		UNWD_GPIO_16,
#else
		0,
#endif
#ifdef UNWD_GPIO_17
		UNWD_GPIO_17,
#else
		0,
#endif
#ifdef UNWD_GPIO_18
		UNWD_GPIO_18,
#else
		0,
#endif
#ifdef UNWD_GPIO_19
		UNWD_GPIO_19,
#else
		0,
#endif
#ifdef UNWD_GPIO_20
		UNWD_GPIO_20,
#else
		0,
#endif
#ifdef UNWD_GPIO_21
		UNWD_GPIO_21,
#else
		0,
#endif
#ifdef UNWD_GPIO_22
		UNWD_GPIO_22,
#else
		0,
#endif
#ifdef UNWD_GPIO_23
		UNWD_GPIO_23,
#else
		0,
#endif
#ifdef UNWD_GPIO_24
		UNWD_GPIO_24,
#else
		0,
#endif
#ifdef UNWD_GPIO_25
		UNWD_GPIO_25,
#else
		0,
#endif
#ifdef UNWD_GPIO_26
		UNWD_GPIO_26,
#else
		0,
#endif
#ifdef UNWD_GPIO_27
		UNWD_GPIO_27,
#else
		0,
#endif
#ifdef UNWD_GPIO_28
		UNWD_GPIO_28,
#else
		0,
#endif
#ifdef UNWD_GPIO_29
		UNWD_GPIO_29,
#else
		0,
#endif
#ifdef UNWD_GPIO_30
		UNWD_GPIO_30,
#else
		0,
#endif
};

static bool set(int num, bool one) {
	gpio_t gpio = unwds_gpio_map[num];
	if (gpio == 0)
		return false;

	gpio_init(gpio, GPIO_OUT);

	if (one)
		gpio_set(gpio);
	else
		gpio_clear(gpio);

	return true;
}

static bool get(int num) {
	gpio_t gpio = unwds_gpio_map[num];
	gpio_init(gpio, GPIO_IN);

	return gpio_read(gpio);
}

static bool toggle(int num) {
	gpio_t gpio = unwds_gpio_map[num];
	if (gpio == 0)
		return false;

	gpio_toggle(gpio);

	return true;
}


void unwds_gpio_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback) {
	(void) non_gpio_pin_map;
	(void) event_callback;
}

static inline void do_reply(module_data_t *reply, unwds_gpio_reply_t reply_code) {
	reply->length = UWNDS_GPIO_DATA_LEN + 1;
	reply->data[0] = UNWDS_GPIO_MODULE_ID;
	reply->data[1] = reply_code;
}

static bool check_pin(module_data_t *reply, int pin) {
	/* Is pin is occupied by other module */
	if (unwds_is_pin_occupied(pin)) {
		do_reply(reply, UNWD_GPIO_REPLY_ERR_PIN);
		return false;
	}

	/* Gpio pin not in range */
	if (pin < 0 || pin >= (sizeof(unwds_gpio_map) / sizeof(gpio_t)) || unwds_gpio_map[pin] == 0) {
		do_reply(reply, UNWD_GPIO_REPLY_ERR_PIN);
		return false;
	}

	return true;
}

bool unwds_gpio_cmd(module_data_t *cmd, module_data_t *reply) {
	if (cmd->length != UWNDS_GPIO_DATA_LEN) {
		do_reply(reply, UNWD_GPIO_REPLY_ERR_FORMAT);
		return false;
	}

	uint8_t value = cmd->data[0];
	uint8_t pin = value & UNWDS_GPIO_PIN_MASK;
	unwds_gpio_action_t act = (value & UNWDS_GPIO_ACT_MASK) >> UNWDS_GPIO_ACT_SHIFT;

	if (!check_pin(reply, pin))
		return false;

	switch (act) {
	case UNWDS_GPIO_GET:
		if (get(pin))
			do_reply(reply, UNWD_GPIO_REPLY_OK_1);
		else
			do_reply(reply, UNWD_GPIO_REPLY_OK_0);

		break;

	case UNWDS_GPIO_SET_0:
	case UNWDS_GPIO_SET_1:
		if (set(pin, act == UNWDS_GPIO_SET_1))
			do_reply(reply, UNWD_GPIO_REPLY_OK);
		else
			do_reply(reply, UNWD_GPIO_REPLY_ERR_PIN);

		break;

	case UNWDS_GPIO_TOGGLE:
		if (toggle(pin))
			do_reply(reply, UNWD_GPIO_REPLY_OK);
		else
			do_reply(reply, UNWD_GPIO_REPLY_ERR_PIN);

		break;
	}

	return true;
}

gpio_t unwds_gpio_pin(int pin) {
	if (pin < 0 || pin >= (sizeof(unwds_gpio_map) / sizeof(gpio_t)))
		return 0;

	return unwds_gpio_map[pin];
}

#ifdef __cplusplus
}
#endif
