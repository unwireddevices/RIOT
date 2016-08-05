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

static const gpio_t gpio_map[] = {
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
	gpio_t gpio = gpio_map[num];
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
	gpio_t gpio = gpio_map[num];
	gpio_init(gpio, GPIO_IN);

	return gpio_read(gpio);
}

static bool toggle(int num) {
	gpio_t gpio = gpio_map[num];
	if (gpio == 0)
		return false;

	gpio_toggle(gpio);

	return true;
}


void unwds_gpio_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback) {
	(void) non_gpio_pin_map;
	(void) event_callback;
}

bool unwds_gpio_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply) {
	if (argc < 2)
		return false;

	char *sub_cmd = argv[1];
	int arg = strtol(argv[2], NULL, 10);

	/* gpio set <num> <1/0> */
	if (strcmp(sub_cmd, "set") == 0 && argc == 3) {
		/* Is pin is occupied by other module */
		if (unwds_is_pin_occupied(arg)) {
			strcpy(reply, "pin occupied");
			return false;
		}

		/* Gpio port not in range */
		if (arg < 0 || arg > 30) {
			strcpy(reply, "pin not in range [1; 30]");
			return false;
		}

		bool one = strtol(argv[3], NULL, 10) == 1;

		if (set(arg, one))
			strcpy(reply, "ok");
		else
			strcpy(reply, "pin not connected");

		return true;
	} else if (strcmp(sub_cmd, "toggle") == 0 && argc == 2) {
		/* Is pin is occupied by other module */
		if (unwds_is_pin_occupied(arg)) {
			strcpy(reply, "pin occupied");
			return false;
		}

		/* Gpio port not in range */
		if (arg <= 0 || arg > 30) {
			strcpy(reply, "pin not in range [1; 30]");
			return false;
		}

		if (toggle(arg))
			strcpy(reply, "ok");
		else
			strcpy(reply, "pin isn't connected");

		return true;
	} else if (strcmp(sub_cmd, "get") == 0 && argc == 2) { /* gpio get <num> */
		/* Is pin is occupied by other module */
		if (unwds_is_pin_occupied(arg)) {
			strcpy(reply, "pin occupied");
			return false;
		}

		/* Gpio port not in range */
		if (arg < 0 || arg >= (sizeof(gpio_map) / sizeof(gpio_t))) {
			strcpy(reply, "pin not in range [1; 30]");
			return false;
		}

		if (gpio_map[arg] == 0) {
			strcpy(reply, "pin isn't connected");
			return false;
		}

		if (get(arg))
			strcpy(reply, "1");
		else
			strcpy(reply, "0");

		return true;
	}

	strcpy(reply, "invalid params");

	return false;
}

#ifdef __cplusplus
}
#endif
