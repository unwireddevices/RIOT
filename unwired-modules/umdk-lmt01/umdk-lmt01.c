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
 * @file		umdk-lmt01.c
 * @brief       umdk-lmt01 module implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "lpm.h"
#include "periph/gpio.h"
#include "board.h"

#include "lmt01.h"

#include "unwds-common.h"
#include "umdk-lmt01.h"
#include "unwds-gpio.h"

#include "thread.h"
#include "xtimer.h"
#include "rtc-timers.h"

static gpio_t en_pins[UMDK_LMT01_MAX_SENSOR_COUNT] = UMDK_LMT01_SENSOR_EN_PINS;
static lmt01_t sensors[UMDK_LMT01_MAX_SENSOR_COUNT];

static uwnds_cb_t *callback;

static kernel_pid_t timer_pid;

static int publish_period_min;

static msg_t timer_msg = {};
static rtctimer_t timer;

static bool is_polled = false;

static void init_sensors(void) {
	int i = 0;

	for (i = 0; i < UMDK_LMT01_MAX_SENSOR_COUNT; i++) {
		lmt01_t *dev = &sensors[i];

		/* Skip disabled */
		if (!en_pins[i])
			continue;

		/* Initialize */
		if (lmt01_init(dev, en_pins[i], UMDK_LMT01_INT_PIN) < 0) {
			printf("[umdk-lmt01] Failed to initialize sensor #%d\n", i);
		}
	}
}

static uint16_t convert_temp(float temp) {
	return (temp + 100) * 16;
}

static void prepare_result(module_data_t *buf) {
	int results = 0;
	int i;

	uint16_t res[4] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };

	for (i = 0; i < UMDK_LMT01_MAX_SENSOR_COUNT; i++) {
		if (!en_pins[i]) {
			continue;
		}

		float temp;
		int pulses;
		if ((pulses = lmt01_get_temp(&sensors[i], &temp)) > 0) {
			printf("[umdk-lmt01] Measured %d pulses on #%d: %.02f\n", pulses, i, temp);
			res[i] = convert_temp(temp);
			results++;
		} else {
			continue;
		}

		/* Delay between sensor switching */
		xtimer_usleep(1e3 * 100);
	}

	buf->data[0] = UNWDS_LMT01_MODULE_ID;
	memcpy(buf->data + 1, (uint8_t *) res, sizeof(res));
	buf->length = sizeof(res) + 1;
}

void *timer_thread(void *arg) {
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    puts("[umdk-lmt01] Periodic publisher thread started");

    while (1) {
        msg_receive(&msg);

        lpm_prevent_sleep = 1;

        rtctimers_remove(&timer);

        module_data_t data = {};
        data.as_ack = is_polled;
        is_polled = false;

        prepare_result(&data);

        /* Notify the application */
        callback(&data);

        lpm_prevent_sleep = 0;

        /* Restart after delay */
        rtctimers_set_msg(&timer, 60 * publish_period_min, &timer_msg, timer_pid);
    }
}

void umdk_lmt01_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback) {
	(void) non_gpio_pin_map;

	callback = event_callback;
	publish_period_min = UMDK_LMT01_PUBLISH_PERIOD_MIN; /* Set to default */

	init_sensors();

	/* Create handler thread */
	char *stack = (char *) allocate_stack();
	if (!stack) {
		puts("umdk-lmt01: unable to allocate memory. Is too many modules enabled?");
		return;
	}

	timer_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, timer_thread, NULL, "lmt01 thread");

    /* Start publishing timer */
	rtctimers_set_msg(&timer, 60 * publish_period_min, &timer_msg, timer_pid);
}

static void reply_fail(module_data_t *reply) {
	reply->length = 6;
	reply->data[0] = UNWDS_LMT01_MODULE_ID;
	reply->data[1] = 'f';
	reply->data[2] = 'a';
	reply->data[3] = 'i';
	reply->data[4] = 'l';
	reply->data[5] = '\0';
}

static void reply_ok(module_data_t *reply) {
	reply->length = 4;
	reply->data[0] = UNWDS_LMT01_MODULE_ID;
	reply->data[1] = 'o';
	reply->data[2] = 'k';
	reply->data[3] = '\0';
}

bool umdk_lmt01_cmd(module_data_t *cmd, module_data_t *reply) {
	if (cmd->length < 1) {
		reply_fail(reply);
		return true;
	}

	umdk_lmt01_cmd_t c = cmd->data[0];
	switch (c) {
	case UMDK_LMT01_CMD_SET_PERIOD: {
		if (cmd->length != 2) {
			reply_fail(reply);
			break;
		}

		uint8_t period = cmd->data[1];
		rtctimers_remove(&timer);

		publish_period_min = period;

		/* Don't restart timer if new period is zero */
		if (publish_period_min) {
			rtctimers_set_msg(&timer, 60 * publish_period_min, &timer_msg, timer_pid);
			printf("[lmt01] Period set to %d minutes\n", publish_period_min);
		} else
			puts("[lmt01] Timer stopped");

		reply_ok(reply);
		break;
	}

	case UMDK_LMT01_CMD_POLL:
		is_polled = true;

		/* Send signal to publisher thread */
		msg_send(&timer_msg, timer_pid);

		return false; /* Don't reply now */

		break;

	case UMDK_LMT01_CMD_SET_GPIOS: {
		uint8_t *gpios = &cmd->data[1];
		int num_gpios = cmd->length - 1;

		if (!num_gpios) {
			reply_fail(reply);
			break;
		}

		int i;
		for (i = 0; i < num_gpios; i++) {
			if (gpios[i]) {
				gpio_t gpio = unwds_gpio_pin(gpios[i]);
				en_pins[i] = gpio;
			} else
				en_pins[i] = 0;	/* Disable this pin */
		}

		/* Re-initialize sensors */
		init_sensors();

		reply_ok(reply);
		break;
	}

	default:
		reply_ok(reply);
		break;
	}

	return true;
}

#ifdef __cplusplus
}
#endif
