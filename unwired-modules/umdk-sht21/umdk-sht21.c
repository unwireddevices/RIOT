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
 * @file		umdk-sht21.c
 * @brief       umdk-sht21 module implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "periph/gpio.h"
#include "periph/i2c.h"

#include "board.h"

#include "sht21.h"

#include "unwds-common.h"
#include "umdk-sht21.h"
#include "unwds-gpio.h"

#include "thread.h"
#include "xtimer.h"

static sht21_t dev;

static uwnds_cb_t *callback;

static kernel_pid_t timer_pid;
static char timer_stack[THREAD_STACKSIZE_MAIN];

static int publish_period_min;

static msg_t timer_msg = {};
static xtimer_t timer;

static bool init_sensor(void) {
	dev.i2c = UMDK_SHT21_I2C;

	return sht21_init(&dev) == 0;
}

static uint16_t convert_temp(float temp) {
	return (temp + 100) * 16;
}

static uint8_t convert_humid(float humid) {
	return humid / 100;
}

static void prepare_result(module_data_t *buf) {
	sht21_measure_t measure = {};
	sht21_measure(&dev, &measure);

	uint16_t temp = convert_temp(measure.temperature);
	uint8_t hum = convert_humid(measure.humidity);

	buf->length = 1 + 2 + 1; /* One byte for module ID, two bytes for temperature, one byte for humidity */

	buf->data[0] = UNWDS_SHT21_MODULE_ID;

	/* Copy measurements into response */
	memcpy(buf->data + 1, (uint8_t *) &temp, 2);
	memcpy(buf->data + 1 + 2, (uint8_t *) &hum, 1);
}

static void *timer_thread(void *arg) {
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    puts("[umdk-sht21] Periodic publisher thread started");

    while (1) {
        msg_receive(&msg);

        xtimer_remove(&timer);

        module_data_t data = {};
        prepare_result(&data);

        /* Notify the application */
        callback(&data);

        /* Restart after delay */
        xtimer_set_msg(&timer, 1e6 * 60 * publish_period_min, &timer_msg, timer_pid);
    }

    return NULL;
}

void umdk_sht21_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback) {
	(void) non_gpio_pin_map;

	callback = event_callback;
	publish_period_min = UMDK_SHT21_PUBLISH_PERIOD_MIN; /* Set to default */

	if (!init_sensor()) {
		puts("[umdk-sht21] Unable to init sensor!");
	}

	/* Create handler thread */
	timer_pid = thread_create(timer_stack, sizeof(timer_stack), THREAD_PRIORITY_MAIN - 1, 0, timer_thread, NULL, "sht21 publisher thread");

    /* Start publishing timer */
	xtimer_set_msg(&timer, 1e6 * 60 * publish_period_min, &timer_msg, timer_pid);
}

bool umdk_sht21_cmd(module_data_t *cmd, module_data_t *reply) {
	if (cmd->length < 1)
		return false;

	umdk_sht21_cmd_t c = cmd->data[0];
	switch (c) {
	case UMDK_SHT21_CMD_SET_PERIOD: {
		if (cmd->length != 2)
			return false;

		uint8_t period = cmd->data[1];
		xtimer_remove(&timer);

		publish_period_min = period;

		/* Don't restart timer if new period is zero */
		if (publish_period_min) {
			xtimer_set_msg(&timer, 1e6 * 60 * publish_period_min, &timer_msg, timer_pid);
			printf("[sht21] Period set to %d seconds\n", publish_period_min);
		} else
			puts("[sht21] Timer stopped");

		reply->length = 4;
		reply->data[0] = UNWDS_SHT21_MODULE_ID;
		reply->data[1] = 'o';
		reply->data[2] = 'k';
		reply->data[3] = '\0';

		break;
	}

	case UMDK_SHT21_CMD_POLL:
		/* Send signal to publisher thread */
		msg_send(&timer_msg, timer_pid);

		return false; /* Don't reply */

		break;

	case UMDK_SHT21_CMD_SET_I2C: {
		i2c_t i2c = (i2c_t) cmd->data[1];
		dev.i2c = i2c;

		init_sensor();

		return false; /* Don't reply */

		break;
	}

	default:
		break;
	}

	return true;
}

#ifdef __cplusplus
}
#endif
