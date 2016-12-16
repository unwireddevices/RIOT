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
 * @author		MC
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
#include "rtc-timers.h"

static sht21_t dev;

static uwnds_cb_t *callback;

static kernel_pid_t timer_pid;

static msg_t timer_msg = {};
static rtctimer_t timer;

static struct {
	uint8_t is_valid;
	uint8_t publish_period_min;
	uint8_t i2c_dev;
} sht21_config;

static bool init_sensor(void) {
	dev.i2c = UMDK_SHT21_I2C;

	printf("[sht21] Initializing sht21 on I2C #%d\n", dev.i2c);

	return sht21_init(&dev) == 0;
}

static uint16_t convert_temp(int temp) {
	return (temp / 1000.0f + 100) * 16;
}

static uint8_t convert_humid(int humid) {
	return round(humid / 1000.0f);
}

static void prepare_result(module_data_t *buf) {
	sht21_measure_t measure = {};
	sht21_measure(&dev, &measure);

	uint16_t temp = convert_temp(measure.temperature);
	uint8_t hum = convert_humid(measure.humidity);

	printf("[sht21] Temperature %d C, humidity: %d%%\n", (temp >> 4)-100, hum);

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

        rtctimers_remove(&timer);

        module_data_t data = {};
        prepare_result(&data);

        /* Notify the application */
        callback(&data);

        /* Restart after delay */
        rtctimers_set_msg(&timer, 60 * sht21_config.publish_period_min, &timer_msg, timer_pid);
    }

    return NULL;
}

static void reset_config(void) {
	sht21_config.is_valid = 0;
	sht21_config.publish_period_min = UMDK_SHT21_PUBLISH_PERIOD_MIN;
	sht21_config.i2c_dev = UMDK_SHT21_I2C;
}

static void init_config(void) {
	reset_config();

	if (!unwds_read_nvram_config(UNWDS_SHT21_MODULE_ID, (uint8_t *) &sht21_config, sizeof(sht21_config)))
		return;

	if ((sht21_config.is_valid == 0xFF) || (sht21_config.is_valid == 0)) {
		reset_config();
		return;
	}

	if (sht21_config.i2c_dev >= I2C_NUMOF) {
		reset_config();
		return;
	}
}

static inline void save_config(void) {
	sht21_config.is_valid = 1;
	unwds_write_nvram_config(UNWDS_SHT21_MODULE_ID, (uint8_t *) &sht21_config, sizeof(sht21_config));
}

void umdk_sht21_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback) {
	(void) non_gpio_pin_map;

	callback = event_callback;

	init_config();
	printf("[sht21] Publish period: %d min\n", sht21_config.publish_period_min);

	if (!init_sensor()) {
		puts("[umdk-sht21] Unable to init sensor!");
	}

	/* Create handler thread */
	char *stack = (char *) allocate_stack();
	if (!stack) {
		puts("umdk-sht21: unable to allocate memory. Are too many modules enabled?");
		return;
	}

	timer_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, timer_thread, NULL, "sht21 thread");

    /* Start publishing timer */
	rtctimers_set_msg(&timer, 60 * sht21_config.publish_period_min, &timer_msg, timer_pid);
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
		rtctimers_remove(&timer);

		sht21_config.publish_period_min = period;
		save_config();

		/* Don't restart timer if new period is zero */
		if (sht21_config.publish_period_min) {
			rtctimers_set_msg(&timer, 60 * sht21_config.publish_period_min, &timer_msg, timer_pid);
			printf("[sht21] Period set to %d minute (s)\n", sht21_config.publish_period_min);
		} else {
			puts("[sht21] Timer stopped");
		}

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

		sht21_config.i2c_dev = i2c;

		init_sensor();

		reply->length = 4;
		reply->data[0] = UNWDS_SHT21_MODULE_ID;
		reply->data[1] = 'o';
		reply->data[2] = 'k';
		reply->data[3] = '\0';

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
