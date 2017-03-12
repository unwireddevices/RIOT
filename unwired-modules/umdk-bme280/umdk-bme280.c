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
 * @file		umdk-bme280.c
 * @brief       umdk-bme280 module implementation
 * @author      Eugene Ponomarev
 * @author		Oleg Artamonov
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

#include "bme280.h"

#include "unwds-common.h"
#include "umdk-bme280.h"
#include "unwds-gpio.h"

#include "thread.h"
#include "rtctimers.h"

static bme280_t dev;

static uwnds_cb_t *callback;

static kernel_pid_t timer_pid;

static msg_t timer_msg = {};
static rtctimer_t timer;

static bool is_polled = false;

static const bme280_params_t bme280_params[] = { BME280_PARAMS_BOARD };

static struct {
	uint8_t is_valid;
	uint8_t publish_period_min;
} bme280_config;

static bool init_sensor(void) {
	printf("[bme280] Initializing BME280 on I2C #%d\n", bme280_params[0].i2c_dev);
    
	return bme280_init(&dev, &bme280_params[0]) == BME280_OK;
}

static void prepare_result(module_data_t *data) {
	int16_t temperature = bme280_read_temperature(&dev); /* degrees C * 100 */
    uint32_t pressure = bme280_read_pressure(&dev); /* Pa */
    uint16_t humidity = bme280_read_humidity(&dev); /* percents * 100 */

    int16_t temp = (5 + temperature)/10; /* 0.1 °C */
    int16_t hum = (5 + humidity)/10; /* 0.1 % */
    uint16_t press = (pressure/100); /* mbar */
    
    char buf[2][10];
    int_to_float_str(buf[0], temp, 1);
    int_to_float_str(buf[1], hum, 1);
    
	printf("[bme280] Temperature %s C, humidity: %s%%, pressure: %d mbar\n", buf[0], buf[1], press);

    /* One byte for module ID, two bytes for temperature, two bytes for humidity, two bytes for pressure */
	data->length = 1 + sizeof(temp) + sizeof(hum) + sizeof(press);

	data->data[0] = UNWDS_BME280_MODULE_ID;

	/* Copy measurements into response */
	memcpy(data->data + 1, (uint8_t *) &temp, sizeof(temp));
	memcpy(data->data + 1 + sizeof(temp), (uint8_t *) &hum, sizeof(hum));
    memcpy(data->data + 1 + sizeof(temp) + sizeof(hum), (uint8_t *) &press, sizeof(press));
}

static void *timer_thread(void *arg) {
    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);

    puts("[umdk-bme280] Periodic publisher thread started");

    while (1) {
        msg_receive(&msg);

        module_data_t data = {};
        data.as_ack = is_polled;
        is_polled = false;

        prepare_result(&data);

        /* Notify the application */
        callback(&data);
        /* Restart after delay */
        rtctimers_set_msg(&timer, 60 * bme280_config.publish_period_min, &timer_msg, timer_pid);
    }

    return NULL;
}

static void reset_config(void) {
	bme280_config.is_valid = 0;
	bme280_config.publish_period_min = UMDK_BME280_PUBLISH_PERIOD_MIN;
}

static void init_config(void) {
	reset_config();

	if (!unwds_read_nvram_config(UNWDS_BME280_MODULE_ID, (uint8_t *) &bme280_config, sizeof(bme280_config)))
		return;

	if ((bme280_config.is_valid == 0xFF) || (bme280_config.is_valid == 0)) {
		reset_config();
		return;
	}
}

static inline void save_config(void) {
	bme280_config.is_valid = 1;
	unwds_write_nvram_config(UNWDS_BME280_MODULE_ID, (uint8_t *) &bme280_config, sizeof(bme280_config));
}

static void set_period (int period) {
    rtctimers_remove(&timer);
    bme280_config.publish_period_min = period;
	save_config();

	/* Don't restart timer if new period is zero */
	if (bme280_config.publish_period_min) {
		rtctimers_set_msg(&timer, 60 * bme280_config.publish_period_min, &timer_msg, timer_pid);
		printf("[bme280] Period set to %d minute (s)\n", bme280_config.publish_period_min);
	} else {
		puts("[bme280] Timer stopped");
	}
}

int umdk_bme280_shell_cmd(int argc, char **argv) {
    if (argc == 1) {
        puts ("bme280 get - get results now");
        puts ("bme280 send - get and send results now");
        puts ("bme280 period <N> - set period to N minutes");
        puts ("bme280 reset - reset settings to default");
        return 0;
    }
    
    char *cmd = argv[1];
	
    if (strcmp(cmd, "get") == 0) {
        module_data_t data = {};
        prepare_result(&data);
    }
    
    if (strcmp(cmd, "send") == 0) {
        is_polled = true;
		/* Send signal to publisher thread */
		msg_send(&timer_msg, timer_pid);
    }
    
    if (strcmp(cmd, "period") == 0) {
        char *val = argv[2];
        set_period(atoi(val));
    }
    
    if (strcmp(cmd, "reset") == 0) {
        reset_config();
        save_config();
    }
    
    return 1;
}

void umdk_bme280_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback) {
	(void) non_gpio_pin_map;

	callback = event_callback;

	init_config();
	printf("[bme280] Publish period: %d min\n", bme280_config.publish_period_min);

	if (!init_sensor()) {
		puts("[umdk-bme280] Unable to init sensor!");
        return;
	}

	/* Create handler thread */
	char *stack = (char *) allocate_stack();
	if (!stack) {
		puts("[umdk-bme280]: unable to allocate memory. Are too many modules enabled?");
		return;
	}
    
    unwds_add_shell_command("bme280", "type 'bme280' for commands list", umdk_bme280_shell_cmd);
    
	timer_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, timer_thread, NULL, "bme280 thread");

    /* Start publishing timer */
	rtctimers_set_msg(&timer, 60 * bme280_config.publish_period_min, &timer_msg, timer_pid);
}

static void reply_fail(module_data_t *reply) {
	reply->length = 2;
	reply->data[0] = UNWDS_BME280_MODULE_ID;
	reply->data[1] = 255;
}

static void reply_ok(module_data_t *reply) {
	reply->length = 2;
	reply->data[0] = UNWDS_BME280_MODULE_ID;
	reply->data[1] = 0;
}

bool umdk_bme280_cmd(module_data_t *cmd, module_data_t *reply) {
	if (cmd->length < 1) {
		reply_fail(reply);
		return true;
	}

	umdk_bme280_cmd_t c = cmd->data[0];
	switch (c) {
	case UMDK_BME280_CMD_SET_PERIOD: {
		if (cmd->length != 2) {
			reply_fail(reply);
			break;
		}

		uint8_t period = cmd->data[1];
		set_period(period);

		reply_ok(reply);
		break;
	}

	case UMDK_BME280_CMD_POLL:
		is_polled = true;

		/* Send signal to publisher thread */
		msg_send(&timer_msg, timer_pid);

		return false; /* Don't reply */

	default:
		reply_fail(reply);
		break;
	}

	return true;
}

#ifdef __cplusplus
}
#endif
