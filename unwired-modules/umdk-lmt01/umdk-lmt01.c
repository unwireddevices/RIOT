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

#include "periph/gpio.h"

#include "board.h"

#include "lmt01.h"

#include "unwds-common.h"
#include "umdk-lmt01.h"

#include "thread.h"
#include "xtimer.h"

static gpio_t en_pins[UMDK_LMT01_MAX_SENSOR_COUNT] = UMDK_LMT01_SENSOR_EN_PINS;
static lmt01_t sensors[UMDK_LMT01_MAX_SENSOR_COUNT];

static bool detected[UMDK_LMT01_MAX_SENSOR_COUNT];
static int num_sensors = 0;

static uwnds_cb_t *callback;

static kernel_pid_t timer_pid;
static char timer_stack[THREAD_STACKSIZE_MAIN];

static int publish_period_s;

static msg_t timer_msg = {};
static xtimer_t timer;

static int init_sensors(void) {
	int i = 0;
	uint8_t detected_count = 0;

	for (i = 0; i < UMDK_LMT01_MAX_SENSOR_COUNT; i++) {
		lmt01_t *dev = &sensors[i];

		/* Initialize */
		if (lmt01_init(dev, en_pins[i], UMDK_LMT01_INT_PIN) < 0) {
			printf("[umdk-lmt01] Failed to initialize sensor #%d\n", i);
		}

		/* Detect */
		printf("[umdk-lmt01] Detecting %d...\n", i);
		if (lmt01_detect(dev, UMDK_LMT01_DETECT_TIMEOUT_MS)) {
			detected[i] = true;
			printf("[umdk-lmt01] Detected sensor #%d\n", i);

			detected_count++;
		}

		/* Delay between sensor switching */
		xtimer_usleep(1e3 * 100);
	}

	return detected_count;
}

static void prepare_result(char *buf) {
	int results = 0;
	int i;

	for (i = 0; i < UMDK_LMT01_MAX_SENSOR_COUNT; i++) {
		if (!detected[i])
			continue;

		float temp;
		int pulses;
		if ((pulses = lmt01_get_temp(&sensors[i], &temp)) > 0) {
			printf("[umdk-lmt01] Measured %d pulses on #%d: %.02f\n", pulses, i, temp);
		} else {
			continue;
		}

		/* Copy temperature value as string */
		char tempbuf[10] = { '\0' };
		snprintf(tempbuf, 10, "%.02f", temp);

		/* Append it to the result */
		strcat(buf, tempbuf);

		/* Append results delimiter */
		if (results++ != num_sensors - 1)
			strcat(buf, ",");

		/* Delay between sensor switching */
		xtimer_usleep(1e3 * 100);
	}

	if (!results) {
		strcat(buf, "!no data!");
	}

	/* Try to re-detect sensors */
	if ((num_sensors = init_sensors()) == 0) {
		puts("[umdk-lmt01] Unable to detect sensor(s)");
	}
}

void *timer_thread(void *arg) {
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    puts("[umdk-lmt01] Periodic publisher thread started");

    while (1) {
        msg_receive(&msg);

        puts("[umdk-lmt01] Publishing...");

        /* Compoese the measuring */
        char buf[30] = {};
        strcpy(buf, "lmt01|");

        prepare_result(buf);

        /* Notify the application */
        callback(buf);

        /* Restart after delay */
        xtimer_set_msg(&timer, 1e6 * publish_period_s, &timer_msg, timer_pid);
    }
}

void umdk_lmt01_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback) {
	/* Disable gpio pins for sensors */
	*non_gpio_pin_map |= 1 << 4;
	*non_gpio_pin_map |= 1 << 5;
	*non_gpio_pin_map |= 1 << 6;
	*non_gpio_pin_map |= 1 << 7;

	callback = event_callback;
	publish_period_s = UMDK_LMT01_PUBLISH_PERIOD_S; /* Set to default */

	if ((num_sensors = init_sensors()) == 0) {
		puts("[umdk-lmt01] Unable to detect sensor(s)");
	}

	/* Create handler thread */
	timer_pid = thread_create(timer_stack, sizeof(timer_stack), THREAD_PRIORITY_MAIN - 1, 0, timer_thread, NULL, "lmt01 publisher thread");

    /* Start publishing timer with default period */
    xtimer_set_msg(&timer, 1e6 * publish_period_s, &timer_msg, timer_pid);
}

bool umdk_lmt01_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply) {
	if (argc < 2)
		return false;

	char *sub_cmd = argv[1];
	int arg = strtol(argv[2], NULL, 10);

	/* lmt01 set_timer <interval_s> */
	if (strcmp(sub_cmd, "set_timer") == 0 && argc == 2) {
		if (arg > 0) {
			/* Restart timer with new period */
			xtimer_remove(&timer);
			publish_period_s = arg;

		    xtimer_set_msg(&timer, 1e6 * publish_period_s, &timer_msg, timer_pid);

			strcpy(reply, "ok");

			return true;
		}
	}

	strcpy(reply, "invalid");

	return false;
}

#ifdef __cplusplus
}
#endif
