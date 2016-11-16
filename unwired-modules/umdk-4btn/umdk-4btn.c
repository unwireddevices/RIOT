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
 * @file		umdk-4btn.c
 * @brief       umdk-4btn module implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "periph/gpio.h"

#include "board.h"

#include "unwds-common.h"
#include "umdk-4btn.h"

#include "thread.h"
#include "xtimer.h"

static kernel_pid_t handler_pid;

static uint32_t last_pressed[4] = {};

static uwnds_cb_t *callback;

void *handler(void *arg) {
    msg_t msg;
    msg_t msg_queue[2];
    msg_init_queue(msg_queue, 2);

    while (1) {
        msg_receive(&msg);
        int btn = msg.type + 1;

        //printf("[4btn] Pressed: %d\n", btn);

        module_data_t data;
        data.length = 2;
        data.data[0] = UNWDS_4BTN_MODULE_ID;
        data.data[1] = btn;

        callback(&data);
    }

	return NULL;
}

static void btn_pressed_int(void *arg) {
	int btn_num = ((int) arg) - 1;

    uint32_t now = xtimer_now();
    /* Timer overflows every ~71 minutes */
	uint32_t overflow = 0;
	if (last_pressed[btn_num] > now) {
		overflow = UINT32_MAX - last_pressed[btn_num];
	}
	/* Don't accept a press of current button if it did occur earlier than last press plus debouncing time */
    if (overflow + now - last_pressed[btn_num] <= UMDK_4BTN_DEBOUNCE_TIME_MS * 1000) {
    	puts("[4btn] Press rejected");
    	return;
	}

    printf("[4btn] Pressed: %d\n", btn_num + 1);
    last_pressed[btn_num] = now;

    msg_t msg;
    msg.type = btn_num;

	msg_send_int(&msg, handler_pid);
}

void umdk_4btn_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback) {
	(void) non_gpio_pin_map;

	callback = event_callback;

	/* Initialize interrupts */
	gpio_init_int(UMDK_4BTN_1, GPIO_IN_PU, GPIO_FALLING, btn_pressed_int, (void *) 1);
	gpio_init_int(UMDK_4BTN_2, GPIO_IN_PU, GPIO_FALLING, btn_pressed_int, (void *) 2);
	gpio_init_int(UMDK_4BTN_3, GPIO_IN_PU, GPIO_FALLING, btn_pressed_int, (void *) 3);
	gpio_init_int(UMDK_4BTN_4, GPIO_IN_PU, GPIO_FALLING, btn_pressed_int, (void *) 4);

	/* Create handler thread */
	char *stack = (char *) allocate_stack();
	if (!stack) {
		puts("umdk-4btn: unable to allocate memory. Is too many modules enabled?");
		return;
	}

	handler_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, handler, NULL, "4btn thread");
}

bool umdk_4btn_cmd(module_data_t *data, module_data_t *reply) {
	return false;
}

#ifdef __cplusplus
}
#endif
