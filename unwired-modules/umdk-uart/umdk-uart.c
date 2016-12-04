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
 * @file		umdk-uart.c
 * @brief       umdk-uart module implementation
 * @author      EP
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "periph/gpio.h"
#include "periph/uart.h"

#include "board.h"

#include "unwds-common.h"
#include "include/umdk-uart.h"

#include "thread.h"
#include "xtimer.h"

static uwnds_cb_t *callback;
static uint8_t rxbuf[UMDK_UART_RXBUF_SIZE] = {};

static volatile uint8_t num_bytes_received;

static kernel_pid_t writer_pid;

static msg_t send_msg;
static msg_t send_msg_ovf;

static xtimer_t send_timer;

#define UMDK_UART_NUM_BAUDRATES 10
static int baudrates[UMDK_UART_NUM_BAUDRATES] = {
		1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
};

typedef struct {
	uint8_t is_valid;
	uint8_t uart_dev;
	uint8_t current_baudrate_idx;
} uart_config_t;

static uart_config_t uart_config = { 0, UMDK_UART_DEV, UMDK_UART_BAUDRATE_NO };

void *writer(void *arg) {
  msg_t msg;
  msg_t msg_queue[128];
  msg_init_queue(msg_queue, 128);

  while (1) {
    msg_receive(&msg);

    module_data_t data;
    data.data[0] = UNWDS_UART_MODULE_ID;
    data.length = 2;

    /* Received payload, send it */
    if (msg.content.value == send_msg.content.value) {
      data.length += num_bytes_received;
      data.data[1] = UMDK_UART_REPLY_RECEIVED;

      memcpy(data.data + 2, rxbuf, num_bytes_received);

      num_bytes_received = 0;
    } else if (msg.content.value == send_msg_ovf.content.value) { /* RX buffer overflowed, send error message */
      data.length = 2;
      data.data[1] = UMDK_UART_REPLY_ERR_OVF;

      num_bytes_received = 0;
    }

    callback(&data);
  }

  return NULL;
}

void rx_cb(void *arg, uint8_t data)
{
	/* Buffer overflow */
	if (num_bytes_received == UMDK_UART_RXBUF_SIZE) {
		num_bytes_received = 0;

		msg_send(&send_msg_ovf, writer_pid);

		return;
	}

	rxbuf[num_bytes_received++] = data;

	/* Schedule sending after timeout */
	xtimer_set_msg(&send_timer, 1e3 * UMDK_UART_SYMBOL_TIMEOUT_MS, &send_msg, writer_pid);
}

static void reset_config(void) {
	uart_config.is_valid = 0;
	uart_config.current_baudrate_idx = UMDK_UART_BAUDRATE_NO;
	uart_config.uart_dev = UMDK_UART_DEV;
}

static void init_config(void) {
	reset_config();

	if (!unwds_read_nvram_config(UNWDS_UART_MODULE_ID, (uint8_t *) &uart_config, sizeof(uart_config)))
		return;

	if ((uart_config.is_valid == 0xFF) || (uart_config.is_valid == 0))  {
		reset_config();
		return;
	}

	if (uart_config.current_baudrate_idx >= UMDK_UART_NUM_BAUDRATES) {
		reset_config();
		return;
	}

	if (uart_config.uart_dev >= UART_NUMOF) {
		reset_config();
		return;
	}
}

static inline void save_config(void) {
	uart_config.is_valid = 1;
	unwds_write_nvram_config(UNWDS_UART_MODULE_ID, (uint8_t *) &uart_config, sizeof(uart_config));
}

void umdk_uart_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;
    callback = event_callback;

    init_config();

    printf("[umdk-uart] Baudrate: %d\n", baudrates[uart_config.current_baudrate_idx]);

    /* Initialize the UART */
    if (uart_init(UART_DEV(uart_config.uart_dev), baudrates[uart_config.current_baudrate_idx], rx_cb, NULL)) {
        return;
    }

    /* Initialize DE/RE pins */
    gpio_init(DE_PIN, GPIO_OUT);
    gpio_init(RE_PIN, GPIO_OUT);

    gpio_clear(DE_PIN);
    gpio_clear(RE_PIN);

    send_msg.content.value = 0;
    send_msg_ovf.content.value = 1;

    char *stack = (char *) allocate_stack();
    if (!stack) {
    	puts("umdk-uart: unable to allocate memory. Is too many modules enabled?");
    	return;
    }

	/* Create handler thread */
	writer_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, writer, NULL, "umdk-uart thread");
}

static void do_reply(module_data_t *reply, umdk_uart_reply_t r)
{
    reply->length = 2;
    reply->data[0] = UNWDS_UART_MODULE_ID;
    reply->data[1] = r;
}

bool umdk_uart_cmd(module_data_t *data, module_data_t *reply)
{
    if (data->length < 1) {
        do_reply(reply, UMDK_UART_REPLY_ERR_FMT);
        return false;
    }

    umdk_uart_prefix_t prefix = data->data[0];
    switch (prefix) {
        case UMDK_UART_SEND_ALL:
            /* Cannot send nothing */
            if (data->length == 1) {
                do_reply(reply, UMDK_UART_REPLY_ERR_FMT);
                return false;
            }

            /* Send data */
            gpio_set(RE_PIN);
            gpio_set(DE_PIN);

            uart_write(UMDK_UART_DEV, (uint8_t *) data->data + 1, data->length - 1);

            gpio_clear(RE_PIN);
            gpio_clear(DE_PIN);

            do_reply(reply, UMDK_UART_REPLY_SENT);
            break;

        case UMDK_UART_SET_BAUDRATE:
            if (data->length != 2) { /* Must be one byte of prefix and one byte of BR index */
                do_reply(reply, UMDK_UART_REPLY_ERR_FMT);
                return false;
            }

            uint8_t br = data->data[1];
            if (br >= UMDK_UART_NUM_BAUDRATES) {	/* This BR index is not supported */
                do_reply(reply, UMDK_UART_REPLY_ERR_FMT);
                return false;
            }

            /* Set baudrate and reinitialize UART */
            gpio_clear(RE_PIN);

            uart_config.current_baudrate_idx = br;
            if (uart_init(uart_config.uart_dev, baudrates[uart_config.current_baudrate_idx], rx_cb, NULL)) {
                do_reply(reply, UMDK_UART_ERR); /* UART error, baud rate not supported? */
                return false;
            }

            save_config();

            gpio_set(RE_PIN);

            do_reply(reply, UMDK_UART_REPLY_BAUDRATE_SET);

        	break;

        default:
        	do_reply(reply, UMDK_UART_REPLY_ERR_FMT);
        	return false;
    }

    return true;
}

#ifdef __cplusplus
}
#endif
