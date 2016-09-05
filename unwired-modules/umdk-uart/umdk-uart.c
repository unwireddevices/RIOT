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

void rx_cb(void *arg, uint8_t data)
{
}

void umdk_uart_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    callback = event_callback;

    /* Initialize the UART */
    if (uart_init(UMDK_UART_DEV, UMDK_UART_BAUDRATE, rx_cb, NULL)) {
        return;
    }
}

static void do_reply(module_data_t *reply, umdk_uart_reply_t r)
{
    reply->length = 1;
    reply->data[0] = r;
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

            uart_write(UMDK_UART_DEV, (uint8_t *) data->data + 1, data->length - 1);
            do_reply(reply, UMDK_UART_SENT);

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
