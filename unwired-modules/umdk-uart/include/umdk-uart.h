/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file		umdk-uart.h
 * @brief       umdk-uart module definitions
 * @author      EP
 */
#ifndef UMDK_UART_H
#define UMDK_UART_H

#include "unwds-common.h"

#define UMDK_UART_RXBUF_SIZE (UNWDS_MAX_DATA_LEN - 1)
#define UMDK_UART_SYMBOL_TIMEOUT_MS 500

#define UMDK_UART_STACK_SIZE 2048

/**
 * @brief   DE/RE pins definitions and handlers
 * @{
 */

#define DE_PIN            UNWD_GPIO_4
#define RE_PIN            UNWD_GPIO_5 

/** @} */

typedef enum {
	UMDK_UART_SEND_ALL = 0,
	UMDK_UART_SET_BAUDRATE = 1,
    UMDK_UART_SET_PARAMETERS = 2,
} umdk_uart_prefix_t;

typedef enum {
	UMDK_UART_REPLY_SENT = 0,
	UMDK_UART_REPLY_RECEIVED = 1,
	UMDK_UART_REPLY_BAUDRATE_SET = 2,
	/* ... */
	UMDK_UART_REPLY_ERR_OVF = 253,	/* RX buffer overflowed */
	UMDK_UART_REPLY_ERR_FMT = 254,
	UMDK_UART_ERR = 255,
} umdk_uart_reply_t;

void umdk_uart_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_uart_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_UART_H */
