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
