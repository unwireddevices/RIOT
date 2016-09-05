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

#define UNWDS_UART_MODULE_ID 7

#define UMDK_UART_DEV UART_DEV(1)
#define UMDK_UART_BAUDRATE 115200

typedef enum {
	UMDK_UART_SEND_ALL = 0,
} umdk_uart_prefix_t;

typedef enum {
	UMDK_UART_SENT = 0,
	/* ... */
	UMDK_UART_REPLY_ERR_FMT = 254,
	UMDK_UART_ERR = 255,
} umdk_uart_reply_t;

void umdk_uart_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_uart_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_UART_H */
