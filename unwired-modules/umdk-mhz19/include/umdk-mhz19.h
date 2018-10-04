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
 * @author      Dmitry Golik
 */
#ifndef UMDK_MHZ19_H
#define UMDK_MHZ19_H

#include "unwds-common.h"

#define UMDK_MHZ19_STACK_SIZE 1024
#define UMDK_MHZ19_READER_STACK_SIZE 2048

#ifndef UMDK_MHZ19_UART
#define UMDK_MHZ19_UART UMDK_UART_DEV
#endif

/** @} */

typedef enum {
    UMDK_MHZ19_ASK = 0,
    UMDK_MHZ19_SET_PERIOD = 1,
} umdk_mhz19_prefix_t;

typedef enum {
	UMDK_MHZ19_REPLY_OK = 0,
	UMDK_MHZ19_REPLY_RECEIVED = 1,

	UMDK_MHZ19_REPLY_ERR_OVF = 253,	/* RX buffer overflowed */
	UMDK_MHZ19_REPLY_ERR_FMT = 254,
	UMDK_MHZ19_ERR = 255,
} umdk_mhz19_reply_t;

void umdk_mhz19_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_mhz19_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_MHZ19_H */
