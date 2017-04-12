/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @author      Dmitry Golik
 */
#ifndef UMDK_MHZ19_H
#define UMDK_MHZ19_H

#include "unwds-common.h"

#define UMDK_MHZ19_RXBUF_SIZE (UNWDS_MAX_DATA_LEN - 1)
#define UMDK_MHZ19_SYMBOL_TIMEOUT_MS 500

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
