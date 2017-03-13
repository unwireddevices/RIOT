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
 * @file		umdk-gpio.h
 * @brief       common declarations for the unwired modules
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_GPIO_H
#define UMDK_GPIO_H

#include "periph/gpio.h"
#include "unwds-common.h"

#define UNWDS_GPIO_DATA_LEN 1

typedef enum {
	UNWD_GPIO_REPLY_OK_0 = 0,
	UNWD_GPIO_REPLY_OK_1 = 1,
	UNWD_GPIO_REPLY_OK = 2,
	UNWD_GPIO_REPLY_ERR_PIN = 3,
	UNWD_GPIO_REPLY_ERR_FORMAT = 4,
} unwds_gpio_reply_t;

typedef enum {
	UNWDS_GPIO_GET = 0,
	UNWDS_GPIO_SET_0 = 1,
	UNWDS_GPIO_SET_1 = 2,
	UNWDS_GPIO_TOGGLE = 3,
} unwds_gpio_action_t;

#define UNWDS_GPIO_PIN_MASK 0x3F
#define UNWDS_GPIO_ACT_MASK 0xC0
#define UNWDS_GPIO_ACT_SHIFT 6

void unwds_gpio_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool unwds_gpio_broadcast(module_data_t *cmd, module_data_t *reply);
bool unwds_gpio_cmd(module_data_t *cmd, module_data_t *reply);

#endif /* UNWDS_GPIO_H */
