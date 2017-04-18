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

#define UMDK_GPIO_DATA_LEN 1

typedef enum {
	UMDK_GPIO_REPLY_OK_0 = 0,
	UMDK_GPIO_REPLY_OK_1 = 1,
	UMDK_GPIO_REPLY_OK = 2,
	UMDK_GPIO_REPLY_ERR_PIN = 3,
	UMDK_GPIO_REPLY_ERR_FORMAT = 4,
    UMDK_GPIO_REPLY_OK_AINAF = 5,
    UMDK_GPIO_REPLY_OK_ALL = 6
} umdk_gpio_reply_t;

typedef enum {
	UMDK_GPIO_GET = 0,
	UMDK_GPIO_SET_0 = 1,
	UMDK_GPIO_SET_1 = 2,
	UMDK_GPIO_TOGGLE = 3,
    UMDK_GPIO_GET_ALL = 4,
} umdk_gpio_action_t;

#define UMDK_GPIO_PIN_MASK 0x1F
#define UMDK_GPIO_ACT_MASK ~UMDK_GPIO_PIN_MASK
#define UMDK_GPIO_ACT_SHIFT 5

void umdk_gpio_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_gpio_broadcast(module_data_t *cmd, module_data_t *reply);
bool umdk_gpio_cmd(module_data_t *cmd, module_data_t *reply);

#endif /* UMDK_GPIO_H */
