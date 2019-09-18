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
    UMDK_GPIO_SET_AUTO = 5,
} umdk_gpio_action_t;

void umdk_gpio_init(uwnds_cb_t *event_callback);
bool umdk_gpio_broadcast(module_data_t *cmd, module_data_t *reply);
bool umdk_gpio_cmd(module_data_t *cmd, module_data_t *reply);

#endif /* UMDK_GPIO_H */
