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
 * @file		umdk-lmt01.h
 * @brief       umdk-lmt01 driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_LMT01_H
#define UMDK_LMT01_H

#include "unwds-common.h"

#define UMDK_LMT01_STACK_SIZE 1024

#define UMDK_LMT01_MAX_SENSOR_COUNT 4
#define UMDK_LMT01_SENSOR_EN_PINS { UNWD_GPIO_4, UNWD_GPIO_5, UNWD_GPIO_25, UNWD_GPIO_26 }
#define UMDK_LMT01_INT_PIN UNWD_GPIO_28
#define UMDK_LMT01_INT_PIN2 UNWD_GPIO_29

#define UMDK_LMT01_SWITCHING_DELAY_MS 20

#define UMDK_LMT01_PUBLISH_PERIOD_MIN 1

typedef enum {
	UMDK_LMT01_CMD_SET_PERIOD = 0,
	UMDK_LMT01_CMD_POLL = 1,
	UMDK_LMT01_CMD_SET_GPIOS = 2,
} umdk_lmt01_cmd_t;

void umdk_lmt01_init(uwnds_cb_t *event_callback);
bool umdk_lmt01_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_LMT01_H */
