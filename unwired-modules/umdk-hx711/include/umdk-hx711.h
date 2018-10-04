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
 * @file		umdk-hx711.h
 * @brief       umdk-hx711 driver module definitions
 */
#ifndef UMDK_HX711_H
#define UMDK_HX711_H

#include "unwds-common.h"

#define UMDK_HX711_STACK_SIZE 1024

#define HX711_DATA_PIN      UNWD_GPIO_16
#define HX711_SCK_PIN       UNWD_GPIO_17

#define UMDK_HX711_PUBLISH_PERIOD_MIN   1

typedef enum {
	UMDK_HX711_CMD_POLL = 0,
    UMDK_HX711_CMD_PERIOD,
    UMDK_HX711_CMD_ZERO,
    UMDK_HX711_CMD_SCALE,
} umdk_hx711_cmd_t;

typedef enum {
	UMDK_HX711_DATA_DATA = 0,
    UMDK_HX711_DATA_OK,
    UMDK_HX711_DATA_ERROR,
} umdk_hx711_data_t;

void umdk_hx711_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_hx711_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_HX711_H */
