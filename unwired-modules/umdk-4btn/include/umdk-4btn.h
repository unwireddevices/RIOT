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
 * @file		umdk-4btn.h
 * @brief       umdk-4btn driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_4BTN_H
#define UMDK_4BTN_H

#include "unwds-common.h"

#define UMDK_4BTN_1 UNWD_GPIO_4
#define UMDK_4BTN_2 UNWD_GPIO_5
#define UMDK_4BTN_3 UNWD_GPIO_6
#define UMDK_4BTN_4 UNWD_GPIO_7

#define UMDK_4BTN_STACK_SIZE 1024

#define UMDK_4BTN_DEBOUNCE_TIME_MS 100

typedef enum {
    UMDK_4BTN_DATA = 0,
    UMDK_4BTN_FAIL = 255
} umdk_4btn_cmd_t;

void umdk_4btn_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_4btn_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_4BTN_H */
