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
 * @file		umdk-counter.h
 * @brief       umdk-counter driver module definitions
 * @author      Mikhail Perkov
 */
#ifndef UMDK_COUNTER_H
#define UMDK_COUNTER_H

#include "unwds-common.h"

#define UMDK_COUNTER_STACK_SIZE 1024

#define UMDK_COUNTER_NUM_SENS  4

#define UMDK_COUNTER_1 UNWD_GPIO_5
#define UMDK_COUNTER_2 UNWD_GPIO_4
#define UMDK_COUNTER_3 UNWD_GPIO_25
#define UMDK_COUNTER_4 UNWD_GPIO_26

#define UMDK_COUNTER_BTN UNWD_GPIO_1

#define UMDK_COUNTER_SLEEP_TIME_MS 100

#define UMDK_COUNTER_VALUE_PERIOD_PER_SEC 3600
#define UMDK_COUNTER_PUBLISH_PERIOD_MIN 1
#define UMDK_COUNTER_PUBLISH_PERIOD_MAX 24

typedef enum {
    DIGITAL = 1,
    ANALOG = 0,
} umdk_counter_signal_t;

typedef enum {
    COUNTING = 0,
    PUBLISHING = 1,
} umdk_counter_msg_t;

typedef enum {
    UMDK_COUNTER_DATA = 0,
    UMDK_COUNTER_CMD_COMMAND = 1,
    UMDK_COUNTER_CMD_POLL = 2,
    UMDK_COUNTER_CMD_RESET = 3,
} umdk_counter_cmd_t;

typedef enum {
    UMDK_COUNTER_REPLY_OK = 0,
    UMDK_COUNTER_REPLY_ERR = 0xFF,
} umdk_counter_reply_t;

void umdk_counter_init(uwnds_cb_t *event_callback);
bool umdk_counter_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_COUNTER_H */
