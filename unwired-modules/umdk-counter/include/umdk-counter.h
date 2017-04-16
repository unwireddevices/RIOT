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
 * @file		umdk-counter.h
 * @brief       umdk-counter driver module definitions
 * @author      Mikhail Perkov
 */
#ifndef UMDK_COUNTER_H
#define UMDK_COUNTER_H

#include "unwds-common.h"

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
    UMDK_COUNTER_CMD_SET_PERIOD = 0,
    UMDK_COUNTER_CMD_POLL = 1,
    UMDK_COUNTER_CMD_RESET = 2,
} umdk_counter_cmd_t;

typedef enum {
    UMDK_COUNTER_REPLY_OK = 0,
    UMDK_COUNTER_REPLY_UNKNOWN_COMMAND = 1,
    UMDK_COUNTER_REPLY_INV_PARAMETER = 2,
} umdk_counter_reply_t;

void umdk_counter_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_counter_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_COUNTER_H */
