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
 * @file		umdk-4counter.h
 * @brief       umdk-4counter driver module definitions
 * @author      Mikhail Perkov
 */
#ifndef UMDK_4COUNTER_H
#define UMDK_4COUNTER_H

#include "unwds-common.h"

#define UNWDS_4COUNTER_MODULE_ID 12

#define UMDK_4COUNT_NUM_SENS  4

#define UMDK_4COUNT_1 UNWD_GPIO_4
#define UMDK_4COUNT_2 UNWD_GPIO_5
#define UMDK_4COUNT_3 UNWD_GPIO_6
#define UMDK_4COUNT_4 UNWD_GPIO_7

#define UMDK_4COUNT_DEBOUNCE_TIME_MS 100

#define UMDK_4COUNT_SLEEP_TIME_SEC 10
#define UMDK_4COUNT_DETECT_COUNT 5

#define UMDK_4COUNT_VALUE_PERIOD_PER_SEC 60
#define UMDK_4COUNT_PUBLISH_PERIOD_MIN 1
#define UMDK_4COUNT_PUBLISH_PERIOD_MAX 24

typedef enum {
  DIGITAL = 1,
  ANALOG = 0,
} umdk_4counter_signal_t;

typedef enum {
  COUNTING = 0,
  PUBLISHING = 1,
} umdk_4counter_msg_t;

typedef enum {
    UMDK_4COUNT_CMD_SET_PERIOD = 0,
    UMDK_4COUNTER_CMD_POLL = 1,
} umdk_4counter_cmd_t;

void umdk_4counter_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_4counter_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_4COUNTER_H */
