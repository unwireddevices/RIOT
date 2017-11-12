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
 * @file		umdk-range.h
 * @brief       umdk-range driver module definitions
 * @author      Dmitry Golik
 */
#ifndef UMDK_RANGE_H
#define UMDK_RANGE_H

#include "unwds-common.h"

#define UMDK_RANGE_PUBLISH_PERIOD_MIN 1

#define UMDK_RANGE_STACK_SIZE 1024

typedef enum {
	UMDK_RANGE_CMD_SET_PERIOD = 0,
	UMDK_RANGE_CMD_POLL = 1,
	UMDK_RANGE_CMD_INIT_SENSOR = 2,
} umdk_range_cmd_t;

void umdk_range_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_range_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_RANGE_H */
