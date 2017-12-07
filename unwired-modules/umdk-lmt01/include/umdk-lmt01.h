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

void umdk_lmt01_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_lmt01_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_LMT01_H */
