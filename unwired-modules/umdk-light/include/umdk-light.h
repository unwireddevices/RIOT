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
 * @file		umdk-light.h
 * @brief       umdk-light driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_LIGHT_H
#define UMDK_LIGHT_H

#include "unwds-common.h"

#define UMDK_LIGHT_STACK_SIZE         1024

#define UMDK_LIGHT_I2C                1

#define UMDK_LIGHT_PUBLISH_PERIOD_MIN 1

typedef enum {
    UMDK_LIGHT_DATA = 0,
	UMDK_LIGHT_CMD_COMMAND = 1,
	UMDK_LIGHT_CMD_POLL = 2,
    UMDK_LIGHT_CMD_FAIL = 0xFF,
} umdk_light_cmd_t;

void umdk_light_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_light_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_LIGHT_H */
