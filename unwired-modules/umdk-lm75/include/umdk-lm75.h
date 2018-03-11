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
 * @file		umdk-lm75.h
 * @brief       umdk-lm75 temperature sensor module
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_LM75_H
#define UMDK_LM75_H

#include "unwds-common.h"

#define UMDK_LM75_I2C 1

typedef enum {
	UMDK_LM75_CMD_POLL = 0,
} umdk_lm75_cmd_t;

void umdk_lm75_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_lm75_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_LM75_H */
