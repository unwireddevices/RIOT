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
 * @file		umdk-sht21.h
 * @brief       umdk-sht21 driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_SHT21_H
#define UMDK_SHT21_H

#include "unwds-common.h"

#define UMDK_SHT21_I2C 1

#define UMDK_SHT21_PUBLISH_PERIOD_MIN 1

#define UMDK_SHT21_STACK_SIZE 1024

typedef enum {
	UMDK_SHT21_CMD_SET_PERIOD = 0,
	UMDK_SHT21_CMD_POLL = 1,
	UMDK_SHT21_CMD_SET_I2C = 2,
} umdk_sht21_cmd_t;

void umdk_sht21_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_sht21_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_SHT21_H */
