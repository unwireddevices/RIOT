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
 * @file		umdk-lsm6ds3.h
 * @brief       umdk-lsm6ds3 temperature sensor module
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_LSM6DS3_H
#define UMDK_LSM6DS3_H

#include "unwds-common.h"

#define UMDK_LSM6DS3_I2C I2C_1

#define UMDK_LSM6DS3_STACK_SIZE 1024

typedef enum {
	UMDK_LSM6DS3_CMD_POLL = 0,
} umdk_lsm6ds3_cmd_t;

void umdk_lsm6ds3_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_lsm6ds3_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_LSM6DS3_H */
