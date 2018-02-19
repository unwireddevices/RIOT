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
 * @file		umdk-adxl345.h
 * @brief       umdk-adxl345 driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_ADXL345_H
#define UMDK_ADXL345_H

#include "unwds-common.h"

#define UMDK_ADXL345_MEASURE_PERIOD_SEC 1

#define UMDK_ADXL345_STACK_SIZE 2048

typedef enum {
	UMDK_ADXL345_CMD_SET_PERIOD = 0,
	UMDK_ADXL345_CMD_POLL = 1,
	UMDK_ADXL345_CMD_SET_I2C = 2,
} umdk_adxl345_cmd_t;

void umdk_adxl345_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_adxl345_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_ADXL345_H */
