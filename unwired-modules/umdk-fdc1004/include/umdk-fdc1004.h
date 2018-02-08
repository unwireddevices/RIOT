/*
 * Copyright (C) 2018 Unwired Devices [info@unwds.com]
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
 * @file		umdk-fdc1004.h
 * @brief       umdk-fdc1004 driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_FDC1004_H
#define UMDK_FDC1004_H

#include "unwds-common.h"

#define UMDK_FDC1004_STACK_SIZE 1024

#define UMDK_FDC1004_I2C 1

#define UMDK_FDC1004_PUBLISH_PERIOD_MIN 1

typedef enum {
	UMDK_FDC1004_CMD_SET_PERIOD = 0,
	UMDK_FDC1004_CMD_POLL = 1,
	UMDK_FDC1004_CMD_SET_I2C = 2,
} umdk_fdc1004_cmd_t;

void umdk_fdc1004_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_fdc1004_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_FDC1004_H */
