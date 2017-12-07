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
 * @file		umdk-opt3001.h
 * @brief       umdk-opt3001 driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_OPT3001_H
#define UMDK_OPT3001_H

#include "unwds-common.h"

#define UMDK_OPT3001_STACK_SIZE 1024

#define UMDK_OPT3001_I2C 1

#define UMDK_OPT3001_PUBLISH_PERIOD_MIN 1

typedef enum {
	UMDK_OPT3001_CMD_SET_PERIOD = 0,
	UMDK_OPT3001_CMD_POLL = 1,
	UMDK_OPT3001_CMD_SET_I2C = 2,
} umdk_opt3001_cmd_t;

void umdk_opt3001_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_opt3001_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_OPT3001_H */
