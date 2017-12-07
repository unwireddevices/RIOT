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
 * @file		umdk-lps331.h
 * @brief       umdk-lps331 driver module definitions
 * @author      Mikhail Churikov
 */
#ifndef UMDK_LPS331_H
#define UMDK_LPS331_H

#include "unwds-common.h"

#define UMDK_LPS331_STACK_SIZE 1024

#define UMDK_LPS331_I2C 1

#define UMDK_LPS331_PUBLISH_PERIOD_MIN 1

typedef enum {
	UMDK_LPS331_CMD_SET_PERIOD = 0,
	UMDK_LPS331_CMD_POLL = 1,
	UMDK_LPS331_CMD_SET_I2C = 2,
} umdk_lps331_cmd_t;

void umdk_lps331_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_lps331_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_SHT21_H */
