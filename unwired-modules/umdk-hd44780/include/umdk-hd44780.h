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
 * @file		umdk-hd44780.h
 * @brief       umdk-hd44780 driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_HD44780_H
#define UMDK_HD44780_H

#include "unwds-common.h"

#define UMDK_HD44780_I2C 1

typedef enum {
	UMDK_HD44780_CMD_PRINT = 0,
} umdk_hd44780_cmd_t;

void umdk_hd44780_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_hd44780_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_HD44780_H */
