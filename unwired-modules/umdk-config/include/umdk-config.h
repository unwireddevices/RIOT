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
 * @file		umdk-config.h
 * @brief       common declarations for the unwired modules
 * @author      Oleg Artamonov
 */
#ifndef UMDK_CONFIG_H
#define UMDK_CONFIG_H

#include "unwds-common.h"

typedef enum {
	UMDK_CONFIG_REPLY_OK = 0,
	UMDK_CONFIG_REPLY_ERR = 253,
} umdk_config_reply_t;

typedef enum {
	UMDK_CONFIG_MODULES = 0,
} umdk_config_action_t;

void umdk_config_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_config_cmd(module_data_t *cmd, module_data_t *reply);

#endif /* UMDK_CONFIG_H */
