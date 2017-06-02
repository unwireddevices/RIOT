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
 * @file		umdk-rssiecho.h
 * @brief       umdk-rssiecho driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_RSSIECHO_H
#define UMDK_RSSIECHO_H

#include "unwds-common.h"

typedef enum {
	UMDK_RSSIECHO_CMD_ECHO = 0,
} umdk_rssiecho_cmd_t;

void umdk_rssiecho_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_rssiecho_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_RSSIECHO_H */
