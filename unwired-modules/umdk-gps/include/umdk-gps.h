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
 * @file		umdk-gps.h
 * @brief       umdk-gps MT3333-based driver module definitions
 * @author      EP
 */
#ifndef UMDK_GPS_H
#define UMDK_GPS_H

#include "unwds-common.h"

#define UMDK_GPS_READER_STACK_SIZE 2048

#ifndef UMDK_GPS_UART
#define UMDK_GPS_UART   1
#endif

#define UMDK_GPS_PUBLISH_PERIOD_MIN   1

typedef enum {
	UMDK_GPS_DATA = 0,
    UMDK_GPS_COMMAND = 1,
	UMDK_GPS_CMD_POLL = 2,
	UMDK_GPS_REPLY_ERROR = 0xFF,
} umdk_gps_cmd_t;

void umdk_gps_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_gps_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_GPS_H */
