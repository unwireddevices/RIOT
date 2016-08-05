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
 * @file		umdk-4btn.h
 * @brief       umdk-gps SL3333-based driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_GPS_H
#define UMDK_GPS_H

#include "unwds-common.h"

#include "sl3333.h"

/**
 * @brief Comment if you don't want to publish arriving GPS data in application automatically
 */
//#define UMDK_GPS_AUTOPUBLISH

#define UMDK_GPS_UART UART_DEV(1)

void umdk_gps_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_gps_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply);

#endif /* UMDK_4BTN_H */
