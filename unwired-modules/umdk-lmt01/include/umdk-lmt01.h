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
 * @file		umdk-lmt01.h
 * @brief       umdk-lmt01 driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_LMT01_H
#define UMDK_LMT01_H

#include "unwds-common.h"

#define UMDK_LMT01_MAX_SENSOR_COUNT 4
#define UMDK_LMT01_SENSOR_EN_PINS { UNWD_GPIO_4, UNWD_GPIO_5, UNWD_GPIO_6, UNWD_GPIO_7 }
#define UMDK_LMT01_INT_PIN UNWD_GPIO_1

#define UMDK_LMT01_DETECT_TIMEOUT_MS 300

#define UMDK_LMT01_PUBLISH_PERIOD_S 10

void umdk_lmt01_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_lmt01_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply);

#endif /* UMDK_LMT01_H */
