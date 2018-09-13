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
 * @file		umdk-inclinometer.h
 * @brief       umdk-inclinometer driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_INCLINOMETER_H
#define UMDK_INCLINOMETER_H

#include "unwds-common.h"

#define UMDK_INCLINOMETER_STACK_SIZE            1024

#define UMDK_INCLINOMETER_I2C                   0

#define UMDK_INCLINOMETER_PUBLISH_PERIOD_SEC    60
#define UMDK_INCLINOMETER_RATE_SEC              10

typedef enum {
    UMDK_INCLINOMETER_DATA = 0,
    UMDK_INCLINOMETER_CONFIG = 1,
    UMDK_INCLINOMETER_ALARM = 2,
    UMDK_INCLINOMETER_FAIL = 255
} umdk_inclinometer_cmd_t;

void umdk_inclinometer_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_inclinometer_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_INCLINOMETER_H */
