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
 * @file		umdk-temp.h
 * @brief       umdk-temp temperature sensor module
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_TEMP_H
#define UMDK_TEMP_H

#include "unwds-common.h"
#include "lm75a.h"

#define UMDK_TEMP_I2C I2C_0

void umdk_temp_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_temp_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply);

#endif /* UMDK_TEMP_H */
