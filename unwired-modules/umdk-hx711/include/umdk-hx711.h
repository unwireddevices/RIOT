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
 * @file		umdk-hx711.h
 * @brief       umdk-hx711 driver module definitions
 */
#ifndef UMDK_HX711_H
#define UMDK_HX711_H

#include "unwds-common.h"

#define UMDK_HX711_STACK_SIZE 1024

#define HX711_DATA_PIN      UNWD_GPIO_16
#define HX711_SCK_PIN       UNWD_GPIO_17

#define UMDK_HX711_PUBLISH_PERIOD_MIN   1

typedef enum {
	UMDK_HX711_CMD_POLL = 0,
    UMDK_HX711_CMD_PERIOD,
    UMDK_HX711_CMD_ZERO,
    UMDK_HX711_CMD_SCALE,
} umdk_hx711_cmd_t;

typedef enum {
	UMDK_HX711_DATA_DATA = 0,
    UMDK_HX711_DATA_OK,
    UMDK_HX711_DATA_ERROR,
} umdk_hx711_data_t;

void umdk_hx711_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_hx711_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_HX711_H */
