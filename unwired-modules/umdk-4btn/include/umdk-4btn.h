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
 * @brief       umdk-4btn driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_4BTN_H
#define UMDK_4BTN_H

#include "unwds-common.h"

#define UMDK_4BTN_1 UNWD_GPIO_4
#define UMDK_4BTN_2 UNWD_GPIO_5
#define UMDK_4BTN_3 UNWD_GPIO_6
#define UMDK_4BTN_4 UNWD_GPIO_7

#define UMDK_4BTN_STACK_SIZE 1024

#define UMDK_4BTN_DEBOUNCE_TIME_MS 100

void umdk_4btn_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_4btn_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_4BTN_H */
