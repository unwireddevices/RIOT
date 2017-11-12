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
 * @file		umdk-pir.h
 * @brief       umdk-pir driver module definitions
 * @author      MC
 */
#ifndef UMDK_PIR_H
#define UMDK_PIR_H

#include "unwds-common.h"

#define UMDK_PIR_STACK_SIZE 1024

#define UMDK_PIR UNWD_GPIO_24

#define UMDK_PIR_DEBOUNCE_TIME_MS 150

void umdk_pir_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_pir_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_PIR_H */
