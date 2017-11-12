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
 * @file		umdk-adc.h
 * @brief       ADCs driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_ADC_H
#define UMDK_ADC_H

#include "unwds-common.h"

#define UMDK_ADC_PUBLISH_PERIOD_MIN 1

#define UMDK_ADC_STACK_SIZE 1024

#define UMDK_ADC_ADC_RESOLUTION ADC_RES_12BIT
#define UMDK_ADC_CONVERT_TO_MILLIVOLTS 1

typedef enum {
	UMDK_ADC_CMD_SET_PERIOD = 0,
	UMDK_ADC_CMD_POLL = 1,
	UMDK_ADC_CMD_SET_GPIO = 2,
	UMDK_ADC_SET_LINES = 3,
} umdk_adc_cmd_t;

typedef enum {
	UMDK_ADC_REPLY_FAIL = 1,
	UMDK_ADC_REPLY_OK = 0,
} umdk_adc_reply_t;

void umdk_adc_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_adc_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_ADC_H */
