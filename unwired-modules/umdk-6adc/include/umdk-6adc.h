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
 * @file		umdk-6adc.h
 * @brief       6 ADCs driver module definitions
 * @author      Eugene Ponomarev
 */
#ifndef UMDK_6ADC_H
#define UMDK_6ADC_H

#include "unwds-common.h"

#define UMDK_6ADC_OUT_PIN UNWD_GPIO_17
#define UMDK_6ADC_PUBLISH_PERIOD_MIN 1

#define UNWDS_6ADC_MODULE_ID 10

#define UMDK_6ADC_ADC_RESOLUTION ADC_RES_10BIT
#define UMDK_6ADC_ADC_VREF_MV 1225.0
#define UMDK_6ADC_CONVERT_TO_MILLIVOLTS 0

typedef enum {
	UMDK_6ADC_CMD_SET_PERIOD = 0,
	UMDK_6ADC_CMD_POLL = 1,
	UMDK_6ADC_CMD_SET_GPIO = 2,
	UMDK_6ADC_SET_LINES = 3,
} umdk_6adc_cmd_t;

void umdk_6adc_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_6adc_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_6ADC_H */
