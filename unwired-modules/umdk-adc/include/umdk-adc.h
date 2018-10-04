/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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
    UMDK_ADC_DATA = 0,
	UMDK_ADC_CMD_COMMAND = 1,
	UMDK_ADC_CMD_POLL = 2,
    UMDK_ADC_FAIL = 0xFF,
} umdk_adc_cmd_t;


void umdk_adc_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_adc_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_ADC_H */
