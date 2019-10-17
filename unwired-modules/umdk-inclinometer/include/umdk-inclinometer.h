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
 * @file		umdk-inclinometer.h
 * @brief       umdk-inclinometer driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_INCLINOMETER_H
#define UMDK_INCLINOMETER_H

#include "unwds-common.h"

#define UMDK_INCLINOMETER_STACK_SIZE            1024

#ifndef UMDK_INCLINOMETER_I2C
#define UMDK_INCLINOMETER_I2C                   1
#endif

#define UMDK_INCLINOMETER_PUBLISH_PERIOD_SEC    60
#define UMDK_INCLINOMETER_RATE_SEC              10

typedef enum {
    UMDK_INCLINOMETER_DATA = 0,
    UMDK_INCLINOMETER_CONFIG = 1,
    UMDK_INCLINOMETER_ALARM = 2,
    UMDK_INCLINOMETER_FAIL = 255
} umdk_inclinometer_cmd_t;

void umdk_inclinometer_init(uwnds_cb_t *event_callback);
bool umdk_inclinometer_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_INCLINOMETER_H */
