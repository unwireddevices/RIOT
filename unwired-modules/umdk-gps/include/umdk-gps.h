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
 * @file		umdk-gps.h
 * @brief       umdk-gps MT3333-based driver module definitions
 * @author      EP
 */
#ifndef UMDK_GPS_H
#define UMDK_GPS_H

#include "unwds-common.h"

#define UMDK_GPS_READER_STACK_SIZE 1024

#ifndef UMDK_GPS_UART
#define UMDK_GPS_UART   1
#endif

#define UMDK_GPS_PUBLISH_PERIOD_MIN   1

typedef enum {
	UMDK_GPS_DATA = 0,
    UMDK_GPS_COMMAND = 1,
	UMDK_GPS_CMD_POLL = 2,
	UMDK_GPS_REPLY_ERROR = 0xFF,
} umdk_gps_cmd_t;

void umdk_gps_init(uwnds_cb_t *event_callback);
bool umdk_gps_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_GPS_H */
