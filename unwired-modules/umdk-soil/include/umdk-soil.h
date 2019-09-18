/*
 * Copyright (C) 2018 Unwired Devices LLC <info@unwds.com>

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
 * @file		umdk-soil.h
 * @brief       umdk-soil module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_SOIL_H
#define UMDK_SOIL_H

#include "unwds-common.h"

#define UMDK_SOIL_STACK_SIZE 1024
#define UMDK_SOIL_READER_STACK_SIZE 2048

#ifndef UMDK_SOIL_UART
#define UMDK_SOIL_UART UMDK_UART_DEV
#endif

#ifndef UMDK_SOIL_POWEREN
#define UMDK_SOIL_POWEREN GPIO_PIN(PORT_B, 5)
#endif

/** @} */

typedef enum {
    UMDK_SOIL_DATA = 0,
    UMDK_SOIL_CONFIG = 1,
    UMDK_SOIL_ALARM = 2,
    UMDK_SOIL_DATA_ERR = 254,
    UMDK_SOIL_FAIL = 255
} umdk_soil_response_t;

typedef enum {
	UMDK_SOIL_CMD_SET_PERIOD = 0,
	UMDK_SOIL_CMD_POLL = 1,
} umdk_soil_cmd_t;

typedef enum {
	UMDK_SOIL_REPLY_OK = 0,
	UMDK_SOIL_ERR = 255,
} umdk_soil_reply_t;

void umdk_soil_init(uwnds_cb_t *event_callback);
bool umdk_soil_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_SOIL_H */
