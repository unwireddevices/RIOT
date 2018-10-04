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
 * @file		umdk-config.h
 * @brief       common declarations for the unwired modules
 * @author      Oleg Artamonov
 */
#ifndef UMDK_CONFIG_H
#define UMDK_CONFIG_H

#include "unwds-common.h"

typedef enum {
	UMDK_CONFIG_REPLY_OK = 0,
	UMDK_CONFIG_REPLY_ERR = 0xFF,
} umdk_config_reply_t;

typedef enum {
	UMDK_CONFIG_MODULES = 0,
    UMDK_REBOOT_DEVICE = 1,
    UMDK_SET_CLASS = 2,
} umdk_config_action_t;

void umdk_config_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_config_cmd(module_data_t *cmd, module_data_t *reply);

#endif /* UMDK_CONFIG_H */
