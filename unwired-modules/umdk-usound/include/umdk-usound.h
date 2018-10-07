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
 * @file        umdk-usound.h
 * @brief       umdk-usound driver module definitions
 * @author      Dmitry Golik
 */
#ifndef UMDK_USOUND_H
#define UMDK_USOUND_H

#include "unwds-common.h"

#define UMDK_USOUND_PUBLISH_PERIOD_MIN 1

#define UMDK_USOUND_STACK_SIZE 2048

#define UMDK_USOUND_PWREN       GPIO_PIN(PORT_B, 1)
#define UMDK_USOUND_ADC_PIN     GPIO_PIN(PORT_A, 5)
#define UMDK_USOUND_ADC_CH      5
#define UMDK_USOUND_SILENCE_PIN GPIO_PIN(PORT_A, 2)
#define UMDK_USOUND_BEEP_PIN    GPIO_PIN(PORT_A, 3)
#define UMDK_USOUND_DISRUPT_PIN GPIO_PIN(PORT_A, 4)


typedef enum {
    UMDK_USOUND_CMD_SET_PERIOD = 0,
    UMDK_USOUND_CMD_POLL = 1,
    UMDK_USOUND_CMD_INIT_SENSOR = 2,
} umdk_usound_cmd_t;

typedef enum {
    UMDK_SOUND_MODE_DISTANCE = 0,
    UMDK_SOUND_MODE_THRESHOLD = 1,
} umdk_sound_mode_t;

void umdk_usound_init(uwnds_cb_t *event_callback);
bool umdk_usound_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_USOUND_H */
