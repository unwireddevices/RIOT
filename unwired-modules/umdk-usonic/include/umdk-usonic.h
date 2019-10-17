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
 * @file        umdk-usonic.h
 * @brief       umdk-usonic driver module definitions
 * @author      Dmitry Golik
 */
#ifndef UMDK_USONIC_H
#define UMDK_USONIC_H

#include "unwds-common.h"
#include "board.h"

#define UMDK_USONIC_PUBLISH_PERIOD_MIN 1

#define UMDK_USONIC_STACK_SIZE 1024

#ifndef UMDK_USONIC_PWREN
#define UMDK_USONIC_PWREN       GPIO_PIN(PORT_B, 1)
#endif

#ifndef UMDK_USONIC_ADC_PIN
#define UMDK_USONIC_ADC_PIN     GPIO_PIN(PORT_A, 5)
#endif

#ifndef UMDK_USONIC_ADC_CH
#define UMDK_USONIC_ADC_CH      5
#endif

#ifndef UMDK_USONIC_SILENCE_PIN
#define UMDK_USONIC_SILENCE_PIN GPIO_PIN(PORT_A, 2)
#endif

#ifndef UMDK_USONIC_BEEP_PIN
#define UMDK_USONIC_BEEP_PIN    GPIO_PIN(PORT_A, 3)
#endif

#ifndef UMDK_USONIC_DISRUPT_PIN
#define UMDK_USONIC_DISRUPT_PIN GPIO_PIN(PORT_A, 4)
#endif


typedef enum {
    UMDK_USONIC_CMD_SET_PERIOD = 0,
    UMDK_USONIC_CMD_POLL = 1,
    UMDK_USONIC_CMD_INIT_SENSOR = 2,
} umdk_usonic_cmd_t;

typedef enum {
    UMDK_USONIC_MODE_DISTANCE = 0,
    UMDK_USONIC_MODE_THRESHOLD = 1,
} umdk_usonic_mode_t;

void umdk_usonic_init(uwnds_cb_t *event_callback);
bool umdk_usonic_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_USONIC_H */
