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
 * @file        umdk-st95.h
 * @brief       umdk-st95 driver module definitions
 * @author      Mikhail Perkov

 */
#ifndef UMDK_ST95_H
#define UMDK_ST95_H

#include "unwds-common.h"

#define UMDK_ST95_STACK_SIZE            2048

#define UMDK_ST95_SPI_DEV               0
#define UMDK_ST95_SPI_CS                UNWD_GPIO_4
#define UMDK_ST95_UART_DEV              1
#define UMDK_ST95_IRQ_IN                UNWD_GPIO_25
#define UMDK_ST95_IRQ_OUT               UNWD_GPIO_26
#define UMDK_ST95_SSI_0                 UNWD_GPIO_24
#define UMDK_ST95_SSI_1                 GPIO_UNDEF

#define ST95_MAX_DATA_BYTES             254

#define UMDK_ST95_UID_OK 1
#define UMDK_ST95_UID_ERROR 0

#define UMDK_ST95_DELAY_DETECT_MS 500

void umdk_st95_init(uwnds_cb_t *event_callback);
bool umdk_st95_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_ST95_H */
