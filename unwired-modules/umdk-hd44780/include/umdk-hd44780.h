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
 * @file		umdk-hd44780.h
 * @brief       umdk-hd44780 driver module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_HD44780_H
#define UMDK_HD44780_H

#include "unwds-common.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "hd44780.h"

/* umdk-16x2 board uses PCF8574 I2C GPIO expander */ 
/* so below are not MCU GPIOs but PCF8574 GPIO numbers */
#define UMDK_HD44780_PARAMS {    \
    .cols   = 16,       \
    .rows   = 2,        \
    .rs     = 4,        \
    .rw     = 5,        \
    .enable = 6,        \
    .data   = {0, 1, 2, 3,     \
               HD44780_RW_OFF, HD44780_RW_OFF, HD44780_RW_OFF, HD44780_RW_OFF}, \
    .backlight = 7,     \
    .i2c_dev = 1,       \
    .i2c_address = 0x20 \
}

static const hd44780_params_t umdk_hd44780_params[] =
{
    UMDK_HD44780_PARAMS,
};

#define UMDK_HD44780_ROWS 2
#define UMDK_HD44780_COLS 16

typedef enum {
	UMDK_HD44780_CMD_PRINT = 0,
} umdk_hd44780_cmd_t;

void umdk_hd44780_init(uwnds_cb_t *event_callback);
bool umdk_hd44780_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_HD44780_H */
