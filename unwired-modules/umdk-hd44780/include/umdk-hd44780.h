/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
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

void umdk_hd44780_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_hd44780_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_HD44780_H */
