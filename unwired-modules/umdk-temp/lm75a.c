/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file		lm75a.h
 * @brief       LM75a temperature sensor driver implementation
 * @author      Eugene Ponomarev
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "thread.h"
#include "assert.h"
#include "periph/i2c.h"

#include "lm75a.h"

#ifdef __cplusplus
extern "C" {
#endif


int lm75a_init(lm75a_t *dev, lm75a_param_t *param)
{
    assert(dev != NULL);
    assert(param != NULL);

    /* Copy parameters */
    dev->params = *param;

    /* Initialize the I2C */
    if (i2c_init_master(dev->params.i2c, I2C_SPEED_NORMAL) < 0) {
        return -1;
    }

    return 0;
}

float_t lm75a_get_ambient_temperature(lm75a_t *dev)
{
    i2c_acquire(dev->params.i2c);

    char temp[2] = {};
    i2c_read_regs(dev->params.i2c, LM75A_ADDRESS, LM75A_REG_ADDR_TEMP, temp, 2);

    uint16_t value = 0;
    uint8_t *ptr = (uint8_t *) &value;

    /* Swap bytes */
    *ptr = temp[1];
    *(ptr + 1) = temp[0];

    /* Shift data (left-aligned) */
    value >>= 5;

    /* Relocate negative bit (11th bit to 16th bit) */
    if (value & 0x0400) {
        value &= 0x03FF;
        value |= 0x8000;
    }

    /* Real value can be calculated with sensor resolution */
    float_t result = (float)value * LM75A_DEGREES_RESOLUTION;

    i2c_release(dev->params.i2c);

    return result;
}

#ifdef __cplusplus
}
#endif
