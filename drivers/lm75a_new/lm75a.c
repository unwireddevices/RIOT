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
 * @author      EP
 */

#include <stdlib.h>
#include <string.h>

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

    /* Modify the actual I2C address with respect to A1-A3 pins state */
    dev->address = LM75A_ADDRESS;
    if (param->a1) {
        dev->address += 1;
    }

    if (param->a2) {
        dev->address += 2;
    }

    if (param->a3) {
        dev->address += 4;
    }

    return 0;
}

float_t lm75a_get_ambient_temperature(lm75a_t *dev)
{
    assert(dev != NULL);

    /* Acquire the I2C bus */
    i2c_acquire(dev->params.i2c);

    /* Read two bytes from the sensor: MSB & LSB of temperature value */
    char temp[2] = {};
    i2c_read_regs(dev->params.i2c, LM75A_ADDRESS, LM75A_REG_ADDR_TEMP, temp, 2);

    uint16_t value = 0;
    uint8_t *ptr = (uint8_t *) &value;

    /* Swap bytes */
    *ptr = temp[1];
    *(ptr + 1) = temp[0];

    /* Shift bits (left-aligned) */
    value >>= 5;

    /* Relocate negative bit (11th bit to 16th bit) */
    if (value & 0x0400) {
        value &= 0x03FF;
        value |= 0x8000;
    }

    /* Scale value to the sensor resolution */
    float_t result = (float)value * LM75A_DEGREES_RESOLUTION;

    /* Release the I2C bus */
    i2c_release(dev->params.i2c);

    return result;
}

#ifdef __cplusplus
}
#endif
