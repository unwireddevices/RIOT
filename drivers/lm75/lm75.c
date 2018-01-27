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
 * @file		lm75.h
 * @brief       LM75a temperature sensor driver implementation
 * @author      EP
 */

#include <stdlib.h>
#include <string.h>

#include "assert.h"
#include "periph/i2c.h"

#include "lm75.h"

#ifdef __cplusplus
extern "C" {
#endif

int lm75_init(lm75_t *dev)
{
    assert(dev != NULL);
	
	/* initialize underlying I2C bus */
    if (i2c_init_master(dev->params.i2c, I2C_SPEED_NORMAL) < 0) {
        /* Release the bus for other threads. */
        puts("[lm75] Error initializing I2C bus");
        i2c_release(dev->params.i2c);
        return -1;
    }

    /* Modify the actual I2C address with respect to A1-A3 pins state */
    dev->address = LM75_ADDRESS;
    if (dev->params.a1) {
        dev->address += 1;
    }

    if (dev->params.a2) {
        dev->address += 2;
    }

    if (dev->params.a3) {
        dev->address += 4;
    }

    return 0;
}

int lm75_get_shutdown_temp(lm75_t *dev)
{
    assert(dev != NULL);

    /* Acquire the I2C bus */
    i2c_acquire(dev->params.i2c);
    
    /* Read two bytes from the sensor: MSB & LSB of temperature value */
    int16_t temp = 0;
    i2c_read_regs(dev->params.i2c, LM75_ADDRESS, LM75_REG_ADDR_TOS, (uint8_t *)&temp, 2);

    /* Release the I2C bus */
    i2c_release(dev->params.i2c);

    /* Swap bytes */
    temp = ((temp >> 8) & 0xff) | ((temp & 0xff) << 8);

    /* Shift bits while preserving the sign */
    temp = (temp & 0x8000) | ((temp >> 7) & 0x1ff);

    /* return value in celsius */
    return (int)temp / 2;
}

void lm75_set_shutdown_temp(lm75_t *dev, int8_t temp)
{
    assert(dev != NULL);
    
    int16_t temp16 = (temp << 8) & 0xff00;
    
    i2c_acquire(dev->params.i2c);
    i2c_write_regs(dev->params.i2c, LM75_ADDRESS, LM75_REG_ADDR_TOS, (uint8_t *)&temp16, 2);
    i2c_release(dev->params.i2c);
}

int lm75_get_hysteresis_temp(lm75_t *dev)
{
    assert(dev != NULL);

    i2c_acquire(dev->params.i2c);
    int16_t temp = 0;
    i2c_read_regs(dev->params.i2c, LM75_ADDRESS, LM75_REG_ADDR_THYST, (uint8_t *)&temp, 2);
    i2c_release(dev->params.i2c);

    temp = ((temp >> 8) & 0xff) | ((temp & 0xff) << 8);
    temp = (temp & 0x8000) | ((temp >> 7) & 0x1ff);

    return (int)temp / 2;
}

void lm75_set_hysteresis_temp(lm75_t *dev, int8_t temp)
{
    assert(dev != NULL);
    
    int16_t temp16 = (temp << 8) & 0xff00;
    
    i2c_acquire(dev->params.i2c);
    i2c_write_regs(dev->params.i2c, LM75_ADDRESS, LM75_REG_ADDR_THYST, (uint8_t *)&temp16, 2);
    i2c_release(dev->params.i2c);
}

int lm75_get_ambient_temperature(lm75_t *dev)
{
    assert(dev != NULL);

    /* Acquire the I2C bus */
    i2c_acquire(dev->params.i2c);
    
    /* Read two bytes from the sensor: MSB & LSB of temperature value */
    int16_t temp = 0;
    i2c_read_regs(dev->params.i2c, LM75_ADDRESS, LM75_REG_ADDR_TEMP, (uint8_t *)&temp, 2);

    /* Release the I2C bus */
    i2c_release(dev->params.i2c);

    /* Swap bytes */
    temp = ((temp >> 8) & 0xff) | ((temp & 0xff) << 8);

    /* Shift bits while preserving the sign */
    temp = (temp & 0x8000) | ((temp >> 5) & 0x3ff);

    /* return value in millicelsius */
    return (int)temp * 125;
}

#ifdef __cplusplus
}
#endif
