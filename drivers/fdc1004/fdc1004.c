/*
 * Copyright (C) 2018 Unwired Devices [info@unwds.com]
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
 * @file		fdc1004.h
 * @brief       FDC1004a temperature sensor driver implementation
 * @author      EP
 */

#include <stdlib.h>
#include <string.h>

#include "assert.h"
#include "periph/i2c.h"

#include "fdc1004.h"

#ifdef __cplusplus
extern "C" {
#endif

int fdc1004_init(fdc1004_t *dev)
{
    assert(dev != NULL);
    
    uint16_t chipid;
	
    i2c_acquire(dev->i2c);

    if (i2c_init_master(dev->i2c, I2C_SPEED_NORMAL) < 0) {
        i2c_release(dev->i2c);
        puts("[fdc1004] Error initializing I2C bus");
        
        return -1;
    }
    
    if (i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_VENDOR_ID, (char *)&chipid, 2) != 2) {
        puts("[fdc1004] Error: sensor not found");
        i2c_release(dev->i2c);
        return -1;
    }
    
    if (chipid != FDC1004_VENDOR_ID) {
        puts("[fdc1004] Error: vendor ID mismatch");
        i2c_release(dev->i2c);
        return -1;
    }
    
    if (i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_DEVICE_ID, (char *)&chipid, 2) != 2) {
        puts("[fdc1004] Error: sensor not found");
        i2c_release(dev->i2c);
        return -1;
    }
    
    if (chipid != FDC1004_DEVICE_ID) {
        puts("[fdc1004] Error: device ID mismatch");
        i2c_release(dev->i2c);
        return -1;
    }
    
    i2c_release(dev->i2c);

    return 0;
}

uint32_t fdc1004_get_capacitance(fdc1004_t *dev, uint8_t channel)
{
    assert(dev != NULL);
    
    channel -= 1;

    /* Acquire the I2C bus */
    i2c_acquire(dev->i2c);
    
    /* Read two bytes from the sensor: MSB & LSB of temperature value */
    uint32_t capacitance = 0;
    i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_MEAS1_MSB + channel, (uint8_t *)&capacitance, 2);
    i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_MEAS1_LSB + channel, ((uint8_t *)&capacitance) + 2, 2);

    /* Release the I2C bus */
    i2c_release(dev->i2c);

    return capacitance;
}

#ifdef __cplusplus
}
#endif
