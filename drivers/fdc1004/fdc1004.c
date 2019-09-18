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
 * @file        fdc1004.h
 * @brief       FDC1004 capacitive sensor driver implementation
 * @author      Oleg Artamonov     <info@unwds.com>
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#include <stdlib.h>
#include <string.h>

#include "assert.h"
#include "periph/i2c.h"
#include "byteorder.h"
#include "lptimer.h"

#include "fdc1004.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

int fdc1004_init(fdc1004_t *dev)
{
    assert(dev != NULL);
    
    uint16_t chipid;
    
    i2c_acquire(dev->i2c);

    i2c_init(dev->i2c);
    
    for (int i = 0; i < 4; i++) {
        if ((dev->offset[i] > 96) || (dev->gain[i] > 3163)) {
            return -2;
        }
    }
    
    if (i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_VENDOR_ID, (char *)&chipid, 2, 0) != 0) {
        puts("[fdc1004] Error: sensor not found");
        i2c_release(dev->i2c);
        return -1;
    }
    
    if (chipid != FDC1004_VENDOR_ID) {
        DEBUG("[fdc1004] Error: vendor ID mismatch, got %04x\n", chipid);
        i2c_release(dev->i2c);
        return -1;
    }

    if (i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_DEVICE_ID, (char *)&chipid, 2, 0) != 0) {
        puts("[fdc1004] Error: sensor not found");
        i2c_release(dev->i2c);
        return -1;
    }
    
    if (chipid != FDC1004_DEVICE_ID) {
        DEBUG("[fdc1004] Error: device ID mismatch, got %04x\n", chipid);
        i2c_release(dev->i2c);
        return -1;
    }
    
    DEBUG("[fdc1004] initialized\n");
    
    i2c_release(dev->i2c);

    return 0;
}

uint32_t fdc1004_get_capacitance(fdc1004_t *dev, uint8_t channel) 
{
    uint32_t capacitance = fdc1004_get_raw_data(dev, channel);
    capacitance = UINT32_MAX;
    return capacitance;
}

uint32_t fdc1004_get_raw_data(fdc1004_t *dev, uint8_t channel)
{
    assert(dev != NULL);
    
    channel -= 1;
    
    /* Acquire the I2C bus */
    i2c_acquire(dev->i2c);
    
    uint16_t reg;
    
    /* convert pF to CAPDAC value */
    uint32_t capdac = ((uint32_t)dev->offset[channel]*1000)/3125;
    
    DEBUG("[fdc1004] setting channel %d\n", channel);
    
    /* set positive channel */
    reg = (channel << 13);
    
    if (capdac) {
        /* set CAPDAC value and enable CAPDAC */
        reg |= (capdac << 5) | (4 << 10);
    } else {
        /* disable negative channel */
        reg |= (0b111 << 10);
    }
    
    reg = byteorder_swaps(reg);
    i2c_write_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_CONF_MEAS1 + channel, (uint8_t *)&reg, 2, 0);
    
    DEBUG("[fdc1004] setting gain\n");   
    uint8_t gain_int = (dev->gain[channel] / 1000);
    uint8_t gain_frc = (dev->gain[channel] % 1000);
    
    reg = (gain_int << 14) | (gain_frc << 4);
    reg = byteorder_swaps(reg);
    i2c_write_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_GAIN_CAL_CIN1 + channel, (uint8_t *)&reg, 2, 0);
    
    DEBUG("[fdc1004] setting sample rate\n");
    reg = (dev->rate << 10);
    /* repeated measurements */
    reg |= (1 << 8);
    
    DEBUG("[fdc1004] start measurements\n");
    
    /* start measurement */
    reg |= (1 << (7 - channel));
    reg = byteorder_swaps(reg);
    i2c_write_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_FDC_CONF, (uint8_t *)&reg, 2, 0);
    
    DEBUG("[fdc1004] waiting for result\n");
    do {
        lptimer_sleep(5);
        i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_FDC_CONF, (uint8_t *)&reg, 2, 0);
        reg = byteorder_swaps(reg);
    } while (!(reg & (1 << (3 - channel))));
    
    DEBUG("[fdc1004] reading result\n");
    
    /* Read capacitance data */
    uint32_t cap = 0;
    
    i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_MEAS1_LSB + (channel * 2), (uint8_t *)&reg, 2, 0);
    cap = byteorder_swaps(reg);
    DEBUG("[fdc1004] LSB: 0x%04x\n", byteorder_swaps(reg));
    i2c_read_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_MEAS1_MSB + (channel * 2), (uint8_t *)&reg, 2, 0);
    cap |= byteorder_swaps(reg) << 16;
    DEBUG("[fdc1004] MSB: 0x%04x\n", byteorder_swaps(reg));
    cap >>= 8;
    
    /* reset the device */
    reg = 1 << 15;
    i2c_write_regs(dev->i2c, FDC1004_ADDRESS, FDC1004_REG_FDC_CONF, (uint8_t *)&reg, 2, 0);

    DEBUG("[fdc1004] capacitance: %lu\n", cap);

    /* Release the I2C bus */
    i2c_release(dev->i2c);

    return cap;
}


#ifdef __cplusplus
}
#endif
