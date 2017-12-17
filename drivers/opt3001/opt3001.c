/* Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @file        opt3001.c
 * @brief       basic driver for OPT3001 sensor
 * @authoh      Oleg Artamonov [info@unwds.com]
 */


#include "opt3001.h"
#include "periph/i2c.h"

#include "rtctimers-millis.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief OPT3001 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param OPT3001 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of an error
 */
int opt3001_init(opt3001_t *dev)
{
    uint16_t chipid;

    assert(dev != NULL);

    /* Acquire I2C bus */
    i2c_acquire(dev->i2c);

    if (i2c_init_master(dev->i2c, I2C_SPEED_NORMAL) < 0) {
        i2c_release(dev->i2c);
        puts("[opt3001 driver] Error initializing I2C bus");
        
        return -1;
    }
    
    if (i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_ID, (char *)&chipid, 2) != 2) {
        puts("[opt3001 driver] Error: sensor not found");
        i2c_release(dev->i2c);
        return -1;
    }
    
    if (chipid != OPT3001_CHIP_ID) {
        puts("[opt3001 driver] Error: ID mismatch");
        i2c_release(dev->i2c);
        return -1;
    }
    /*
    i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_CONFIG, (char *)&chipid, 2);
    
    if (chipid != OPT3001_CFG_DEFAULT) {
        puts("[opt3001 driver] Error: default config mismatch");
        i2c_release(dev->i2c);
        return -1;
    }
    */
    i2c_release(dev->i2c);
    
    return 0;
}

static uint32_t read_sensor(opt3001_t *dev) {
    uint16_t data;
    data = OPT3001_CFG;
    i2c_write_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_CONFIG, (char *)&data, 2);
    
    /* wait till measurement is finished */
    int i = 100;
    do {
        /* 10 ms delay */
        rtctimers_millis_sleep(10);
        
        i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_CONFIG, (char *)&data, 2);
        if (data & OPT3001_CFG_CRF) {
            break;
        }
        
        i--;
    } while (i);
    
    if (i == 0) {
        return 0;
    }

    /* read result */
    i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_RESULT, (char *)&data, 2);
    
    /* swap bytes, as OPT3001 sends MSB first and I2C driver expects LSB first */
    data = ((data >> 8) | (data << 8));

    /* calculate lux per LSB */
    uint8_t exp = (data >> 12) & 0x0F;
    uint32_t lsb_size_x100 = (1 << exp);
    
    /* remove 4-bit exponent and leave 12-bit mantissa only */
    data &= 0x0FFF;
    
    return (((uint32_t)data * lsb_size_x100) / 100);
}

/**
 * @brief Gets OPT3001 measure.
 *
 * @param[in] dev pointer to the initialized OPT3001 device
 * @param[in] measure pointer to the allocated memory
 * for retval
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * opt3001_measure_t measure;
 *
 * opt3001_init(opt3001);
 * opt3001_measure(opt3001, &measure)
 */
uint32_t opt3001_measure(opt3001_t *dev, opt3001_measure_t *measure)
{
    assert(dev != NULL);

    i2c_acquire(dev->i2c);
    uint32_t lum = read_sensor(dev);
    i2c_release(dev->i2c);

    measure->luminocity = lum;

    return 0;
}


#ifdef __cplusplus
}
#endif
