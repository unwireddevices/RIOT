/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_lmp91000
 * @{
 *
 * @file
 * @brief       LMP91000 Sensor AFE System driver implementation
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include "lmp91000.h"
#include "lmp91000_internal.h"

#define ENABLE_DEBUG                    (0)
#include "debug.h"

#if ENABLE_DEBUG
    #define PRINTBUFF _printbuff
    static void _printbuff(uint8_t *buff, unsigned len)
    {
        while (len) {
            len--;
            printf("%02X ", *buff++);
        }
        printf("\n");
    }
#else
    #define PRINTBUFF(...)
#endif


/**
 * @brief This function for writing buffer onto the device
 * 
 * @param[in] dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[in] buffer Pointer to the buffer to send to the M24SR
 * @param[in] len    Number of byte to send
 * 
 * @return Error code
 */
static int _write_i2c(const lmp91000_t *dev, uint8_t reg, uint8_t value);


/**
 * @brief This function for reading buffer from a device
 * 
 * @param dev    Pointer to M24SR NFC eeprom device descriptor
 * @param buffer Pointer to the buffer to read from the M24SR
 * @param len    Number of byte to read
 * 
 * @return Error code
 */
static int _read_i2c(const lmp91000_t *dev, uint8_t reg, uint8_t *value);





static int _write_i2c(const lmp91000_t *dev, uint8_t reg, uint8_t value) {
    int ret = -1;

    DEBUG("m24sr: -> ");
    PRINTBUFF(buffer, len);

    i2c_acquire(dev->params.i2c);
    ret = i2c_write_bytes(dev->params.i2c, dev->params.i2c_addr, buffer, len, 0);
    i2c_release(dev->params.i2c);

    return ret;
}

static int _read_i2c(const lmp91000_t *dev, uint8_t reg, uint8_t *value) {
    int ret = -1;

    i2c_acquire(dev->params.i2c);
    ret = i2c_read_bytes(dev->params.i2c, dev->params.i2c_addr, buffer, len, 0);
    i2c_release(dev->params.i2c);

    DEBUG("m24sr: <- ");
    PRINTBUFF(buffer, len);
    
    return ret;
}

static int _lmp91000_status(lmp91000_t *dev);
static int _lmp91000_lock(lmp91000_t *dev);
static int _lmp91000_unlock(lmp91000_t *dev);





static int write(const lmp91000_t *dev, uint8_t reg, uint8_t value) {

}

static int read(const lmp91000_t *dev, uint8_t reg, uint8_t *value) {

} 


static int _lmp91000_status(lmp91000_t *dev) {

}

static int _lmp91000_lock(lmp91000_t *dev) {

}

static int _lmp91000_unlock(lmp91000_t *dev) {

}




int lmp91000_init_hw(lmp91000_t *dev, const lmp91000_params_t params) {
}

int lmp91000_get_configure(lmp91000_t *dev, uint8_t _tiacn, uint8_t _refcn, uint8_t _modecn) {

}

int lmp91000_set_configure(lmp91000_t *dev, uint8_t _tiacn, uint8_t _refcn, uint8_t _modecn) {

}
