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

#include "xtimer.h"

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
 * @brief This function for writing register onto the device
 * 
 * @param[in] dev   Pointer to LMP91000 device descriptor
 * @param[in] reg   Register address to write
 * @param[in] value Data in byte to write
 * 
 * @return Error code
 */
static int _write_i2c(const lmp91000_t *dev, uint8_t reg, uint8_t value);


/**
 * @brief This function for reading register from a device
 * 
 * @param dev   Pointer to LMP91000 device descriptor
 * @param reg   Register address to read
 * @param value Memory location to store received data
 * 
 * @return Error code
 */
static int _read_i2c(const lmp91000_t *dev, uint8_t reg, uint8_t *value);


/**
 * @brief The function of checking the readiness of the LMP91000 for data exchange over I2C
 * 
 * @param dev Pointer to LMP91000 device descriptor
 * 
 * @return Error code
 */
static int _lmp91000_is_ready(lmp91000_t *dev);

/**
 * @brief TIACN and REFCN write lock function
 * 
 * @param dev Pointer to LMP91000 device descriptor
 * 
 * @return Error code
 */
static int _lmp91000_lock(lmp91000_t *dev);

/**
 * @brief TIACN and REFCN write unlock function
 * 
 * @param dev Pointer to LMP91000 device descriptor
 * 
 * @return Error code
 */
static int _lmp91000_unlock(lmp91000_t *dev);


static int _write_i2c(const lmp91000_t *dev, uint8_t reg, uint8_t value) {
    int ret = -1;

    DEBUG("LMP91000 [REG %02X]: -> ", reg);
    PRINTBUFF(&value, 1);

    i2c_acquire(dev->params.i2c);
    ret = i2c_write_reg(dev->params.i2c, dev->params.i2c_addr, reg, value, 0);
    i2c_release(dev->params.i2c);

    return ret;
}

static int _read_i2c(const lmp91000_t *dev, uint8_t reg, uint8_t *value) {
    int ret = -1;

    i2c_acquire(dev->params.i2c);
    ret = i2c_read_reg(dev->params.i2c, dev->params.i2c_addr, reg, value, 0);
    i2c_release(dev->params.i2c);

    DEBUG("LMP91000 [REG %02X]: <- ", reg);
    PRINTBUFF(value, 1);
    
    return ret;
}

static int _lmp91000_is_ready(lmp91000_t *dev) {
    int ret = LMP91000_NODEV;
    uint8_t value = LMP91000_NOT_READY;
    uint32_t current_timestamp = 0;
    const uint32_t start_timestamp = (xtimer_now_usec() / US_PER_MS);

    /* Wait until LMP91000 is not ready or timeout occurs */
    do {
        DEBUG("Waiting...\n");
        ret = _read_i2c(dev, LMP91000_STATUS, &value);
        current_timestamp = (xtimer_now_usec() / US_PER_MS);
    } while (((current_timestamp - start_timestamp) < LMP91000_I2C_TIMEOUT) && (value != LMP91000_READY));
    
    if (((current_timestamp - start_timestamp) > LMP91000_I2C_TIMEOUT) || (value != LMP91000_READY)) {
        ret = LMP91000_NODEV;
    }
    else {
        ret = LMP91000_OK;
    }
    
    return ret;
}

static int _lmp91000_lock(lmp91000_t *dev) {
    int ret = LMP91000_OK;
    
    ret = _write_i2c(dev, LMP91000_LOCK, LMP91000_READ_ONLY_MODE);
    if (ret < 0) {
        ret = LMP91000_NOBUS;
    }
    return ret; 
}

static int _lmp91000_unlock(lmp91000_t *dev) {
    int ret = LMP91000_OK;
    
    ret = _write_i2c(dev, LMP91000_LOCK, LMP91000_WRITE_MODE);
    if (ret < 0) {
        ret = LMP91000_NOBUS;
    }
    return ret; 
}

int lmp91000_init_hw(lmp91000_t *dev, const lmp91000_params_t params) {
    int ret = LMP91000_OK;
    
    dev->params = params;

    /* Configure GPIO pins for Module Enable*/
    if (dev->params.module_en_pin != GPIO_UNDEF) {
        ret = gpio_init(dev->params.module_en_pin, GPIO_OUT);
        if (ret < 0){
            DEBUG("[lmp91000] ERROR: failed to initialize MODULE EN pin\n");
            return LMP91000_ERROR;
        }
    } else {
        ret = LMP91000_ERROR_PARAM;
    }

    return ret;
}

int lmp91000_set_configure(lmp91000_t *dev, lmp91000_config_t reg_config) {
    int ret = LMP91000_OK;
    uint8_t tmp = 0x00;

    gpio_clear(dev->params.module_en_pin);

    if (_lmp91000_is_ready(dev) == LMP91000_OK) {
        ret = _lmp91000_unlock(dev);
        if (ret != LMP91000_OK) {
            gpio_set(dev->params.module_en_pin);
            return LMP91000_NOBUS;
        }

        tmp = (reg_config.tiacn.tia_gain << 2)|(reg_config.tiacn.r_load);
        ret =  _write_i2c(dev, LMP91000_TIACN, tmp);
        if (ret != LMP91000_OK) {
            gpio_set(dev->params.module_en_pin);
            return LMP91000_NOBUS;
        }

        tmp = 0x00;
        tmp = (reg_config.refcn.ref_source << 7)|(reg_config.refcn.int_z << 5)|
              (reg_config.refcn.bias_sign << 4)|(reg_config.refcn.bias);
        ret = _write_i2c(dev, LMP91000_REFCN, tmp);
        if (ret != LMP91000_OK) {
            gpio_set(dev->params.module_en_pin);
            return LMP91000_NOBUS;
        }

        ret = _lmp91000_lock(dev);
        if (ret != LMP91000_OK) {
            gpio_set(dev->params.module_en_pin);
            return LMP91000_NOBUS;
        }

        tmp = 0x00;
        tmp = (reg_config.modecn.fet_short << 7)|(reg_config.modecn.op_mode);
        ret = _write_i2c(dev, LMP91000_MODECN, tmp);
        if (ret != LMP91000_OK) {
            gpio_set(dev->params.module_en_pin);
            return LMP91000_NOBUS;
        }       
        
    } else {
        ret = LMP91000_NODEV;
    }
    gpio_set(dev->params.module_en_pin);
    return ret;
}


int lmp91000_set_operation_mode(lmp91000_t *dev, uint8_t op_mode) {
    int ret = LMP91000_OK;
    uint8_t tmp = 0x00;

    gpio_clear(dev->params.module_en_pin);

    if (_lmp91000_is_ready(dev) == LMP91000_OK) {
        
        ret = _read_i2c(dev, LMP91000_MODECN, &tmp);
        if (ret != LMP91000_OK) {
            gpio_set(dev->params.module_en_pin);
            return LMP91000_NOBUS;
        }

        tmp &= ~LMP91000_MASK_MODECN_OP_MODE;
        tmp |= op_mode;
        ret = _write_i2c(dev, LMP91000_MODECN, tmp);
        if (ret != LMP91000_OK) {
            gpio_set(dev->params.module_en_pin);
            return LMP91000_NOBUS;
        }       
        
    } else {
        ret = LMP91000_NODEV;
    }
    gpio_set(dev->params.module_en_pin);
    return ret;
}
