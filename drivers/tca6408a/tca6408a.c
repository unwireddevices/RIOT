/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_tca6408a
 * @{
 *
 * @file
 * @brief       TCA6408A I/O Expander driver implementation
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 * @}
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "assert.h"

#include "tca6408a.h"
#include "tca6408a_internal.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

#include "log.h"

#if ENABLE_DEBUG
    #define PRINTBUFF _printbuff
    static void _printbuff(uint8_t *buff, unsigned len)
    {
        while (len) {
            len--;
            printf("%02Xh ", *buff++);
        }
        printf("\n");
    }
#else
    #define PRINTBUFF(...)
#endif

/**
 * @brief Write a data to registers in the TCA6408A.
 *
 * @param[in] dev     Device descriptor
 * @param[in] reg     The source register starting address
 * @param[in] data    The values of the source registers will be written here
 * 
 * @return            Error status
 */
static int _write(const tca6408a_t *dev, const uint8_t reg, uint8_t *data);

/**
 * @brief Read sequential registers from the TCA6408A.
 *
 * @param[in]  dev     Device descriptor
 * @param[in]  reg     The source register starting address
 * @param[out] data    The values of the source registers will be written here
 *
 * @return             Error status
 */
static int _read(const tca6408a_t *dev, const uint8_t reg, uint8_t *data);


static int _read(const tca6408a_t *dev, const uint8_t reg, uint8_t *data)
{
    int status = 0x00;

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);
    /* Perform the transaction */
    status = i2c_read_regs(dev->params.i2c_dev, dev->params.i2c_addr, (uint16_t)reg, data, 1, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);

    DEBUG("TCA6408A [REG %02X]: <- ", reg);
    PRINTBUFF(data, 1);

    return status;
}

static int _write(const tca6408a_t *dev, const uint8_t reg, uint8_t *data)
{
    int status = 0x00;

    DEBUG("TCA6408A [REG %02X]: -> ", reg);
    PRINTBUFF(data, 1);

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);
    /* Perform the transaction */
    status = i2c_write_regs(dev->params.i2c_dev, dev->params.i2c_addr, (uint16_t)reg, data, 1, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);
    return status;
}


int tca6408a_init(tca6408a_t *dev, const tca6408a_params_t *params)
{
    /* Initialization of parameters dependent on the connection interface */
    dev->params = *params;

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);

    /* Initialize I2C interface */
    i2c_init(dev->params.i2c_dev);

    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);

    /* Initialize default value */

    /* Setting Configuration Register (all pins as input) */
    uint8_t regval = 0xFF;
    if (_write(dev, TCA6408A_CONFIG_REG, &regval) < 0) {
        return -TCA6408A_ERROR_I2C;
    }

    /* Clearing Output Port Register */
    regval = 0x00;
    if (_write(dev, TCA6408A_OUTPUT_REG, &regval) < 0) {
        return -TCA6408A_ERROR_I2C;
    }

    /* Clearing Polarity Inversion Register*/
    regval = 0x00;
    if (_write(dev, TCA6408A_POLARITY_REG, &regval) < 0) {
        return -TCA6408A_ERROR_I2C;
    }
	return TCA6408A_OK;
}

int tca6408a_read_input(const tca6408a_t *dev, const uint8_t pin_num, uint8_t *value)
{
    uint8_t regval = 0x00;

    if (pin_num > 8) {
        return -TCA6408A_WRONG_PARAM;
    }

    if (_read(dev,TCA6408A_INPUT_REG, &regval) < 0) {
        return -TCA6408A_ERROR_I2C;
    }

    *value = (regval & (1 << pin_num));

    return TCA6408A_OK;
}

int tca6408a_write_config(const tca6408a_t *dev, const uint8_t pin_num, const tca6408a_mode_pin_t mode)
{
    uint8_t value = 0x00;

    if (pin_num > 8) {
        return -TCA6408A_WRONG_PARAM;
    }

    if (_read(dev, TCA6408A_INPUT_REG, &value) < 0) {
        return -TCA6408A_ERROR_I2C;
    }

    value &= ~(1 << pin_num);
    if (mode == TCA6408A_PIN_INPUT) {
        value |= (1 << pin_num);
    }

    if(_write(dev, TCA6408A_CONFIG_REG, &value) < 0) {
        return -TCA6408A_ERROR_I2C;
    }

    return TCA6408A_OK;
}


int tca6408a_write_output(const tca6408a_t *dev, const uint8_t pin_num, const bool enable)
{
    uint8_t regval = 0x00;

    if (pin_num > 8) {
        return -TCA6408A_WRONG_PARAM;
    }

    if (_read(dev, TCA6408A_OUTPUT_REG, &regval) < 0) {
        return -TCA6408A_ERROR_I2C;
    }
    
    regval &= ~(1 << pin_num);
    if (enable) {
        regval |= (1 << pin_num);
    }

    if (_write(dev, TCA6408A_OUTPUT_REG, &regval) < 0) {
        return -TCA6408A_ERROR_I2C;
    }

    return TCA6408A_OK;
}

int tca6408a_write_polarity(const tca6408a_t *dev, const uint8_t pin_num, const bool invert)
{
    uint8_t regval = 0x00;

    if (pin_num > 8) {
        return -TCA6408A_WRONG_PARAM;
    }

    if (_read(dev, TCA6408A_POLARITY_REG, &regval) < 0) {
        return -TCA6408A_ERROR_I2C;
    }

    regval &= ~(1 << pin_num);
    if (invert) {
        regval |= (1 << pin_num);
    }

    if (_write(dev, TCA6408A_POLARITY_REG, &regval) < 0) {
        return -TCA6408A_ERROR_I2C;
    }

    return TCA6408A_OK;
}