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
 * @brief       LM75A temperature sensor driver
 * @author      EP
 */
#ifndef LM75A_H_
#define LM75A_H_

#include "thread.h"
#include "ringbuffer.h"
#include "periph/i2c.h"

#include <stdlib.h>
#include <math.h>

/**
 * @brief Initial LM72A address on I2C bus
 */
#define LM75A_ADDRESS 0x48

/**
 * @brief Resolution of the sensor (or value in degrees per LSB)
 */
#define LM75A_DEGREES_RESOLUTION (0.125f)

/**
 * @brief Address of the temperature value register
 */
#define LM75A_REG_ADDR_TEMP 0

/**
 * @brief Structure that holds the LM75A driver parameters
 */
typedef struct {
    i2c_t i2c;          /**< The I2C device descriptor on which the LM75A module is attached */
    bool a1, a2, a3;    /**< State of A1-A3 pins to modify actual I2C device address */
} lm75a_param_t;

/**
 * @brief Structure that holds the LM75A driver internal state and parameters
 */
typedef struct {
    lm75a_param_t params;   /**< Holds driver parameters */
    uint8_t address;        /**< Actual device address on I2C bus with respect to A1-A3 pin state */
} lm75a_t;

/**
 * @brief LM75A driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param LM75A driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int lm75a_init(lm75a_t *dev, lm75a_param_t *param);

/**
 * @brief Gets an ambient temperature in Celsius degrees
 *
 * @param[in] dev pointer to the initialized LM75A device
 */
int16_t lm75a_get_ambient_temperature(lm75a_t *dev);

#endif /* LM75A_H_ */
