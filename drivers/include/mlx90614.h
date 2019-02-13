/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
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
 * @file		mlx90614.h
 * @brief       MLX90614 temperature sensor driver
 * @author      Oleg Artamonov
 */
#ifndef MLX90614_H_
#define MLX90614_H_

#include "periph/i2c.h"

#include <stdlib.h>

/**
 * @brief Initial LM72A address on I2C bus
 */
#define MLX90614_I2C_ADDRESS    0x5A

#define MLX90614_REG_ADDR_CONF  0x005
#define MLX90614_REG_ADDR_ADDR  0x00E

#define MLX90614_RAM_ADDR_AMB   0x006
#define MLX90614_RAM_ADDR_OBJ1  0x007
#define MLX90614_RAM_ADDR_OBJ2  0x008

/**
 * @brief Structure that holds the MLX90614 driver parameters
 */
typedef struct {
    i2c_t i2c;          /**< The I2C device descriptor on which the MLX90614 module is attached */
} mlx90614_param_t;

/**
 * @brief Structure that holds the MLX90614 driver internal state and parameters
 */
typedef struct {
    mlx90614_param_t params;   /**< Holds driver parameters */
} mlx90614_t;

/**
 * @brief MLX90614 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param MLX90614 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int mlx90614_init(mlx90614_t *dev);

/**
 * @brief Gets an ambient temperature in Celsius degrees
 *
 * @param[in] dev pointer to the initialized MLX90614 device
 */
int16_t mlx90614_get_ambient_temperature(mlx90614_t *dev);

int16_t mlx90614_get_object_temperature(mlx90614_t *dev);

int16_t mlx90614_get_object2_temperature(mlx90614_t *dev);

#endif /* MLX90614_H_ */
