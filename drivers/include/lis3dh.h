/*
 * Copyright (C) 2015 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_lis3dh LIS3DH accelerometer
 * @ingroup     drivers_sensors
 * @brief       Device driver for the LIS3DH accelerometer
 * @{
 *
 * @file
 * @brief       Device driver interface for the LIS3DH accelerometer
 *
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 */

#ifndef LIS3DH_H
#define LIS3DH_H

#include <stdint.h>

#if defined (MODULE_LIS3DH_SPI)
#include "periph/spi.h"
#elif defined (MODULE_LIS3DH_I2C)
#include "periph/i2c.h"
#endif

#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Configuration parameters for LIS3DH devices
 */
#if defined (MODULE_LIS3DH_SPI)
typedef struct {
    spi_t     spi;              /**< SPI device the sensor is connected to */
    spi_clk_t clk;              /**< designated clock speed of the SPI bus */
    gpio_t    cs;               /**< Chip select pin */
    gpio_t    int1;             /**< INT1 pin */
    gpio_t    int2;             /**< INT2 (DRDY) pin */
    uint8_t   scale;            /**< Default sensor scale: 2, 4, 8, or 16 (G) */
    uint8_t   odr;              /**< Default sensor ODR setting: LIS3DH_ODR_xxxHz */
    uint8_t   op_mode;          /**< Resolution */
    
} lis3dh_params_t;
#elif defined (MODULE_LIS3DH_I2C)
typedef struct {
    i2c_t   i2c;                /**< I2C device */
    uint8_t addr;               /**< I2C address */
    gpio_t  int1;               /**< INT1 pin */
    gpio_t  int2;               /**< INT2 (DRDY) pin */
    uint8_t scale;              /**< Default sensor scale: 2, 4, 8, or 16 (G) */
    uint8_t odr;                /**< Default sensor ODR setting: LIS3DH_ODR_xxxHz */
    uint8_t op_mode;            /**< Resolution */
    
} lis3dh_params_t;
#endif

/**
 * @brief   Device descriptor for LIS3DH sensors
 */
typedef struct {
    lis3dh_params_t params;     /**< Device initialization parameters */
    uint16_t scale;             /**< Internal sensor scale */
} lis3dh_t;

/**
 * @brief   Result vector for accelerometer measurement
 */
typedef struct
{
    int32_t axis_x;             /**< Acceleration in the X direction in milli-G */
    int32_t axis_y;             /**< Acceleration in the Y direction in milli-G */
    int32_t axis_z;             /**< Acceleration in the Z direction in milli-G */
} __attribute__((packed)) lis3dh_acceleration_t;

/**
 * @brief   Initialize a LIS3DH sensor instance
 *
 * @param[in]  dev          Device descriptor of sensor to initialize
 * @param[in]  params       Configuration parameters
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_init(lis3dh_t *dev, const lis3dh_params_t *params);

/**
 * @brief   Read 3D acceleration data from the accelerometer
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[out] acc_data     Accelerometer data output buffer
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_read_xyz(const lis3dh_t *dev, lis3dh_data_t *acc_data);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_H */
/** @} */
