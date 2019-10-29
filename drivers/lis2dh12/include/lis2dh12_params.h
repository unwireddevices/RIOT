/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lis2dh12
 * @brief       Default configuration for STMicro LIS2DH12 accelerometer
 * @author      Alexander Ugorelov <info@unwds.com>
 * @file
 * @{
 */
#ifndef LIS2DH12_PARAMS_H
#define LIS2DH12_PARAMS_H

#include "board.h"
#include "lis2dh12.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for LIS2DH12 devices
 * @{
 */
#ifndef LIS2DH12_PARAM_I2C
#define LIS2DH12_PARAM_I2C                              (I2C_DEV(0))
#endif
#ifndef LIS2DH12_PARAM_ADDR
#define LIS2DH12_PARAM_ADDR                             (LIS2DH12_I2C_SAD_L)
#endif
#ifndef LIS2DH12_PARAM_SCALE
#define LIS2DH12_PARAM_SCALE                            (LIS2DH12_SCALE_2G)
#endif
#ifndef LIS2DH12_PARAM_ODR
#define LIS2DH12_PARAM_ODR                             (LIS2DH12_ODR_1HZ)
#endif
#ifndef LIS2DH12_PARAM_RES
#define LIS2DH12_PARAM_RES                              (LIS2DH12_HR_12BIT)
#endif

#ifndef LIS2DH12_PARAMS
#define LIS2DH12_PARAMS              { .i2c_dev  = LIS2DH12_PARAM_I2C,   \
                                       .i2c_addr = LIS2DH12_PARAM_ADDR,  \
                                       .scale    = LIS2DH12_PARAM_SCALE, \
                                       .odr      = LIS2DH12_PARAM_ODR,   \
                                       .res      = LIS2DH12_PARAM_RES }
#endif

/**@}*/

/**
 * @brief   LIS2DH12 configuration
 */
static const lis2dh12_params_t lis2dh12_params[] =
{
    LIS2DH12_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* LIS2DH12_PARAMS_H */
/** @} */
