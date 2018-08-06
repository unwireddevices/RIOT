/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_lis2hh12
 * @{
 *
 * @file
 * @brief       Default configuration for LIS2DH12 devices
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#ifndef LIS2HH12_PARAMS_H
#define LIS2HH12_PARAMS_H

#include "board.h"
#include "lis2hh12.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for LIS2HH12 devices
 * @{
 */

#ifndef LIS2HH12_PARAM_I2C
#define LIS2HH12_PARAM_I2C          (I2C_DEV(0))
#endif
#ifndef LIS2HH12_PARAM_SCALE
#define LIS2HH12_PARAM_SCALE        LIS2HH12_SCALE_2G
#endif
#ifndef LIS2HH12_PARAM_RATE
#define LIS2HH12_PARAM_RATE         LIS2HH12_RATE_100HZ
#endif

#ifndef LIS2HH12_PARAMS
#define LIS2HH12_PARAMS             { .i2c = LIS2HH12_PARAM_I2C,     \
                                      .odr = LIS2HH12_RATE_100HZ, \
                                      .scale  = LIS2HH12_SCALE_2G }
#endif

#ifndef LIS2HH12_SAULINFO
#define LIS2HH12_SAULINFO           { .name = "lis2hh12" }
#endif
/**@}*/

/**
 * @brief   LIS2DH12 configuration
 */
static const lis2hh12_params_t lis2hh12_params[] =
{
    LIS2HH12_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t lis2hh12_saul_info[] =
{
    LIS2HH12_SAULINFO
};

#ifdef __cplusplus
}
#endif

#endif /* LIS2DH12_PARAMS_H */
/** @} */
