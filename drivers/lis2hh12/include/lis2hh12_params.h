/*
 * Copyright (C) 2016-2019 Unwired Devices
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
 * @brief       Default configuration for LIS2HH12 devices
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
#define LIS2HH12_ACC_SAD0L          (0x02)
#define LIS2HH12_ACC_SAD0H          (0x01)
#define LIS2HH12_ACC_I2C_SADROOT    (0x07)

/* I2C address if acc SA0 pin to GND */
#define LIS2HH12_ACC_I2C_SAD_L      ((LIS2HH12_ACC_I2C_SADROOT << 2)| \
                                      LIS2HH12_ACC_SAD0L)

/* I2C address if acc SA0 pin to Vdd */
#define LIS2HH12_ACC_I2C_SAD_H      ((LIS2HH12_ACC_I2C_SADROOT << 2)| \
                                      LIS2HH12_ACC_SAD0H)

#ifndef LIS2HH12_PARAM_I2C
#define LIS2HH12_PARAM_I2C          (I2C_DEV(0))
#endif
#ifndef LIS2HH12_PARAM_ADDR
#define LIS2HH12_PARAM_ADDR         (LIS2HH12_ACC_I2C_SAD_L)
#endif
#ifndef LIS2HH12_PARAM_SCALE
#define LIS2HH12_PARAM_SCALE        LIS2HH12_SCALE_2G
#endif
#ifndef LIS2HH12_PARAM_ODR
#define LIS2HH12_PARAM_ODR          LIS2HH12_ODR_400HZ
#endif
#ifndef LIS2HH12_PARAM_RES
#define LIS2HH12_PARAM_RES          LIS2HH12_RES_HR
#endif
#ifndef LIS2HH12_PARAM_PIN_INT1
#define LIS2HH12_PARAM_PIN_INT1     GPIO_UNDEF
#endif
#ifndef LIS2HH12_PARAM_INT1_MODE
#define LIS2HH12_PARAM_INT1_MODE    (INT1_DISABLE)
#endif

#ifndef LIS2HH12_PARAMS
#define LIS2HH12_PARAMS             { .i2c_dev    = LIS2HH12_PARAM_I2C,       \
                                      .i2c_addr   = LIS2HH12_PARAM_ADDR,      \
                                      .odr        = LIS2HH12_PARAM_ODR,       \
                                      .scale      = LIS2HH12_PARAM_SCALE,     \
                                      .res        = LIS2HH12_PARAM_RES,       \
                                      .int1_pin   = LIS2HH12_PARAM_PIN_INT1,  \
                                      .int1_mode  = LIS2HH12_PARAM_INT1_MODE, \
                                    }
#endif

#ifndef LIS2HH12_SAULINFO
#define LIS2HH12_SAULINFO           { .name = "lis2hh12" }
#endif
/**@}*/

/**
 * @brief   LIS2HH12 configuration
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

#endif /* LIS2HH12_PARAMS_H */
/** @} */
