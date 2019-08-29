/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_fdc2212
 * @brief       Default configuration for TI FDC2212 digital capacitor sensors
 * @author      Alexander Ugorelov <info@unwds.com>
 * @file
 * @{
 */
#ifndef __FDC2212_PARAMS_H__
#define __FDC2212_PARAMS_H__

#include "board.h"
#include "fdc2212.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for FDC2212 devices
 * @{
 */

#ifndef FDC2212_PARAM_I2C
#define FDC2212_PARAM_I2C                               (I2C_DEV(0))
#endif
#ifndef FDC2212_PARAM_ADDR
#define FDC2212_PARAM_ADDR                              (FDC2212_CAP_I2C_ADDR_L)
#endif
#ifndef FDC2212_PARAM_SHUTDOWN_PIN
#define FDC2212_PARAM_SHUTDOWN_PIN                      (GPIO_UNDEF)
#endif


#ifndef FDC2212_PARAMS
#define FDC2212_PARAMS              { .i2c_dev      = FDC2212_PARAM_I2C,  \
                                      .i2c_addr     = FDC2212_PARAM_ADDR, \
                                      .shutdown_pin = FDC2212_PARAM_SHUTDOWN_PIN}
#endif
/**@}*/

/**
 * @brief   FDC2212 configuration
 */
static const fdc2212_params_t fdc2212_params[] =
{
    FDC2212_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* __FDC2212_PARAMS_H__ */
/** @} */