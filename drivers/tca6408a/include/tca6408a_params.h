/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_tda6408a
 * @brief       Default configuration for TI TCA6408A I/O Expander 
 * @author      Alexander Ugorelov <info@unwds.com>
 * @file
 * @{
 */
#ifndef _TCA6408A_PARAMS_H
#define _TCA6408A_PARAMS_H

#include "board.h"
#include "tca6408a.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for TCA6408A devices
 * @{
 */
#ifndef TCA6408A_PARAM_I2C
#define TCA6408A_PARAM_I2C                              (I2C_DEV(0))
#endif
#ifndef TCA6408A_PARAM_ADDR
#define TCA6408A_PARAM_ADDR                             (TCA6408A_I2C_SAD_L)
#endif

#ifndef TCA6408A_PARAMS
#define TCA6408A_PARAMS              { .i2c_dev  = TCA6408A_PARAM_I2C, \
                                       .i2c_addr = TCA6408A_PARAM_ADDR }
#endif

/**@}*/

/**
 * @brief   TCA6408A configuration
 */
static const tca6408a_params_t tca6408a_params[] =
{
    TCA6408A_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* _TCA6408A_PARAMS_H */
/** @} */
