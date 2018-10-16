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
 * @brief       Default configuration for LMP91000 devices
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */
#ifndef LMP91000_PARAMS_H
#define LMP91000_PARAMS_H

#include "board.h"
#include "lmp91000.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for LMP91000 devices
 * @{
 */

#ifndef LMP91000_PARAM_I2C
#define LMP91000_PARAM_I2C                  (I2C_DEV(0))
#endif

#ifndef LMP91000_PARAM_ADDR
#define LMP91000_PARAM_ADDR                 (0x48)
#endif

#ifndef LMP91000_PARAM_MODULE_EN_PIN
#define LMP91000_PARAM_MODULE_EN_PIN        (GPIO_UNDEF)
#endif

#ifndef LMP91000_PARAMS
#define LMP91000_PARAMS         {                                                   \
                                    .i2c            = LMP91000_PARAM_I2C,           \
                                    .i2c_addr       = LMP91000_PARAM_ADDR,          \
                                    .module_en_pin  = LMP91000_PARAM_MODULE_EN_PIN  \
                                }
#endif

/**@}*/

/**
 * @brief    LMP91000 configuration
 */
static const lmp91000_params_t lmp91000_params[] =
{
    LMP91000_PARAMS
};


#ifdef __cplusplus
}
#endif

#endif /* LMP91000_PARAMS_H */
/** @} */
