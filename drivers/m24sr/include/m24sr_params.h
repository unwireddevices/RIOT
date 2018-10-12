/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_m24sr
 * @{
 *
 * @file
 * @brief       Default configuration for M24SRxx devices
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#ifndef M24SR_PARAMS_H
#define M24SR_PARAMS_H

#include "board.h"
#include "m24sr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for M24SR devices
 * @{
 */

#ifndef M24SR_PARAM_I2C
#define M24SR_PARAM_I2C             (I2C_DEV(0))
#endif

#ifndef M24SR_PARAM_ADDR
#define M24SR_PARAM_ADDR            (0xAC)
#endif

#ifndef M24SR_PARAM_GPO_PIN
#define M24SR_PARAM_GPO_PIN         (GPIO_UNDEF)
#endif

#ifndef M24SR_PARAM_RF_DIS_PIN
#define M24SR_PARAM_RF_DIS_PIN      (GPIO_UNDEF)
#endif

#ifndef M24SR_PARAM_PWR_EN_PIN
#define M24SR_PARAM_PWR_EN_PIN      (GPIO_UNDEF)
#endif

#ifndef M24SR_PRIORITY_OPEN
#define M24SR_PRIORITY_OPEN         (I2C_OPEN_SESSION)
#endif

#ifndef M24SR_CLOSE_TOKEN_MODE
#define M24SR_CLOSE_TOKEN_MODE      (I2C_TOKEN_RELEASE_SW)
#endif 

#ifndef M24SR_PARAMS
#define M24SR_PARAMS            {                                               \
                                    .i2c            = M24SR_PARAM_I2C,          \
                                    .i2c_addr       = M24SR_PARAM_ADDR,         \
                                    .gpo_pin        = M24SR_PARAM_GPO_PIN,      \
                                    .rfdisable_pin  = M24SR_PARAM_RF_DIS_PIN,   \
                                    .pwr_en_pin     = M24SR_PARAM_PWR_EN_PIN,   \
                                    .priority       = M24SR_PRIORITY_OPEN,      \
                                    .token_mode     = M24SR_CLOSE_TOKEN_MODE    \
                                }
#endif

/**@}*/

/**
 * @brief    M24SRxx configuration
 */
static const m24sr_params_t m24sr_params[] =
{
     M24SR_PARAMS
};


#ifdef __cplusplus
}
#endif

#endif /* M24SR_PARAMS_H */
/** @} */
