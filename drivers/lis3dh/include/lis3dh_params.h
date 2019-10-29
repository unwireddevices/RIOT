/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lis3dh
 *
 * @{
 * @file
 * @brief       Default configuration for LIS3DH devices
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef LIS3DH_PARAMS_H
#define LIS3DH_PARAMS_H

#include "board.h"
#include "lis3dh.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif


#if defined (MODULE_LIS3DH_SPI)
/**
 * @name    Set default configuration parameters
 * @{
 */
#ifndef LIS3DH_PARAM_SPI
#define LIS3DH_PARAM_SPI            (SPI_DEV(0))
#endif
#ifndef LIS3DH_PARAM_CS
#define LIS3DH_PARAM_CS             (GPIO_PIN(0, 0))
#endif
#ifndef LIS3DH_PARAM_CLK
#define LIS3DH_PARAM_CLK            (SPI_CLK_5MHZ)
#endif
#ifndef LIS3DH_PARAM_INT1
#define LIS3DH_PARAM_INT1           (GPIO_PIN(0, 1))
#endif
#ifndef LIS3DH_PARAM_SCALE
#define LIS3DH_PARAM_SCALE          (4)
#endif
#ifndef LIS3DH_PARAM_ODR
#define LIS3DH_PARAM_ODR            (LIS3DH_ODR_100HZ)
#endif

#ifndef LIS3DH_PARAMS
#define LIS3DH_PARAMS               { .spi   = LIS3DH_PARAM_SPI,   \
                                      .cs    = LIS3DH_PARAM_CS,    \
                                      .clk   = LIS3DH_PARAM_CLK,   \
                                      .int1  = LIS3DH_PARAM_INT1,  \
                                      .scale = LIS3DH_PARAM_SCALE, \
                                      .odr   = LIS3DH_PARAM_ODR }
#endif
#ifndef LIS3DH_SAUL_INFO
#define LIS3DH_SAUL_INFO            { .name = "lis3dh" }
#endif
/**@}*/

/**
 * @brief   Allocate some memory to store the actual configuration
 */
static const lis3dh_params_t lis3dh_params[] =
{
    LIS3DH_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t lis3dh_saul_info[] =
{
    LIS3DH_SAUL_INFO
};
#elif defined (MODULE_LIS3DH_I2C)
/**
 * @name    Set default configuration parameters for LIS3DH devices
 * @{
 */
#define LIS3DH_SAD0L               (0x00)
#define LIS3DH_SAD0H               (0x01)
#define LIS3DH_I2C_SADROOT         (0x03)

/* I2C address if acc SA0 pin to GND */
#define LIS3DH_I2C_SAD_L           ((LIS3DH_I2C_SADROOT << 3)| \
                                      LIS3DH_SAD0L)

/* I2C address if acc SA0 pin to Vdd */
#define LIS3DH_I2C_SAD_H           ((LIS3DH_I2C_SADROOT << 3)| \
                                      LIS3DH_SAD0H)


/**
 * @name    Set default configuration parameters
 * @{
 */
#ifndef LIS3DH_PARAM_I2C
#define LIS3DH_PARAM_I2C            (I2C_DEV(0))
#endif
#ifndef LIS3DH_PARAM_ADDR
#define LIS3DH_PARAM_ADDR           (LIS3DH_I2C_SAD_L)
#endif
#ifndef LIS3DH_PARAM_INT1
#define LIS3DH_PARAM_INT1           GPIO_UNDEF
#endif
#ifndef LIS3DH_PARAM_INT1_MODE
#define LIS3DH_PARAM_INT1_MODE      (I1_DISABLE)
#endif
#ifndef LIS3DH_PARAM_SCALE
#define LIS3DH_PARAM_SCALE          (LIS3DH_SCALE_2G)
#endif
#ifndef LIS3DH_PARAM_ODR
#define LIS3DH_PARAM_ODR            (LIS3DH_ODR_1HZ)
#endif
#ifndef LIS3DH_PARAM_RES
#define LIS3DH_PARAM_RES            (LIS3DH_HR_12BIT)
#endif


#ifndef LIS3DH_PARAMS
#define LIS3DH_PARAMS               { .i2c_dev   = LIS3DH_PARAM_I2C,       \
                                      .i2c_addr  = LIS3DH_PARAM_ADDR,      \
                                      .int1      = LIS3DH_PARAM_INT1,      \
                                      .int1_mode = LIS3DH_PARAM_INT1_MODE, \
                                      .scale     = LIS3DH_PARAM_SCALE,     \
                                      .odr       = LIS3DH_PARAM_ODR,       \
                                      .res       = LIS3DH_PARAM_RES}
#endif
#ifndef LIS3DH_SAUL_INFO
#define LIS3DH_SAUL_INFO            { .name = "lis3dh" }
#endif
/**@}*/

/**
 * @brief   Allocate some memory to store the actual configuration
 */
static const lis3dh_params_t lis3dh_params[] =
{
    LIS3DH_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t lis3dh_saul_info[] =
{
    LIS3DH_SAUL_INFO
};
#endif



#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_PARAMS_H */
/** @} */
