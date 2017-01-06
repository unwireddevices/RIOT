/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file		sht21.h
 * @brief       driver for SHT21 sensor
 * @author      SmallSharky
 */
#ifndef SHT21_H_
#define SHT21_H_ 1

#include "thread.h"
#include "periph/i2c.h"

#include <stdlib.h>
#include <math.h>

/**
 * @brief Initial SHT21 address on I2C bus
 */
#define SHT21_ADDRESS 0x40

/**
 * @brief SHT21 commands
 */
#define SHT21_REG_T_HOLD        0xE3
#define SHT21_REG_T_NACK        0xF3

#define SHT21_REG_RH_HOLD       0xE5
#define SHT21_REG_RH_NACK       0xF5

#define SHT21_REG_USER_WRITE    0xE6
#define SHT21_REG_USER_READ     0xE7

#define SHT21_REG_SOFT_RESET    0xFE

/*---------------------------------------------------------------------------*/
/**
 *  * \name SHT21 register addresses and values
 *   * @{
 *    */
#define SHT21_USER_REG_READ             (0xE7)
#define SHT21_USER_REG_WRITE            (0xE6)
#define SHT21_USER_REG_RESERVED_BITS    (0x38)

#define SHT21_STATUS_MASK               (0xFC)

#define SHT21_RESOLUTION_12b_14b        ((0 << 7) | (0 << 0))
#define SHT21_RESOLUTION_8b_12b         ((0 << 7) | (1 << 0))
#define SHT21_RESOLUTION_10b_13b        ((1 << 7) | (0 << 0))
#define SHT21_RESOLUTION_11b_11b        ((1 << 7) | (1 << 0))
#define SHT21_BATTERY_ABOVE_2V25        (0 << 6)
#define SHT21_BATTERY_BELOW_2V25        (1 << 6)
#define SHT21_ONCHIP_HEATER_ENABLE      (1 << 2)
#define SHT21_ONCHIP_HEATER_DISABLE     (0 << 2)
#define SHT21_OTP_RELOAD_ENABLE         (0 << 1)
#define SHT21_OTP_RELOAD_DISABLE        (1 << 1)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 *  * \name SHT21 configuration values
 *   * @{
 *    */
#define SHT21_DEFAULT_CONFIG            (SHT21_RESOLUTION_12b_14b | \
                                         SHT21_ONCHIP_HEATER_DISABLE | \
                                         SHT21_BATTERY_ABOVE_2V25 | \
                                         SHT21_OTP_RELOAD_DISABLE)

#define SHT21_USER_CONFIG               (SHT21_RESOLUTION_8b_12b | \
                                         SHT21_ONCHIP_HEATER_DISABLE | \
                                         SHT21_BATTERY_ABOVE_2V25 | \
                                         SHT21_OTP_RELOAD_DISABLE)
/** @} */
/*---------------------------------------------------------------------------*/

/**
 * @brief Structure that holds the SHT21 driver internal state and parameters
 */
typedef struct {
    i2c_t i2c;    /**< Holds driver parameters */

} sht21_t;


typedef struct {
    int temperature;        /**< temperature in mC */
    int humidity;           /**< humidity in 1/1000 % (promille) */
} sht21_measure_t;

/**
 * @brief SHT21 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param SHT21 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int sht21_init(sht21_t *dev);

/**
 * @brief Get SHT21 measure
 *
 * @param[in] dev pointer to the initialized SHT21 device
 * @param[in] measure pointer to the allocated memory
 *
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * sht21_measure_t measure;
 *
 * sht21_init(sht21);
 * sht21_measure(sht21, &measure)
 */
uint32_t sht21_measure(sht21_t *dev, sht21_measure_t *measure);

#endif
