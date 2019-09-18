/*
 * Copyright (C) 2018 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_fdc1004 FDC1004 capacitive sensor 
 * @ingroup     drivers_sensors
 * @brief       Driver for the TI FDC1004 capacitive sensor
 *
 * @{
 * @file        fdc1004.h
 * @brief       Device driver interface for FDC1004 capacitive sensor
 * @author      Oleg Artamonov     <info@unwds.com>
 * @author      Alexander Ugorelov <info@unwds.com>
 */
#ifndef FDC1004_H_
#define FDC1004_H_

#include <stdlib.h>

#include "periph/i2c.h"

/**
 * @brief FDC1004 address on I2C bus
 */
#define FDC1004_ADDRESS                 0x50

/**
 * @brief FDC1004 registers
 */
#define FDC1004_REG_MEAS1_MSB           0x00
#define FDC1004_REG_MEAS1_LSB           0x01
#define FDC1004_REG_MEAS2_MSB           0x02
#define FDC1004_REG_MEAS2_LSB           0x03
#define FDC1004_REG_MEAS3_MSB           0x04
#define FDC1004_REG_MEAS3_LSB           0x05
#define FDC1004_REG_MEAS4_MSB           0x06
#define FDC1004_REG_MEAS4_LSB           0x07
#define FDC1004_REG_CONF_MEAS1          0x08
#define FDC1004_REG_CONF_MEAS2          0x09
#define FDC1004_REG_CONF_MEAS3          0x0A
#define FDC1004_REG_CONF_MEAS4          0x0B
#define FDC1004_REG_FDC_CONF            0x0C
#define FDC1004_REG_OFFSET_CAL_CIN1     0x0D
#define FDC1004_REG_OFFSET_CAL_CIN2     0x0E
#define FDC1004_REG_OFFSET_CAL_CIN3     0x0F
#define FDC1004_REG_OFFSET_CAL_CIN4     0x10
#define FDC1004_REG_GAIN_CAL_CIN1       0x11
#define FDC1004_REG_GAIN_CAL_CIN2       0x12
#define FDC1004_REG_GAIN_CAL_CIN3       0x13
#define FDC1004_REG_GAIN_CAL_CIN4       0x14
#define FDC1004_REG_VENDOR_ID           0xFE
#define FDC1004_REG_DEVICE_ID           0xFF

#define FDC1004_VENDOR_ID               0x4954
#define FDC1004_DEVICE_ID               0x0410

typedef enum {
    FDC1004_RATE_100SS = 1,
    FDC1004_RATE_200SS = 2,
    FDC1004_RATE_400SS = 3
} fdc1004_rate_t;

/**
 * @brief Structure that holds the FDC1004 driver internal state and parameters
 */
typedef struct {
    i2c_t    i2c;                   /**< Holds I2C bus number */
    uint8_t  offset[4];             /**< Channel offset, pF, 0-96 */
    uint16_t gain[4];               /**< Channel gain, x1E3, 0-3163 */
    fdc1004_rate_t rate;            /**< FDC1004 sampling rate */
} fdc1004_t;

/**
 * @brief FDC1004 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param FDC1004 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of error
 */
int fdc1004_init(fdc1004_t *dev);

/**
 * @brief Gets capacitance measurement
 * 
 * @param[in] dev     pointer to the initialized FDC1004 device
 * @param[in] channel FDC1004 channel number (1-4)
 *
 * @return capacitance
 * 
 * TODO: Does not work for future use.
 */
uint32_t fdc1004_get_capacitance(fdc1004_t *dev, uint8_t channeln);

/**
 * @brief Get raw measurement
 * 
 * @param dev      pointer to the initialized FDC1004 device
 * @param channel  FDC1004 channel number (1-4)
 *
 * @return         raw data for channel number
 */
uint32_t fdc1004_get_raw_data(fdc1004_t *dev, uint8_t channel);

#endif /* FDC1004_H_ */
/** @} */