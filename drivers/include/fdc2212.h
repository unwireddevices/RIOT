/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_fdc2212 FDC2212 capacitive sensor 
 * @ingroup     drivers_sensors
 * @brief       Device Driver for for the TI FDC2212 digital capacitive sensor
 *
 * @{
 * @file        fdc2212.h
 * @brief       Device driver interface for FDC2212 digital capacitive sensor
 * @author      Alexander Ugorelov <info@unwds.com>
 */
#ifndef __FDC2212_H__
#define __FDC2212_H__

#include <stdint.h>

#include "periph/i2c.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* I2C address if capsens ADDR pin to GND */
#define FDC2212_CAP_I2C_ADDR_L                          (0x2A)

/* I2C address if capsens ADDR pin to Vdd */
#define FDC2212_CAP_I2C_ADDR_H                          (0x2B)

/* Number of measurement channels */
/* TODO: Works with only one channel number 0 */
#define FDC2212_NUM_OF_CHANNELS                         (1)

/**
 * @brief   FDC2212 Sensor drive current
 */
typedef enum {
    FDC2212_IDRIVE_0P016,                               /**< 00000: 0.016mA */
    FDC2212_IDRIVE_0P018,                               /**< 00001: 0.018mA */
    FDC2212_IDRIVE_0P021,                               /**< 00010: 0.021mA */
    FDC2212_IDRIVE_0P025,                               /**< 00011: 0.025mA */
    FDC2212_IDRIVE_0P028,                               /**< 00100: 0.028mA */
    FDC2212_IDRIVE_0P033,                               /**< 00101: 0.033mA */
    FDC2212_IDRIVE_0P038,                               /**< 00110: 0.038mA */
    FDC2212_IDRIVE_0P044,                               /**< 00111: 0.044mA */
    FDC2212_IDRIVE_0P052,                               /**< 01000: 0.052mA */
    FDC2212_IDRIVE_0P060,                               /**< 01001: 0.060mA */
    FDC2212_IDRIVE_0P069,                               /**< 01010: 0.069mA */
    FDC2212_IDRIVE_0P081,                               /**< 01011: 0.081mA */
    FDC2212_IDRIVE_0P093,                               /**< 01100: 0.093mA */
    FDC2212_IDRIVE_0P108,                               /**< 01101: 0.108mA */
    FDC2212_IDRIVE_0P126,                               /**< 01110: 0.126mA */
    FDC2212_IDRIVE_0P146,                               /**< 01111: 0.146mA */
    FDC2212_IDRIVE_0P169,                               /**< 10000: 0.169mA */
    FDC2212_IDRIVE_0P196,                               /**< 10001: 0.196mA */
    FDC2212_IDRIVE_0P228,                               /**< 10010: 0.228mA */
    FDC2212_IDRIVE_0P264,                               /**< 10011: 0.264mA */
    FDC2212_IDRIVE_0P307,                               /**< 10100: 0.307mA */
    FDC2212_IDRIVE_0P356,                               /**< 10101: 0.356mA */
    FDC2212_IDRIVE_0P413,                               /**< 10110: 0.413mA */
    FDC2212_IDRIVE_0P479,                               /**< 10111: 0.479mA */
    FDC2212_IDRIVE_0P555,                               /**< 11000: 0.555mA */
    FDC2212_IDRIVE_0P644,                               /**< 11001: 0.644mA */
    FDC2212_IDRIVE_0P747,                               /**< 11010: 0.747mA */
    FDC2212_IDRIVE_0P867,                               /**< 11011: 0.867mA */
    FDC2212_IDRIVE_1P006,                               /**< 11100: 1.006mA */
    FDC2212_IDRIVE_1P167,                               /**< 11101: 1.167mA */
    FDC2212_IDRIVE_1P354,                               /**< 11110: 1.354mA */
    FDC2212_IDRIVE_1P571,                               /**< 11111: 1.571mA */
} fdc2212_idrive_t;

/**
 * @brief   FDC2212 Input deglitch filter bandwidth
 */
typedef enum {
    FDC2212_DEGLITCH_1MHZ   = 0x0001,
    FDC2212_DEGLITCH_3P3MHZ = 0x0004,
    FDC2212_DEGLITCH_10MHZ  = 0x0005,
    FDC2212_DEGLITCH_33MHZ  = 0x0007,
} fdc2212_deglitch_t;


/**
 * @brief   Driver error codes (returned as negative values)
 */
typedef enum {
    FDC2212_OK,                                         /**< no error */
    FDC2212_ERROR_I2C,                                  /**< I2C communication failure */
    FDC2212_ERROR_NO_DEV,                               /**< device not available */
    FDC2212_ERROR_SETTING_INV,                          /**< invalid setting*/
    FDC2212_ERROR_NO_NEW_DATA,                          /**< no new data */
} fdc2212_error_codes_t;

/**
 * @brief   FDC2212 device initialization parameters
 */
typedef struct {
    i2c_t   i2c_dev;                /**< I2C device, clock stretching required (default I2C_DEV(0)) */
    uint8_t i2c_addr;               /**< I2C address (default FDC2212_CAP_I2C_ADDR_L) */
    gpio_t  shutdown_pin;           /**< Shutdown signal pin (default GPIO_UNDEF) */
} fdc2212_params_t;


/**
 * @brief   FDC2212 sensor device data structure
 */
typedef struct {
    fdc2212_params_t params;                        /**< device initialization parameters */

    uint16_t ref_count[FDC2212_NUM_OF_CHANNELS];           /**< */
    uint16_t settle_count[FDC2212_NUM_OF_CHANNELS];        /**< */
    uint8_t  freq_in_sel[FDC2212_NUM_OF_CHANNELS];         /**< */
    uint16_t freq_divider[FDC2212_NUM_OF_CHANNELS];        /**< */
    fdc2212_idrive_t idrive[FDC2212_NUM_OF_CHANNELS];      /**< */
} fdc2212_t;

/**
 * @brief   Initialize a FDC2212 sensor device
 *
 * @param[in/out]   dev     Device descriptor of FDC2212 device to be initialized
 * @param[in]       params  Configuration parameters used by initialization
 *
 * @retval  FDC2212_OK      on success
 * @retval  FDC2212_ERROR_* on error, see #fdc2212_error_codes_t
 */
int fdc2212_init(fdc2212_t *dev, const fdc2212_params_t *params);

/**
 * @brief   Data-ready status function
 *
 * @param[in]   dev     Device descriptor of FDC2212 device 
 *
 * @retval  FDC2212_OK                when new data are available
 * @retval  FDC2212_ERROR_NO_NEW_DATA when no new data are available
 * @retval  FDC2212_ERROR_*           otherwise, see #fdc2212_error_codes_t.
 */
int fdc2212_data_ready(const fdc2212_t *dev);

/**
 * @brief   Read the raw data from sensor
 *
 * @param[in]   dev         Device descriptor of FDC2212 device
 * @param[in]   channel     Current channel
 * @param[out]  raw_data    Raw data
 *
 * @retval  FDC2212_OK      on success
 * @retval  FDC2212_ERROR_* on error, see #fdc2212_error_codes_t
 */
int fdc2212_read_raw_data(const fdc2212_t *dev, uint8_t channel, uint32_t *raw_data);

#ifdef __cplusplus
}
#endif

#endif /* __FDC2212_H__ */
/** @} */