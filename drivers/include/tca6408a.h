/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_tca6408a TI TCA6408A I/O Expander driver
 * @ingroup     
 * @brief       Driver for TI TCA6408A I/O Expander.
 *
 * @{
 *
 * @file
 * @brief       Public interface for TCA6408A driver
 * 
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef _TCA6408A_H_
#define _TCA6408A_H_

#include <stdint.h>
#include <stdbool.h>

#include "periph/i2c.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for TCA6408A devices
 * @{
 */
#define TCA6408A_SAD0L               (0x00)     // ADDR tied to Vcc
#define TCA6408A_SAD0H               (0x01)     // ADDR tied to GND
#define TCA6408A_I2C_SADROOT         (0x20)

/* I2C address if acc SA0 pin to GND */
#define TCA6408A_I2C_SAD_L           (TCA6408A_I2C_SADROOT | TCA6408A_SAD0L)

/* I2C address if acc SA0 pin to Vdd */
#define TCA6408A_I2C_SAD_H           (TCA6408A_I2C_SADROOT | TCA6408A_SAD0H)

/**
 * @brief   TCA6408A pin mode
 */
typedef enum {
    TCA6408A_PIN_OUTPUT = 0x00,
    TCA6408A_PIN_INPUT  = 0x01,
} tca6408a_mode_pin_t;

/**
 * @brief   TCA6408A configuration parameters
 */
typedef struct {
    i2c_t                 i2c_dev;                      /**< I2C device, clock stretching required (default I2C_DEV(0)) */
    uint8_t               i2c_addr;                     /**< I2C address (default TCA6408A_I2C_SAD_L) */
} tca6408a_params_t;

/**
 * @brief   TCA6408A device descriptor
 */
typedef struct {
    tca6408a_params_t params;                           /**< Device initialization parameters */
} tca6408a_t;

/**
 * @brief   Driver error codes (returned as negative values)
 */
typedef enum {
    TCA6408A_OK,                                        /**< no error */
    TCA6408A_ERROR_I2C,                                 /**< I2C communication failure */
    TCA6408A_ERROR_NO_DEV,                              /**< device not available */
    TCA6408A_WRONG_PARAM,                               /**< wrong parameter */
} tca6408a_error_codes_t;

/**
 * @brief   Initialize the given TCA6408A sensor device
 *
 * @param[in/out] dev       Device descriptor to initialize
 * @param[in]     params    Configuration parameters
 *
 * @return                  Error status
 */
int tca6408a_init(tca6408a_t *dev, const tca6408a_params_t *params);

/**
 * @brief   Read input register values
 *
 * @param[in]  dev       Device descriptor
 * @param[in]  pin_num   Pin number
 * @param[out] value     Pin value
 *
 * @return                  Error status
 */
int tca6408a_read_input(const tca6408a_t *dev, const uint8_t pin_num, uint8_t *value);

/**
 * @brief   Writes config value
 *
 * @param[in] dev       Device descriptor
 * @param[in] pin_num   Pin number
 * @param[in] mode      Pin's configuration parameters
 *
 * @return                  Error status
 */
int tca6408a_write_config(const tca6408a_t *dev, const uint8_t pin_num, const tca6408a_mode_pin_t mode);

/**
 * @brief   Writes output value
 *
 * @param[in] dev       Device descriptor
 * @param[in] pin_num   Pin number
 * @param[in] enable    Pin value (true - high, false - low)
 *
 * @return                  Error status
 */
int tca6408a_write_output(const tca6408a_t *dev, const uint8_t pin_num, const bool enable);

/**
 * @brief   Writes polarity inversion value
 *
 * @param[in] dev       Device descriptor
 * @param[in] pin_num   Pin number
 * @param[in] value     Pin polarity inversion value
 *
 * @return                  Error status
 */
int tca6408a_write_polarity(const tca6408a_t *dev, const uint8_t pin_num, const bool invert);


#ifdef __cplusplus
}
#endif

#endif /* _TCA6408A_H_ */
/** @} */
