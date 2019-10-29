/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_lis2dh12 LIS2DH12 Accelerometer
 * @ingroup     drivers_sensors
 * @brief       Driver for the STM LIS2DH12 accelerometer
 *
 * This device driver provides a minimal interface to LIS2HH12 devices. As of
 * now, it only provides very basic access to the device. The LIS2HH12's
 * FIFO is bypassed, so the driver might not be sufficient for use cases where
 * the complete history of readings is of interest.
 *
 * Also, the current version of the driver supports only interfacing the sensor
 * via I2C.
 *
 * @{
 * @file
 * @brief       Interface definition for the STMicro LIS2DH12 accelerometer
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef LIS2DH12_H
#define LIS2DH12_H

#include <stdint.h>
#include <stdbool.h>

#include "periph/i2c.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SPI support is not implemented (yet), so throw an error when selected */
#ifdef MODULE_LIS2DH12_SPI
#error "LIS2HH12 error: SPI mode is not supported yet."
#endif

/**
 * @name    Set default configuration parameters for LIS2DH12 devices
 * @{
 */
#define LIS2DH12_SAD0L               (0x00)
#define LIS2DH12_SAD0H               (0x01)
#define LIS2DH12_I2C_SADROOT         (0x03)

/* I2C address if acc SA0 pin to GND */
#define LIS2DH12_I2C_SAD_L           ((LIS2DH12_I2C_SADROOT << 3) | \
                                       LIS2DH12_SAD0L)

/* I2C address if acc SA0 pin to Vdd */
#define LIS2DH12_I2C_SAD_H           ((LIS2DH12_I2C_SADROOT << 3) | \
                                       LIS2DH12_SAD0H)

/**
 * @brief   Available scale values
 */
typedef enum {
    LIS2DH12_SCALE_2G  = 0x00,                          /**< +- 2g */
    LIS2DH12_SCALE_4G  = 0x01,                          /**< +- 4g */
    LIS2DH12_SCALE_8G  = 0x02,                          /**< +- 8g */
    LIS2DH12_SCALE_16G = 0x03,                          /**< +- 16g */
} lis2dh12_scale_t;

/**
 * @brief   Available sampling rates
 */
typedef enum {
    LIS2DH12_ODR_POWER_DOWN = 0x00,                    /**< b0000: Power-down mode                       */
    LIS2DH12_ODR_1HZ        = 0x01,                    /**< b0001: HR / Normal / Low-power mode (1 Hz)   */
    LIS2DH12_ODR_10HZ       = 0x02,                    /**< b0010: HR / Normal / Low-power mode (10 Hz)  */
    LIS2DH12_ODR_25HZ       = 0x03,                    /**< b0011: HR / Normal / Low-power mode (25 Hz)  */
    LIS2DH12_ODR_50HZ       = 0x04,                    /**< b0100: HR / Normal / Low-power mode (50 Hz)  */
    LIS2DH12_ODR_100HZ      = 0x05,                    /**< b0101: HR / Normal / Low-power mode (100 Hz) */
    LIS2DH12_ODR_200HZ      = 0x06,                    /**< b0110: HR / Normal / Low-power mode (200 Hz) */
    LIS2DH12_ODR_400HZ      = 0x07,                    /**< b0111: HR / Normal / Low-power mode (400 Hz) */
    LIS2DH12_ODR_1620Hz     = 0x08,                    /**< b1000: Low-power mode (1.620 kHz)            */
    LIS2DH12_ODR_5376Hz     = 0x09,                    /**< b1001: HR/ Normal (1.344 kHz) Low-power mode (5.376 kHz) */
} lis2dh12_odr_t;

/**
 * @brief   Resolution (operating) modes
 */
typedef enum {
    LIS2DH12_HR_12BIT   = 0x00,                         /**< High-resolution mode */
    LIS2DH12_NM_10BIT   = 0x01,                         /**< Normal mode */
    LIS2DH12_LP_8BIT    = 0x02,                         /**< Low-power mode */
} lis2dh12_res_t;

/**
 * @brief   LIS2DH12 configuration parameters
 */
typedef struct {
    i2c_t               i2c_dev;                        /**< I2C device, clock stretching required (default I2C_DEV(0)) */
    uint8_t             i2c_addr;                       /**< I2C address (default LIS2DH12_I2C_SAD_L) */
    lis2dh12_scale_t    scale;                          /**< Sensor scale: 2, 4, 8, or 16 (G) */
    lis2dh12_odr_t      odr;                            /**< Sensor ODR setting: LIS2DH12_ODR_xxxHz */
    lis2dh12_res_t      res;                            /**< Resolution (operation mode) */
} lis2dh12_params_t;

/**
 * @brief   LIS2DH12 device descriptor
 */
typedef struct {
    lis2dh12_params_t params;                           /**<Device initialization parameters */
} lis2dh12_t;

/**
 * @brief   Driver error codes (returned as negative values)
 */
typedef enum {
    LIS2DH12_OK,                                        /**< no error */
    LIS2DH12_ERROR_I2C,                                 /**< I2C communication failure */
    LIS2DH12_ERROR_NO_DEV,                              /**< device not available */
    LIS2DH12_ERROR_NO_NEW_DATA,                         /**< no new data (last valid data returned) */
    LIS2DH12_ERROR_NOT_SUPPORTED,                       /**< function is not supported */
} lis2dh12_error_codes_t;

/**
 * @brief   Result vector for accelerometer measurement
 */
typedef struct {
    int16_t axis_x;                                     /**< Acceleration in the X direction in milli-G */
    int16_t axis_y;                                     /**< Acceleration in the Y direction in milli-G */
    int16_t axis_z;                                     /**< Acceleration in the Z direction in milli-G */
} lis2dh12_data_t;

/**
 * @brief   Initialize the given LIS2DH12 sensor device
 *
 * @param[in/out] dev       Device descriptor of sensor to initialize
 * @param[in]     params    Configuration parameters
 *
 * @return                  Error status
 */
int lis2dh12_init(lis2dh12_t *dev, const lis2dh12_params_t *params);

/**
 * @brief   Read 3D acceleration data from the accelerometer
 *
 * @param[in]  dev            Device descriptor of sensor
 * @param[out] acceleration   Accelerometer data output
 *
 * @return                    Error status
 */
int lis2dh12_read_xyz(lis2dh12_t *dev, lis2dh12_data_t *acceleration);

/**
 * @brief   Read temperature from the accelerometer
 * 
 * @param dev                Device descriptor of sensor
 * @param temperature_degC   Temperature output
 * 
 * @return                   Error status
 */
int lis2dh12_read_temp(lis2dh12_t *dev, int16_t *temperature_degC);

/**
 * @brief   Power on the given device
 * 
 * @param[in/out] dev   Device descriptor of sensor
 * 
 * @return           Error status
 */
int lis2dh12_poweron(lis2dh12_t *dev);

/**
 * @brief   Power off the given device
 *
 * @param[in/out] dev   Device descriptor of sensor
 *
 * @return  Error status
 */
int lis2dh12_poweroff(lis2dh12_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* LIS2DH12_H */
/** @} */
