/*
 * Copyright (C) 2015 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_lis3dh LIS3DH accelerometer
 * @ingroup     drivers_sensors
 * @brief       Device driver for the LIS3DH accelerometer
 * @{
 *
 * @file
 * @brief       Device driver interface for the LIS3DH accelerometer
 *
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 */

#ifndef LIS3DH_H
#define LIS3DH_H

#include <stdint.h>

#if defined (MODULE_LIS3DH_SPI)
#include "periph/spi.h"
#elif defined (MODULE_LIS3DH_I2C)
#include "periph/i2c.h"
#endif

#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   LIS2HH12 configuration parameters
 */
typedef struct {
    i2c_t i2c;                      /**< I2C device                */
    uint8_t i2c_addr;               /**< I2C address               */
    lis2hh12_odr_t odr;             /**< Output data range         */
    lis2hh12_scale_t scale;         /**< Sampling sensitivity used */
    lis2hh12_res_t resolution;      /**< Resolution */
} lis2hh12_params_t;



/**
 * @brief   Configuration parameters for LIS3DH devices
 */
#if defined (MODULE_LIS3DH_SPI)
typedef struct {
    spi_t     spi;              /**< SPI device the sensor is connected to */
    spi_clk_t clk;              /**< designated clock speed of the SPI bus */
    gpio_t    cs;               /**< Chip select pin */
    gpio_t    int1;             /**< INT1 pin */
    gpio_t    int2;             /**< INT2 (DRDY) pin */
    uint8_t   scale;            /**< Default sensor scale: 2, 4, 8, or 16 (G) */
    uint8_t   odr;              /**< Default sensor ODR setting: LIS3DH_ODR_xxxHz */
    
} lis3dh_params_t;
#elif defined (MODULE_LIS3DH_I2C)
typedef struct {
    i2c_t   i2c;                /**< I2C device */
    uint8_t addr;               /**< I2C address */
    gpio_t  int1;               /**< INT1 pin */
    gpio_t  int2;               /**< INT2 (DRDY) pin */
    uint8_t scale;              /**< Default sensor scale: 2, 4, 8, or 16 (G) */
    uint8_t odr;                /**< Default sensor ODR setting: LIS3DH_ODR_xxxHz */
    
} lis3dh_params_t;
#endif

/**
 * @brief   Device descriptor for LIS3DH sensors
 */
typedef struct {
    lis3dh_params_t params; /**< Device initialization parameters */
    uint16_t scale;         /**< Internal sensor scale */
} lis3dh_t;

/**
 * @brief   Result vector for accelerometer measurement
 */
typedef struct __attribute__((packed))
{
    int16_t acc_x;          /**< Acceleration in the X direction in milli-G */
    int16_t acc_y;          /**< Acceleration in the Y direction in milli-G */
    int16_t acc_z;          /**< Acceleration in the Z direction in milli-G */
} lis3dh_data_t;

/**
 * @brief   Initialize a LIS3DH sensor instance
 *
 * @param[in]  dev          Device descriptor of sensor to initialize
 * @param[in]  params       Configuration parameters
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_init(lis3dh_t *dev, const lis3dh_params_t *params);

/**
 * @brief   Read 3D acceleration data from the accelerometer
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[out] acc_data     Accelerometer data output buffer
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_read_xyz(const lis3dh_t *dev, lis3dh_data_t *acc_data);

/**
 * @brief   Read auxiliary ADC channel 1 data from the accelerometer
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[out] out          The value of ADC1 (OUT_1_{L,H}) will be written to this buffer
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_read_aux_adc1(const lis3dh_t *dev, int16_t *out);

/**
 * @brief   Read auxiliary ADC channel 2 data from the accelerometer
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[out] out          The value of ADC2 (OUT_2_{L,H}) will be written to this buffer
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_read_aux_adc2(const lis3dh_t *dev, int16_t *out);

/**
 * @brief   Read auxiliary ADC channel 3 data from the accelerometer
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[out] out          The value of ADC3 (OUT_3_{L,H}) will be written to this buffer
 *
 * @note The internal temperature sensor is connected to the third channel on
 *       the auxiliary ADC when the TEMP_EN bit of TEMP_CFG_REG is set.
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_read_aux_adc3(const lis3dh_t *dev, int16_t *out);

/**
 * @brief   Turn on/off power to the auxiliary ADC in LIS3DH.
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[in]  enable       Power state of the auxiliary ADC
 * @param[in]  temperature  If not zero, switch the ADC mux so that a
 *                          temperature reading is available on OUT_3_L, OUT_3_H.
 *
 * @note This ADC is only used for the temperature reading and the external ADC
 *       pins. The accelerometer ADC is turned on by lis3dh_set_odr().
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_set_aux_adc(const lis3dh_t *dev, const uint8_t enable, const uint8_t temperature);

/**
 * @brief   Enable/disable accelerometer axes.
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[in]  axes         An OR-ed combination of LIS3DH_AXES_X,
 *                          LIS3DH_AXES_Y, LIS3DH_AXES_Z.
 *
 * @note The macro LIS3DH_AXES_XYZ is a convenience shortcut to enable all axes.
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_set_axes(const lis3dh_t *dev, const uint8_t axes);

/**
 * @brief   Enable/disable the FIFO.
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[in]  mode         FIFO mode, see data sheet for details.
 * @param[in]  watermark    Watermark level for FIFO level interrupts
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_set_fifo(const lis3dh_t *dev, const uint8_t mode, const uint8_t watermark);

/**
 * @brief   Set the output data rate of the sensor.
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[in]  odr          Chosen output data rate.
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_set_odr(const lis3dh_t *dev, const uint8_t odr);

/**
 * @brief   Set the full scale range of the sensor.
 *
 * Valid values for scale are 2, 4, 8, 16 and represents the full range of the
 * sensor.
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[in]  scale        The chosen sensitivity scale.
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_set_scale(lis3dh_t *dev, const uint8_t scale);

/**
 * @brief   Set INT1 pin function
 *
 * Set the bits of CTRL_REG3 for choosing sources for the INT1 pin.
 *
 * @param[in]  dev          Device descriptor of sensor
 * @param[in]  mode         CTRL_REG3 value, see data sheet for details.
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int lis3dh_set_int1(const lis3dh_t *dev, const uint8_t mode);

/**
 * @brief   Get the current number of elements in the FIFO
 *
 * @param[in]  dev          Device descriptor of sensor
 *
 * @return                  number of elements in device FIFO on success
 * @return                  -1 on error
 */
int lis3dh_get_fifo_level(const lis3dh_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_H */
/** @} */
