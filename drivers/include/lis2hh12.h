/*
 * Copyright (C) 2016-2019 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    drivers_lis2hh12 LIS2HH12 Accelerometer
 * @ingroup     drivers_sensors
 * @brief       Driver for the STM LIS2HH12 accelerometer
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
 * @brief       Interface definition for the STM LIS2HH12 accelerometer
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 */

#ifndef LIS2HH12_H
#define LIS2HH12_H

#include "saul.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SPI support is not implemented (yet), so throw an error when selected */
#ifdef MODULE_LIS2HH12_SPI
#error "LIS2HH12 error: SPI mode is not supported yet."
#endif

/**
 * @brief   3d data container of the LIS2HH12 sensor
 */
typedef struct {
    int32_t x_axis;                  /**< Data from x-axis */
    int32_t y_axis;                  /**< Data from y_axis */
    int32_t z_axis;                  /**< Data from z_axis */
} __attribute__((packed)) lis2hh12_data_t;

/**
 * @brief   Available scale values
 */
typedef enum {
    LIS2HH12_SCALE_2G  = 0x00,      /**< +- 2g */
    LIS2HH12_SCALE_4G  = 0x20,      /**< +- 4g */
    LIS2HH12_SCALE_8G  = 0x30,      /**< +- 8g */
} lis2hh12_scale_t;

/**
 * @brief   Output data rate [Hz] for LIS2HH12
 */
typedef enum {
    LIS2HH12_ODR_PWRDWN = 0x00,     /**< Power-down mode    */
    LIS2HH12_ODR_10HZ   = 0x10,     /**< Active mode 10 Hz  */
    LIS2HH12_ODR_50HZ   = 0x20,     /**< Active mode 50 Hz  */
    LIS2HH12_ODR_100HZ  = 0x30,     /**< Active mode 100 Hz */
    LIS2HH12_ODR_200HZ  = 0x40,     /**< Active mode 200 Hz */
    LIS2HH12_ODR_400HZ  = 0x50,     /**< Active mode 400 Hz */
    LIS2HH12_ODR_800HZ  = 0x60,     /**< Active mode 800 Hz */
} lis2hh12_odr_t;

/**
 * @brief   LIS2HH12 high-res mode
 */
typedef enum {
    LIS2HH12_RES_NORMAL = 0x00,     /**< Normal mode          */
    LIS2HH12_RES_HR     = 0x80,     /**< High resolution mode */
} lis2hh12_res_t;

/**
 * @brief LIS2HH12 type of pin interrupt
 */
typedef enum {
    LIS2HH12_PUSH_PULL       = 0x00,
    LIS2HH12_OPEN_DRAIN      = 0x01,
} lis2hh12_pp_od_t;

/**
 *  @brief LIS2HH12 type of interrupt signal
 */
typedef enum {
    LIS2HH12_ACTIVE_HIGH     = 0x00,
    LIS2HH12_ACTIVE_LOW      = 0x02,
} lis2hh12_pin_pol_t;

/**
 * @brief   Interrupt 1 pin mode
 */
typedef enum {
    INT1_DISABLE = 0x00,            /**< Disable all on INT1 */
    INT1_DRDY    = 0x01,            /**< Data Ready signal on INT1 */
    INT1_FTH     = 0x02,            /**< FIFO threshold signal on INT1*/
    INT1_OVR     = 0x04,            /**< FIFO overrun signal on INT1 */
    INT1_IG1     = 0x08,            /**< Interrupt generator 1 on INT1 */
    INT1_IG2     = 0x10,            /**< Interrupt generator 2 on INT1 */
    INT1_INACT   = 0x20,            /**< Inactivity interrupt on INT1 */
} lis2hh12_int1_md_t;



/**
 * @brief   LIS2HH12 interrupt callback
 */
typedef void (*lis2hh12_int_cb_t)(void *);


/**
 * @brief   LIS2HH12 configuration parameters
 */
typedef struct {
    i2c_t               i2c_dev;        /**< I2C device                */
    uint8_t             i2c_addr;       /**< I2C address               */
    lis2hh12_odr_t      odr;            /**< Output data range         */
    lis2hh12_scale_t    scale;          /**< Sampling sensitivity used */
    lis2hh12_res_t      res;            /**< Resolution                */
    gpio_t              int1_pin;       /**< INT1 pin                  */
    lis2hh12_int1_md_t  int1_mode;      /**< INT1 mode                 */
} lis2hh12_params_t;


/**
 * @brief   LIS2HH12 device descriptor
 */
typedef struct {
    lis2hh12_params_t  params;      /**< device configuration */
    lis2hh12_int_cb_t  int1_cb;     /**< alert callback */
    void               *int1_arg;   /**< alert callback param */
} lis2hh12_t;

/**
 * @brief   Status and error return codes
 */
enum {
    LIS2HH12_OK     =  0,           /**< everything was fine */
    LIS2HH12_NOBUS  = -1,           /**< bus interface error */
    LIS2HH12_NODEV  = -2,           /**< unable to talk to device */
    LIS2HH12_NOCOM  = -3,           /**< communication failed */
    LIS2HH12_NODATA = -4,           /**< no data available */
    LIS2HH12_ERROR  = -5            /**< any error */
};

/**
 * @brief   Export the SAUL interface for this driver
 */
extern const saul_driver_t lis2hh12_saul_driver;

/**
 * @brief   Initialize the given LIS2HH12 sensor device
 *
 * @param[out] dev      device descriptor
 * @param[in]  params   static device configuration
 *
 * @return  LIS2HH12_OK on success
 * @return  LIS2HH12_NOBUS on bus errors
 * @return  LIS2HH12_NODEV if no LIS2HH12 device was found on the bus
 */
int lis2hh12_init(lis2hh12_t *dev, const lis2hh12_params_t *params, lis2hh12_int_cb_t cb, void *arg);

/**
 * @brief   Read acceleration data from the given device
 *
 * @param[in]  dev      device descriptor
 * @param[out] data     acceleration data in milli-g
 *
 * @return  LIS2HH12_OK on success
 * @return  LIS2HH12_NOBUS on bus error
 */
int lis2hh12_read_xyz(const lis2hh12_t *dev, lis2hh12_data_t *data);

/**
 * @brief   Read acceleration data from the given device
 *
 * @param[in]  dev      device descriptor
 * @param[out] value     temperature
 *
 * @return  LIS2HH12_OK on success
 * @return  LIS2HH12_NOBUS on bus error
 */
int lis2hh12_read_temp(const lis2hh12_t *dev, int16_t *value);


/**
 * @brief   Power on the given device
 *
 * @param[in] dev       device descriptor
 *
 * @return  LIS2HH12_OK on success
 * @return  LIS2HH12_NOBUS on bus error
 */
int lis2hh12_poweron(const lis2hh12_t *dev);

/**
 * @brief   Power off the given device
 *
 * @param[in] dev       device descriptor
 *
 * @return  LIS2HH12_OK on success
 * @return  LIS2HH12_NOBUS on bus error
 */
int lis2hh12_poweroff(const lis2hh12_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* LIS2HH12_H */
/** @} */
