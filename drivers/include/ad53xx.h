/*
 * Copyright (C) 2017 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ad53xx AD53xx SPI DAC
 * @ingroup     drivers_sensors
 * @brief       Driver for the Analog Devices AD5308/AD5318/AD5328 DACs.
 * @{
 *
 * @file
 * @brief       Interface definition for the AD53xx SPI DACs
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#ifndef AD53XX_H_
#define AD53XX_H_

#include <stdint.h>
#include <stdbool.h>
#include "periph/spi.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Device descriptor for AD53xx sensors.
 */
typedef struct {
    spi_t spi;              /**< SPI bus the sensor is connected to */
    gpio_t cs;              /**< CS pin GPIO handle */
    gpio_t ldac;            /**< LDAC pin GPIO handle */
    bool initialized;       /**< sensor status, true if sensor is initialized */
} ad53xx_t;

/* select DAC model */
#define AD53XX_AD5318

/**
 * @brief Initialize the AD53xx sensor driver.
 *
 * @note The SPI bus is expected to have been initialized when adt7310_init is called.
 *
 * @param[in]  dev          pointer to sensor device descriptor
 * @param[in]  spi          SPI bus the sensor is connected to
 * @param[in]  cs           GPIO pin the chip select signal is connected to
 * @param[in]  ldac         GPIO pin the chip LDAC signal is connected to
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ad53xx_init(ad53xx_t *dev, spi_t spi, gpio_t cs, gpio_t ldac);

int ad53xx_set_single(ad53xx_t *dev, uint8_t channel, uint16_t data);
int ad53xx_set_all(ad53xx_t *dev, uint16_t *data);

#ifdef __cplusplus
}
#endif

#endif /* AD53XX_H_ */
/** @} */
