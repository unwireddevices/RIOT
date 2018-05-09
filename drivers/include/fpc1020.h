/*
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (c) 2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_fpc1020 FPC1020 capacitive fingerprint sensor
 * @ingroup     drivers_sensors
 * @brief       Device driver interface for the FPC1020 sensor.
 *
 * @author      Oleg Artamonov <info@unwds.com>
 */

#ifndef FPC1020_H
#define FPC1020_H

#include <inttypes.h>
#include "periph/spi.h"
#include "fpc1020_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FPC102X_ERROR_NO_ERROR = 0,
    FPC102X_ERROR_IRQ_NOT_CLEARED,
    FPC102X_ERROR_IRQ_TIMEOUT,
    FPC102X_ERROR_HWID_MISMATCH,
    FPC102X_ERROR_IMAGE_CAPTURE,
} fpc1020_errors_t;

/**
 * @brief Device descriptor for FPC1020 sensors.
 */
typedef struct {
    spi_t spi;                                  /**< SPI bus the sensor is connected to */
    gpio_t cs;                                  /**< SPI CS pin */
    gpio_t reset;                               /**< RESET pin */
    gpio_t irq;                                 /**< IRQ pin */
    uint8_t revision;                           /**< FPC1020A chip revision */
    uint8_t image[FPC1020_MAX_IMAGE_SIZE];      /**< Image buffer, default size is 192*192 = 36864 bytes */
} fpc1020_t;

/**
 * @brief Initialize the FPC1020 sensor driver.
 *
 * @note The SPI bus is expected to have been initialized when adt7310_init is called.
 *
 * @param[in]  dev          pointer to sensor device descriptor
 * @param[in]  spi          SPI bus the sensor is connected to
 * @param[in]  cs           GPIO pin the chip SPI CS is connected to
 * @param[in]  reset        GPIO pin the chip RESET signal is connected to
 * @param[in]  irq          GPIO pin the chip IRQ signal is connected to
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int fpc1020_init(fpc1020_t *dev, spi_t spi, gpio_t cs, gpio_t reset, gpio_t irq);

#ifdef __cplusplus
}
#endif

#endif /* FPC1020_H */
/** @} */
