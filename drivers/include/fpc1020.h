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

#define FPC1020_PIXEL_COLUMNS                   192
#define FPC1020_PIXEL_ROWS                      192
#define FPC1020_ADC_GROUP_SIZE                  8
#define FPC1020_MAX_IMAGE_SIZE                  36864
#define FPC1020_SPI_SPEED                       SPI_CLK_1MHZ
#define FPC1020_EXT_HWID_CHECK_ID1020A_ROWS     5
#define FPC1020_BUFFER_MAX_IMAGES               3
#define FPC1020_WAKEUP_DETECT_ZONE_COUNT        2

typedef enum {
    FPC102X_ERROR_NO_ERROR = 0,
    FPC102X_ERROR_IRQ_NOT_CLEARED,
    FPC102X_ERROR_IRQ_TIMEOUT,
    FPC102X_ERROR_HWID_MISMATCH,
    FPC102X_ERROR_IMAGE_CAPTURE,
    FPC102X_ERROR_OUT_OF_BUFFER,
} fpc1020_errors_t;

typedef struct fpc1020_setup {
	uint8_t adc_gain[FPC1020_BUFFER_MAX_IMAGES];
	uint8_t adc_shift[FPC1020_BUFFER_MAX_IMAGES];
	uint16_t pxl_ctrl[FPC1020_BUFFER_MAX_IMAGES];
	uint8_t capture_settings_mux;
	uint8_t capture_count;
	fpc1020_capture_mode_t capture_mode;
	uint8_t capture_row_start;	/* Row 0-191        */
	uint8_t capture_row_count;	/* Rows <= 192      */
	uint8_t capture_col_start;	/* ADC group 0-23   */
	uint8_t capture_col_groups;	/* ADC groups, 1-24 */
	uint8_t capture_finger_up_threshold;
	uint8_t capture_finger_down_threshold;
	uint8_t finger_detect_threshold;
	uint8_t wakeup_detect_rows[FPC1020_WAKEUP_DETECT_ZONE_COUNT];
	uint8_t wakeup_detect_cols[FPC1020_WAKEUP_DETECT_ZONE_COUNT];
	bool finger_auto_threshold;
} fpc1020_setup_t;

typedef enum {
	FPC1020_CONDITIONS_DRY = 0,
	FPC1020_CONDITIONS_WET = 1,
	FPC1020_CONDITIONS_NORMAL = 2,
} fpc1020_conditions_t;

/**
 * @brief Device descriptor for FPC1020 sensors.
 */
typedef struct {
    spi_t spi;                                  /**< SPI bus the sensor is connected to */
    gpio_t cs;                                  /**< SPI CS pin */
    gpio_t reset;                               /**< RESET pin */
    gpio_t irq;                                 /**< IRQ pin */
    uint8_t revision;                           /**< FPC1020A chip revision */
    uint8_t fp_threshold;                       /**< Finger detection threshold */
    uint8_t fp_conditions;                      /**< Ambient conditions */
    fpc1020_setup_t setup;                      /**< Default settings */
    uint8_t *image;                             /**< Image buffer */
} fpc1020_t;

/**
 * @brief Initialize the FPC1020 sensor driver.
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

/**
 * @brief Gets fingerprint image.
 *
 * Image will be put to dev->image
 *
 * @param[in]  dev          pointer to sensor device descriptor
 *
 * @return                  image size on success
 * @return                  <0 on error
 */
int fpc1020_get_fingerprint(fpc1020_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* FPC1020_H */
/** @} */
