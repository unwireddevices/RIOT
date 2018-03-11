/*
 * Copyright (C) 2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_tic33 TIC33 LCD Display
 * @ingroup     drivers_displays
 * @brief       Driver for the TIC33 LCD Display.
 * @{
 *
 * @file
 * @brief       Interface definition for the TIC33 LCD display driver
 *
 * @author      Oleg Artamonov
 */

#ifndef TIC33_H_
#define TIC33_H_

#include <stdint.h>
#include <stdbool.h>
#include "periph/spi.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define TIC33_MAX_CHARACTERS 9
#define TIC33_LCLK_PERIOD 10

/**
 * @brief Device descriptor for AD53xx sensors.
 */
typedef struct {
    gpio_t din;             /**< DIN pin GPIO handle */
    gpio_t dclk;            /**< DCLK pin GPIO handle */
    gpio_t lclk;            /**< LCLK pin GPIO handle */
    gpio_t load;            /**< LOAD pin GPIO handle */
    bool   clock_enable;    /**< Toggle lclk every TIC33_LCLK_PERIOD ms if true */
} tic33_t;

/**
 * @brief Initialize the TIC33 display driver.
 *
 * @param[in]  dev          pointer to device descriptor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int tic33_init(tic33_t *dev);

int tic33_puts(tic33_t *dev, char *str);
void tic33_clear(tic33_t *dev);
void tic33_lclk_toggle(tic33_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* TIC33_H_ */
/** @} */
