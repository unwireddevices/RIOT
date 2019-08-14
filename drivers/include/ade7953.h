/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup
 * @ingroup
 * @brief
 * @{
 * @file		ade7953.h
 * @brief       driver for ADE7953
 * @author      Mikhail Perkov
 */
#ifndef ADE7953_H_
#define ADE7953_H_

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "periph/gpio.h"
#include "periph/spi.h"


#define ADE7953_MAX_BYTE_BUFF      64

/**
 * @brief ADE7953 return codes
*/
#define ADE7953_OK			        0
#define ADE7953_ERROR		        1

/**
 * @brief   ADE7953 hardware and global parameters.
 */
typedef struct { 
    uint8_t spi;        /**< SPI device */
    gpio_t cs_spi;      /**< SPI NSS pin */
    gpio_t irq;         /**< Interrupt input */
    gpio_t reset;       /**< Reset */
} ade7953_params_t;

/**
 * @brief   ADE7953 callback
 */
typedef void (*ade7953_cb_t)(void *);

/**
 * @brief   ADE7953 device descriptor
 */
typedef struct {
    ade7953_params_t params;   /**< device driver configuration */
    ade7953_cb_t cb;           /**< callback */
    void *arg;              /**< callback param */
} ade7953_t;

/**
 * @brief ADE7953 driver initialization routine
 *
 * @param[in]   dev Pointer to ST95 device descriptor
 * @param[in]   params Pointer to static ST95 device configuration
 *
 * @return 0 if initialization succeeded
 * @return >0 in case of an error
 */
int ade7953_init(ade7953_t *dev, ade7953_params_t * params);

#endif /* ADE7953_H_ */
