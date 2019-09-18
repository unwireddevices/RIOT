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
#ifndef ADE7953_H
#define ADE7953_H

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "periph/gpio.h"
#include "periph/spi.h"


#define ADE7953_MAX_BYTE_BUFF       8

#define ADE7953_CMD_READ 	        0x80
#define ADE7953_CMD_WRITE           0x00

#define ADE7953_VRMS_OFFSET         0xFFF913  // -1773

#define ADE7953_SQRT2	            (1414213)	// sqrt(2) * 1000000
#define ADE7953_1000000	            (1000000)	// 1000000
// /* Vrms = Sample / coeff_vrms [V] */
/* coeff_vrms =  (9032007 * sqrt(2)) / ((500 / 1000) * 2093)*/
#define ADE7953_FULL_SCALE_VRMS	    9032007UL	        // Full-scale VRMS
#define ADE7953_RESISTOR_DIVIDER    (2093)              // (4 * 523Ohm) + 1KOhm
#define FULL_SCALE_VOLT	            (500 / 1000)	    // Full-scale +/-500 mV (500/1000 V) (gain == 1 !!)
//#define ADE7953_COEFF_VRMS	        ((ADE7953_FULL_SCALE_VRMS * ADE7953_SQRT2 * 10) / (ADE7953_RESISTOR_DIVIDER * 5 * ADE7953_1000000))
#define ADE7953_COEFF_VRMS	        (12205625) // 12205.625 * 1000

// /* Irms = Sample / coeff_irms [mA]*/
/* coeff_irms =  (9032007 * sqrt(2) * (2 * Rb)) / (500 * 2500)*/
#define ADE7953_FULL_SCALE_IRMS	    9032007UL	        // Full-scale VRMS
#define ADE7953_TRANS_RATIO         (2500)              // Transformation ratio = 2500
#define ADE7953_RB                  ((2 * 5900))              // Rb = 2 * 5900 mOhm
#define FULL_SCALE_CURR	            (500)	    // Full-scale +/-500 mV (gain == 1 !!)
#define FULL_SCALE_CURR_A	        (500 / 1000)// Full-scale +/- 0.5 V (gain == 1 !!)
//#define ADE7953_COEFF_IRMS	        ((ADE7953_FULL_SCALE_IRMS * ADE7953_SQRT2 * ADE7953_RB) / (1000 * ADE7953_TRANS_RATIO * FULL_SCALE_CURR * ADE7953_1000000))
#define ADE7953_COEFF_IRMS	(120578) // 120.578 * 1000

#define ADE7953_FULL_SCALE_ENERGY	4862401UL	    // Full-scale ENERGY
/* coeff_energy =  (4862401 * sqrt(2)* sqrt(2) * (2 * Rb)) / (0.5 * 0.5 * 2500 * 2093)*/
//#define ADE7953_COEFF_ENERGY ((ADE7953_FULL_SCALE_ENERGY * 2 * ADE7953_RB * FULL_SCALE_CURR_A) / (ADE7953_RESISTOR_DIVIDER * ADE7953_TRANS_RATIO * FULL_SCALE_VOLT))
#define ADE7953_COEFF_ENERGY (87723) // 87.723 * 1000


#define ADE7953_GPIO_PIN_RESET  0
#define ADE7953_GPIO_PIN_SET    1

/**
 * @brief ADE7953 return codes
*/
#define ADE7953_OK			        0
#define ADE7953_ERROR		        1

/**
 * @brief ADE7953 states
*/
#define ADE7953_STATE_READY 0
#define ADE7953_STATE_STOP 1

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

uint32_t ade7953_get_irms(const ade7953_t * dev);
uint32_t ade7953_get_vrms(const ade7953_t * dev);
uint32_t ade7953_get_volt(const ade7953_t * dev);
uint32_t ade7953_get_curr(const ade7953_t * dev);
int32_t ade7953_get_awatt(const ade7953_t * dev);

#endif /* ADE7953_H */
