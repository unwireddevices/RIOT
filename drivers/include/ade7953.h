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


#define ADE7953_MAX_BYTE_BUFF      16

/* Register Addresses */
/* 8-bits Registers */
#define ADE7953_UNLOCK_REG_8    0x0FE   /* Addres Unlock register for unlock Register 0x120 address */
#define ADE7953_PGA_V_8         0x007   /* Voltage channel gain configuration (Bits[2:0]) */
#define ADE7953_PGA_IA_8        0x008   /* Current Channel A gain configuration (Bits[2:0]) */
#define ADE7953_VERSION_8       0x702   /*  Contains the silicon version number */
/* 16-bits Registers */
#define ADE7953_SETUP_REG_16    0x120   /* This register should be set to 0x30 */
/* 32-bits Registers */
#define ADE7953_VPEAK_32       0x326   /* Voltage channel peak */
#define ADE7953_RSTVPEAK_32    0x327   /* Read voltage peak with reset */
#define ADE7953_IAPEAK_32      0x328   /* Current Channel A peak */
#define ADE7953_RSTIAPEAK_32   0x329   /* Read Current Channel A peak with reset */
#define ADE7953_IRQENA_32      0x32C   /* Interrupt enable (Current Channel A) */
#define ADE7953_IRMSA_32       0x31A   /* IRMS register (Current Channel A) */
#define ADE7953_VRMS_32        0x31C   /* VRMS register */
#define ADE7953_AENERGYA_32    0x31E   /* Active energy (Current Channel A) */
#define ADE7953_IRQSTATA_32    0x32D   /* Interrupt status (Current Channel A) */
#define ADE7953_RSTIRQSTATA_32 0x32E   /* Reset interrupt status (Current Channel A) */
#define ADE7953_IA_32          0x316   /* Instantaneous current (Current Channel A) */
#define ADE7953_V_32           0x318   /* Instantaneous voltage (voltage channel) */

/* Register Interrupt enable (IRQENA) bits */
#define ADE7953_IRQENA_AEHFA   0x000001 /* interrupt when the active energy is half full (Current Channel A) */
#define ADE7953_IRQENA_RESET   0x100000 /* This interrupt is always enabled and cannot be disabled */

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

uint32_t ade7953_get_version(const ade7953_t * dev);

uint32_t ade7953_get_aenergy(const ade7953_t * dev);
uint32_t ade7953_get_irms(const ade7953_t * dev);
uint32_t ade7953_get_vrms(const ade7953_t * dev);

uint32_t ade7953_get_volt(const ade7953_t * dev);
uint32_t ade7953_get_curr(const ade7953_t * dev);

#endif /* ADE7953_H_ */
