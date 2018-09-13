/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_m24srxx
 * @{
 *
 * @file
 * @brief       Definition for the M24SRxx NFC memory
 * 
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */
#ifndef _M24SR_H
#define _M24SR_H


#include "periph/i2c.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   M24SR configuration parameters
 */
typedef struct {
    i2c_t   i2c;                /**< I2C device   */
    uint8_t i2c_addr;           /**< I2C address */
    gpio_t  gpo_pin;            /**< Interrupt GPO   */
    gpio_t  rfdisable_pin;      /**< GPIO to switch RF on/off */
    gpio_t  pwr_en_pin;         /**< GPIO to switch power on/off*/
} m24sr_params_t;



typedef struct {
    uint16_t chipsize;
    uint16_t type;
} m24sr_memory_t;

/**
 * @brief   M24SR device descriptor
 */
typedef struct {
    m24sr_params_t params;      /**< device configuration */
    m24sr_memory_t memory;      /**< device memory parameters */
} m24sr_t;




/**
 * @brief   Status and error return codes
 */
enum {
    M24SR_OK      =  0,             /**< everything was fine */
    M24SR_NOBUS   = -1,             /**< bus interface error */
    M24SR_NODEV   = -2,             /**< unable to talk to device */
    M24SR_ERROR   = -3,             /**< any error memory */
    M24SR_ERROR_PARAM = -4,         /**< error parameter */
};

int m24sr_eeprom_init(m24sr_t *dev, const m24sr_params_t *params);
int m24sr_eeprom_read(m24sr_t *dev, void *dest, uint32_t addr, uint32_t size);
int m24sr_eeprom_write(m24sr_t *dev, void *src, uint32_t addr, uint32_t size);
int m24sr_eeprom_erase(m24sr_t *dev, uint32_t addr, uint32_t size);
int m24sr_eeprom_power(m24sr_t *dev, uint8_t power);

#ifdef __cplusplus
}
#endif

#endif /* _M24SR_H */
