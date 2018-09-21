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
 * @brief Type token release
 */
typedef enum {
  I2C_TOKEN_RELEASE_HW = 0,
  I2C_TOKEN_RELEASE_SW,
  I2C_TOKEN_RELEASE_NUM
} m24sr_token_mode_t;

/**
 * @brief Type open session
 */
typedef enum {
  I2C_OPEN_SESSION = 0,
  I2C_KILL_RF,
  I2C_PRIORITY_NUM
} m24sr_priority_t;




/**
  * @brief  Synchronization Mechanism structure 
  */
typedef enum{
    M24SR_WAITING_TIME_UNKNOWN = 0,
    M24SR_WAITING_TIME_POLLING,
    M24SR_WAITING_TIME_GPO,
    M24SR_INTERRUPT_GPO,
} m24sr_wait_mode_t;    



/**
 * @brief   M24SR configuration parameters
 */
typedef struct {
    i2c_t   i2c;                /**< I2C device   */
    uint8_t i2c_addr;           /**< I2C address */
    gpio_t  gpo_pin;            /**< Interrupt GPO   */
    gpio_t  rfdisable_pin;      /**< GPIO to switch RF on/off */
    gpio_t  pwr_en_pin;         /**< GPIO to switch power on/off*/
    m24sr_priority_t priority;             /**<*/
    m24sr_token_mode_t token_mode;      /**<*/
} m24sr_params_t;



typedef struct {
    uint16_t chipsize;
    uint16_t type;
    uint16_t max_read_byte;
    uint16_t max_write_byte;
    uint8_t uid[7];
} m24sr_memory_t;


/**
 * @brief   M24SR alert callback
 */
typedef void (*m24sr_cb_t)(void *);


/**
 * @brief   M24SR device descriptor
 */
typedef struct {
    m24sr_params_t params;              /**< device configuration */
    m24sr_memory_t memory;              /**< device memory parameters */
    // m24sr_cb_t cb;                      /**< alert callback */
    // void *arg;                          /**< alert callback param */
    m24sr_wait_mode_t synchro_mode;        // @TODO static m24sr_waiting_time_mode_t synchro_mode = M24SR_WAITING_TIME_POLLING;
    uint8_t event_ready;                 /**< check if an event was received */
    // mutex_t event_lock;                     /**< mutex for waiting for event */
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
    M24SR_WRONG_CRC = -5,
};

int m24sr_eeprom_init(m24sr_t *dev, const m24sr_params_t *params);
int m24sr_eeprom_read(m24sr_t *dev, void *dest, uint16_t addr, uint16_t size);
int m24sr_eeprom_write(m24sr_t *dev, void *src, uint16_t addr, uint16_t size);
//int m24sr_eeprom_erase(m24sr_t *dev, uint16_t addr, uint16_t size);
int m24sr_eeprom_erase_all(m24sr_t *dev);
int m24sr_eeprom_power(m24sr_t *dev, uint8_t power);

#ifdef __cplusplus
}
#endif

#endif /* _M24SR_H */
