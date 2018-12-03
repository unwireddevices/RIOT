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
 * @brief Type token release enumeration
 */
typedef enum {
  I2C_TOKEN_RELEASE_HW = 0,             /**< Hardware release token */
  I2C_TOKEN_RELEASE_SW,                 /**< Software release token */
  I2C_TOKEN_RELEASE_NUM                 /**< Number of token release types */
} m24sr_token_mode_t;

/**
 * @brief Type open session enumeration
 */
typedef enum {
  I2C_OPEN_SESSION = 0,                 /**< Get I2C session*/
  I2C_KILL_RF,                          /**< Kill RF session */
  I2C_PRIORITY_NUM                      /**< Number of session open types */
} m24sr_priority_t;                    


/**
  * @brief  Synchronization Mechanism enumeration 
  */
typedef enum{
    M24SR_WAITING_TIME_UNKNOWN = 0,     /**< Unknown synchronization mechanism */
    M24SR_WAITING_TIME_POLLING,         /**< I2C polling synchronization mechanism */
    M24SR_WAITING_TIME_GPO,             /**< GPO pin polling synchronization mechanism */
    M24SR_INTERRUPT_GPO,                /**< GPO pin interrupt synchronization mechanism */
    M24SR_WAITING_TIME_MODE_NUM         /**< Number of Synchronization Mechanism types */
} m24sr_wait_mode_t;    


/**
 * @brief   M24SR configuration parameters
 */
typedef struct {
    i2c_t   i2c;                        /**< I2C device  */
    uint8_t i2c_addr;                   /**< I2C address */
    gpio_t  gpo_pin;                    /**< Interrupt GPO */
    gpio_t  rfdisable_pin;              /**< GPIO to switch RF on/off */
    gpio_t  pwr_en_pin;                 /**< GPIO to switch power on/off */
    m24sr_priority_t priority;          /**< Type open session */
    m24sr_token_mode_t token_mode;      /**< Type token release */
} m24sr_params_t;


/**
 * @brief M24SR memory information
 */
typedef struct {
    uint16_t chipsize;                  /**< Size of memory */
    uint16_t type;                      /**< Type of memory */
    uint16_t max_read_byte;             /**< The maximum number of read bytes */
    uint16_t max_write_byte;            /**< The maximum number of write bytes */
    uint8_t uid[7];                     /**< Unique identification number */
} m24sr_memory_t;


/**
 * @brief  M24SR power states enumeration
 */
enum m24sr_power_state {
    M24SR_POWER_UP = 0,                 /**< Power up */
    M24SR_POWER_DOWN,                   /**< Power down */
    M24SR_POWER_STATE_NUM 
};

/**
 * @brief   M24SR device descriptor
 */
typedef struct {
    m24sr_params_t params;              /**< Вevice configuration */
    m24sr_memory_t memory;              /**< Вevice memory parameters */
    m24sr_wait_mode_t synchro_mode;     /**< Type synchronization mechanism */
    volatile uint8_t event_ready;       /**< Ready to send response flag */
} m24sr_t;




/**
 * @brief   Status and error return codes enumeration
 */
enum {
    M24SR_OK          =  0,             /**< Everything was fine */
    M24SR_NOBUS       = -1,             /**< Bus interface error */
    M24SR_NODEV       = -2,             /**< Unable to talk to device */
    M24SR_ERROR       = -3,             /**< Any error memory */
    M24SR_ERROR_PARAM = -4,             /**< Error parameter */
    M24SR_WRONG_CRC   = -5,             /**< Wrong CRC */
};


/**
 * @brief   Initialize the given M24SR NFC eeprom
 * 
 * @param[out] dev      Pointer to M24SR NFC eeprom device descriptor
 * @param[in]  params   Pointer to static device configuration
 * 
 * @return  Error code
 */
int m24sr_eeprom_init(m24sr_t *dev, const m24sr_params_t *params);

/**
 * @brief Copy data from M24SR NFC eeprom to system memory.
 * 
 * @param[in]  dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[out] dest   Pointer to the first byte in the system memory address space
 * @param[in]  addr   Starting address in the M24SR NFC eeprom device address space
 * @param[in]  size   Number of bytes to copy
 * 
 * @return Number of bytes read or error code
 */
int m24sr_eeprom_read(m24sr_t *dev, void *dest, uint16_t addr, uint16_t size);

/**
 * @brief Copy data from system memory to M24SR NFC eeprom.
 * 
 * @param[in]  dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[in]  src    Pointer to the first byte in the system memory address space
 * @param[in]  addr   Starting address in the M24SR NFC eeprom device address space
 * @param[in]  size   Number of bytes to copy
 *
 * @return Number of bytes written or error code
 */
int m24sr_eeprom_write(m24sr_t *dev, void *src, uint16_t addr, uint16_t size);

/**
 * @brief Erase M24SR NFC eeprom
 * 
 * @param[in]  dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[in]  addr   Starting address in the M24SR NFC eeprom device address space
 * @param[in]  size   Number of bytes to erase
 * 
 * @return Error code
 */
int m24sr_eeprom_erase(m24sr_t *dev, uint16_t addr, uint16_t size);

/**
 * @brief Fully erase data from M24SR NFC eeprom
 * 
 * @param[in]  dev    Pointer to M24SR NFC eeprom device descriptor
 * 
 * @return Error code
 */
int m24sr_eeprom_erase_all(m24sr_t *dev);

/**
 * @brief Control power of  M24SR NFC eeprom
 * 
 * @param[in] dev     Pointer to M24SR NFC eeprom device descriptor
 * @param[in] power   Power state to apply
 * 
 * @return Error code
 */
int m24sr_eeprom_power(m24sr_t *dev, enum m24sr_power_state power);

#ifdef __cplusplus
}
#endif

#endif /* _M24SR_H */
