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
 
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _M24SR_CORE_H
#define _M24SR_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "m24sr.h"

/** @brief Get Most Significant Byte
  * @param  val: number where MSB must be extracted
  * @return MSB
  */ 
#define GET_MSB(val)                                         ( (uint8_t) ((val & 0xFF00 )>>8)) 

/** @brief Get Least Significant Byte
  * @param  val: number where LSB must be extracted
  * @return LSB
  */ 
#define GET_LSB(val)                                         ( (uint8_t) (val & 0x00FF )) 

/** @brief Used to toggle the block number by adding 0 or 1 to default block number value
  * @param  val: number to know if incrementation is needed
  * @return  0 or 1 if incrementation needed
  */
#define TOGGLE(val)                                         ((val != 0x00)? 0x00 : 0x01)

/**
  * @brief  APDU-Header command structure
  */
typedef struct {
    uint8_t CLA;                            /**< Command class */
    uint8_t INS;                            /**< Operation code */
    uint8_t P1;                             /**< Selection Mode */
    uint8_t P2;                             /**< Selection Option */
} __attribute__((packed)) cmd_apdu_header_t;


/**
  * @brief  APDU-Body command structure
  */
typedef struct {
    uint8_t LC;                             /**< Data field length */ 
    uint8_t *data;                          /**< Command parameters */ 
    uint8_t LE;                             /**< Expected length of data to be returned */
} __attribute__((packed)) cmd_apdu_body_t;

/**
  * @brief  APDU Command structure 
  */
typedef struct {
    cmd_apdu_header_t header;               /**< APDU-Header */
    cmd_apdu_body_t   body;                 /**< APDU-Body */
} __attribute__((packed)) cmd_apdu_t;

/**
  * @brief  SC response structure
  */
typedef struct {
    uint8_t *data ;                         /**< Data returned from the card */
    uint8_t SW1;                            /**< Command Processing status */
    uint8_t SW2;                            /**< Command Processing qualification */
} __attribute__((packed)) resp_apdu_t;



/**
  * @brief Capability Container File structure
  */
typedef struct {
        uint16_t cc_file_len;               /**< Number of bytes of CC file */
        uint8_t  version;                   /**< Mapping version */
        uint16_t max_read_byte;             /**< Maximum number of bytes that can be read */
        uint16_t max_write_byte;            /**< Maximum number of bytes that can be written*/
        uint8_t  t_field;                   /**< NDEF file control TLV: T field */
        uint8_t  l_field;                   /**< NDEF file control TLV: L field */
        uint16_t ndef_file_id;              /**< NDEF file control TLV: FileID */
        uint16_t ndef_file_max_size;        /**< NDEF file control TLV: Maximum NDEF file size */
        uint8_t  read_access;               /**< NDEF file control TLV: Read access */
        uint8_t  write_access;              /**< NDEF file control TLV: Write access */
}  __attribute__((packed)) cc_file_info_t;



/**
  * @brief System file structure
  */
typedef struct {
        uint16_t sys_file_len;              /**< Length system file */
        uint8_t  i2c_protect;               /**< I2C protect */
        uint8_t  i2c_watchdog;              /**< I2C watchdog*/
        uint8_t  gpo;                       /**< GPO */
        uint8_t  reserved;                  /**< ST reserved */
        uint8_t  rf_enable;                 /**< RF enable */
        uint8_t  ndef_file_num;             /**< NDEF File number (RFU)*/
        uint8_t  UID[7];                    /**< UID: 0x02 0x82 0xZZ 0xZZ 0xZZ 0xZZ 0xZZ*/
        uint16_t memory_size;               /**< Memory Size */
        uint8_t  prod_code;                 /**< Product Code */
}  __attribute__((packed)) sys_file_info_t;


/**
  * @brief  GPO mode enumeration
  */
typedef enum {
    RF_GPO = 0,                             /**< GPO mode RF*/
    I2C_GPO,                                /**< GPO mode I2C*/
    GPO_HW_MODE_NUM                         /**< Number of GPO mode types */
} m24sr_gpo_hw_mode_t;

/**
  * @brief  GPO configuration enumeration
  */
typedef enum {
    HIGH_IMPEDANCE = 0,                     /**< High impedance */
    SESSION_OPENED,                         /**< Session open*/
    WIP,                                    /**< Writing in progress*/
    I2C_ANSWER_READY,                       /**< I2C ready response*/
    INTERRUPT,                              /**< Interrupt */
    STATE_CONTROL,                          /**< State Control */
    RF_BUSY,                                /**< RF busy */
    GPO_CONFIG_NUM                          /**< Number of GPO configuration types */
} m24sr_gpo_config_t;



/**
 * @brief  This function generates an I2C Token release
 * 
 * @param[in] dev Pointer to M24SR NFC eeprom device descriptor
 * @return Error code 
 */
int m24sr_release_i2c_token(const m24sr_t *dev);

/**
 * @brief This functions sends the command buffer
 * 
 * @param[in] dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[in] buffer Pointer to the buffer to send to the M24SR
 * @param[in] len    Number of byte to send
 * @return Error code 
 */ 
int m24sr_send_i2c_cmd(const m24sr_t *dev, uint8_t *buffer, uint8_t len);

/**
 * @brief This functions returns M24SR_OK when a response is ready
 * 
 * @param[in] dev Pointer to M24SR NFC eeprom device descriptor
 * @return Error code
 */
int m24sr_is_answer_rdy(m24sr_t *dev);

/**
 * @brief This functions polls the I2C interface
 * 
 * @param[in] dev Pointer to M24SR NFC eeprom device descriptor
 * @return Error code
 */
int m24sr_poll_i2c (const m24sr_t *dev);

/**
 * @brief This functions reads a response of the M24SR device
 * 
 * @param[in]  dev    Pointer to M24SR NFC eeprom device descriptor
 * @param[out] buffer Pointer on the buffer to retrieve M24SR response
 * @param[in]  len    Number of byte to read (shall be >= 5)
 * @return Error code 
 */
int m24sr_rcv_i2c_response(const m24sr_t *dev, uint8_t *buffer, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* _M24SR_CORE_H */    