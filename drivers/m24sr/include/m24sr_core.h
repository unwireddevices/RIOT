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
  * @retval MSB
  */ 
#define GET_MSB(val)                                         ( (uint8_t) ((val & 0xFF00 )>>8)) 

/** @brief Get Least Significant Byte
  * @param  val: number where LSB must be extracted
  * @retval LSB
  */ 
#define GET_LSB(val)                                         ( (uint8_t) (val & 0x00FF )) 

/** @brief Used to toggle the block number by adding 0 or 1 to default block number value
  * @param  val: number to know if incrementation is needed
  * @retval  0 or 1 if incrementation needed
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
    cmd_apdu_header_t header;
    cmd_apdu_body_t   body;
} __attribute__((packed)) cmd_apdu_t;

/**
  * @brief  SC response structure
  */
typedef struct {
    uint8_t *data ;                        /**< Data returned from the card */
    uint8_t SW1;                            /**< Command Processing status */
    uint8_t SW2;                            /**< Command Processing qualification */
} __attribute__((packed)) resp_apdu_t;



/**
  * @brief CC File structure
  */
typedef struct {
    uint16_t cc_file_len;
    uint8_t  version;
    uint16_t max_read_byte;
    uint16_t max_write_byte;
    uint8_t  t_field;
    uint8_t  l_field;
    uint16_t ndef_file_id;
    uint16_t ndef_file_max_size;
    uint8_t  read_access;
    uint8_t  write_access;
} __attribute__((packed)) cc_file_info_t;



/**
  * @brief System file structure
  */
typedef struct {
  uint16_t sys_file_len;
  uint8_t  i2c_protect;
  uint8_t  i2c_watchdog;
  uint8_t  gpo;
  uint8_t  reserved;
  uint8_t  rf_enable;
  uint8_t  ndef_file_num;
  uint8_t  UID[7];
  uint16_t memory_size;
  uint8_t  prod_code;  
}__attribute__((packed)) sys_file_info_t;


/**
  * @brief  GPO mode structure 
  */
typedef enum {
    RF_GPO = 0,
    I2C_GPO
} m24sr_gpo_hw_mode_t;

/**
  * @brief  GPO state structure 
  */
typedef enum {
    HIGH_IMPEDANCE = 0,
    SESSION_OPENED,
    WIP,
    I2C_ANSWER_READY,
    INTERRUPT,
    STATE_CONTROL
} m24sr_gpo_mode_t;


int m24sr_release_i2c_token(const m24sr_t *dev);
int m24sr_send_i2c_cmd(const m24sr_t *dev, uint8_t *buffer, uint8_t len);
int m24sr_is_answer_rdy(const m24sr_t *dev);
int m24sr_poll_i2c (const m24sr_t *dev);
int m24sr_rcv_i2c_response(const m24sr_t *dev, uint8_t *buffer, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* _M24SR_CORE_H */    