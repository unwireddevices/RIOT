/*
 * Copyright (C) 2018 Unwired Devices [info@unwds.com]
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
 * @file		umdk-modbus.h
 * @brief       umdk-modbus module definitions
 * @author      Mikhail Perkov
 */
#ifndef UMDK_MODBUS_H
#define UMDK_MODBUS_H

#include "unwds-common.h"

#define UMDK_MODBUS_DEV 1
#define UMDK_MODBUS_BAUDRATE_MAX 115200
#define UMDK_MODBUS_BAUDRATE_MIN 1200
#define UMDK_MODBUS_BAUDRATE_DEF 19200 /* 19200 */

#define MODBUS_CRC16_INIT 0xFFFF
#define MODBUS_CRC16_POLY 0xA001

#define UMDK_MODBUS_DATA_SIZE 64
#define UMDK_MODBUS_BUFF_SIZE (UMDK_MODBUS_DATA_SIZE + 4)

#define UMDK_MODBUS_STACK_SIZE (2*UMDK_MODBUS_BUFF_SIZE + 2048)

#define UMDK_MODBUS_DE_PIN UNWD_GPIO_29
#define UMDK_MODBUS_RE_PIN UNWD_GPIO_30

#define MODBUS_MAX_ID 247

#define MODBUS_FRAME_BIT (11)
#define MODBUS_WAIT_FRAME (4)
#define UMDK_MODBUS_USEC_IN_SEC 1000000
#define UMDK_MODBUS_MS_IN_SEC 1000
#define UMDK_MODBUS_TIMEWAIT_DEF_USEC 1750

#define UMDK_MODBUS_RECIEVE_TIME_MIN_MS 30
#define UMDK_MODBUS_TIME_NO_RESPONSE_MS 1000

#define UMDK_MODBUS_RX_ALLOW     1
#define UMDK_MODBUS_RX_NOT_ALLOW 0

#define UMDK_MODBUS_RECIEVED     1
#define UMDK_MODBUS_NOT_RECIEVED 0

#define UMDK_MODBUS_RESPONSE    1
#define UMDK_MODBUS_NO_RESPONSE 0

#define UMDK_MODBUS_RADIO    1
#define UMDK_MODBUS_NO_RADIO 0

/**
 * @brief Thread messages values
 */
typedef enum {
    UMDK_MODBUS_MSG_RADIO           = 0,
	UMDK_MODBUS_MSG_OVERFLOW        = 1,
    UMDK_MODBUS_MSG_NO_RESPONSE     = 2,
} modbus_msg_t;

/**
 * @brief Reply messages values
 */
typedef enum {
	UMDK_MODBUS_OK_REPLY 			= 0x00,
	UMDK_MODBUS_ERROR_REPLY			= 0x01,
	UMDK_MODBUS_NO_RESPONSE_REPLY   = 0x02,
	UMDK_MODBUS_OVERFLOW_REPLY   	= 0x03,
	UMDK_MODBUS_INVALID_FORMAT   	= 0x04,
    UMDK_MODBUS_INVALID_CMD_REPLY   = 0xFF,
} umdk_modbus_reply_t;

/**
 * @brief Modbus configurations structure
 */
typedef struct {
	uint8_t uart_dev;
	uint32_t baudrate;
    uint8_t databits;
    uint8_t parity;
    uint8_t stopbits;
} umdk_modbus_config_t;

/**
 * @brief Modbus package configurations structure
 */
typedef struct {
	volatile uint8_t flag_rx;
	volatile uint8_t rx_allow;
	volatile uint8_t radio;
	uint8_t response;
    
    bool is_true;
	
	uint8_t length_rx;
	uint8_t length_tx;
} umdk_modbus_config_pack_t;

/**
 * @brief Commands list
 */
typedef enum {
	UMDK_MODBUS_SET_PARAMS 	= 0xFF,
	UMDK_MODBUS_SET_DEVICE	= 0xFE,
	MODBUS_MAX_CMD			= 0x7F,

} umdk_modbus_cmd_t;

void umdk_modbus_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_modbus_cmd(module_data_t *cmd, module_data_t *reply);

#endif /* UMDK_MODBUS_H */
