/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @file		umdk-electro.h
 * @brief       umdk-electro driver module definitions
 * @author      Mikhail Perkov
 */
#ifndef UMDK_ELECTRO_H
#define UMDK_ELECTRO_H

#include "unwds-common.h"

#define UMDK_ELECTRO_DEV 1
#define UMDK_ELECTRO_BAUDRATE_NO 3 /* 9600 */

#define RS_485_DE_PIN UNWD_GPIO_4
#define RS_485_RE_PIN UNWD_GPIO_5

#define UMDK_ELECTRO_NUM_BAUDRATES 10

#define UMDK_ELECTRO_NUM_CMD_MERCURY 23
#define MAX_NUM_TARIFFS 4

#define UMDK_ELECTRO_BUF_SIZE 42
#define UMDK_ELECTRO_MAX_LENGTH_DATA 35

#define UMDK_MERCURY_ADDR_DEF 0x00000000

#define UMDK_ELECTRO_DELAY_BYTE 6
#define UMDK_ELECTRO_DELAY_BIT (UMDK_ELECTRO_DELAY_BYTE * 8)
#define USEC_IN_SEC 1000

#define FLAG_ALLOW_REPLY 0
#define FLAG_NOT_ALLOW_REPLY 1

#define PACK_DELAY_USEC 5000
#define MAX_BYTES_IM_RADIO 30

#define MERCURY_CRC16_INIT 0xFFFF

typedef struct {
	uint8_t is_valid;
	uint8_t uart_dev;
	uint8_t current_baudrate_idx;
	uint32_t addr;
} umdk_electro_config_t;

typedef struct {
    uint32_t addr;
    uint8_t cmd;
    uint8_t data[UMDK_ELECTRO_MAX_LENGTH_DATA];

    uint16_t crc;
} umdk_electro_pack_t;


typedef struct {
  uint8_t cmd;

  uint8_t length_tx_data;
  uint8_t length_rx_data;
} umdk_electro_cmd_mercury_t;

typedef enum {
    MERCURY_CMD_INIT_ADDR = 0xFF,		/* Set address at initialization */

    MERCURY_CMD_GET_ADDR = 0x00,		/* Read the address */
    MERCURY_CMD_GET_SERIAL = 0x01,		/* Read the serial number */
    MERCURY_CMD_SET_NEW_ADDR = 0x2,		/* Set new address */
    MERCURY_CMD_GET_CURR_TARIFF = 0x3,		/* Read the current tariff */
    MERCURY_CMD_GET_LAST_OPEN = 0x4,		/* Read the time of last opening */
    MERCURY_CMD_GET_LAST_CLOSE = 0x5,		/* Read the time of last closing */
    MERCURY_CMD_GET_U_I_P = 0x6,		/* Read the value of the voltage, current and power */
    MERCURY_CMD_GET_TIMEDATE = 0x7,		/* Read the internal time and date */
    MERCURY_CMD_GET_LIMIT_POWER = 0x8,		/* Read the limit of power */
    MERCURY_CMD_GET_CURR_POWER_LOAD = 0x09,	/* Read the current power load */
    MERCURY_CMD_GET_TOTAL_VALUE = 0x0A,		/* Read the total values of power after reset */
    MERCURY_CMD_GET_LAST_POWER_OFF = 0xB,	/* Read the time of last power off */
    MERCURY_CMD_GET_LAST_POWER_ON = 0xC,	/* Read the time of last power on */
    MERCURY_CMD_GET_HOLIDAYS = 0xD,		/* Read the table of holidays */
    MERCURY_CMD_GET_SCHEDULE = 0xE,		/* Read the schedule of tariffs */
    MERCURY_CMD_GET_VALUE = 0x0F,		/* Read the month's value */
    MERCURY_CMD_GET_NUM_TARIFFS = 0x10,		/* Read the number of tariffs */
    MERCURY_CMD_SET_SPEED = 0x11,		/* Set baudrate */
    MERCURY_CMD_SET_NUM_TARIFFS = 0x12,		/* Set number of tariffs */
    MERCURY_CMD_SET_TARIFF = 0x13,		/* Set the tariff */
    MERCURY_CMD_SET_HOLIDAYS = 0x14,		/* Set the table of holidays */
    MERCURY_CMD_SET_SCHEDULE = 0x15,		/* Set the schedule of tariffs */
    MERCURY_CMD_GET_WORKING_TIME = 0x16,	/* Read the total working time of battery and device */
} mercury_cmd_t;


typedef enum {
  MERCURY_DECODE_DEFAULT = 0x0,
  MERCURY_DECODE_VALUE = 0x1,
  MERCURY_DECODE_TIMEDATE = 0x2,
  MERCURY_DECODE_SCHEDULE = 0x3,
  MERCURY_DECODE_UIP = 0x4,
  MERCURY_DECODE_WORKING_TIME = 0x5,
  MERCURY_DECODE_LIMIT_POWER = 0x6,
  MERCURY_DECODE_CURR_POWER = 0x7,
}flag_decode_t;


void umdk_electro_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_electro_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_ELECTRO_H */
