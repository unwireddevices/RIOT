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
 * @file        umdk-st95.h
 * @brief       umdk-st95 driver module definitions
 * @author      Mikhail Perkov

 */
#ifndef UMDK_ST95_H
#define UMDK_ST95_H

#include "unwds-common.h"

#define UMDK_ST95_STACK_SIZE            2048

#define UMDK_ST95_SPI_DEV               0
#define UMDK_ST95_SPI_CS                UNWD_GPIO_4
#define UMDK_ST95_UART_DEV              1
#define UMDK_ST95_IRQ_IN                UNWD_GPIO_25
#define UMDK_ST95_IRQ_OUT               UNWD_GPIO_26
#define UMDK_ST95_SSI_0                 UNWD_GPIO_24
#define UMDK_ST95_SSI_1                 GPIO_UNDEF

#define UMDK_ST95_UART_BAUD_DEF         57600
#define UMDK_ST95_UART_TIME_RX_USEC     500
#define UMDK_ST95_SPI_CLK               SPI_CLK_1MHZ

#define ST95_RAMP_UP_TIME_MS            9
#define ST95_HFO_SETUP_TIME_MS          9
#define ST95_ECHO_WAIT_TIME_MS          10

#define ST95_MAX_DATA_BYTES             254
#define UMDK_ST95_DETECT_MS             1000
#define UMDK_ST95_NO_RESPONSE_TIME_MS   2500

#define ST95_TX_RATE_106                0
#define ST95_RX_RATE_106                0
#define ST95_TX_RATE_212                1
#define ST95_RX_RATE_212                1
#define ST95_TX_RATE_424                2
#define ST95_RX_RATE_424                2
#define ST95_TX_RATE_14443A             ST95_TX_RATE_106
#define ST95_RX_RATE_14443A             ST95_RX_RATE_106


/**
 * @brief Thread messages values
 */
typedef enum {
    UMDK_ST95_MSG_RADIO     = 0,
    UMDK_ST95_MSG_ECHO      = 1,
    UMDK_ST95_MSG_CALIBR    = 2,
    UMDK_ST95_MSG_IDN       = 3,
    UMDK_ST95_MSG_UID       = 4,
    UMDK_ST95_MSG_ANTICOL   = 5,
    UMDK_ST95_MSG_IDLE      = 6,
    UMDK_ST95_MSG_PROTOCOL  = 7,
} st95_msg_t;

/**
 * @brief Pack RX state values
 */
typedef enum {
    UMDK_ST95_NOT_RECIEVED  = 0,
    UMDK_ST95_RECIEVED      = 1,
} st95_rx_state_t;

/**
 * @brief ST95 Pack state values
 */
typedef enum {
    UMDK_ST95_PACK_OK       = 1,
    UMDK_ST95_PACK_ERROR    = 0,
} st95_pack_state_t;


/**
 * @brief   ST95 commands list
 */
typedef enum {
    ST95_CMD_IDN            = 0x01,
    ST95_CMD_PROTOCOL       = 0x02,
    ST95_CMD_SEND_RECV      = 0x04,
    ST95_CMD_IDLE           = 0x07,
    ST95_CMD_READ_REG       = 0x08,
    ST95_CMD_WRITE_REG      = 0x09,
    ST95_CMD_BAUDRATE       = 0x0A,
    ST95_CMD_ECHO           = 0x55,
} st95_cmd_t;

/**
 * @brief   Protocol select commands
 */
typedef enum {
    FIELD_OFF           = 0x00,
    ISO_15693           = 0x01,
    ISO_14443A          = 0x02,
    ISO_14443B          = 0x03,
    ISO_18092           = 0x04,
} st95_protocol_t;


/**
 * @brief   Mask Protocols select
 */
typedef enum {
    NO_SELECT_PROTOCOL  = 0x00,
    ISO14443A_SELECT    = 0x01,
    ISO14443B_SELECT    = 0x02,
    ISO15693_SELECT     = 0x04,
    ISO18092_SELECT     = 0x08,
    SELECT_ALL_PROTOCOL = 0x0F,
} umdk_st95_select_protocol_t;

/**
 * @brief   ST95 hardware and global parameters.
 */
typedef struct {
    uint8_t spi;        /**< SPI device */
    gpio_t cs_spi;      /**< SPI NSS pin */
    uint8_t uart;       /**< UART device */
    gpio_t irq_in;      /**< Interrupt input (UART RX pin) */
    gpio_t irq_out;     /**< Interrupt output (UART TX pin) */
    gpio_t ssi_0;       /**< Select serial communication interface */
    gpio_t ssi_1;       /**< Select serial communication interface */
} st95_params_t;

/**
 * @brief ST95 interfaces
 */
typedef enum {
    ST95_IFACE_UART    = 1,
    ST95_IFACE_SPI     = 2,
} st95_iface_t;

/**
 * @brief ST95 modes
 */
typedef enum {
    ST95_MODE_WRITER    = 1,
    ST95_MODE_READER    = 2,
} st95_mode_t;

/**
 * @brief ST95 configurations structure
 */
typedef struct {
    st95_iface_t iface;
    st95_mode_t mode;
    uint8_t protocol;
 } umdk_st95_config_t;

void umdk_st95_init(uwnds_cb_t *event_callback);
bool umdk_st95_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_ST95_H */
