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
 * @file		st95_params.h
 * @brief       driver for ST95
 * @author      Mikhail Perkov
 */

#ifndef ST95_PARAMS_H
#define ST95_PARAMS_H


#ifdef __cplusplus
extern "C" {
#endif
#define ST95_SPI_CLK                    SPI_CLK_1MHZ

#define ST95_PULSE_NEGATIVE_USEC        1000
#define ST95_HFO_SETUP_TIME_MS          10
#define ST95_RAMP_UP_TIME_MS            10
#define ST95_DELAY_POWER_ON_MS          100

#define ST95_NO_RESPONSE_TIME_MS        5000
#define ST95_NO_RESPONSE_TIME_MIN_USEC  250

#define ST95_NUMB_TRY_INIT              5

#define ST95_SLEEP_MODE                 0
#define ST95_READY_MODE                 1

/**
 * @brief   ST95 device communincation states
 */
typedef struct {
    volatile bool data_rx;      /**< Flag interrupt */
    volatile bool timeout;      /**< Response timeout */
    volatile uint8_t mode;      /**< Sleep mode */
} st95_state_t;

/**
 * @brief   ST95 SPI control communincation bytes */
typedef enum {
    ST95_CTRT_SPI_SEND      = 0x00,
    ST95_CTRT_SPI_RESET     = 0x01,
    ST95_CTRT_SPI_READ      = 0x02,
    ST95_CTRT_SPI_POLL      = 0x03,
} st95_spi_ctrl_t;


/**
 * @brief   ST95 commands list
 */
typedef enum {
    ST95_CMD_IDN                = 0x01,
    ST95_CMD_PROTOCOL           = 0x02,
    ST95_CMD_POLL_FIELD         = 0x03,     // Only ST95 (Card Emulation mode)
    ST95_CMD_SEND_RECV          = 0x04,
    ST95_CMD_LISTEN             = 0x05,     // Only ST95(Card Emulation mode)
    ST95_CMD_SEND               = 0x06,     // Only ST95(Card Emulation mode)
    ST95_CMD_IDLE               = 0x07,
    ST95_CMD_READ_REG           = 0x08,
    ST95_CMD_WRITE_REG          = 0x09,
    ST95_CMD_BAUDRATE           = 0x0A,     // Only CR95 (UART baudrate)
    ST95_CMD_SUBFREQ            = 0x0B,     // Only ST95 (iso18092)
    ST95_CMD_ANTICOL_FILTER     = 0x0D,     // Only ST95 (iso14443A)
    ST95_CMD_ECHO               = 0x55,
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
    
    CARD_ISO14443A      = 0x12,
    CARD_ISO14443B      = 0x13, // ST Reserved
    CARD_ISO18092       = 0x14, // ST Reserved
} st95_protocol_t;


#ifdef __cplusplus
}
#endif

#endif /* ST95_PARAMS_H */
/** @} */
