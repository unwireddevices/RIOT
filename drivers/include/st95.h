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
 * @file		st95.h
 * @brief       driver for ST95
 * @author      Mikhail Perkov
 */
#ifndef ST95_H_
#define ST95_H_

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "periph/gpio.h"
#include "periph/spi.h"

#define ST95_IFACE_UART         1
#define ST95_IFACE_SPI          2

#define ST95_MAX_BYTE_BUFF      256

#define ST95_RESULT_CODE_OK     0x80
#define ST95_RESULT_CODE_ACK    0x90
#define ST95_BYTE_ACK           0x0A
#define ST95_BYTE_NACK          0x00
#define ST95_RESULT_BYTE        0x24

/* Offset definitions for global buffers */
#define ST95_COMMAND_OFFSET		0
#define ST95_LENGTH_OFFSET		1
#define ST95_DATA_OFFSET		2

#define ST95_TX_RATE_106         0
#define ST95_RX_RATE_106         0
#define ST95_TX_RATE_212         1
#define ST95_RX_RATE_212         1
#define ST95_TX_RATE_424         2
#define ST95_RX_RATE_424         2
#define ST95_TX_RATE_848         3
#define ST95_RX_RATE_848         3

/* Read/Write(RR/WR) Register parameters */
#define ST95_READ_ADDR_1        0x69        // Register address
#define ST95_READ_ADDR_2        0x62        // Register address
#define ST95_REG_SIZE           0x01        // Register size
#define ST95_ST_RESERVED        0x00        // ST Reserved

#define ST95_WR_ARC_ADDR        0x68        // Analog Register Configuration address index
#define ST95_WR_TIMER_WINDOW    0x3A        // Timer Window value
#define ST95_WR_AUTODETECT      0x0A        // AutoDetect filter control value

#define ST95_WR_FLAG_INC        0x01        // Flag Increment address after Write command
#define ST95_WR_FLAG_NOT_INC    0x00        // Flag not Increment address after Write command

#define ST95_WR_PTR_MODUL_GAIN  0x01        // Index pointing to the Modulation and Gain in ARC_B

#define ST95_WR_TIMER_WINDOW_CONFIRM 0x04        // Timer Window value confirmation
#define ST95_WR_TIMER_WINDOW_VAL 0x5F       // Timer Window value

/* Possible Modulation index values [%] */
#define ST95_WR_MODULATION_10   0x01        // 10%
#define ST95_WR_MODULATION_17   0x02        // 17%
#define ST95_WR_MODULATION_25   0x03        // 25%
#define ST95_WR_MODULATION_30   0x04        // 30%
#define ST95_WR_MODULATION_33   0x05        // 33%
#define ST95_WR_MODULATION_36   0x06        // 36%
#define ST95_WR_MODULATION_95   0x0D        // 95%
/* Possible receiver Gain values [dB] */
#define ST95_WR_GAIN_34_DB      0x00        // 34 Db
#define ST95_WR_GAIN_32_DB      0x01        // 32 Db
#define ST95_WR_GAIN_27_DB      0x03        // 27 Db
#define ST95_WR_GAIN_20_DB      0x07        // 20 Db
#define ST95_WR_GAIN_8_DB       0x0F        // 8 Db


/**
 * @brief ST95 return codes
*/
#define ST95_OK			        0
#define ST95_WAKE_UP            ST95_OK
#define ST95_ERROR		        1
#define ST95_NO_DEVICE	        2

/**
 * @brief   ST95 hardware and global parameters.
 */
typedef struct {
    uint8_t iface;      /**< Iface (SPI or UART) */
    uint8_t uart;       /**< UART device */
    uint32_t baudrate;  /**< Baudrate UART device */
    
    uint8_t spi;        /**< SPI device */
    gpio_t cs_spi;      /**< SPI NSS pin */
    gpio_t irq_in;      /**< Interrupt input */
    gpio_t irq_out;     /**< Interrupt output */
    gpio_t ssi_0;       /**< Select serial communication interface */
    gpio_t ssi_1;       /**< Select serial communication interface */
    gpio_t vcc;         /**< Vcc enable */
    
    uint8_t dac_l;      /**< DacDataL value (Lower compare value for tag detection) */
    uint8_t dac_h;      /**< DacDataH value (Higher compare value for tag detection) */
} st95_params_t;

/**
 * @brief   ST95 wake up callback
 */
typedef void (*st95_cb_t)(void *);

/**
 * @brief   ST95 device descriptor
 */
typedef struct {
    st95_params_t params;   /**< device driver configuration */
    st95_cb_t cb;           /**< callback */
    void *arg;              /**< callback param */
} st95_t;

/**
 * @brief ST95 driver initialization routine
 *
 * @param[in]   dev Pointer to ST95 device descriptor
 * @param[in]   params Pointer to static ST95 device configuration
 *
 * @return 0 if initialization succeeded
 * @return >0 in case of an error
 */
int st95_init(st95_t *dev, st95_params_t * params);

void st95_spi_reset(const st95_t * dev);

void st95_sleep(st95_t * dev);

int st95_is_wake_up(const st95_t * dev);

int st95_idn(const st95_t * dev, uint8_t * idn, uint8_t * length);

int st95_write_data(const st95_t * dev, uint8_t * data, uint16_t length);
int st95_read_data(const st95_t * dev, uint8_t * data, uint16_t length);
int st95_get_uid(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak);
int st95_set_uid(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak);


int _st95_select_iso14443a(const st95_t * dev, uint8_t * params, uint8_t length_params);
uint8_t _st95_cmd_write_reg(const st95_t * dev, uint8_t size_tx, uint8_t addr, uint8_t flag, uint8_t * data_tx);
uint8_t _st95_modify_modulation_gain(const st95_t * dev, uint8_t modul, uint8_t gain);
uint8_t _st95_set_timer_window(const st95_t * dev, uint8_t timer_w);

int _st95_cmd_send_receive(const st95_t * dev, uint8_t *data_tx, uint8_t size_tx, uint8_t params, uint8_t * rxbuff, uint16_t size_rx_buff);
#endif /* ST95_H_ */
