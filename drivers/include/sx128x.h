/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_sx128x Semtech SX1280 and SX1281 radios driver
 * @ingroup     drivers_netdev
 * @brief       Driver for Semtech SX1280 and SX1281 radios.
 *
 * This module contains the driver for radio devices of the Semtech SX128x
 * series (SX1280 and SX1281).
 * Only LoRa long range modem is supported at the moment.
 *
 * SX128x modules are designed to be used in the ISM radio frequency (RF) band.
 * This RF band depends on different regional regulations worldwide.
 * Be careful to configure the device to use a RF frequency allowed in your
 * region.
 *
 *
 * @{
 *
 * @file
 * @brief       Public interface for SX128X driver
 * 
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef _SX128X_H_
#define _SX128X_H_

#include "lptimer.h"
#include "net/netdev.h"
#include "periph/gpio.h"
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   LoRa configuration structure.
 */
typedef struct {
    uint16_t preamble_len;              /**< Length of preamble header */
    int8_t   power;                     /**< Signal power */
    uint8_t  bandwidth;                 /**< Signal bandwidth */
    uint8_t  datarate;                  /**< Spreading factor rate, e.g datarate */
    uint8_t  coderate;                  /**< Error coding rate */
    uint8_t  freq_hop_period;           /**< Frequency hop period */
    uint8_t  flags;                     /**< Boolean flags */
    uint32_t rx_timeout;                /**< RX timeout in milliseconds */
    uint32_t tx_timeout;                /**< TX timeout in milliseconds */
} sx128x_lora_settings_t;

/**
 * @brief   Radio settings.
 */
typedef struct {
    uint32_t channel;                   /**< Radio channel */
    uint8_t state;                      /**< Radio state */
    uint8_t modem;                      /**< Driver model (FSK or LoRa) */
    sx128x_lora_settings_t lora;        /**< LoRa settings */
} sx128x_radio_settings_t;

typedef enum {
    SX127X_MODEM_SX1280 = 0,
    SX127X_MODEM_SX1281 = 1,
} sx128x_modem_chip_t;

/**
 * @brief   SX128X internal data.
 */
typedef struct {
    /* Data that will be passed to events handler in application */
    lptimer_t tx_timeout_timer;         /**< TX operation timeout timer */
    lptimer_t rx_timeout_timer;         /**< RX operation timeout timer */
    uint32_t last_channel;              /**< Last channel in frequency hopping sequence */
    sx128x_modem_chip_t modem_chip;     /**< Modem model */
    bool is_last_cad_success;           /**< Sign of success of last CAD operation (activity detected) */
} sx128x_internal_t;

/**
 * @brief   SX128X hardware and global parameters.
 */
typedef struct {
    spi_t  spi;                         /**< SPI device */
    gpio_t nss_pin;                     /**< SPI NSS pin */
    gpio_t reset_pin;                   /**< Reset pin */
    gpio_t busy                         /**< Busy pin */
    gpio_t dio1_pin;                    /**< Interrupt line DIO1 */
    gpio_t dio2_pin;                    /**< Interrupt line DIO2 */
    gpio_t dio3_pin;                    /**< Interrupt line DIO3 */
} sx128x_params_t;

/**
 * @brief   SX128X IRQ flags.
 */
typedef uint8_t sx128x_flags_t;

/**
 * @brief   SX128X device descriptor.
 * @extends netdev_t
 */
typedef struct {
    netdev_t netdev;                    /**< Netdev parent struct */
    sx128x_radio_settings_t settings;   /**< Radio settings */
    sx128x_params_t params;             /**< Device driver parameters */
    sx128x_internal_t _internal;        /**< Internal sx128x data used within the driver */
    sx128x_flags_t irq;                 /**< Device IRQ flags */
} sx128x_t;

#ifdef __cplusplus
}
#endif

#endif /* _SX128X_H_ */
/** @} */