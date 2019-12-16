/*
 * Copyright (C) 2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_loramac LoRaMAC
 * @ingroup     net
 * @brief       LoRaMAC definitions
 *
 * @{
 *
 * @file
 * @brief       LoRaMAC header definitions
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef NET_LORAMAC_H
#define NET_LORAMAC_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup net_loramac_conf   LoRaMAC compile configurations
 * @ingroup config
 * @{
 */
/**
 * @brief   Default device EUI
 *
 *          8 bytes key, required for join procedure
 */
#ifndef LORAMAC_DEV_EUI_DEFAULT
#define LORAMAC_DEV_EUI_DEFAULT        { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default application EUI
 *
 *          8 bytes key, required for join procedure
 */
#ifndef LORAMAC_APP_EUI_DEFAULT
#define LORAMAC_APP_EUI_DEFAULT        { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default application key
 *
 *          16 bytes key, required for join procedure
 */
#ifndef LORAMAC_APP_KEY_DEFAULT
#define LORAMAC_APP_KEY_DEFAULT        { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default application session key
 *
 *          16 bytes key, only required for ABP join procedure type
 */
#ifndef LORAMAC_APP_SKEY_DEFAULT
#define LORAMAC_APP_SKEY_DEFAULT       { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default network session key
 *
 *          16 bytes key, only required for ABP join procedure type.
 */
#ifndef LORAMAC_NWK_SKEY_DEFAULT
#define LORAMAC_NWK_SKEY_DEFAULT       { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default device address
 */
#ifndef LORAMAC_DEV_ADDR_DEFAULT
#define LORAMAC_DEV_ADDR_DEFAULT       { 0x00, 0x00, 0x00, 0x00 }
#endif
/** @} */

/**
 * @name    LoRaMAC default values
 * @{
 */
/**
 * @brief   Default device class (A, B or C)
 */
#ifndef LORAMAC_DEFAULT_DEVICE_CLASS
#define LORAMAC_DEFAULT_DEVICE_CLASS           (LORAMAC_CLASS_A)
#endif

/**
 * @brief   Default NetID (only valid with ABP join procedure)
 */
#ifndef LORAMAC_DEFAULT_NETID
#define LORAMAC_DEFAULT_NETID                  (1U)
#endif

/**
 * @brief   Default network type (public or private)
 */
#ifndef LORAMAC_DEFAULT_PUBLIC_NETWORK
#define LORAMAC_DEFAULT_PUBLIC_NETWORK         (true)
#endif
/**
 * @brief   Default datarate (only valid for EU)
 */
#ifndef LORAMAC_DEFAULT_DR
#define LORAMAC_DEFAULT_DR                     (0)
#endif

/**
 * @brief   Default MAC TX power (14dBm in EU)
 */
#ifndef LORAMAC_DEFAULT_TX_POWER
#define LORAMAC_DEFAULT_TX_POWER               (0)
#endif

/**
 * @brief   Default MAC TX port (from 1 to 223)
 */
#ifndef LORAMAC_DEFAULT_TX_PORT
#define LORAMAC_DEFAULT_TX_PORT                (2U)
#endif

/**
 * @brief   Default MAC TX mode (confirmable or unconfirmable)
 */
 #ifndef LORAMAC_DEFAULT_TX_MODE
 #define LORAMAC_DEFAULT_TX_MODE               (LORAMAC_TX_CNF)
 #endif

/**
 * @brief   Default MAC TX power (14dBm in EU)
 */
#ifndef LORAMAC_DEFAULT_TX_POWER
#define LORAMAC_DEFAULT_TX_POWER               (LORAMAC_TX_PWR_1)
#endif

/**
 * @brief   Default adaptive datarate state
 */
#ifndef LORAMAC_DEFAULT_ADR
#define LORAMAC_DEFAULT_ADR                    (false)
#endif

/**
 * @brief   Default uplink retransmission
 */
#ifndef LORAMAC_DEFAULT_RETX
#define LORAMAC_DEFAULT_RETX                   (5U)
#endif

/**
 * @brief   Default link check interval (in seconds)
 *
 *          0 means the link check process is disabled
 */
#ifndef LORAMAC_DEFAULT_LINKCHK
#define LORAMAC_DEFAULT_LINKCHK                (0U)
#endif

/**
 * @brief   Default first RX window delay (in ms)
 */
#ifndef LORAMAC_DEFAULT_RX1_DELAY
#define LORAMAC_DEFAULT_RX1_DELAY              (1000U)
#endif

/**
 * @brief   Default second RX window delay (in ms)
 */
#define LORAMAC_DEFAULT_RX2_DELAY              (1000U + LORAMAC_DEFAULT_RX1_DELAY)

/**
 * @brief   Default automatic reply status
 */
#ifndef LORAMAC_DEFAULT_AR
#define LORAMAC_DEFAULT_AR                     (false)
#endif

/**
 * @brief   Default second RX window datarate index
 */
#ifndef LORAMAC_DEFAULT_RX2_DR
#define LORAMAC_DEFAULT_RX2_DR                 (0)
#endif

/**
 * @brief   Default LoRaWAN region
 */
#ifndef LORAMAC_DEFAULT_REGION
#define LORAMAC_DEFAULT_REGION                  (0)
#endif

/**
 * @brief   Default LoRaMAC join procedure
 */
#ifndef LORAMAC_DEFAULT_JOIN_PROCEDURE
#define LORAMAC_DEFAULT_JOIN_PROCEDURE         (LORAMAC_JOIN_OTAA)
#endif

/**
 * @brief   Default LoRaMAC join accept delay 1 (in seconds)
 */
#ifndef LORAMAC_DEFAULT_JOIN_DELAY1
#define LORAMAC_DEFAULT_JOIN_DELAY1            (5U)
#endif

/**
 * @brief   Default LoRaMAC join accept delay 2
 */
#ifndef LORAMAC_DEFAULT_JOIN_DELAY2
#define LORAMAC_DEFAULT_JOIN_DELAY2            (6U)
#endif

/**
 * @brief   Default max FCNT gap
 */
#ifndef LORAMAC_DEFAULT_MAX_FCNT_GAP
#define LORAMAC_DEFAULT_MAX_FCNT_GAP           (16384U)
#endif

/**
 * @brief   Default adaptive datarate ACK limit (in s)
 */
#ifndef LORAMAC_DEFAULT_ADR_ACK_LIMIT
#define LORAMAC_DEFAULT_ADR_ACK_LIMIT          (64U)
#endif

/**
 * @brief   Default adaptive datarate ACK delay (in s)
 */
#ifndef LORAMAC_DEFAULT_ADR_ACK_DELAY
#define LORAMAC_DEFAULT_ADR_ACK_DELAY          (32U)
#endif

/**
 * @brief   Default adaptive datarate timeout
 */
#ifndef LORAMAC_DEFAULT_ADR_TIMEOUT
#define LORAMAC_DEFAULT_ADR_TIMEOUT            (3U)
#endif

/**
 * @brief   Default maximum system overall timing error
 */
#ifndef LORAMAC_DEFAULT_SYSTEM_MAX_RX_ERROR
#define LORAMAC_DEFAULT_SYSTEM_MAX_RX_ERROR    (50)
#endif

/**
 * @brief   Default minimum RX symbols to detect a frame
 */
#ifndef LORAMAC_DEFAULT_MIN_RX_SYMBOLS
#define LORAMAC_DEFAULT_MIN_RX_SYMBOLS         (12)
#endif
/** @} */

/**
 * @name    LoRaMAC constants
 * @{
 */
/**
 * @brief   Device EUI length in bytes
 */
#define LORAMAC_DEVEUI_LEN             (8U)

/**
 * @brief   Device address length in bytes
 */
#define LORAMAC_DEVADDR_LEN            (4U)

/**
 * @brief   Application EUI length in bytes
 */
#define LORAMAC_APPEUI_LEN             (8U)

/**
 * @brief   Application key length in bytes
 */
#define LORAMAC_APPKEY_LEN             (16U)

/**
 * @brief   Application session key length in bytes
 */
#define LORAMAC_APPSKEY_LEN            (16U)

/**
 * @brief   Network session key length in bytes
 */
#define LORAMAC_NWKSKEY_LEN            (16U)

/**
 * @brief   Minimum port value
 */
#define LORAMAC_PORT_MIN               (1U)

/**
 * @brief   Maximmu port value
 */
#define LORAMAC_PORT_MAX               (223U)

/**
 * @brief Application Nonce length in bytes
 */
#define LORAMAC_APP_NONCE_LEN          (3U)

/**
 * @brief Network ID length in bytes
 */
#define LORAMAC_NETWORK_ID_LEN         (3U)

/** @} */

/**
 * @name    LoRaMAC parameters indexes
 */

/**
 * @brief   Device class
 */
typedef enum {
    LORAMAC_CLASS_A,                   /**< Class A device */
    LORAMAC_CLASS_B,                   /**< Class B device */
    LORAMAC_CLASS_C,                   /**< Class C device */
} loramac_class_t;

/**
 * @brief   LoRaMAC network join procedure type
 */
typedef enum {
    LORAMAC_JOIN_OTAA,                 /**< Other-the-air activation */
    LORAMAC_JOIN_ABP,                  /**< Activation by personnalization */
} loramac_join_mode_t;

/**
 * @brief   LoRaMAC transmission mode
 */
typedef enum {
    LORAMAC_TX_CNF,                    /**< Confirmable transmission mode */
    LORAMAC_TX_UNCNF,                  /**< Unconfirmable transmission mode */
} loramac_tx_mode_t;

/**
 * @brief   A LoRaMAC network channel
 */
 typedef struct {
    uint32_t freq;                     /**< Center frequency in Hz */
    uint8_t index;                     /**< Channel index in defined list */
    uint8_t bw;                        /**< Bandwidth index */
    uint8_t dr_min;                    /**< Minimum datarate index */
    uint8_t dr_max;                    /**< Maximum datarate index */
    uint8_t dcycle;                    /**< Duty cycle to use on this channel (1 to 100) */
} loramac_channel_t;

#ifdef __cplusplus
}
#endif

#endif /* NET_LORAMAC_H */
/** @} */
