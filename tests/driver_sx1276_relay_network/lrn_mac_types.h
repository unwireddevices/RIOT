/*
 * Copyright (C) 2016 Unwired Devices
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
 * @file		lrn_mac_types.h
 * @brief       LRN MAC types declaration
 * @author      Unwired Devices
 */
#ifndef TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_MAC_TYPES_H_
#define TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_MAC_TYPES_H_

#include "crypto/aes.h"

/**
 * Broadcasting address.
 */
#define LRN_ADDR_UNDEFINED 0x00

typedef uint8_t lrn_mhdr_t;		/**< 1 byte LoRaWAN MHDR */
typedef uint8_t lrn_addr_t;     /**< 1 byte node address */
typedef uint8_t lrn_frame_id_t;	/**< 1 byte frame ID */
typedef uint32_t lrn_mic_t;     /**< 4 byte message integrity check value */

/**
 * @biref Possible data rates.
 */
typedef enum {
	DR0 = 0,
	DR1,
	DR2,
	DR3,
	DR4,
	DR5,
	DR6,
} lrn_datarate_t;

/**
 * @brief Reserved value for the LoRaWAN MHDR field
 */
#define MHDR_LORAWAN_RESERVED 0xFF // TODO: select right one

/**
 * @brief Types of a frame
 */
typedef enum {
	LRN_TYPE_CMD = 0,
	LRN_TYPE_DATA,

	/* LRN specific types */
	LRN_NH_ACK,			/**< Next hop acknowledge. Next hop node is successfully retransmitted the frame */
	LRN_DST_ACK,		/**< Destination acknowledge. Destination node is successfully received the message */

	LRN_JOIN_REQ,	/**< Join request */
	LRN_JOIN_ACK,	/**< Join acknowledge */
} lrn_type_t;

/**
 * LRN Frame header.
 */
typedef struct {
    uint8_t mhdr;                   /**< Reserved value for the MHDR to be LoRaWAN compatible */

    lrn_mic_t mic : 24;                  /**< Frame's message integrity check value*/

    lrn_addr_t dest_addr;           /**< Destination address */
    lrn_addr_t src_addr;            /**< Source address */
    lrn_addr_t next_hop;			/**< Address of the next hop */

    lrn_addr_t retr_src;			/**< Retransmission source */
    lrn_type_t type;				/**< Type of a frame */

    lrn_frame_id_t fid;				/**< Frame serial number for frame acknowledge */
} lrn_header_t;

/**
 * Type of the payload length field.
 */
typedef uint8_t lrn_payload_len_t;

/**
 * Size of the frame.
 */
#define LRN_FRAME_SIZE 255

/**
 * Minimum size of the LRN frame. This is a size of a frame with empty payload
 */
#define LRN_FRAME_MINIMUM_SIZE (sizeof(lrn_header_t) + sizeof(lrn_payload_len_t))

/**
 * Maximum size of the payload in LRN
 */
#define LRN_PAYLOAD_SIZE_MAX (LRN_FRAME_SIZE - LRN_FRAME_MINIMUM_SIZE - AES_BLOCK_SIZE - 2)

#define LRN_PAYLOAD_BUF_SIZE (LRN_FRAME_SIZE - LRN_FRAME_MINIMUM_SIZE)

/**
 * LRN frame payload.
 */
typedef struct {
    lrn_payload_len_t len;              /**< Length of the payload */
    uint8_t data[LRN_PAYLOAD_BUF_SIZE]; /**< Payload data */
} lrn_payload_t;


/**
 * LRN Frame.
 */
typedef struct {
    lrn_header_t header;            /**< LRN frame header */
    lrn_payload_t payload;          /**< LRN frame payload */
} lrn_frame_t;

#endif /* TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_MAC_TYPES_H_ */
