/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file		ls-mac-types.h
 * @brief       LoRa Star MAC types
 * @author      Eugene Ponomarev
 */
#ifndef UNWIRED_MODULES_LORALAN_MAC_TYPES_H_
#define UNWIRED_MODULES_LORALAN_MAC_TYPES_H_

#include <stdbool.h>
#include "crypto/aes.h"

/**
 * @brief Undefined/multicast address.
 */
#define LS_ADDR_UNDEFINED 0xFFFFFFFF

/**
 * @brief Types of values
 */
typedef uint8_t ls_mhdr_t;			/**< 1 byte LoRaWAN MHDR */
typedef uint32_t ls_addr_t;     	/**< 4 byte node address */
typedef uint16_t ls_frame_id_t;		/**< 1 byte frame ID */
typedef uint16_t ls_bc_frame_id_t;	/**< 2 byte broadcast frame ID */
typedef uint32_t ls_mic_t;     		/**< 4 byte message integrity check value */
typedef uint8_t ls_channel_t;		/**< 1 byte channel number */
typedef uint32_t ls_nonce_t;        /**< 4 byte nounce value */

/**
 * End device class
 */
typedef enum {
	LS_ED_CLASS_A = 0,
	LS_ED_CLASS_B,
	LS_ED_CLASS_C
} ls_node_class_t;

/**
 * @biref Possible data rates.
 */
typedef enum {
	LS_DR0 = 0,
	LS_DR1,
	LS_DR2,
	LS_DR3,
	LS_DR4,
	LS_DR5,
	LS_DR6,
} ls_datarate_t;

/**
 * @brief Reserved value for the LoRaWAN MHDR field
 */
#define MHDR_LORAWAN_RESERVED 0xFF // TODO: select right one

/**
 * @brief Types of a frame
 */
typedef enum {
	LS_DL,				/**< Downlink application data frame */
	LS_UL_ACK,			/**< Uplink acknowledge from the end device */

	LS_UL_CONF,			/**< Uplink application data confirmed */
	LS_UL_UNC,			/**< Uplink application data unconfirmed */

	LS_DL_ACK,			/**< Downlink application data acknowledge for confirmed messages */


	LS_UL_UNC_ACK,		/**< Uplink application data unconfirmed with ACK for previous app. data */
	LS_DL_ACK_W_DATA,	/**< Downlink appdata acknowledge with additional app. data */

	LS_DL_BROADCAST,	/**< Unconfirmed downlink broadcasted message */

	LS_UL_JOIN_REQ,	    /**< Join request */
	LS_DL_JOIN_ACK,	    /**< Join acknowledge */

	RFU4,
	LS_DL_INVITE,		/**< Individual join invitation for the class C (always listening) devices */
    
    LS_UL_TIME_REQ,     /**< Time synchronization request */
    LS_DL_TIME_ACK,     /**< Time synchronization reply */

	/* Reserved for future use */
	LS_RFU5,
} ls_type_t;

/**
 * LoRa-Star Frame header.
 */
typedef struct {
    uint8_t mhdr;					/**< Reserved value of the MHDR for the LoRaWAN coexistence */
    ls_mic_t mic : 24;				/**< Frame's Message Integrity Check value*/
    ls_addr_t dev_addr;				/**< Destination address */
    ls_type_t type;					/**< Type of a frame */
    uint8_t fid;				/**< Frame serial number */
    uint8_t status;                 /**< Node status */
} ls_header_r1_t;

typedef struct {
    uint8_t mhdr;					/**< Reserved value of the MHDR for the LoRaWAN coexistence */
    ls_mic_t mic : 24;				/**< Frame's Message Integrity Check value*/
    ls_addr_t dev_addr;				/**< Destination address */
    ls_frame_id_t fid;				/**< Frame serial number */
    ls_type_t type;					/**< Type of a frame */
    uint8_t status;                 /**< Node status */
} ls_header_t;

/**
 * Type of the payload length field.
 */
typedef uint8_t ls_payload_len_t;

/**
 * Size of the frame.
 */
#define LS_FRAME_SIZE 255

/**
 * Minimum size of the LS frame. This is a size of a frame with empty payload
 */
#define LS_FRAME_MINIMUM_SIZE (sizeof(ls_header_t) + sizeof(ls_payload_len_t))

/**
 * Maximum size of the payload in LS
 */
#define LS_PAYLOAD_SIZE_MAX (LS_FRAME_SIZE - LS_FRAME_MINIMUM_SIZE - AES_BLOCK_SIZE)
#define LS_PAYLOAD_BUF_SIZE (LS_FRAME_SIZE - LS_FRAME_MINIMUM_SIZE)

/**
 * LS frame payload.
 */
typedef struct {
    ls_payload_len_t len;              /**< Length of the payload */
    uint8_t data[LS_PAYLOAD_BUF_SIZE]; /**< Payload data */
} ls_payload_t;

/**
 * LS Frame.
 */

#if defined(UNWDS_MAC_LORAWAN)
typedef struct {
    uint8_t data[32];              /**< Payload data, 32 bytes max */
    uint8_t length;                /**< Payload length */
    uint8_t fport;                 /**< LoRaWAN FPort */
    bool    retransmit;            /**< Frame already transmitted, but ACK failed */
} ls_frame_t;
#else
typedef struct {
    ls_header_t header;            /**< LS frame header */
    ls_payload_t payload;          /**< LS frame payload */
} ls_frame_t;
#endif

typedef struct {
    ls_header_r1_t header;         /**< LS frame header */
    ls_payload_t payload;          /**< LS frame payload */
} ls_frame_r1_t;

/**
 * LS join request.
 */
typedef struct {
	uint64_t dev_id;			/**< Unique device identifier */
	uint64_t app_id;			/**< Unique application identifier */

	uint32_t dev_nonce;			/**< Random number generated on the device */
	ls_node_class_t node_class;	/**< Node's device class */
} ls_join_req_t;

/**
 * LS join acknowledge.
 */
typedef struct {
	uint64_t dev_id;
	ls_addr_t addr;
	uint32_t app_nonce;
} ls_join_ack_t;

typedef struct {
	uint8_t status_byte;
} ls_device_status_t;

/**
 * @brief Individual join invitation for class C devices
 */
typedef struct {
	uint64_t dev_id;
} ls_invite_t;

/**
 * @brief Current time value
 */
typedef struct {
	time_t gate_time;    /**< Current time on a gateway on a moment of request */
} ls_time_req_ack_t;

#endif /* UNWIRED_MODULES_LORALAN_MAC_TYPES_H_ */
