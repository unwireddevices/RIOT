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
 * @file        lrn.h
 * @brief       LoRa Relay Network module definitions
 * @author      Unwired Devices
 */

#ifndef TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_LRN_H_
#define TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_LRN_H_

#include "crypto/aes.h"

#include "sx1276.h"

#include "lrn_frame_fifo.h"
#include "lrn_routing.h"
#include "lrn_mac_types.h"
#include "lrn_vfl.h"

#define LRN_SX1276_LISTENER_STACKSIZE      (2 * THREAD_STACKSIZE_DEFAULT)
#define LRN_FRAME_Q_HANDLER_STACKSIZE      (2 * THREAD_STACKSIZE_DEFAULT)

/**
 * @brief Receive window time. Milliseconds
 */
#define LRN_RX_WINDOW_TIME 2500

/**
 * @brief Mimimum delay between TX operations from uplink queue. In milliseconds
 */
#define LRN_TX_DELAY_MIN 100

/**
 * @brief Maximum delay between TX operations from uplink queue. In milliseconds
 */
#define LRN_TX_DELAY_MAX 1000

/**
 * @brief RSSI threshold for free channel.
 */
#define LRN_CHANNEL_FREE_RSSI -100

/**
 * @brief State of the LRN stack
 */
typedef enum {
	LRN_SLEEP = 0,

	LRN_TRANSMITTING,
	LRN_TRANSMITTING_AFTER_LISTENING,
	LRN_LISTENING,

	LRN_FAULT,
} lrn_state_t;

/**
 * LoRa Relay Network driver state
 */
typedef struct {
	lrn_state_t state;		/**< LRN stack state */
    lrn_addr_t dev_addr;	/**< Device address */

    lrn_addr_t gate_addr;	/**< LRN network gateway address */
    bool is_gateway;		/**< Is this device a gateway */

    bool joined;			/**< Is joined to the network */

    /**
     * Data rates for TX and RX modes.
     */
    lrn_datarate_t datarate_tx;
    lrn_datarate_t datarate_rx;

    /**
     * LRN routing table for this node
     */
    lrn_routing_table_entry_t *routing_table;
    int	routing_table_size;	/**< Routing table size */

    /**
     * Cryptographic key used in AES-128-CBC payload encryption and frame integrity check.
     */
    uint8_t crypto_key[AES_KEY_SIZE];

    /**
     * Reception of data callback function.
     */
    void (*data_recv_cb)(lrn_addr_t from, uint8_t* data, uint8_t data_len);

    /**
     * Reception of command callback function.
     */
    void (*cmd_recv_cb)(lrn_addr_t from, uint8_t* cmd_data, uint8_t cmd_data_len);

    /**
     * Node is joined to the network callback
     */
    void (*joined_cb)(uint8_t num_nodes);

	/* Internal data */
    sx1276_t *_sx1276;                       	/**< LoRa transceiver handler */
    char _sx1276_listener_thread_stack[LRN_SX1276_LISTENER_STACKSIZE];	/**< SX1276 events listener thread stack */
    msg_t _sx1276_event_queue[10];

    /* RX window timer */
    xtimer_t rx_window_timer;

    /* Internal uplink frame queue */
    char _uq_thread_stack[LRN_FRAME_Q_HANDLER_STACKSIZE];
    kernel_pid_t _uq_thread_pid;
    msg_t _uq_msg_queue[LRN_MAX_FRAME_FIFO_SIZE];

    lrn_frame_fifo_t _uplink_queue;

    /* Last frame ID */
    lrn_frame_id_t _last_fid;

    lrn_vfl_t _vfl;			/**< Frames that awaiting acknowledge */
    xtimer_t _vfl_timer;	/**< Timer for the acknowledge timeouts */

    /* Internal downlink frame queue */
    /*char _dq_thread_stack[LRN_FRAME_Q_HANDLER_STACKSIZE];
    kernel_pid_t _dq_thread_pid;
    msg_t _dq_msg_queue[LRN_MAX_FRAME_FIFO_SIZE];

    lrn_frame_fifo_t _downlink_queue;*/
} lrn_t;

/**
 * @brief Initializes LRN stack.
 *
 *
 * @param   [IN]    lrn                 the LRN stack state structure
 * @param   [IN]    app_listener_thread LRN events listener on application side thread PID
 */
void lrn_init(lrn_t *lrn);

/**
 * @brief Sends JOIN request and joins to the network
 *
 * @param	[IN]	lrn		the LRN stack state structure
 */
void lrn_join(lrn_t *lrn);

/**
 * @brief Send frame into the network.
 *
 * @param	[IN]	lrn		the LRN stack state structure
 * @param	[IN]	dest	destination device address
 * @param	[IN]	*buf	payload buffer
 * @param	[IN]	buflen	length of the payload buffer
 */
void lrn_send(lrn_t *lrn, lrn_addr_t src, lrn_addr_t dest, lrn_type_t type, lrn_frame_id_t fid, uint8_t *buf, uint8_t buflen);

/**
 * @brief Sets LRN stack to listening of incoming frames.
 *
 * @param	*lrn			the LRN stack state
 */
void lrn_listen(lrn_t *lrn);

/**
 * @brief Sets LRN stack into sleep mode.
 *
 * @param	lrn	the LRN stack state
 */
void lrn_sleep(lrn_t *lrn);

#endif /* TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_LRN_H_ */
