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
 * @file        ls-gate.h
 * @brief       LoRaLAN gateway device definitions
 * @author      Eugene Ponomarev
 * @author      Oleg Artamonov
 */
#ifndef UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_H_
#define UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_H_

#include "mutex.h"

#include "ls-mac-types.h"
#include "ls-crypto.h"
#include "ls-gate-device-list.h"
#include "ls-frame-fifo.h"

#include "xtimer.h"
#include "net/netdev.h"
#include "sx127x_internal.h"
#include "sx127x_params.h"
#include "sx127x_netdev.h"

/**
 * @brief Delay before sending frame from queue
 */
#define LS_GATE_TX_DELAY_MS 1000//500

/**
 * @brief Length of first RX window [us]
 */
#define LS_GATE_RX1_LENGTH (1e6 * 4)

/**
 * @brief Ping timeout in seconds.
 */
#define LS_PING_TIMEOUT_S 3

/**
 * @brief Ping counter increment period in microseconds.
 */
#define LS_PING_TIMEOUT (1e6 * LS_PING_TIMEOUT_S)

/**
 * @brief Difference in ping count. If the node skipped LS_MAX_PING_DIFFERENCE pings it considered as dead and would be kicked from the network
 */
#define LS_MAX_PING_DIFFERENCE (60 * 60 * 12)  /* 36 hours */

#define LS_TX_DELAY_MIN_MS 100
#define LS_TX_DELAY_MAX_MS 1000

// TODO: check LoRaWAN
#define LS_RX2_DR LS_DR3
#define LS_RX2_CH 0

/**
 * @brief RSSI of the channel considered free.
 */
#define LS_CHANNEL_FREE_RSSI -100

/**
 * @brief LoRa-Star stack status.
 */
typedef enum {
    LS_GATE_SLEEP = 0,
    LS_GATE_TRANSMITTING,
    LS_GATE_LISTENING,
    LS_GATE_FAULT,
} ls_gate_status_t;

typedef enum {
    LS_GATE_PING = 0,

    LS_GATE_RX1_EXPIRED,
} ls_gate_tim_cmd_t;

/**
 * @brief LoRa-Star stack initialization result.
 *
 */
typedef enum {
    LS_INIT_E_SX1276_THREAD = 1,        /**< Unable to start sx1276 event handler thread */
    LS_INIT_E_SX127X_DEVICE,            /**< Unable to initialize SX127x device */
    LS_GATE_E_INIT,                     /**< Gate initialization error */
    LS_INIT_E_TIM_THREAD,               /**< Unable to start timeout handler thread */
    LS_GATE_E_NODEV,                    /**< Unable to send frame - device with address specified is not joined */
    LS_E_PQ_OVERFLOW,                   /**< Unable to queue frame for sending - queue is overflowed */
    LS_INIT_E_UQ_THREAD,                /**< Unable to create uplink queue handler thread */
    LS_GATE_OK,                         /**< Initialized successfully */
} ls_gate_init_status_t;

/**
 * @brief LoRa-Star stack settings.
 *
 * Could be stored in non-volatile memory
 */
typedef struct {
    uint64_t gate_id;                   /**< Unique node ID */
    uint8_t *join_key;                  /**< Join MIC key */
    uint32_t keepalive_period_ms;       /**< Period of calling `keepalive_cb` [milliseconds] */
} ls_gate_settings_t;

/**
 * @brief Holds internal channel-related data such as transceiver handler, thread stack, etc.
 */
typedef struct {
    netdev_t *device;                   /**< Transceiver instance for this channel */
    void *gate;                         /**< Gate instance pointer */
    ls_frame_t current_frame;           /**< Memory for current frame */
    mutex_t channel_mutex;              /**< Mutex on the channel */
    ls_frame_fifo_t ul_fifo;            /**< Uplink frame queue */
    xtimer_t    rx_window1;             /**< First receive window timer */
} ls_channel_internal_t;

typedef enum {
    LS_GATE_CHANNEL_STATE_IDLE = 0,
    LS_GATE_CHANNEL_STATE_RX,
    LS_GATE_CHANNEL_STATE_TX,
} ls_channel_state_t;

/**
 * @brief Holds channel-related information.
 *
 * One sx1276 transceiver per channel
 */
typedef struct {
    ls_datarate_t dr;                   /**< Data rate for this channel */
    uint32_t frequency;                 /**< LoRa frequency */
    int16_t    last_rssi;               /**< RSSI of last received packet on this channel */
    ls_channel_state_t state;           /**< State of the channel */
    ls_channel_internal_t _internal;    /**< Internal channel-specific data */
} ls_gate_channel_t;

#define LS_UQ_HANDLER_STACKSIZE            (2048)
#define LS_UQ_MSG_QUEUE_SIZE            64

#define LS_TIM_HANDLER_STACKSIZE        (2048)
#define LS_TIM_MSG_QUEUE_SIZE           8

/**
 * @brief Lora-Star gate stack internal data.
 */
typedef struct {
    uint32_t ping_count;                /**< Ping count, increments every PING_TIMEOUT us */
    xtimer_t ping_timer;                /**< Timer for periodic ping count increment */
    xtimer_t keepalive_timer;           /**< Timer for periodic keepalive callback calls */

    /* Timeout message handler data */
    kernel_pid_t tim_thread_pid;
    char tim_thread_stack[LS_TIM_HANDLER_STACKSIZE];

    /* Uplink queue handler data */
    kernel_pid_t uq_thread_pid;
    char uq_thread_stack[LS_TIM_HANDLER_STACKSIZE];

} ls_gate_internal_t;

/**
 * @briefLoRa-Star gate stack state.
 */
typedef struct {
    ls_gate_settings_t settings;        /**< Network settings, could be stored in NVRAM */
    ls_gate_status_t status;            /**< Current LS stack status */
    ls_gate_channel_t *channels;        /**< Array of channels used by this gate */
    size_t num_channels;                /**< Number of channels available */
    /* Callback functions */
    bool (*accept_node_join_cb)(uint64_t dev_id, uint64_t app_id);
    uint32_t (*node_joined_cb) (ls_gate_node_t *node);
    void (*node_kicked_cb) (ls_gate_node_t *node);
    void (*app_data_received_cb) (ls_gate_node_t *node, ls_gate_channel_t *ch, uint8_t *buf, size_t bufsize, uint8_t status);
    void (*app_data_ack_cb)(ls_gate_node_t *node, ls_gate_channel_t *ch);
    /* Gate will call this callback periodically to make sure that watchdog timer (if used) is reset in time  */
    void (*keepalive_cb)(void);

    /*
     * Request for pending frames for specified node.
     * This function will be called when node has pending frames and within it's receive window
     */
    void (*pending_frames_req) (ls_gate_node_t *node);

    ls_gate_devices_t devices;        /**< Devices list */
    ls_gate_internal_t _internal;
} ls_gate_t;

/**
 * @brief Initializes the internal gate structures, channels, transceivers, start listening threads.
 */
int ls_gate_init(ls_gate_t *ls);

/**
 * @brief Sends an answer to the node in channel assigned to the node.
 */
int ls_gate_send_to(ls_gate_t *ls, ls_addr_t devaddr, uint8_t *buf, size_t bufsize);

/**
 * @brief Sends invitation to join for class C devices on all channels
 */
int ls_gate_invite(ls_gate_t *ls, uint64_t nodeid);

/**
 * @brief Broadcasts a packet to all channels.
 */
int ls_gate_broadcast(ls_gate_t *ls, uint8_t *buf, size_t bufsize);

/**
 * @brief Puts gate into sleep mode.
 */
void ls_gate_sleep(ls_gate_t *ls);

#endif /* UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_H_ */
