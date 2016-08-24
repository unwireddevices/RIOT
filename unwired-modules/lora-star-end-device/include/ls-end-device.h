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
 * @file		ls.h
 * @brief       LoRa-Star definitions
 * @author      Eugene Ponomarev
 */
#ifndef UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_H_
#define UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_H_

#include "ls-frame-fifo.h"
#include "ls-mac-types.h"
#include "ls-crypto.h"

#include "sx1276.h"

#define LS_TX_DELAY_MIN_MS 100
#define LS_TX_DELAY_MAX_MS 1000

#define LS_RX2_DR LS_DR3
#define LS_RX2_CH 0

/**
 * @brief RSSI of the channel considered free.
 */
#define LS_CHANNEL_FREE_RSSI -100

/**
 * @brief Maximum time of awaiting for the confirmed application data acknowledge in microseconds.
 */
#define LS_ACK_TIMEOUT	1e6 * 15

/**
 * @brief Maximum time of awaiting for the link check request acknowledge in microseconds.
 */
#define LS_LNKCHK_TIMEOUT	1e6 * 15

/**
 * @brief Maximum time of awaiting for the join request acknowledge in microseconds.
 */
#define LS_JOIN_TIMEOUT 	1e6 * 15

/**
 * @brief Duration of the first receive window in microseconds.
 */
#define LS_RX_DELAY1		1e6 * 2

/**
 * @brief Duration of the second receive window in microseconds.
 */
#define LS_RX_DELAY2		1e6 * 2

/**
 * Sleep request delay
 */
#define LS_ED_SLEEP_REQUEST_DELAY 1e6 * 1

// TODO: optimize these values to reduce memory consumption
#define LS_SX1276_LISTENER_STACKSIZE	(2 * THREAD_STACKSIZE_DEFAULT)
#define LS_UQ_HANDLER_STACKSIZE			(2 * THREAD_STACKSIZE_DEFAULT)
#define LS_TIM_HANDLER_STACKSIZE		(1 * THREAD_STACKSIZE_DEFAULT)

#define LS_TIM_MSG_QUEUE_SIZE 10

typedef enum {
	LS_ED_RX1_EXPIRED = 0,
	LS_ED_RX2_EXPIRED,

	LS_ED_JOIN_REQ_EXPIRED,
	LS_ED_LNKCHK_REQ_EXPIRED,

	LS_ED_APPDATA_ACK_EXPIRED,

	LS_ED_LNKCHK_BEGIN,

	LS_ED_SLEEP_REQUEST,
} ls_ed_tim_cmd_t;

/**
 * @brief LoRa-Star stack status.
 */
typedef enum {
	LS_ED_SLEEP = 0,

	LS_ED_TRANSMITTING,
	LS_ED_LISTENING,

	LS_ED_IDLE,

	LS_ED_FAULT,
} ls_ed_status_t;

/**
 * @brief LoRa-Star stack initialization result.
 *
 */
typedef enum {
	LS_INIT_E_SX1276_THREAD = 1,	/**< Unable to start sx1276 event handler thread */
	LS_INIT_E_FQ_THREAD,			/**< Unable to start uplink frame queue handler thread */
	LS_INIT_E_TIM_THREAD,			/**< Unable to start rx window timing handler thread */
	LS_SEND_E_FQ_OVERFLOW,			/**< Uplink frame queue is overflowed */

	LS_OK,							/**< Initialized successfully */

} ls_init_status_t;

/**
 * @brief Action to do after periodic link check failed
 */
typedef enum {
	LS_ED_REJOIN = 0,		/**< Try to rejoin if link check is failed */
	LS_ED_NOTIFY			/**< Notify application if link check is failed */
} ls_ed_lnkchk_action_t;

/**
 * @brief LoRa-Star stack settings.
 *
 * Could be stored in non-volatile memory
 */
typedef struct {
	uint64_t node_id;							/**< Unique node ID */
	uint64_t app_id;							/**< Unique application ID */

	ls_datarate_t dr;							/**< End-device data rate */
	ls_channel_t channel;						/**< Channel for the end-device */

	ls_crypto_t crypto;							/**< Cryptography settings */

	uint8_t max_retr;							/**< Maximum number of retransmissions */

	uint8_t lnkchk_period_s;					/**< Periodic link check interval is seconds */
	ls_ed_lnkchk_action_t lnkchk_failed_action;	/**< Action to do if link check is failed */

	ls_node_class_t class;						/**< Device class */
	uint64_t ability;							/**< Device abilities set up by the used modules */
} ls_ed_settings_t;

typedef struct {
	sx1276_t *sx1276;		/**< Pointer to the radio PHY structure */

    char sx1276_listener_thread_stack[LS_SX1276_LISTENER_STACKSIZE];	/**< SX1276 events listener thread stack */
    msg_t sx1276_event_queue[16];

    /* Timers for first and second RX windows */
    xtimer_t rx_window1, rx_window2;

    /* In second RX window, use the default settings */
    bool use_rx_window_2_settings;

    /* Uplink frame queue */
    ls_frame_fifo_t uplink_queue;

    char uq_thread_stack[LS_UQ_HANDLER_STACKSIZE];
    kernel_pid_t uq_thread_pid;
    msg_t uq_msg_queue[LS_MAX_FRAME_FIFO_SIZE];

    /* Various timeout timers message handler */
    char tim_thread_stack[LS_TIM_HANDLER_STACKSIZE];
	kernel_pid_t tim_thread_pid;
	msg_t tim_msg_queue[LS_TIM_MSG_QUEUE_SIZE];

	/* Device nonce from the last join procedure */
	uint32_t last_nonce;

	/* Device address assigned by a gate from the last join procedure */
	ls_addr_t dev_addr;

	/* Device is joined to the network */
	bool is_joined;

	/* Join request expiration timer */
	xtimer_t join_req_expired;

	/* Link check request expiration timer */
	xtimer_t lnkchk_expired;

	/* Confirmation timeout */
	xtimer_t conf_ack_expired;

	/* Number of retransmitting tries */
	uint8_t num_retr;

	/* Last application data packet to retransmit after confirmation timeout */
	ls_payload_t last_app_msg;

	/* Last frame ID */
	ls_frame_id_t last_fid;

	/* Timer for the periodic link check */
	xtimer_t lnkchk_timer;

	/* Delay before next wakeup in microseconds */
	uint32_t wakeup_delay;

	/* Wakeup message to send for timeout handling thread */
	msg_t *wakeup_msg;

	/* Sleep request timer */
	xtimer_t sleep_req_timer;

	/* Wakeup timer */
	xtimer_t wakeup_timer;
} ls_ed_internal_t;

/**
 * LoRa-Star stack state
 */
typedef struct {
	ls_ed_settings_t settings;		/**< Network settings, could be stored in EEPROM */
	ls_ed_status_t state;			/**< Current LS stack status */
	ls_device_status_t status;		/**< Device status */

	/* Callback functions */
	void (*joined_cb)(void);
	void (*join_timeout_cb)(void);

	void (*link_good_cb)(void);
	void (*lnkchk_timeout_cb)(void);

	void (*appdata_received_cb)(uint8_t *buf, size_t buflen);
	void (*appdata_send_failed_cb)(void);

	void (*standby_mode_cb)(uint32_t wakeup_after);

	ls_ed_internal_t _internal;	/**< Internal data for the LS stack*/
} ls_ed_t;

int ls_ed_init(ls_ed_t *ls);

int ls_ed_send_app_data(ls_ed_t *ls, uint8_t *buf, size_t buflen, bool confirmed);

int ls_ed_join(ls_ed_t *ls);

void ls_ed_unjoin(ls_ed_t *ls);

void ls_ed_sleep(ls_ed_t *ls, bool lowpower);

void ls_ed_lnkchk(ls_ed_t *ls);

#endif /* UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_H_ */
