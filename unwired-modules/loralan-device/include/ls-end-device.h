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

#include "rtctimers.h"

#include "ls-frame-fifo.h"
#include "appdata-fifo.h"
#include "ls-mac-types.h"
#include "ls-crypto.h"

#include "sx1276.h"

#define LS_RX2_DR LS_DR3
#define LS_RX2_CH 0

/**
 * @brief Maximum number of RX window reopens.
 * To prevent class C devices stuck on endless RX reopening while frames awaiting in queue
 */
#define LS_ED_RX_NUM_REOPEN 2

/**
 * @brief Maximum time of awaiting for the confirmed application data acknowledge in seconds.
 */
#define LS_ACK_TIMEOUT	15

/**
 * @brief Maximum time of awaiting for the join request acknowledge in seconds.
 */
#define LS_JOIN_TIMEOUT 	15

/**
 * @brief Duration of the first receive window in seconds.
 */
#define LS_RX_DELAY1		3

/**
 * @brief Duration of the second receive window in seconds.
 */
#define LS_RX_DELAY2		1

/**
 * Sleep request delay
 */
#define LS_ED_SLEEP_REQUEST_DELAY 1

// TODO: optimize these values to reduce memory consumption
#if defined (UNWDS_BUILD_MINIMAL)
    #define LS_UQ_HANDLER_STACKSIZE			(1536)
    #define LS_TIM_HANDLER_STACKSIZE		(768)
    #define LS_TIM_MSG_QUEUE_SIZE 4
#else
    #define LS_UQ_HANDLER_STACKSIZE			(2 * THREAD_STACKSIZE_DEFAULT)
    #define LS_TIM_HANDLER_STACKSIZE		(1 * THREAD_STACKSIZE_DEFAULT)
    #define LS_TIM_MSG_QUEUE_SIZE 8
#endif

typedef enum {
	LS_ED_RX1_EXPIRED = 0,
	LS_ED_RX2_EXPIRED,

	LS_ED_JOIN_REQ_EXPIRED,

	LS_ED_APPDATA_ACK_EXPIRED,
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
	LS_SEND_E_NOT_JOINED,			/**< Not joined to the network */
    LS_SEND_E_FIFO_ERROR,

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
    uint32_t ability[8];						/**< Device abilities set up by the used modules */
	const uint32_t *channels_table;				/**< Table of frequencies [Hz] of possible channels */
	size_t channels_table_size;					/**< Size of the channels table */
    ls_crypto_t crypto;							/**< Cryptography settings */
	ls_datarate_t dr;							/**< End-device data rate */
	ls_channel_t channel;						/**< Channel for the end-device */
	ls_node_class_t class;						/**< Device class */
	uint8_t max_retr;							/**< Maximum number of retransmissions */
	bool no_join;								/**< Statically personalized device, no join required */
    bool auto_shutdown;
} ls_ed_settings_t;

typedef struct {
	sx1276_t *sx1276;		/**< Pointer to the radio PHY structure */

    msg_t sx1276_event_queue[16];

    /* Timers for first and second RX windows */
    rtctimers_t rx_window1, rx_window2;

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
	rtctimers_t join_req_expired;

	/* Confirmation timeout */
	rtctimers_t conf_ack_expired;

	/* Number of retransmitting tries */
	uint8_t num_retr;

	/* Last frame ID */
	ls_frame_id_t last_fid;

	/* Last frame needs to be confirmed
     * Blocking sending of other frames from queue until current frame is confirmed */
	bool confirmation_required;

	/* Current frame to send (to reduce stack consumption) */
	ls_frame_t current_frame;
	mutex_t curr_frame_mutex; /**< Mutex on current frame */

	int16_t last_rssi;		  /**< RSSI value of the last frame received */
    
    bool last_cad_success;     /**< last Channel Activity Detection result */
    
	uint8_t num_reopened;		/**< Number of RX window reopening */

	/*
	 * Since network's uplink queue stores network frames, we should use separate queue of raw app. data
	 * messages to preserve them from losing if network key is changed and encrypted frames are invalid.
	 */
	appdata_fifo_t appdata_fifo; /**< Application data FIFO */
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

	bool (*appdata_received_cb)(uint8_t *buf, size_t buflen);
	void (*appdata_send_failed_cb)(void);

	bool (*broadcast_appdata_received_cb)(uint8_t *buf, size_t buflen);

	ls_ed_internal_t _internal;	/**< Internal data for the LS stack*/
} ls_ed_t;

int ls_ed_init(ls_ed_t *ls);

void ls_ed_poweroff(ls_ed_t *ls);

int ls_ed_send_app_data(ls_ed_t *ls, uint8_t *buf, size_t buflen, bool confirmed, bool with_ack, bool delayed);

int ls_ed_join(ls_ed_t *ls);

void ls_ed_unjoin(ls_ed_t *ls);

void ls_ed_sleep(ls_ed_t *ls);

#endif /* UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_H_ */
