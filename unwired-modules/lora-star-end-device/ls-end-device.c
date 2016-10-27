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
 * @file		ls_end_device.c
 * @brief       Implementation of LoRa-Star stack for end-device
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "random.h"
#include "assert.h"
#include "thread.h"
#include "mutex.h"

#include "lpm.h"
#include "periph/rtc.h"

#include "ls-mac-types.h"
#include "ls-mac.h"
#include "ls-end-device.h"

static msg_t msg_rx1;
static msg_t msg_rx2;

static msg_t msg_join_timeout;
static msg_t msg_lnkchk_timeout;

static msg_t msg_ack_timeout;

static msg_t msg_lnkchk_begin;

/**
 * Data rates table.
 */
static uint8_t datarate_table[7][3] = {
    { SX1276_SF12, SX1276_BW_125_KHZ, SX1276_CR_4_5 },       /* DR0 */
    { SX1276_SF11, SX1276_BW_125_KHZ, SX1276_CR_4_5 },       /* DR1 */
    { SX1276_SF10, SX1276_BW_125_KHZ, SX1276_CR_4_5 },       /* DR2 */
    { SX1276_SF9, SX1276_BW_125_KHZ, SX1276_CR_4_5 },        /* DR3 */
    { SX1276_SF8, SX1276_BW_125_KHZ, SX1276_CR_4_5 },        /* DR4 */
    { SX1276_SF7, SX1276_BW_125_KHZ, SX1276_CR_4_5 },        /* DR5 */
    { SX1276_SF7, SX1276_BW_250_KHZ, SX1276_CR_4_5 },        /* DR6 */
};

static void configure_sx1276(ls_ed_t *ls, bool tx)
{
    /* Choose data rate */
    ls_datarate_t dr = (!ls->_internal.use_rx_window_2_settings) ? ls->settings.dr : LS_RX2_DR;
    uint8_t *datarate = datarate_table[dr];

    /* Setup channel */
    ls_channel_t ch = (!ls->_internal.use_rx_window_2_settings) ? ls->settings.channel : LS_RX2_CH;

    sx1276_set_channel(ls->_internal.sx1276, ls->settings.channels_table[ch]);

    /* Setup transceiver settings according to datarate */
    sx1276_lora_settings_t settings;

    settings.datarate = datarate[0];
    settings.bandwidth = datarate[1];
    settings.coderate = datarate[2];

    settings.crc_on = true;
    settings.freq_hop_on = false;
    settings.hop_period = 0;
    settings.implicit_header = false;
    settings.iq_inverted = false;
    settings.low_datarate_optimize = false;
    settings.payload_len = 0;
    settings.power = TX_OUTPUT_POWER;
    settings.preamble_len = LORA_PREAMBLE_LENGTH;
    settings.rx_continuous = true;
    settings.tx_timeout = 1e6 * 30; // 30 sec
    settings.rx_timeout = LORA_SYMBOL_TIMEOUT;

    sx1276_configure_lora(ls->_internal.sx1276, &settings);
}

static void enter_rx(ls_ed_t *ls)
{
    assert(ls != NULL);
    assert(ls->_internal.sx1276 != NULL);

    /* Don't touch anything if we're already in reception mode */
    if (ls->state == LS_ED_LISTENING)
    	return;

    ls->state = LS_ED_LISTENING;

    configure_sx1276(ls, false);
    sx1276_set_rx(ls->_internal.sx1276, ls->_internal.sx1276->settings.lora.rx_timeout);
}

static inline void schedule_tx(ls_ed_t *ls)
{
    msg_t msg;

    /* Send message to the frame queue thread to initiate frame transmission */
    msg_try_send(&msg, ls->_internal.uq_thread_pid);
}

static void open_rx_windows(ls_ed_t *ls)
{
    assert(ls != NULL);

    /* Open 2 RX windows if class A device */
    if (ls->settings.class == LS_ED_CLASS_A) {
        xtimer_set_msg(&ls->_internal.rx_window1, LS_RX_DELAY1, &msg_rx1, ls->_internal.tim_thread_pid);
        xtimer_set_msg(&ls->_internal.rx_window2, LS_RX_DELAY1 + LS_RX_DELAY2, &msg_rx2, ls->_internal.tim_thread_pid);

        /* Enter reception mode */
        enter_rx(ls);
    }
    else if (ls->settings.class == LS_ED_CLASS_B) {
        xtimer_set_msg(&ls->_internal.rx_window1, LS_RX_DELAY1, &msg_rx1, ls->_internal.tim_thread_pid);
        enter_rx(ls);
    }
}

static int send_frame(ls_ed_t *ls, ls_type_t type, uint8_t *buf, size_t buflen)
{
    assert(ls != NULL);

    configure_sx1276(ls, true); /* Configure for TX */

    mutex_lock(&ls->_internal.curr_frame_mutex);

    ls_frame_t *frame = &ls->_internal.current_frame;
    ls_assemble_frame(ls->_internal.dev_addr, type, buf, buflen, frame);

    /* Enqueue frame */
    if (!ls_frame_fifo_push(&ls->_internal.uplink_queue, frame)) {
    	mutex_unlock(&ls->_internal.curr_frame_mutex);
        return -LS_SEND_E_FQ_OVERFLOW;
    }

    /* Schedule RX windows */
    if (ls->settings.class != LS_ED_CLASS_A) {
        open_rx_windows(ls);
    }
    else if (ls->state != LS_ED_LISTENING) {
        schedule_tx(ls);
    }

    mutex_unlock(&ls->_internal.curr_frame_mutex);

    return LS_OK;
}

static void close_rx_windows(ls_ed_t *ls)
{
    assert(ls != NULL);

    /* Clear the default settings flag just in case the second rx window is not expired yet */
    ls->_internal.use_rx_window_2_settings = false;

    puts("ls: both rx windows closed"); // XXX: debug

    /* Remove windows */
    xtimer_remove(&ls->_internal.rx_window1);
    xtimer_remove(&ls->_internal.rx_window2);

    /* Put SX1276 into sleep */
    ls_ed_sleep(ls);

    /* Schedule next transmission */
    schedule_tx(ls);
}

static bool frame_recv(ls_ed_t *ls, ls_frame_t *frame)
{
    switch (frame->header.type) {
        case LS_DL_ACK:                             /* Downlink frame acknowledge for confirmed messages */
            puts("ls-ed: confirmation received");   // XXX: debug

            /* Remove timeout timer */
            xtimer_remove(&ls->_internal.conf_ack_expired);

            /* Remove link check timer */
            xtimer_remove(&ls->_internal.lnkchk_expired);

            return true;

        case LS_DL:         /* Downlink frame */
            /* Must be joined to the network first */
            if (!ls->_internal.is_joined) {
                return false;
            }

            if (!ls_validate_frame_mic(ls->settings.crypto.mic_key, frame)) {
                return false;
            }

            ls_decrypt_frame_payload(ls->settings.crypto.aes_key, &frame->payload);

            /* Send acknowledge */
            send_frame(ls, LS_UL_ACK, NULL, 0);

            /* Notify application about the data */
            if (ls->appdata_received_cb != NULL) {
                ls->appdata_received_cb(frame->payload.data, frame->payload.len);
            }

            return true;

        case LS_DL_JOIN_ACK: /* Downlink join acknowledge */
            if (frame->payload.len != sizeof(ls_join_ack_t)) {
                return false;
            }

            if (!ls_validate_frame_mic(ls->settings.crypto.join_key, frame)) {
                return false;
            }

            ls_decrypt_frame_payload(ls->settings.crypto.join_key, &frame->payload);

            ls_join_ack_t ack;
            memcpy(&ack, frame->payload.data, sizeof(ls_join_ack_t));

            /* This join ack is not for us */
            if (ack.dev_id != ls->settings.node_id) {
                return false;
            }

            /* Setup device address */
            ls->_internal.dev_addr = ack.addr;

            /* This join ack for us, derive encryption keys and save */
            ls_derive_keys(ls->_internal.last_nonce, ack.app_nonce, ack.addr, ls->settings.crypto.mic_key, ls->settings.crypto.aes_key);

            /* Remove timeout timer */
            xtimer_remove(&ls->_internal.join_req_expired);

            /* Make device joined */
            ls->_internal.is_joined = true;

            /* Notify application code via callback */
            if (ls->joined_cb != NULL) {
                ls->joined_cb();
            }

            /* Clear wakeup reason */
            ls->_internal.wakeup_msg = NULL;

            /* Start periodic link check if needed */
            if (ls->settings.lnkchk_period_s != 0) {
            	xtimer_set_msg(&ls->_internal.lnkchk_timer, 1e6 * ls->settings.lnkchk_period_s, &msg_lnkchk_begin, ls->_internal.tim_thread_pid);
            }

            return true;

        case LS_DL_LNKCHK: /* Link check acknowledge */
            /* Remove timeout timer */
            xtimer_remove(&ls->_internal.lnkchk_expired);

            if (ls->link_good_cb != NULL) {
                ls->link_good_cb();
            }

            return true;

        case LS_DL_LNKCHK_P: /* Link check acknowledged with frames pending */
        	puts("ls-ed: waiting for next pending frame");

            /* Remove timeout timer */
        	xtimer_remove(&ls->_internal.lnkchk_expired);

            if (ls->link_good_cb != NULL) {
                ls->link_good_cb();
            }

            /* False because we must wait for another frame which is pending */
            return false;

        default:
        case LS_UL_CONF:        /* Uplink data confirmed */
        case LS_UL_UNC:         /* Uplink data unconfirmed */
        case LS_UL_JOIN_REQ:    /* Join request */
            /* Not interested in frames from other devices */
            return false;
    }
}

static void sx1276_handler(void *arg, sx1276_event_type_t event_type)
{
    assert(arg != NULL);

    sx1276_t *dev = (sx1276_t *) arg;
    ls_ed_t *ls = (ls_ed_t *) dev->callback_arg;

	sx1276_rx_packet_t *packet = (sx1276_rx_packet_t *) &dev->_internal.last_packet;

	switch (event_type) {
		case SX1276_RX_DONE:
			printf("RX: %u bytes, | RSSI: %d\n", packet->size, packet->rssi_value);

			ls_frame_t *frame = (ls_frame_t *) packet->content;

			/* Check frame format */
			if (ls_validate_frame(packet->content, packet->size)) {
				/* Process new frame */
				if (frame_recv(ls, frame)) {
					/* Class A devices closes RX window after each received packet */
					if (ls->settings.class == LS_ED_CLASS_A) {
						close_rx_windows(ls);
					}
				}
			}
			else {
				puts("ls-ed: malformed data discarded"); // XXX: debug

			}
			break;

		case SX1276_RX_ERROR_CRC:
			puts("sx1276: RX CRC failed");
			break;

		case SX1276_TX_DONE:
			puts("sx1276: transmission done.");

			/* Open RX windows after each transmitted packet */
			open_rx_windows(ls);

			break;

		case SX1276_RX_TIMEOUT:
			puts("sx1276: RX timeout");

			close_rx_windows(ls);
			ls_ed_sleep(ls);
			break;

		case SX1276_TX_TIMEOUT:
			puts("sx1276: TX timeout");
			ls_ed_sleep(ls);

			break;

		default:
			printf("sx1276: received event #%d\n", (int) event_type);
			ls_ed_sleep(ls);
			break;
	}
}

/**
 * Uplink frame queue handler thread body.
 */
static void *uq_handler(void *arg)
{
    assert(arg != NULL);

    puts("ls: uplink frame queue handler thread started"); // XXX: debug

    ls_ed_t *ls = (ls_ed_t *) arg;
    msg_init_queue(ls->_internal.uq_msg_queue, sizeof(ls->_internal.uq_msg_queue));
    msg_t msg;

    while (1) {
        msg_receive(&msg);

        if (ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
            ls->state = LS_ED_IDLE;
            continue;
        }

        /* Get frame from queue */
        ls_frame_t *f;
        ls_frame_t frame;
        if (!ls_frame_fifo_pop(&ls->_internal.uplink_queue, &frame)) {
            continue;
        }

        f = &frame;

        /* Wakeup peripherals */
        if (ls->wakeup_cb)
        	ls->wakeup_cb();

        ls->state = LS_ED_TRANSMITTING;

        /* Configure to sleep */
        sx1276_set_sleep(ls->_internal.sx1276);

        /* Configure SX1276 to TX */
        configure_sx1276(ls, true);

        size_t header_size = sizeof(ls_header_t) + sizeof(ls_payload_len_t);
        size_t payload_size = 0;

        frame.header.fid = ls->_internal.last_fid++;

        /* Apply cryptography procedures */
        if (f->header.type != LS_UL_JOIN_REQ) {
            ls_encrypt_frame(ls->settings.crypto.mic_key, ls->settings.crypto.aes_key, f, &payload_size);
        }
        else {
            ls_encrypt_frame(ls->settings.crypto.join_key, ls->settings.crypto.join_key, f, &payload_size);
        }

        // XXX: debug
        printf(">mhdr=0x%02X, mic=0x%04X, addr=0x%02X, type=0x%02X, fid=0x%02X (%d bytes)\n", (unsigned int) f->header.mhdr,
               (unsigned int) f->header.mic, (unsigned int) f->header.dev_addr,
               (unsigned int) f->header.type,
               (unsigned int) f->header.fid, header_size + payload_size);

        /* Send frame into LoRa PHY */
        sx1276_send(ls->_internal.sx1276, (uint8_t *) f, header_size + payload_size);
    }

    return NULL;
}

/**
 * @brief Creates uplink frame queue handler thread.
 */
static bool create_uq_handler_thread(ls_ed_t *ls)
{
    puts("ls_init: creating uplink frame queue handler thread...");

    kernel_pid_t pid_fq = thread_create(ls->_internal.uq_thread_stack, sizeof(ls->_internal.uq_thread_stack), THREAD_PRIORITY_MAIN - 2,
                                        THREAD_CREATE_STACKTEST, uq_handler, ls,
                                        "uplink frame queue handler thread");

    if (pid_fq <= KERNEL_PID_UNDEF) {
        puts("ls_init: creation of uplink rame queue handler thread failed");
        return false;
    }

    ls->_internal.uq_thread_pid = pid_fq;

    return true;
}

static void *tim_handler(void *arg)
{
    assert(arg != NULL);

    ls_ed_t *ls = (ls_ed_t *) arg;
    msg_init_queue(ls->_internal.tim_msg_queue, sizeof(ls->_internal.tim_msg_queue));
    msg_t msg;

    while (1) {
        msg_receive(&msg);

        if (ls->wakeup_cb != NULL)
        	ls->wakeup_cb();

        ls_ed_tim_cmd_t cmd = (ls_ed_tim_cmd_t) msg.content.value;

        switch (cmd) {
            case LS_ED_RX1_EXPIRED:
                puts("ls: first RX window expired"); // XXX: debug

                if (ls->settings.class == LS_ED_CLASS_A) {
                    /* Use default settings for the transceiver in second RX window */
                    ls->_internal.use_rx_window_2_settings = true;

                    /* Enter reception mode */
                    enter_rx(ls);
                }
                else if (ls->settings.class == LS_ED_CLASS_B) {
                    /* Transmit next frame from queue */
                    if (!ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
                    	puts("ls: sending next frame");
                        schedule_tx(ls);
                    }
                    else {
                    	puts("ls: staying in RX mode");
                        enter_rx(ls);
                    }
                }

                break;

            case LS_ED_RX2_EXPIRED:
                puts("ls-ed: second RX window expired"); // XXX: debug

                /* Clear the default settings flag */
                ls->_internal.use_rx_window_2_settings = false;

                /* Transmit next frame from queue */
                if (!ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
                    schedule_tx(ls);
                } else {
					/* Put transceiver into sleep with low power mode */
					ls_ed_sleep(ls);
                }

                break;

            case LS_ED_JOIN_REQ_EXPIRED:
                puts("ls-ed: join request expired"); // XXX: debug

                /* Connection is lost, clear uplink queue */


                /* Notify application about the event */
                if (ls->join_timeout_cb != NULL) {
                    ls->join_timeout_cb();
                }

                break;


            case LS_ED_LNKCHK_REQ_EXPIRED:
                puts("ls-ed: link check request expired"); // XXX: debug

                /* Unjoin the device */
                ls_ed_unjoin(ls);

                /* Select action */
                switch (ls->settings.lnkchk_failed_action) {
                    case LS_ED_REJOIN:
                        ls_ed_join(ls);
                        break;

                    case LS_ED_NOTIFY:
                        if (ls->lnkchk_timeout_cb != NULL) {
                            ls->lnkchk_timeout_cb();
                        }
                        break;
                }

                break;

            case LS_ED_APPDATA_ACK_EXPIRED:
                puts("ls-ed: appdata confirmation timeout");

                /* Retransmit data */
                if (ls->_internal.num_retr > ls->settings.max_retr) {
                    /* Stop retransmitting */
                    ls->_internal.num_retr = 0;

                    if (ls->appdata_send_failed_cb != NULL) {
                        ls->appdata_send_failed_cb();
                    }
                }
                else {
                    /* Do a retransmission */
                    ls->_internal.num_retr++;
                    ls_ed_send_app_data(ls, ls->_internal.last_app_msg.data, ls->_internal.last_app_msg.len, true);
                }

                break;

            case LS_ED_LNKCHK_BEGIN:
                puts("ls-ed: periodic link check begins");

                /* Start link check */
                ls_ed_lnkchk(ls);

                /* Restart periodic link check */
                if (ls->settings.lnkchk_period_s != 0) {
                	xtimer_set_msg(&ls->_internal.lnkchk_timer, 1e6 * ls->settings.lnkchk_period_s, &msg_lnkchk_begin, ls->_internal.tim_thread_pid);
                }

                break;
        }
    }


    return NULL;
}

/**
 * @brief Creates timeout timers message handler thread.
 */
static bool create_tim_handler_thread(ls_ed_t *ls)
{
    puts("ls_init: creating timeouts handler thread...");

    kernel_pid_t pid_tim = thread_create(ls->_internal.tim_thread_stack, sizeof(ls->_internal.tim_thread_stack), THREAD_PRIORITY_MAIN - 2,
                                         THREAD_CREATE_STACKTEST, tim_handler, ls,
                                         "LS timeouts handler thread");

    if (pid_tim <= KERNEL_PID_UNDEF) {
        puts("ls_init: creation of timer handler thread failed");
        return false;
    }

    ls->_internal.tim_thread_pid = pid_tim;

    return true;
}

int ls_ed_init(ls_ed_t *ls)
{
    assert(ls != NULL);
    assert(ls->_internal.sx1276 != NULL);

    ls->_internal.last_fid = 0;
    ls->_internal.num_retr = 0;
    ls->_internal.is_joined = false;
    ls->_internal.dev_addr = LS_ADDR_UNDEFINED;
    ls->_internal.wakeup_msg = NULL;
    ls->_internal.wakeup_delay = 0;

    mutex_init(&ls->_internal.curr_frame_mutex);
    memset(&ls->status, 0, sizeof(ls_device_status_t));

    /* Initialize uplink frame queue */
    ls_frame_fifo_init(&ls->_internal.uplink_queue);

    /* Start threads */
    if (!create_uq_handler_thread(ls)) {
        ls->state = LS_ED_FAULT;
        return -LS_INIT_E_SX1276_THREAD;
    }

    if (!create_tim_handler_thread(ls)) {
        ls->state = LS_ED_FAULT;
        return -LS_INIT_E_TIM_THREAD;
    }

    msg_rx1.content.value = LS_ED_RX1_EXPIRED;
    msg_rx2.content.value = LS_ED_RX2_EXPIRED;
    msg_join_timeout.content.value = LS_ED_JOIN_REQ_EXPIRED;
    msg_lnkchk_timeout.content.value = LS_ED_LNKCHK_REQ_EXPIRED;
    msg_ack_timeout.content.value = LS_ED_APPDATA_ACK_EXPIRED;
    msg_lnkchk_begin.content.value = LS_ED_LNKCHK_BEGIN;

    /* Setup event callback and stack state as it's argument */
    ls->_internal.sx1276->sx1276_event_cb = sx1276_handler;
    ls->_internal.sx1276->callback_arg = ls;

    /* Initialize the transceiver */
    sx1276_init(ls->_internal.sx1276);

    /* Initialize random number generator */
    random_init(sx1276_random(ls->_internal.sx1276));

    ls_ed_sleep(ls);

    return LS_OK;
}

int ls_ed_send_app_data(ls_ed_t *ls, uint8_t *buf, size_t buflen, bool confirmed)
{
    assert(ls != NULL);
    assert(buf != NULL);

    if (ls->wakeup_cb != NULL)
    	ls->wakeup_cb();

    /* Has to be joined to network */
    if (!ls->_internal.is_joined) {
    	if (ls->standby_mode_cb)
    		ls->standby_mode_cb();

    	return -LS_SEND_E_NOT_JOINED;
    }

    int res = send_frame(ls, (confirmed) ? LS_UL_CONF : LS_UL_UNC, buf, buflen);
    if (res < 0) {
        return res;
    }

    if (confirmed) {
        xtimer_set_msg(&ls->_internal.conf_ack_expired, LS_ACK_TIMEOUT, &msg_ack_timeout, ls->_internal.tim_thread_pid);

        /* Save current message for retransmission */
        memcpy(ls->_internal.last_app_msg.data, buf, buflen);
        ls->_internal.last_app_msg.len = buflen;
    }

    return LS_OK;
}

void ls_ed_unjoin(ls_ed_t *ls)
{
    xtimer_remove(&ls->_internal.join_req_expired);

    ls->_internal.dev_addr = LS_ADDR_UNDEFINED;
    ls->_internal.is_joined = false;
}

int ls_ed_join(ls_ed_t *ls)
{
    assert(ls != NULL);

    if (ls->wakeup_cb != NULL)
    	ls->wakeup_cb();

    /* Leave network if we're currently joined */
    ls_ed_unjoin(ls);

    /* Compose a join request */
    ls_join_req_t req;
    req.app_id = ls->settings.app_id;
    req.dev_id = ls->settings.node_id;

    req.node_class = ls->settings.class;
    req.dev_nonce = sx1276_random(ls->_internal.sx1276);

    req.node_ability = ls->settings.ability;

    ls->_internal.last_nonce = req.dev_nonce;

    /* Send request */
    send_frame(ls, LS_UL_JOIN_REQ, (uint8_t *) &req, sizeof(ls_join_req_t));

    /* Launch timeout timer */
    xtimer_set_msg(&ls->_internal.join_req_expired, LS_JOIN_TIMEOUT, &msg_join_timeout, ls->_internal.tim_thread_pid);

    return LS_OK;
}

void ls_ed_lnkchk(ls_ed_t *ls)
{
    assert(ls != NULL);

    /* The device has to be joined */
    if (!ls->_internal.is_joined) {
        return;
    }

    /* Send link check request */
    ls_lnkchk_req_t req;
    memcpy(&req.status, &ls->status, sizeof(ls_device_status_t));

    send_frame(ls, LS_UL_LNKCHK, (uint8_t *) &req, sizeof(ls_lnkchk_req_t));

    /* Launch link check request acknowledge timeout timer */
    xtimer_set_msg(&ls->_internal.lnkchk_expired, LS_LNKCHK_TIMEOUT, &msg_lnkchk_timeout, ls->_internal.tim_thread_pid);
}

void ls_ed_sleep(ls_ed_t *ls)
{
    assert(ls != NULL);

    ls->state = LS_ED_SLEEP;
    sx1276_set_sleep(ls->_internal.sx1276);

    if (ls->standby_mode_cb != NULL)
    	ls->standby_mode_cb();
}

#ifdef __cplusplus
}
#endif
