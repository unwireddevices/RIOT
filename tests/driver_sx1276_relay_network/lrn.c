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
 * @file        lrn.c
 * @brief       LoRa Relay Network stack implementation
 * @author      Unwired Devices
 */
#include <stdlib.h>

#include "thread.h"
#include "random.h"

#include "lrn.h"
#include "lrn_mac.h"
#include "lrn_routing.h"
#include "lrn_frame_fifo.h"

#include "lrn_crypto.h"

#include "sx1276.h"

#ifdef __cplusplus
extern "C" {
#endif

static uint8_t datarate_table[7][3] = {
    { SF12, BW_125_KHZ, CR_4_5 },       /* DR0 */
    { SF11, BW_125_KHZ, CR_4_5 },       /* DR1 */
    { SF10, BW_125_KHZ, CR_4_5 },       /* DR2 */
    { SF9, BW_125_KHZ, CR_4_5 },        /* DR3 */
    { SF8, BW_125_KHZ, CR_4_5 },        /* DR4 */
    { SF7, BW_125_KHZ, CR_4_5 },        /* DR5 */
    { SF7, BW_250_KHZ, CR_4_5 },        /* DR6 */
};

static void configure_sx1276(lrn_t *lrn, bool tx);

static inline void _schedule_tx(lrn_t *lrn)
{
    msg_t msg;

    /* Send message to the frame queue thread to initiate frame transmission */
    msg_try_send(&msg, lrn->_uq_thread_pid);
}

/**
 * Puts frame into uplink queue.
 */
static void _enqueue_frame(lrn_t *lrn, lrn_frame_t *frame)
{
    /* Push new frame into queue */
    bool was_empty = lrn_frame_fifo_empty(&lrn->_uplink_queue);

    if (lrn_frame_fifo_push(&lrn->_uplink_queue, frame)) {
        if (was_empty) {
            _schedule_tx(lrn);
        }
    }
    else {
        puts("lrn: frame queue overflow!"); // XXX: debug
    }
}

/**
 * Frame acknowledge timeout expired.
 */
void _frame_expired_cb(void *arg, lrn_frame_t *f)
{
    puts("lrn: timeout. Retransmitting!");
    lrn_t *lrn = (lrn_t *)arg;
    _enqueue_frame(lrn, f);
}

/**
 * @brief handles received frame.
 */
static void _frame_recv(lrn_t *lrn, lrn_frame_t *frame)
{
    switch (frame->header.type) {
        case LRN_TYPE_CMD:
            /* Call the application code */
            lrn->cmd_recv_cb(frame->header.src_addr, frame->payload.data, frame->payload.len);
            break;

        case LRN_TYPE_DATA:
            /* Call the application code */
            lrn->data_recv_cb(frame->header.src_addr, frame->payload.data, frame->payload.len);
            break;

        case LRN_JOIN_REQ:
        	if (lrn->is_gateway) {
        		/* Join request is accepted by a gateway, send a routing table and other network information to it */
        		lrn_join_ack_t ack;
        		ack.table_size = lrn->routing_table_size;

        		/* Copy routing table to the frame's payload */
        		memcpy(ack.table, lrn->routing_table, sizeof(ack.table));

        		lrn_send(lrn, lrn->dev_addr, frame->header.src_addr, LRN_JOIN_ACK, frame->header.fid, (uint8_t *) &ack, sizeof(ack));
        	}
            break;

        case LRN_JOIN_ACK:
        	/* Gateway don't care about join accepts */
        	if (lrn->is_gateway)
        		break;

        	lrn_join_ack_t *ack = (lrn_join_ack_t *) &frame->payload.data;

        	lrn->routing_table_size = ack->table_size;
        	lrn->routing_table = malloc(sizeof(lrn_routing_table_entry_t) * ack->table_size);
        	memcpy(lrn->routing_table, ack->table, sizeof(lrn_routing_table_entry_t) * ack->table_size);

        	lrn->joined = true;

        	if (lrn->joined_cb != NULL)
        		lrn->joined_cb(lrn->routing_table_size);

            break;

        case LRN_NH_ACK:
            puts("lrn: NH ACK received");
            lrn_vfl_remove(&lrn->_vfl, frame->header.fid, true);
            break;

        case LRN_DST_ACK:
            puts("lrn: DST ACK received");
            lrn_vfl_remove(&lrn->_vfl, frame->header.fid, false);
            break;

    }
}

static inline void _send_ack(lrn_t *lrn, lrn_addr_t dest, lrn_type_t type, lrn_frame_id_t fid)
{
    lrn_send(lrn, lrn->dev_addr, dest, type, fid, NULL, 0);
}

static void *frame_queue_handler(void *arg)
{
    puts("lrn: uplink frame queue handler thread started"); // XXX: debug

    lrn_t *lrn = (lrn_t *) arg;
    msg_init_queue(lrn->_uq_msg_queue, sizeof(lrn->_uq_msg_queue));
    msg_t msg;
    while (1) {
        msg_receive(&msg);

        if (lrn_frame_fifo_empty(&lrn->_uplink_queue)) {
            continue;
        }

        uint32_t millis = random_uint32_range(LRN_TX_DELAY_MIN, LRN_TX_DELAY_MAX);
        printf("lrn: tx delay: %d millis\n", (unsigned int) millis); // XXX: debug

        xtimer_usleep(1e3 * millis);

        /* Sleeping while channel is occupied */
        while (!sx1276_is_channel_free(lrn->_sx1276, lrn->_sx1276->settings.channel, LRN_CHANNEL_FREE_RSSI)) {
            puts("channel is occupied!"); // XXX: debug

            millis = random_uint32_range(LRN_TX_DELAY_MIN, LRN_TX_DELAY_MAX);
            printf("lrn: tx delay: %d millis\n", (unsigned int) millis); // XXX: debug
            xtimer_usleep(1e3 * millis);
        }

        /* Get frame from queue */
        lrn_frame_t *f;
        lrn_frame_t frame;
        if (!lrn_frame_fifo_pop(&lrn->_uplink_queue, &frame)) {
            continue;
        }

        f = &frame;

        /* Check the previous state */
        if (lrn->state == LRN_LISTENING || lrn->state == LRN_TRANSMITTING_AFTER_LISTENING) {
            lrn->state = LRN_TRANSMITTING_AFTER_LISTENING;
        }
        else {
            lrn->state = LRN_TRANSMITTING;
        }

        /* Add frame to VFL */
        if (f->header.type != LRN_NH_ACK) {
            /* Always wait for the NH ACK from the recipient */
            lrn_vfl_add(&lrn->_vfl, f, true, false);

            /* If the frame that we sending is not DST ACK and we're not retransmitting, wait for DST ACK from the recipient */
            if (f->header.type != LRN_DST_ACK && f->header.src_addr == lrn->dev_addr) {
                lrn_vfl_add(&lrn->_vfl, f, false, true);
            }
        }

        // XXX: debug
        printf("mhdr=0x%02X, mic=0x%04X, 0x%02X <- 0x%02X <- 0x%02X, type=0x%02X, fid=0x%02X\n", (unsigned int) f->header.mhdr,
               (unsigned int) f->header.mic,
               (unsigned int) f->header.dest_addr, (unsigned int) f->header.next_hop, (unsigned int) f->header.src_addr,
               (unsigned int) f->header.type, (unsigned int) f->header.fid);

        /* Configure to sleep */
        sx1276_set_sleep(lrn->_sx1276);

        /* Configure SX1276 with current TX datarate */
        configure_sx1276(lrn, true);

        uint8_t header_size = sizeof(lrn_header_t) + sizeof(lrn_payload_len_t);
        uint8_t payload_size = encrypt_frame_payload(lrn, &f->payload);

        if (f->payload.len > 0) /* An AES-CBC IV is appended to non-empty payload, so increase the size correspondingly */
        	payload_size += AES_BLOCK_SIZE;

        f->header.mic = calculate_mic(lrn, &frame, payload_size); /* Calculate frame's message integrity/authority check */

        sx1276_send(lrn->_sx1276, (uint8_t *) f, header_size + payload_size);
    }

    return NULL;
}

static void *event_handler_thread(void *arg)
{
    puts("lrn: sx1276 event handler thread started");

    lrn_t *lrn = (lrn_t *) arg;

    msg_init_queue(lrn->_sx1276_event_queue, sizeof(lrn->_sx1276_event_queue));
    msg_t msg;

    while (1) {
        msg_receive(&msg);

        sx1276_event_t *event = (sx1276_event_t *) msg.content.ptr;
        sx1276_rx_packet_t *packet = (sx1276_rx_packet_t *) event->event_data;

        switch (event->type) {
            case RX_DONE:

                printf("RX: %u bytes, | RSSI: %d\n", packet->size, packet->rssi_value);

                /* Copy packet's data as a frame to our stack */
                lrn_frame_t frame;
                memcpy(&frame, packet->content, packet->size);

                /* It's necessary to free the memory for the content because it was allocated dynamically in the sx1276 library */
                free(packet->content);

                /* Validate frame */
                if (check_frame(lrn, (uint8_t *) packet->content, packet->size)) {

                	/* Decrypt frame payload */
                	decrypt_frame_payload(lrn, &frame.payload);

                    /* It's broadcast frame or we're next hop for this frame */
                    if (frame.header.next_hop == lrn->dev_addr
                        || frame.header.next_hop == LRN_ADDR_UNDEFINED) {

                        /* We're destination point for this frame, so digest it */
                        if (frame.header.dest_addr == lrn->dev_addr) {
                            _frame_recv(lrn, &frame);

                            /* We don't want to acknowledge the acknowledge that is for us */
                            if (frame.header.type != LRN_NH_ACK && frame.header.type != LRN_DST_ACK) {
                                puts("lrn: sending dest. ACK"); // XXX: debug

                                /* Send destination acknowledge */
                                _send_ack(lrn, frame.header.src_addr, LRN_DST_ACK, frame.header.fid);
                            }

                            /* Send next hop acknowledge */
                            if (frame.header.next_hop == lrn->dev_addr && frame.header.type != LRN_NH_ACK) {
                                _send_ack(lrn, (frame.header.retr_src == LRN_ADDR_UNDEFINED)
                                          ? frame.header.src_addr
                                          : frame.header.retr_src, LRN_NH_ACK, frame.header.fid);
                            }
                        }
                        else { /* Retransmit frame further if we're not a destination point for it */
                            /* It's join request, retransmit it to the gateway if we're closer to it */
                            if (frame.header.dest_addr == LRN_ADDR_UNDEFINED && frame.header.type == LRN_JOIN_REQ) {
                            	if (lrn->is_gateway && lrn_routing_nohops(lrn->routing_table, lrn->routing_table_size, lrn->dev_addr, frame.header.src_addr) == 1) {
                            		_frame_recv(lrn, &frame);
                            		break;
                            	}

                            	frame.header.dest_addr = lrn_routing_find_gateway_addr(lrn->routing_table, lrn->routing_table_size, lrn->dev_addr, frame.header.src_addr);

								/* Don't send nothing */
								if (frame.header.dest_addr == LRN_ADDR_UNDEFINED)
									break;
                            }

                            puts("lrn: retransmitting...");

                            /* Send frame */
                            lrn_send(lrn, frame.header.src_addr, frame.header.dest_addr, frame.header.type, frame.header.fid, frame.payload.data, frame.payload.len);

                            /* Send ACK to the sender */
                            puts("lrn: sending hop ACK"); // XXX: debug
                            _send_ack(lrn, (frame.header.retr_src == LRN_ADDR_UNDEFINED)
                                      ? frame.header.src_addr
                                      : frame.header.retr_src, LRN_NH_ACK, frame.header.fid);
                        }
                    }
                    else {
                        puts("lrn: valid frame discarded."); // XXX: debug
                    }

                    break;
                }
                else {
                    puts("lrn: frame discarded."); // XXX: debug
                }
                break;

            case RX_ERROR_CRC:
                puts("sx1276: RX CRC failed");
                break;

            case TX_DONE:
                puts("sx1276: transmission done.");

                /* Frame queue is not empty, send next frame */
                if (!lrn_frame_fifo_empty(&lrn->_uplink_queue)) {
                    _schedule_tx(lrn);
                }

                if (lrn->state == LRN_TRANSMITTING_AFTER_LISTENING) {
                    puts("lrn: returning into listening...");

                    /* Back into listening */
                    lrn_listen(lrn);
                }
                else {
                    /* Back to sleep */
                    lrn_sleep(lrn);
                }

                break;

            case RX_TIMEOUT:
                puts("sx1276: RX timeout");
                break;

            case TX_TIMEOUT:
                puts("sx1276: TX timeout");
                break;

            default:
                printf("sx1276: received event #%d\n", (int) event->type);
                break;
        }
    }

    return NULL;
}

static void configure_sx1276(lrn_t *lrn, bool tx)
{
    lrn_datarate_t dr = (tx) ? lrn->datarate_tx : lrn->datarate_rx;
    uint8_t *datarate = datarate_table[dr];

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

    sx1276_configure_lora(lrn->_sx1276, &settings);
}

void lrn_init(lrn_t *lrn)
{
    if (lrn == NULL) {
        return;
    }

    if (lrn->_sx1276 == NULL) {
        return;
    }

    /* Init uplink frame queue */
    lrn_frame_fifo_init(&lrn->_uplink_queue);

    /* Create uplink frame queue handler thread */
    puts("lrn_init: uplink frame queue handler thread...");

    kernel_pid_t pid_fq = thread_create(lrn->_uq_thread_stack, sizeof(lrn->_uq_thread_stack), THREAD_PRIORITY_MAIN - 2,
                                        THREAD_CREATE_STACKTEST, frame_queue_handler, lrn,
                                        "uplink frame queue handler thread");

    if (pid_fq <= KERNEL_PID_UNDEF) {
        puts("lrn_init: creation of uplink rame queue handler thread failed");
        return;
    }

    lrn->_uq_thread_pid = pid_fq;

    /* Initialize downlink queue */
    //lrn_frame_fifo_init(&lrn->_downlink_queue);

    // TODO: create downlink queue handler

    /* Create sx1276 layer events listener thread */
    puts("lrn_init: creating sx1276 event listener...");

    kernel_pid_t pid = thread_create(lrn->_sx1276_listener_thread_stack, sizeof(lrn->_sx1276_listener_thread_stack), THREAD_PRIORITY_MAIN - 1,
                                     THREAD_CREATE_STACKTEST, event_handler_thread, lrn,
                                     "sx1276 event handler thread");

    if (pid <= KERNEL_PID_UNDEF) {
        puts("lrn_init: creation of sx1276 event listener thread failed");
        return;
    }

    lrn->_sx1276->event_handler_thread_pid = pid;

    /* Initialize frame victim list */
    lrn->_last_fid = 0;

    lrn->_vfl.frame_ack_expired_cb = _frame_expired_cb;
    lrn->_vfl.frame_ack_expired_cb_arg = lrn;

    lrn->_vfl.timer = lrn->_vfl_timer;
    lrn_vfl_init(&lrn->_vfl);

    //lrn->rx_window_timer.arg = lrn;
    //lrn->rx_window_timer.callback =


    /* Initialize the transceiver */
    sx1276_init(lrn->_sx1276);

    /* Initialize random number generator */
    random_init(sx1276_random(lrn->_sx1276));

    lrn_sleep(lrn);
}

void lrn_join(lrn_t *lrn) {
	/* Send join request */
	lrn_send(lrn, lrn->dev_addr, LRN_ADDR_UNDEFINED, LRN_JOIN_REQ, lrn->_last_fid, (uint8_t *) NULL, 0);
}

void lrn_send(lrn_t *lrn, lrn_addr_t src, lrn_addr_t dest, lrn_type_t type, lrn_frame_id_t fid, uint8_t *buf, uint8_t buflen)
{
    lrn_routing_table_entry_t e = lrn_routing_next_hop(lrn->routing_table, lrn->routing_table_size, lrn->dev_addr, dest);
    lrn_addr_t next_hop = e.addr;

    lrn_frame_t frame = prepare_frame(lrn, dest, src, next_hop, type, buf, buflen);

    frame.header.fid = fid;

    /* Setup retransmission source if we're retransmitting this frame from other node */
    if (src != lrn->dev_addr) {
        frame.header.retr_src = lrn->dev_addr;
    }
    else {
        frame.header.retr_src = LRN_ADDR_UNDEFINED; /* Not retransmitting */

    }
    /* Check the previous state */
    if (lrn->state == LRN_LISTENING || lrn->state == LRN_TRANSMITTING_AFTER_LISTENING) {
        lrn->state = LRN_TRANSMITTING_AFTER_LISTENING;
    }
    else {
        lrn->state = LRN_TRANSMITTING;
    }

    _enqueue_frame(lrn, &frame);
}

void lrn_listen(lrn_t *lrn)
{
    if (lrn == NULL) {
        return;
    }

    lrn->state = LRN_LISTENING;

    configure_sx1276(lrn, false);
    sx1276_set_rx(lrn->_sx1276, lrn->_sx1276->settings.lora.rx_timeout);
}

void lrn_sleep(lrn_t *lrn)
{
    if (lrn == NULL) {
        return;
    }

    lrn->state = LRN_SLEEP;
    sx1276_set_sleep(lrn->_sx1276);
}

#ifdef __cplusplus
}
#endif
