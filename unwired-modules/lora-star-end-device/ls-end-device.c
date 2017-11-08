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
 * @brief       Implementation of LoRaLAN stack for end-device
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
#include "periph/adc.h"

#include "ls-mac-types.h"
#include "ls-mac.h"
#include "ls-end-device.h"
#include "board.h"
#include "rtctimers.h"
#include "rtctimers-millis.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

static msg_t msg_rx1;
static msg_t msg_rx2;

static msg_t msg_join_timeout;
static msg_t msg_ack_timeout;

/**
 * Data rates table.
 */
static const uint8_t datarate_table[7][3] = {
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
    const uint8_t *datarate = datarate_table[dr];

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

static void anticollision_delay(void) {
	/* Pseudorandom delay up to 8 seconds for collision avoidance */
	unsigned int delay = random_uint32_range(1, 8); // XXX: return to original

	DEBUG("[LoRa] random delay %d s\n", (unsigned int) (delay));
	rtctimers_sleep(delay);

	DEBUG("[LoRa] delay end\n");
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
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);
    
    /* Send message to the frame queue thread to initiate frame transmission */
    msg_try_send(&msg, ls->_internal.uq_thread_pid);
}

static void open_rx_windows(ls_ed_t *ls)
{
    assert(ls != NULL);

    /* Open 2 RX windows if class A device */
    if (ls->settings.class == LS_ED_CLASS_A) {
        rtctimers_set_msg(&ls->_internal.rx_window1, LS_RX_DELAY1, &msg_rx1, ls->_internal.tim_thread_pid);
        rtctimers_set_msg(&ls->_internal.rx_window2, LS_RX_DELAY1 + LS_RX_DELAY2, &msg_rx2, ls->_internal.tim_thread_pid);

        /* Enter reception mode */
        enter_rx(ls);
    }

	/* For a while, B and C are the same */
    else if (ls->settings.class == LS_ED_CLASS_B) {
        rtctimers_set_msg(&ls->_internal.rx_window1, LS_RX_DELAY1, &msg_rx1, ls->_internal.tim_thread_pid);
        enter_rx(ls);
    }
	else if (ls->settings.class == LS_ED_CLASS_C) {
        rtctimers_set_msg(&ls->_internal.rx_window1, LS_RX_DELAY1, &msg_rx1, ls->_internal.tim_thread_pid);
        enter_rx(ls);
    }
}

static void send_next(ls_ed_t *ls) {
	ls->_internal.confirmation_required = false;

    /* Schedule RX windows */
    if (ls->settings.class != LS_ED_CLASS_A) {
        open_rx_windows(ls);
    }
    else if (ls->state == LS_ED_IDLE || ls->state == LS_ED_SLEEP) {
    	puts("[LoRa] scheduling TX");
        schedule_tx(ls);
    }
}

static uint8_t get_node_status(void)
{
    if (adc_init(ADC_LINE(ADC_VREF_INDEX)) < 0) {
        return 0;
    }
    uint16_t vdd;
    vdd = adc_sample(ADC_LINE(ADC_VREF_INDEX), ADC_RES_12BIT);
    vdd -= 2000; /* 2000 mV min voltage */
    vdd /= 50; /* 50 mV per bit resolution */
    
    if (adc_init(ADC_LINE(ADC_TEMPERATURE_INDEX)) < 0) {
        return ((uint8_t)vdd & 0x1F);
    }
    int16_t temp;
    temp = (adc_sample(ADC_LINE(ADC_TEMPERATURE_INDEX), ADC_RES_12BIT) + 5) / 10;
    temp += 40; /* -40 is the minimim */
    temp = (temp + 10) / 20; /* 20 deg C per bit */  
    
    return (uint8_t)((vdd & 0x1F) | ((temp & 0x7) << 5));
}

static int send_frame(ls_ed_t *ls, ls_type_t type, uint8_t *buf, size_t buflen)
{
    assert(ls != NULL);

    mutex_lock(&ls->_internal.curr_frame_mutex);

    ls_frame_t *frame = &ls->_internal.current_frame;
    ls_assemble_frame(ls->_internal.dev_addr, type, buf, buflen, frame);

    frame->header.fid = ls->_internal.last_fid;
    frame->header.status = get_node_status();

    /* Enqueue frame */
    if (!ls_frame_fifo_push(&ls->_internal.uplink_queue, frame)) {
    	mutex_unlock(&ls->_internal.curr_frame_mutex);
        return -LS_SEND_E_FQ_OVERFLOW;
    }

    send_next(ls);

    mutex_unlock(&ls->_internal.curr_frame_mutex);

    return LS_OK;
}

static void close_rx_windows(ls_ed_t *ls)
{
    assert(ls != NULL);

    ls->_internal.num_reopened = 0;

    /* Clear the default settings flag just in case the second rx window is not expired yet */
    ls->_internal.use_rx_window_2_settings = false;

    DEBUG("[LoRa] both rx windows closed\n");

    /* Remove windows */
    rtctimers_remove(&ls->_internal.rx_window1);
    rtctimers_remove(&ls->_internal.rx_window2);

    /* Put SX1276 into sleep */
    ls_ed_sleep(ls);

    /* Schedule next transmission */
    schedule_tx(ls);
}

static bool ack_recv(ls_ed_t *ls, ls_frame_t  *frame) {
    if (frame->header.dev_addr != ls->_internal.dev_addr) {
    	return false;
    }

	/* Pop frame from uplink queue */
	ls_frame_fifo_pop(&ls->_internal.uplink_queue, NULL);

	/* Advance frame ID */
	ls->_internal.last_fid++;

	/* Reset number of retries */
	ls->_internal.num_retr = 0;

	puts("[LoRa] confirmation received");   // XXX: debug

	ls->_internal.confirmation_required = false;

	/* Remove timeout timer */
	rtctimers_remove(&ls->_internal.conf_ack_expired);

	/* Close RX window only if we haven't pending frames (regular app. data acknowledge received) */
	bool close_rx_window = frame->header.type == LS_DL_ACK;
	return close_rx_window;
}

static void data_recv(ls_ed_t *ls, ls_frame_t *frame) {
    if (frame->header.dev_addr != ls->_internal.dev_addr) {
    	return;
    }

	/* Notify application about the data */
	if (ls->appdata_received_cb != NULL) {
		/* Send acknowledge if app. data wasn't sent already */
		if (ls->appdata_received_cb(frame->payload.data, frame->payload.len)) {
			send_frame(ls, LS_UL_ACK, NULL, 0);
		}
	}
}

static bool frame_recv(ls_ed_t *ls, ls_frame_t *frame)
{
    switch (frame->header.type) {
    	case LS_DL_BROADCAST: { /* Downlink broadcast message */
    		/* Validate and decipher incoming broadcast message */
            if (!ls_validate_frame_mic(ls->settings.crypto.join_key, frame)) {
                return false;
            }
            ls_decrypt_frame_payload(ls->settings.crypto.join_key, &frame->payload);

            /* Notify application code about incoming data */
            if (ls->broadcast_appdata_received_cb != NULL) {
            	return ls->broadcast_appdata_received_cb(frame->payload.data, frame->payload.len);
            }

            return false;
    	}
    	break;

    	case LS_DL_ACK_W_DATA: /* Acknowledge with additional data */
            /* Must be joined to the network first */
            if (!ls->settings.no_join && !ls->_internal.is_joined) {
                return false;
            }

            if (frame->header.dev_addr != ls->_internal.dev_addr) {
            	return false;
            }

            if (!ls_validate_frame_mic(ls->settings.crypto.mic_key, frame)) {
                return false;
            }
            
            /* Remove app data we've got ACK for from FIFO */
            if (!appdata_fifo_empty(&ls->_internal.appdata_fifo)) {
                DEBUG("[LoRa] remove FIFO entry\n");
                appdata_fifo_pop(&ls->_internal.appdata_fifo, NULL);
            }

            ls_decrypt_frame_payload(ls->settings.crypto.aes_key, &frame->payload);

            bool close_rx_window = ack_recv(ls, frame);
            data_recv(ls, frame);

            return close_rx_window;

    		break;

        case LS_DL_ACK:                             /* Downlink frame acknowledge for confirmed messages */
            if (frame->header.dev_addr != ls->_internal.dev_addr) {
            	return false;
            }

            if (!ls_validate_frame_mic(ls->settings.crypto.mic_key, frame)) {
                return false;
            }
            
            /* Remove app data we've got ACK for from FIFO */
            if (!appdata_fifo_empty(&ls->_internal.appdata_fifo)) {
                DEBUG("[LoRa] remove FIFO entry\n");
                appdata_fifo_pop(&ls->_internal.appdata_fifo, NULL);
            }

            return ack_recv(ls, frame);

        case LS_DL:         /* Downlink frame */
            /* Must be joined to the network first */
            if (!ls->settings.no_join && !ls->_internal.is_joined) {
                return false;
            }

            if (frame->header.dev_addr != ls->_internal.dev_addr) {
            	return false;
            }

            if (!ls_validate_frame_mic(ls->settings.crypto.mic_key, frame)) {
                return false;
            }

            ls_decrypt_frame_payload(ls->settings.crypto.aes_key, &frame->payload);

            data_recv(ls, frame);
            return true;

        case LS_DL_JOIN_ACK: { /* Downlink join acknowledge */
        	/* Joins are disabled */
        	if (ls->settings.no_join)
        		return false;

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
            rtctimers_remove(&ls->_internal.join_req_expired);

            /* Make device joined */
            ls->_internal.is_joined = true;

            /* Notify application code via callback */
            if (ls->joined_cb != NULL) {
                ls->joined_cb();
            }

            /* Check for queued data to send after join */
            appdata_fifo_t *fifo = &ls->_internal.appdata_fifo;
            DEBUG("[LoRa] checking FIFO\n");

    		while(!appdata_fifo_empty(fifo)) {
    			appdata_fifo_entry_t e;

    			if (!appdata_fifo_pop(fifo, &e)) {
    				break;
    			}

    			printf("[LoRa] sending delayed app. data [fid: %d, size: %d]\n", e.id, e.size);

    			ls_ed_send_app_data(ls, e.data, e.size, e.is_confirmed, e.is_with_ack, true);
    		}

            return true;
        }

        case LS_DL_INVITE: { /* Individual join invitation for class C devices */
        	if (ls->settings.class != LS_ED_CLASS_C) /* Only for class C */
        		return false;

        	/* Validate and decrypt frame */
            if (frame->payload.len != sizeof(ls_invite_t)) {
                return false;
            }

            if (!ls_validate_frame_mic(ls->settings.crypto.join_key, frame)) {
                return false;
            }

            ls_decrypt_frame_payload(ls->settings.crypto.join_key, &frame->payload);

            /* Check device ID */
            ls_invite_t ack;
            memcpy(&ack, frame->payload.data, sizeof(ls_invite_t));

            /* This is not for us */
            if (ack.dev_id != ls->settings.node_id) {
                return false;
            }

            puts("[LoRa] invited to join, rejoining..."); // XXX: debug

            /* Stop rejoin timeout */
            rtctimers_remove(&ls->_internal.join_req_expired);

			anticollision_delay();

            /* Proceed to join procedure as requested */
            ls_ed_join(ls);
        	return false;
        }

        default:
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

			/* Save RSSI value of received frame */
			ls->_internal.last_rssi = packet->rssi_value;

			ls_frame_t *frame = (ls_frame_t *) packet->content;

			/* Check frame format */
			if (ls_validate_frame(packet->content, packet->size)) {
				/* Process new frame */
				if (frame_recv(ls, frame)) {
					/* Class A devices closes RX window after each received packet */
					if (ls->settings.class == LS_ED_CLASS_A) {
						close_rx_windows(ls);
					}
				} else {
					if (ls->_internal.num_reopened++ < LS_ED_RX_NUM_REOPEN) {
						puts("[LoRa] first RX window reopened");

						/* Reopen RX window */
						rtctimers_remove(&ls->_internal.rx_window1);
						rtctimers_remove(&ls->_internal.rx_window2);

						open_rx_windows(ls);
					} else {
						puts("[LoRa] forcing RX window expiring");

						ls->_internal.num_reopened = 0;
						rtctimers_remove(&ls->_internal.rx_window1);
						rtctimers_remove(&ls->_internal.rx_window2);

						/* Notify timeouts thread about RX window expiration */
						msg_send(&msg_rx1, ls->_internal.tim_thread_pid);
					}
				}
			}
			else {
				puts("[LoRa] malformed data discarded"); // XXX: debug
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
            
        case SX1276_CAD_DONE:
            DEBUG("sx1276: CAD done\n");
            ls->_internal.last_cad_success = dev->_internal.is_last_cad_success;
            ls_ed_sleep(ls);
            break;
            
        case SX1276_CAD_DETECTED:
            DEBUG("sx1276: CAD detected\n");
            ls->_internal.last_cad_success = dev->_internal.is_last_cad_success;
            ls_ed_sleep(ls);
            break;

		default:
			printf("sx1276: received event #%d\n", (int) event_type);
			ls_ed_sleep(ls);
			break;
	}
}

static void get_type_str(ls_type_t type, char *str) {
	switch (type) {
	case LS_UL_ACK:		/**< Uplink acknowledge from the end device */
		strcpy(str, "ACK");
		break;

	case LS_UL_CONF:		/**< Uplink application data confirmed */
		strcpy(str, "CONF");
		break;


	case LS_UL_UNC:		/**< Uplink application data unconfirmed */
		strcpy(str, "UNC");
		break;

	case LS_UL_UNC_ACK:	/**< Uplink application data unconfirmed with ACK for previous app. data */
		strcpy(str, "UNC_ACK");
		break;

	case LS_UL_JOIN_REQ:	/**< Join request */
		strcpy(str, "JOIN_REQ");
		break;

	default:
		strcpy(str, "?");
		break;
	}
}

/**
 * Uplink frame queue handler thread body.
 */
static void *uq_handler(void *arg)
{
    assert(arg != NULL);

    puts("[LoRa] uplink frame queue handler thread started"); // XXX: debug

    ls_ed_t *ls = (ls_ed_t *) arg;
    msg_init_queue(ls->_internal.uq_msg_queue, sizeof(ls->_internal.uq_msg_queue));
    msg_t msg;

    while (1) {
        msg_receive(&msg);

        if (ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
            ls->state = LS_ED_IDLE;
            continue;
        }
        
        int delay_ms = 5 + ((100 + 10*ls->settings.dr) >> ls->settings.dr);
        /* REG_LR_MODEMSTAT doesn't seems to work properly
        int total_time = 0;
        sx1276_modem_status_t status;
        while ((status = sx1276_get_modem_status(ls->_internal.sx1276)) != SX1276_MODEM_CLEAR) {
            DEBUG("[LoRa] modem status is %u, postpone transmission\n", (uint8_t) status);
            rtctimers_millis_sleep(delay_ms);
            total_time += delay_ms;
            if (total_time > 2000) {
                DEBUG("[LoRa] force switching from RX to TX\n");
                break;
            }
        }
        */
        
        /* Listen Before Talk with LoRa CAD support */
        DEBUG("[LoRa] checking channel activity\n");
        int cad_tries = 0;
        for (int k = 0; k < 10; k++) {
            sx1276_start_cad(ls->_internal.sx1276, SX1276_MODE_CADDONE);
            rtctimers_millis_sleep(delay_ms);
            if (ls->_internal.last_cad_success) {
                DEBUG("[LoRa] channel activity detected\n");
                
                /* send anyway if we tried too many times */
                cad_tries++;
                if (cad_tries > 5) {
                    break;
                }
                
                /* otherwise restart CAD after a pause */
                rtctimers_millis_sleep(5*delay_ms);
                k = 0;
            }
        }
        DEBUG("[LoRa] sending frame\n");

        /* Get frame from queue top */
        ls_frame_t *f;
        ls_frame_t frame;
        if (!ls_frame_fifo_peek(&ls->_internal.uplink_queue, &frame)) {
            continue;
        }

        f = &frame;

        ls->_internal.confirmation_required = (f->header.type == LS_UL_CONF);

        /* Current frame is not confirmed app. data */
        if (!ls->_internal.confirmation_required) {
        	/* Remove frame from queue */
        	ls_frame_fifo_pop(&ls->_internal.uplink_queue, NULL);

        	/* Update frame's FID to the last one and advance it */
        	f->header.fid = ls->_internal.last_fid++;
        } else {
            /* Update frame's FID to the last one */
            f->header.fid = ls->_internal.last_fid;

            /* Start retransmission timer */
            rtctimers_set_msg(&ls->_internal.conf_ack_expired, LS_ACK_TIMEOUT, &msg_ack_timeout, ls->_internal.tim_thread_pid);
        }

        ls->state = LS_ED_TRANSMITTING;

        /* Configure to sleep */
        sx1276_set_sleep(ls->_internal.sx1276);

        /* Configure SX1276 to TX */
        configure_sx1276(ls, true);

        size_t header_size = sizeof(ls_header_t) + sizeof(ls_payload_len_t);
        size_t payload_size = 0;

        /* Apply cryptography procedures */
        if (f->header.type != LS_UL_JOIN_REQ) {
            ls_encrypt_frame(ls->settings.crypto.mic_key, ls->settings.crypto.aes_key, f, &payload_size);
        }
        else {
            ls_encrypt_frame(ls->settings.crypto.join_key, ls->settings.crypto.join_key, f, &payload_size);
        }

        // XXX: debug
        char type_str[10] = {};
        get_type_str(f->header.type, type_str);

#if ENABLE_DEBUG
        printf(">mhdr=0x%02X, mic=0x%04X, addr=0x%02X, <%s> fid=0x%02X (%d bytes) [%d left]\n", (unsigned int) f->header.mhdr,
               (unsigned int) f->header.mic, (unsigned int) f->header.dev_addr,
               type_str,
               (unsigned int) f->header.fid, header_size + payload_size,
			   ls_frame_fifo_size(&ls->_internal.uplink_queue));
#endif
        /* Configure for TX */
        configure_sx1276(ls, true);

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
                                        "uplink queue thread");

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

        ls_ed_tim_cmd_t cmd = (ls_ed_tim_cmd_t) msg.content.value;

        switch (cmd) {
            case LS_ED_RX1_EXPIRED:
                DEBUG("[LoRa] first RX window expired\n");

                if (ls->settings.class == LS_ED_CLASS_A) {
                    /* Use default settings for the transceiver in second RX window */
                    ls->_internal.use_rx_window_2_settings = true;

                    /* Enter reception mode */
                    enter_rx(ls);
                }
				// for a while, classes B and C are the same
                else if (ls->settings.class == LS_ED_CLASS_B) {
                    /* Transmit next frame from queue */
                    if (!ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
                    	puts("[LoRa] sending next frame");
                    	schedule_tx(ls);
                    }
                    else {
                    	puts("[LoRa] staying in RX mode");
                        enter_rx(ls);
                    }
				}
				else if (ls->settings.class == LS_ED_CLASS_C) {
                    /* Transmit next frame from queue */
                    if (!ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
                    	/* If current frame in a head of a queue doesn't awaiting confirmation, schedule it for sending
                    	 * Otherwise, current frame will be retransmitted after confirmation timeout
                    	 */
                    	if (!ls->_internal.confirmation_required)
                    		schedule_tx(ls);
                    	else {
                    		enter_rx(ls);
                    		DEBUG("[LoRa] awaiting confirmation\n");
                    	}
                    }
                    else {
                    	puts("[LoRa] staying in RX mode");
                        enter_rx(ls);
                    }
                }

                break;

            case LS_ED_RX2_EXPIRED:
                DEBUG("[LoRa] second RX window expired\n"); // XXX: debug

                /* Clear the default settings flag */
                ls->_internal.use_rx_window_2_settings = false;

                /* Transmit next frame from queue */
                if (!ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
                	/* If current frame in a head of a queue doesn't awaiting confirmation, schedule it for sending
                	 * Otherwise, current frame will be retransmitted after confirmation timeout
                	 */
                	if (!ls->_internal.confirmation_required)
                		schedule_tx(ls);
                	else
                		puts("[LoRa] awaiting confirmation");
                } else {
					/* Put transceiver into sleep with low power mode */
					ls_ed_sleep(ls);
                }

                break;

            case LS_ED_JOIN_REQ_EXPIRED:
                puts("[LoRa] join request expired"); // XXX: debug

                /* Connection is lost, clear uplink queue */


                /* Notify application about the event */
                if (ls->join_timeout_cb != NULL) {
                    ls->join_timeout_cb();
                }

                break;

            case LS_ED_APPDATA_ACK_EXPIRED:
                puts("[LoRa] appdata confirmation timeout");

                /* Retransmit data */
                if (ls->_internal.num_retr >= ls->settings.max_retr) {
                    /* Stop retransmitting */
                    ls->_internal.num_retr = 0;

                    if (ls->appdata_send_failed_cb != NULL) {
                        ls->appdata_send_failed_cb();
                    }
                }
                else {
                    /* Do a retransmission */
                	anticollision_delay();
                    ls->_internal.num_retr++;
                    ls->state = LS_ED_IDLE;

                    send_next(ls);
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
                                         "LS timeouts thread");

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

    if (!ls->settings.no_join)
    	ls->_internal.dev_addr = LS_ADDR_UNDEFINED;

    mutex_init(&ls->_internal.curr_frame_mutex);
    memset(&ls->status, 0, sizeof(ls_device_status_t));

    /* Initialize appdata queue */
    appdata_fifo_init(&ls->_internal.appdata_fifo);

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
    msg_ack_timeout.content.value = LS_ED_APPDATA_ACK_EXPIRED;

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

void ls_ed_poweroff(ls_ed_t *ls) {
    sx1276_set_sleep(ls->_internal.sx1276);
}

int ls_ed_send_app_data(ls_ed_t *ls, uint8_t *buf, size_t buflen, bool confirmed, bool with_ack, bool delayed)
{
    assert(ls != NULL);
    assert(buf != NULL);

    
    /* Store current app. data in FIFO buffer
     * Data will be removed on receiving ACK from gate */
    if (!delayed && ((confirmed) || (!ls->settings.no_join && !ls->_internal.is_joined))) {
        DEBUG("[LoRa] pushing data to FIFO\n");
        appdata_fifo_t *fifo = &ls->_internal.appdata_fifo;

        /* Last data has priority, so we can pop oldest item from the queue if it's full */
        if (appdata_fifo_full(fifo)) {
            appdata_fifo_pop(fifo, NULL);
        }

        appdata_fifo_push(fifo, buf, buflen, ls->_internal.last_fid, confirmed, with_ack);
        
        /* Not joined to the network, delay appdata frame until device is joined */    
        if (!confirmed) {
            puts("[LoRa] data delayed until node is joined");
            return -LS_SEND_E_NOT_JOINED;
        }
    }

    ls->_internal.confirmation_required = false;

    ls_type_t type = (with_ack) ? LS_UL_UNC_ACK : (confirmed) ? LS_UL_CONF : LS_UL_UNC;

    int res = send_frame(ls, type, buf, buflen);
    if (res < 0) {
        return res;
    }

    return LS_OK;
}

void ls_ed_unjoin(ls_ed_t *ls)
{
	if (ls->settings.no_join)
		return;

    /* Clear uplink queue */
    ls_frame_fifo_clear(&ls->_internal.uplink_queue);
    ls->_internal.confirmation_required = false;

	/* Stop timers */
    rtctimers_remove(&ls->_internal.join_req_expired);
    rtctimers_remove(&ls->_internal.conf_ack_expired);
    rtctimers_remove(&ls->_internal.rx_window1);
    rtctimers_remove(&ls->_internal.rx_window2);

    /* Forget network address */
    ls->_internal.dev_addr = LS_ADDR_UNDEFINED;

    /* Forget session cryptographic keys */
    memset(ls->settings.crypto.aes_key, 0, AES_KEY_SIZE);
    memset(ls->settings.crypto.mic_key, 0, AES_KEY_SIZE);

    /* Mark as not joined */
    ls->_internal.is_joined = false;
}

int ls_ed_join(ls_ed_t *ls)
{
    assert(ls != NULL);

    /* Enter IDLE state */
    ls->state = LS_ED_IDLE;

    /* Leave network if we're currently joined */
    ls_ed_unjoin(ls);

    /* Compose a join request */
    ls_join_req_t req;
    req.app_id = ls->settings.app_id;
    req.dev_id = ls->settings.node_id;

    req.node_class = ls->settings.class;
    
    /* nonce must not be 0 */
    do {
        req.dev_nonce = sx1276_random(ls->_internal.sx1276);
    } while (req.dev_nonce == 0);

    ls->_internal.last_nonce = req.dev_nonce;

    /* Reset frame ID */
    ls->_internal.last_fid = 0;

    /* Send request */
    send_frame(ls, LS_UL_JOIN_REQ, (uint8_t *) &req, sizeof(ls_join_req_t));

    /* Launch timeout timer */
    rtctimers_set_msg(&ls->_internal.join_req_expired, LS_JOIN_TIMEOUT, &msg_join_timeout, ls->_internal.tim_thread_pid);

    return LS_OK;
}

void ls_ed_sleep(ls_ed_t *ls)
{
    assert(ls != NULL);

    if (ls->settings.class == LS_ED_CLASS_A) {
        DEBUG("[LoRa] Class A device, sleeping\n");
        ls->state = LS_ED_SLEEP;
        sx1276_set_sleep(ls->_internal.sx1276);
    } else {
        DEBUG("[LoRa] Class C device, not sleeping\n");
    }
}

#ifdef __cplusplus
}
#endif
