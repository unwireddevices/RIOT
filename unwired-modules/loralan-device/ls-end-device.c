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

#include "ls-init-device.h"
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

static void configure_sx1276(ls_ed_t *ls, bool tx)
{
    ls_datarate_t dr = (!ls->_internal.use_rx_window_2_settings) ? ls->settings.dr : LS_RX2_DR;
    ls_channel_t ch = (!ls->_internal.use_rx_window_2_settings) ? ls->settings.channel : LS_RX2_CH;
    
    ls_setup_sx1276(ls->_internal.sx1276, dr, ls->settings.channels_table[ch]);
    
    DEBUG("[LoRa] configure_sx1276: SX1276 configured\n");
}

static void anticollision_delay(void) {
	/* Pseudorandom delay up to 8 seconds for collision avoidance */
	unsigned int delay = random_uint32_range(1, 8); // XXX: return to original

	DEBUG("[LoRa] anticollision_delay: random delay %d s\n", (unsigned int) (delay));
	rtctimers_sleep(delay);

	DEBUG("[LoRa] anticollision_delay: delay end\n");
}

static void enter_rx(ls_ed_t *ls)
{
    assert(ls != NULL);
    assert(ls->_internal.sx1276 != NULL);

    /* Don't touch anything if we're already in reception mode */
    if (ls->state == LS_ED_LISTENING) {
        DEBUG("[LoRa] enter_rx: already listening\n");
    	return;
    }

    ls->state = LS_ED_LISTENING;

    DEBUG("[LoRa] enter_rx: configure SX1276\n");
    configure_sx1276(ls, false);
    sx1276_set_rx(ls->_internal.sx1276, ls->_internal.sx1276->settings.lora.rx_timeout);
    DEBUG("[LoRa] enter_rx: SX1276 configured\n");
}

static inline void schedule_tx(ls_ed_t *ls)
{
    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);
    
    /* Send message to the frame queue thread to initiate frame transmission */
    DEBUG("[LoRa] schedule_tx: sending message to uplink queue\n");
    msg_try_send(&msg, ls->_internal.uq_thread_pid);
}

static void open_rx_windows(ls_ed_t *ls)
{
    assert(ls != NULL);
    
    DEBUG("[LoRa] open_rx_windows\n");

    /* Open 2 RX windows if class A device */
    if (ls->settings.class == LS_ED_CLASS_A) {
        DEBUG("[LoRa] open_rx_windows: class A\n");
        rtctimers_set_msg(&ls->_internal.rx_window1, LS_RX_DELAY1, &msg_rx1, ls->_internal.tim_thread_pid);
        rtctimers_set_msg(&ls->_internal.rx_window2, LS_RX_DELAY1 + LS_RX_DELAY2, &msg_rx2, ls->_internal.tim_thread_pid);

        /* Enter reception mode */
        enter_rx(ls);
    }

	/* For a while, B and C are the same */
	else if ((ls->settings.class == LS_ED_CLASS_C) || (ls->settings.class == LS_ED_CLASS_B)) {
        DEBUG("[LoRa] open_rx_windows: class C\n");
        rtctimers_set_msg(&ls->_internal.rx_window1, LS_RX_DELAY1, &msg_rx1, ls->_internal.tim_thread_pid);
        enter_rx(ls);
    }
}

static void send_next(ls_ed_t *ls) {
    anticollision_delay();
	ls->_internal.confirmation_required = false;
    
    DEBUG("[LoRa] send_next: sending frame\n");

    /* Schedule RX windows */
    if (ls->settings.class != LS_ED_CLASS_A) {
        puts("[LoRa] scheduling RX");
        open_rx_windows(ls);
    }
    else if (ls->state == LS_ED_IDLE || ls->state == LS_ED_SLEEP) {
    	puts("[LoRa] scheduling TX");
        schedule_tx(ls);
    }
}

static uint8_t get_node_status(void)
{
    DEBUG("[LoRa] get_node_status: battery and voltage\n");
    
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
    
    DEBUG("[LoRa] get_node_status: V = %d, T = %d\n", vdd, temp);
    
    return (uint8_t)((vdd & 0x1F) | ((temp & 0x7) << 5));
}

static int send_frame(ls_ed_t *ls, ls_type_t type, uint8_t *buf, size_t buflen)
{
    assert(ls != NULL);
    DEBUG("[LoRa] send_frame: sending frame\n");

    mutex_lock(&ls->_internal.curr_frame_mutex);

    ls_frame_t *frame = &ls->_internal.current_frame;
    ls_assemble_frame(ls->_internal.dev_addr, type, buf, buflen, frame);

    frame->header.fid = ls->_internal.last_fid;
    frame->header.status = get_node_status();

    /* Enqueue frame */
    if (ls_frame_fifo_full(&ls->_internal.uplink_queue)) {
        DEBUG("[LoRa] remove oldest frame from queue\n");
        ls_frame_fifo_pop(&ls->_internal.uplink_queue, NULL);
    }
    
    if (!ls_frame_fifo_push(&ls->_internal.uplink_queue, frame)) {
    	mutex_unlock(&ls->_internal.curr_frame_mutex);
        DEBUG("[LoRa] send_frame: FIFO error\n");
        return -LS_SEND_E_FIFO_ERROR;
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

    DEBUG("[LoRa] close_rx_windows: rx windows closed\n");

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
        DEBUG("[LoRa] ack_recv: address mismatch\n");
    	return false;
    }

	/* Pop frame from uplink queue */
	ls_frame_fifo_pop(&ls->_internal.uplink_queue, NULL);

	/* Advance frame ID */
	ls->_internal.last_fid++;

	/* Reset number of retries */
	ls->_internal.num_retr = 0;

	puts("[LoRa] ack_recv: confirmation received");

	ls->_internal.confirmation_required = false;

	/* Remove timeout timer */
	rtctimers_remove(&ls->_internal.conf_ack_expired);

	/* Close RX window only if we haven't pending frames (regular app. data acknowledge received) */
	bool close_rx_window = frame->header.type == LS_DL_ACK;
	return close_rx_window;
}

static void data_recv(ls_ed_t *ls, ls_frame_t *frame) {
    if (frame->header.dev_addr != ls->_internal.dev_addr) {
        DEBUG("[LoRa] data_recv: address mismatch\n");
    	return;
    }
    
    DEBUG("[LoRa] data_recv: data received\n");

	/* Notify application about the data */
	if (ls->appdata_received_cb != NULL) {
		/* Send acknowledge if app. data wasn't sent already */
		if (ls->appdata_received_cb(frame->payload.data, frame->payload.len)) {
            DEBUG("[LoRa] data_recv: sending ACK\n");
			send_frame(ls, LS_UL_ACK, NULL, 0);
		}
	}
}

static bool frame_recv(ls_ed_t *ls, ls_frame_t *frame)
{
#if ENABLE_DEBUG
    char debug_frame_type[20];
    switch (frame->header.type) {
    	case LS_DL:
            snprintf(debug_frame_type, 20, "LS_DL");
            break;
        case LS_DL_ACK:
            snprintf(debug_frame_type, 20, "LS_DL_ACK");
            break;
        case LS_DL_ACK_W_DATA:
            snprintf(debug_frame_type, 20, "LS_DL_ACK_W_DATA");
            break;
        case LS_DL_BROADCAST:
            snprintf(debug_frame_type, 20, "LS_DL_BROADCAST");
            break;
        case LS_UL_JOIN_REQ:
            snprintf(debug_frame_type, 20, "LS_UL_JOIN_REQ");
            break;
        case LS_DL_JOIN_ACK:
            snprintf(debug_frame_type, 20, "LS_DL_JOIN_ACK");
            break;
        case LS_DL_INVITE:
            snprintf(debug_frame_type, 20, "LS_DL_INVITE");
            break;
        default:
            snprintf(debug_frame_type, 20, "UNKNOWN");
            break;
    }
    DEBUG("[LoRa] frame_recv: downlink frame received, type %s\n", debug_frame_type);
#endif

    if ((frame->header.type == LS_DL_ACK_W_DATA) ||
        (frame->header.type == LS_DL_ACK) ||
        (frame->header.type == LS_DL)) {
        if (!ls->settings.no_join && !ls->_internal.is_joined) {
            DEBUG("[LoRa] frame_recv: not joined\n");
            return false;
        }

        if (frame->header.dev_addr != ls->_internal.dev_addr) {
            DEBUG("[LoRa] frame_recv: address mismatch\n");
            return false;
        }

        if (!ls_validate_frame_mic(ls->settings.crypto.mic_key, frame)) {
            DEBUG("[LoRa] frame_recv: invalid MIC\n");
            return false;
        }
    } else {
        if (!ls_validate_frame_mic(ls->settings.crypto.join_key, frame)) {
            DEBUG("[LoRa] frame_recv: invalid MIC\n");
            return false;
        }
    }
    
    switch (frame->header.type) {
    	case LS_DL_BROADCAST: { /* Downlink broadcast message */
    		/* Validate and decipher incoming broadcast message */
            DEBUG("[LoRa] frame_recv: broadcast message\n");
            
            ls_decrypt_frame_payload(ls->settings.crypto.join_key, &frame->payload);

            /* Notify application code about incoming data */
            if (ls->broadcast_appdata_received_cb != NULL) {
                DEBUG("[LoRa] frame_recv: notify application\n");
            	return ls->broadcast_appdata_received_cb(frame->payload.data, frame->payload.len);
            }
            return false;
    	}
    	break;

    	case LS_DL_ACK_W_DATA: /* Acknowledge with additional data */
            /* Must be joined to the network first */
            DEBUG("[LoRa] frame_recv: ack with data received\n");

            /* Remove app data we've got ACK for from FIFO */
            if (!appdata_fifo_empty(&ls->_internal.appdata_fifo)) {
                DEBUG("[LoRa] frame_recv: remove FIFO entry\n");
                appdata_fifo_pop(&ls->_internal.appdata_fifo, NULL);
            }

            DEBUG("[LoRa] frame_recv: decrypting payload\n");
            ls_decrypt_frame_payload(ls->settings.crypto.aes_key, &frame->payload);

            bool close_rx_window = ack_recv(ls, frame);
            data_recv(ls, frame);

            return close_rx_window;

    		break;

        case LS_DL_ACK:                             /* Downlink frame acknowledge for confirmed messages */
            DEBUG("[LoRa] frame_recv: ack received\n");

            /* Remove app data we've got ACK for from FIFO */
            if (!appdata_fifo_empty(&ls->_internal.appdata_fifo)) {
                DEBUG("[LoRa] frame_recv: remove FIFO entry\n");
                appdata_fifo_pop(&ls->_internal.appdata_fifo, NULL);
            }

            return ack_recv(ls, frame);

        case LS_DL:         /* Downlink frame */
            DEBUG("[LoRa] frame_recv: donwlink frame received\n");
            DEBUG("[LoRa] frame_recv: decrypting payload\n");
            ls_decrypt_frame_payload(ls->settings.crypto.aes_key, &frame->payload);

            data_recv(ls, frame);
            return true;

        case LS_DL_JOIN_ACK: { /* Downlink join acknowledge */
            DEBUG("[LoRa] frame_recv: join ACK received\n");
        	/* Joins are disabled */
        	if (ls->settings.no_join) {
                DEBUG("[LoRa] frame_recv: OTA disabled\n");
        		return false;
            }

            if (frame->payload.len != sizeof(ls_join_ack_t)) {
                DEBUG("[LoRa] frame_recv: incorrect payload length\n");
                return false;
            }

            DEBUG("[LoRa] frame_recv: decrypting payload\n");
            ls_decrypt_frame_payload(ls->settings.crypto.join_key, &frame->payload);

            ls_join_ack_t ack;
            memcpy(&ack, frame->payload.data, sizeof(ls_join_ack_t));

            /* This join ack is not for us */
            if (ack.dev_id != ls->settings.node_id) {
                DEBUG("[LoRa] frame_recv: join address mismatch\n");
                return false;
            }

            /* Setup device address */
            ls->_internal.dev_addr = ack.addr;

            /* This join ack for us, derive encryption keys and save */
            ls_derive_keys(ls->_internal.last_nonce, ack.app_nonce, ack.addr, ls->settings.crypto.mic_key, ls->settings.crypto.aes_key);

            /* Remove timeout timer */
            DEBUG("[LoRa] frame_recv: remove join timeout timer\n");
            rtctimers_remove(&ls->_internal.join_req_expired);

            /* Make device joined */
            ls->_internal.is_joined = true;

            /* Notify application code via callback */
            DEBUG("[LoRa] frame_recv: notify application\n");
            if (ls->joined_cb != NULL) {
                ls->joined_cb();
            }

            /* Check for queued data to send after join */
            appdata_fifo_t *fifo = &ls->_internal.appdata_fifo;
            DEBUG("[LoRa] frame_recv: checking FIFO after join\n");

    		while(!appdata_fifo_empty(fifo)) {
    			appdata_fifo_entry_t e;

    			if (!appdata_fifo_pop(fifo, &e)) {
    				break;
    			}

    			printf("[LoRa] frame_recv: sending delayed app. data [fid: %d, size: %d]\n", e.id, e.size);

    			ls_ed_send_app_data(ls, e.data, e.size, e.is_confirmed, e.is_with_ack, true);
    		}

            return true;
        }

        case LS_DL_INVITE: {
            /* Individual join invitation for class C devices */
            DEBUG("[LoRa] frame_recv: join invitation receinved\n");
            
        	if (ls->settings.class != LS_ED_CLASS_C) {
                /* Only for class C */
                DEBUG("[LoRa] frame_recv: not class C device\n");
        		return false;
            }

        	/* Validate and decrypt frame */
            if (frame->payload.len != sizeof(ls_invite_t)) {
                DEBUG("[LoRa] frame_recv: invalid payload length\n");
                return false;
            }

            DEBUG("[LoRa] frame_recv: decrypting payload\n");
            ls_decrypt_frame_payload(ls->settings.crypto.join_key, &frame->payload);

            /* Check device ID */
            ls_invite_t ack;
            memcpy(&ack, frame->payload.data, sizeof(ls_invite_t));

            /* This is not for us */
            if (ack.dev_id != ls->settings.node_id) {
                DEBUG("[LoRa] frame_recv: invitation address mismatch\n");
                return false;
            }

            puts("[LoRa] invited to join, rejoining..."); // XXX: debug

            /* Stop rejoin timeout */
            DEBUG("[LoRa] frame_recv: remove join timeout timer\n");
            rtctimers_remove(&ls->_internal.join_req_expired);

			anticollision_delay();

            /* Proceed to join procedure as requested */
            DEBUG("[LoRa] frame_recv: join\n");
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
            DEBUG("[LoRa] state = IDLE\n");
            ls->state = LS_ED_IDLE;

			/* Save RSSI value of received frame */
			ls->_internal.last_rssi = packet->rssi_value;

			ls_frame_t *frame = (ls_frame_t *) packet->content;

			/* Check frame format */
			if (ls_validate_frame(packet->content, packet->size)) {
                DEBUG("[LoRa] sx1276_handler: data valid\n");
				/* Process new frame */
				if (frame_recv(ls, frame)) {
					/* Class A devices closes RX window after each received packet */
					if (ls->settings.class == LS_ED_CLASS_A) {
                        DEBUG("[LoRa] class A close RX window\n");
						close_rx_windows(ls);
					}
				} else {
					if (ls->_internal.num_reopened++ < LS_ED_RX_NUM_REOPEN) {
						DEBUG("[LoRa] first RX window reopened\n");

						/* Reopen RX window */
						rtctimers_remove(&ls->_internal.rx_window1);
						rtctimers_remove(&ls->_internal.rx_window2);

						open_rx_windows(ls);
					} else {
						DEBUG("[LoRa] forcing RX window expiring\n");

						ls->_internal.num_reopened = 0;
						rtctimers_remove(&ls->_internal.rx_window1);
						rtctimers_remove(&ls->_internal.rx_window2);

						/* Notify timeouts thread about RX window expiration */
						msg_send(&msg_rx1, ls->_internal.tim_thread_pid);
					}
				}
			}
			else {
				DEBUG("[LoRa] sx1276_handler: malformed data discarded\n");
			}

			break;

		case SX1276_RX_ERROR_CRC:
			puts("sx1276: RX CRC failed");
            DEBUG("[LoRa] state = IDLE\n");
            ls->state = LS_ED_IDLE;
			break;

		case SX1276_TX_DONE:
			puts("sx1276: transmission done.");

			/* Open RX windows after each transmitted packet */
			open_rx_windows(ls);

			break;

		case SX1276_RX_TIMEOUT:
			puts("sx1276: RX timeout");
            DEBUG("[LoRa] state = IDLE\n");
            ls->state = LS_ED_IDLE;
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
            
        case SX1276_VALID_HEADER:
            puts("[LoRa] header received, switch to RX state");
            ls->state = LS_ED_LISTENING;
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

    DEBUG("[LoRa] uq_handler: uplink frame queue handler thread started\n");

    ls_ed_t *ls = (ls_ed_t *) arg;
    msg_init_queue(ls->_internal.uq_msg_queue, sizeof(ls->_internal.uq_msg_queue));
    msg_t msg;

    while (1) {
        msg_receive(&msg);
        DEBUG("[LoRa] uq_handler: message received\n");

        if (ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
            ls->state = LS_ED_IDLE;
            DEBUG("[LoRa] uq_handler: FIFO is empty\n");
            continue;
        }

        /* Get frame from queue top */
        ls_frame_t *f;
        ls_frame_t frame;
        if (!ls_frame_fifo_peek(&ls->_internal.uplink_queue, &frame)) {
            DEBUG("[LoRa] uq_handler: error getting frame from FIFO\n");
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

        DEBUG("[LoRa] uq_handler: reconfigure transceiver\n");
        /* Configure to sleep */
        sx1276_set_sleep(ls->_internal.sx1276);

        /* Configure SX1276 to TX */
        configure_sx1276(ls, true);

        size_t header_size = sizeof(ls_header_t) + sizeof(ls_payload_len_t);
        size_t payload_size = 0;

        /* Apply cryptography procedures */
        if (f->header.type != LS_UL_JOIN_REQ) {
            DEBUG("[LoRa] uq_handler: encrypt regular frame\n");
            ls_encrypt_frame(ls->settings.crypto.mic_key, ls->settings.crypto.aes_key, f, &payload_size);
        }
        else {
            DEBUG("[LoRa] uq_handler: encrypt join request\n");
            ls_encrypt_frame(ls->settings.crypto.join_key, ls->settings.crypto.join_key, f, &payload_size);
        }
        /* Listen Before Talk with LoRa CAD support */
        /* delays between CAD requests */
        int delay_ms = 5 + ((100 + 10*ls->settings.dr) >> ls->settings.dr);
        
        DEBUG("[LoRa] uq_handler: checking channel activity every %d ms\n", delay_ms);
        
        int cad_tries = 0;
        for (int k = 0; k < 10; k++) {
            ls->_internal.last_cad_success = 0;
            sx1276_start_cad(ls->_internal.sx1276, SX1276_MODE_CADDETECT);
            //xtimer_spin(xtimer_ticks_from_usec(delay_ms * 1000));
            rtctimers_millis_sleep(delay_ms);
            if (ls->_internal.last_cad_success) {
                DEBUG("[LoRa] uq_handler: channel activity detected\n");
                
                /* send anyway if we tried too many times */
                cad_tries++;
                if (cad_tries > 5) {
                    break;
                }
                
                /* otherwise restart CAD after a pause */
                if ((f->header.type == LS_UL_ACK) || (f->header.type == LS_UL_UNC_ACK)) {
                    rtctimers_sleep(1);
                } else {
                    rtctimers_sleep(3);
                }
                /* xtimer_spin(xtimer_ticks_from_usec(delay_ms * 1000 * 5)); */
                k = 0;
            }
        }
        DEBUG("[LoRa] uq_handler: sending data to transceiver\n");


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
        DEBUG("[LoRa] uq_handler: transceiver configured\n");
        /* Send frame into LoRa PHY */
        sx1276_send(ls->_internal.sx1276, (uint8_t *) f, header_size + payload_size);
        DEBUG("[LoRa] uq_handler: data sent\n");
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
        puts("ls_init: creation of uplink frame queue handler thread failed");
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
                DEBUG("[LoRa] tim_handler: RX1 window expired\n");

                if (ls->settings.class == LS_ED_CLASS_A) {
                    /* Use default settings for the transceiver in second RX window */
                    ls->_internal.use_rx_window_2_settings = true;

                    /* Enter reception mode */
                    DEBUG("[LoRa] tim_handler: class A, enter reception mode\n");
                    enter_rx(ls);
                }
				// for a while, classes B and C are the same
				else if ((ls->settings.class == LS_ED_CLASS_C) || (ls->settings.class == LS_ED_CLASS_B))  {
                    /* Transmit next frame from queue */
                    if (!ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
                    	/* If current frame in a head of a queue doesn't awaiting confirmation, schedule it for sending
                    	 * Otherwise, current frame will be retransmitted after confirmation timeout
                    	 */
                    	if (!ls->_internal.confirmation_required) {
                            DEBUG("[LoRa] tim_handler: schedule current frame transmission\n");
                    		schedule_tx(ls);
                        } else {
                    		enter_rx(ls);
                    		DEBUG("[LoRa] tim_handler: awaiting confirmation\n");
                    	}
                    }
                    else {
                    	puts("[LoRa] tim_handler: staying in RX mode");
                        enter_rx(ls);
                    }
                }

                break;

            case LS_ED_RX2_EXPIRED:
                puts("[LoRa] tim_handler: RX2 window expired"); // XXX: debug

                /* Clear the default settings flag */
                ls->_internal.use_rx_window_2_settings = false;

                /* Transmit next frame from queue */
                if (!ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
                	/* If current frame in a head of a queue doesn't awaiting confirmation, schedule it for sending
                	 * Otherwise, current frame will be retransmitted after confirmation timeout
                	 */
                	if (!ls->_internal.confirmation_required) {
                        DEBUG("[LoRa] tim_handler: schedule current frame transmission\n");
                		schedule_tx(ls);
                    } else {
                		DEBUG("[LoRa] tim_handler: awaiting confirmation");
                    }
                } else {
					/* Put transceiver into sleep with low power mode */
                    DEBUG("[LoRa] tim_handler: put transciever to sleep\n");
					ls_ed_sleep(ls);
                }

                break;

            case LS_ED_JOIN_REQ_EXPIRED:
                puts("[LoRa] tim_handler: join request expired"); // XXX: debug

                /* Connection is lost, clear uplink queue */


                /* Notify application about the event */
                if (ls->join_timeout_cb != NULL) {
                    ls->join_timeout_cb();
                }

                break;

            case LS_ED_APPDATA_ACK_EXPIRED:
                puts("[LoRa] tim_handler: appdata confirmation timeout");

                /* Retransmit data */
                if (ls->_internal.num_retr >= ls->settings.max_retr) {
                    /* Stop retransmitting */
                    DEBUG("[LoRa] tim_handler: stop retransmitting\n");
                    ls->_internal.num_retr = 0;

                    if (ls->appdata_send_failed_cb != NULL) {
                        ls->appdata_send_failed_cb();
                    }
                }
                else {
                    /* Do a retransmission */
                    DEBUG("[LoRa] tim_handler: do a retransmission\n");
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
    DEBUG("[LoRa] create_tim_handler_thread: creating LS timeouts thread\n");

    kernel_pid_t pid_tim = thread_create(ls->_internal.tim_thread_stack, sizeof(ls->_internal.tim_thread_stack), THREAD_PRIORITY_MAIN - 2,
                                         THREAD_CREATE_STACKTEST, tim_handler, ls,
                                         "LS timeouts thread");

    if (pid_tim <= KERNEL_PID_UNDEF) {
        DEBUG("[LoRa] create_tim_handler_thread: fail\n");
        return false;
    } else {
        DEBUG("[LoRa] create_tim_handler_thread: success\n");
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

    DEBUG("[LoRa] ls_ed_init: init SX1276\n");
    /* Initialize the transceiver */
    sx1276_init(ls->_internal.sx1276);

    DEBUG("[LoRa] ls_ed_init: init RNG\n");
    /* Initialize random number generator */
    random_init(sx1276_random(ls->_internal.sx1276));

    ls_ed_sleep(ls);

    return LS_OK;
}

void ls_ed_poweroff(ls_ed_t *ls) {
    DEBUG("[LoRa] ls_ed_poweroff: set transceiver to sleep\n");
    sx1276_set_sleep(ls->_internal.sx1276);
}

int ls_ed_send_app_data(ls_ed_t *ls, uint8_t *buf, size_t buflen, bool confirmed, bool with_ack, bool delayed)
{
    assert(ls != NULL);
    assert(buf != NULL);

    
    /* Store current app. data in FIFO buffer
     * Data will be removed on receiving ACK from gate */
    if (!delayed && (confirmed || (!ls->settings.no_join && !ls->_internal.is_joined))) {
        DEBUG("[LoRa] ls_ed_send_app_data: pushing data to FIFO\n");
        appdata_fifo_t *fifo = &ls->_internal.appdata_fifo;

        /* Last data has priority, so we can pop oldest item from the queue if it's full */
        if (appdata_fifo_full(fifo)) {
            appdata_fifo_pop(fifo, NULL);
        }

        appdata_fifo_push(fifo, buf, buflen, ls->_internal.last_fid, confirmed, with_ack);
        
        /* Not joined to the network, delay appdata frame until device is joined */
        if (!ls->settings.no_join && !ls->_internal.is_joined) {
            DEBUG("[LoRa] ls_ed_send_app_data: data delayed until node is joined\n");
            return -LS_SEND_E_NOT_JOINED;
        }
        
        if (appdata_fifo_size(fifo) > 1) {
            DEBUG("[LoRa] ls_ed_send_app_data: FIFO is not empty, postpone new data\n");
            
            appdata_fifo_entry_t e;
            appdata_fifo_peek(fifo, &e);
            DEBUG("[LoRa] ls_ed_send_app_data: send oldest data instead [fid: %d, size: %d]\n", e.id, e.size);
            buf = e.data;
            buflen = e.size;
            confirmed = e.is_confirmed;
            with_ack = e.is_with_ack;
        }
    }

    ls->_internal.confirmation_required = false;

    ls_type_t type = (with_ack) ? LS_UL_UNC_ACK : (confirmed) ? LS_UL_CONF : LS_UL_UNC;

    DEBUG("[LoRa] ls_ed_send_app_data: send_frame\n");
    int res = send_frame(ls, type, buf, buflen);
    if (res < 0) {
        return res;
    }

    return LS_OK;
}

void ls_ed_unjoin(ls_ed_t *ls)
{
    DEBUG("[LoRa] ls_ed_unjoin: unjoin node\n");
    
	if (ls->settings.no_join) {
        DEBUG("[LoRa] ls_ed_unjoin: OTA disabled\n");
		return;
    }

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
    
    DEBUG("[LoRa] ls_ed_unjoin: node unjoined\n");
}

int ls_ed_join(ls_ed_t *ls)
{
    assert(ls != NULL);
    DEBUG("[LoRa] ls_ed_join: joining\n");

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
    DEBUG("[LoRa] ls_ed_join: send join request\n");
    send_frame(ls, LS_UL_JOIN_REQ, (uint8_t *) &req, sizeof(ls_join_req_t));

    /* Launch timeout timer */
    DEBUG("[LoRa] ls_ed_join: set join timeout timer\n");
    rtctimers_set_msg(&ls->_internal.join_req_expired, LS_JOIN_TIMEOUT, &msg_join_timeout, ls->_internal.tim_thread_pid);

    return LS_OK;
}

void ls_ed_sleep(ls_ed_t *ls)
{
    assert(ls != NULL);
    
    DEBUG("[LoRa] ls_ed_sleep: put transceiver to sleep\n");

    if (ls->settings.class == LS_ED_CLASS_A) {
        ls->state = LS_ED_SLEEP;
        sx1276_set_sleep(ls->_internal.sx1276);
    } else {
        DEBUG("[LoRa] ls_ed_sleep: ignore sleep, not a Class A\n");
    }
}

#ifdef __cplusplus
}
#endif
