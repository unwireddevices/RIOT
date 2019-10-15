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
 * @file		ls-end-device.c
 * @brief       LoRaLAN stack implementation for the node device
 * @author      Eugene Ponomarev
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "random.h"
#include "assert.h"
#include "thread.h"
#include "mutex.h"

#include "periph/adc.h"
#include "net/netdev/lora.h"
#include "ls-init-device.h"
#include "ls-mac-types.h"
#include "ls-mac.h"
#include "ls-end-device.h"
#include "board.h"
#include "lptimer.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define SX127X_LORA_MSG_QUEUE   (16U)
#define SX127X_STACKSIZE        (2*THREAD_STACKSIZE_DEFAULT)
#define MSG_TYPE_ISR            (0x3456)

#if !defined(UNWDS_MAC_LORAWAN)

static char isr_stack[SX127X_STACKSIZE];
static kernel_pid_t isr_pid;

static msg_t msg_rx1;
static msg_t msg_rx2;

static msg_t msg_join_timeout;
static msg_t msg_ack_timeout;

static ls_ed_t *p_ls;

static void configure_sx127x(ls_ed_t *ls)
{
    ls_datarate_t dr = (!ls->_internal.use_rx_window_2_settings) ? ls->settings.dr : LS_RX2_DR;
    ls_channel_t ch = (!ls->_internal.use_rx_window_2_settings) ? ls->settings.channel : LS_RX2_CH;
    
    ls_setup_sx127x(ls->_internal.device, dr, ls->settings.channels_table[ch]);
    
    DEBUG("[LoRa] SX127X configured\n");
}

static void anticollision_delay(void) {
	/* Pseudorandom delay up to 8 seconds for collision avoidance */
	unsigned int delay = random_uint32_range(1000, 8000);

	DEBUG("[LoRa] random delay %d ms\n", (unsigned int) (delay));
	lptimer_sleep(delay);

	DEBUG("[LoRa] delay end\n");
}

static void enter_rx(ls_ed_t *ls)
{
    assert(ls != NULL);
    assert(ls->_internal.device != NULL);

    /* Don't touch anything if we're already in reception mode */
    if (ls->state == LS_ED_LISTENING) {
        DEBUG("[LoRa] already listening\n");
    	return;
    }

    ls->state = LS_ED_LISTENING;

    DEBUG("[LoRa] configure SX127X\n");
    configure_sx127x(ls);
    uint8_t state = NETOPT_STATE_RX;
    ls->_internal.device->driver->set(ls->_internal.device, NETOPT_STATE, &state, sizeof(uint8_t));
    DEBUG("[LoRa] SX127X configured\n");
}

static inline void schedule_tx(ls_ed_t *ls)
{
    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);
    
    /* Send message to the frame queue thread to initiate frame transmission */
    DEBUG("[LoRa] sending message to uplink queue\n");
    msg_try_send(&msg, ls->_internal.uq_thread_pid);
}

static void open_rx_windows(ls_ed_t *ls)
{
    assert(ls != NULL);
    
    DEBUG("[LoRa] open_rx_windows\n");

    lptimer_set_msg(&ls->_internal.rx_window1, 1000*LS_RX_DELAY1, &msg_rx1, ls->_internal.tim_thread_pid);
    
    /* Open second RX windows if class A device */
    if (ls->settings.class == LS_ED_CLASS_A) {
        DEBUG("[LoRa] class A second window\n");        
        lptimer_set_msg(&ls->_internal.rx_window2, 1000*(LS_RX_DELAY1 + LS_RX_DELAY2), &msg_rx2, ls->_internal.tim_thread_pid);
    }
    
    enter_rx(ls);
}

static void send_next(ls_ed_t *ls) {
    anticollision_delay();
	ls->_internal.confirmation_required = false;
    
    DEBUG("[LoRa] sending frame\n");

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
    DEBUG("[LoRa] battery and voltage\n");

#if defined(ADC_VREF_INDEX) && defined(ADC_TEMPERATURE_INDEX)
    if (adc_init(ADC_LINE(ADC_VREF_INDEX)) < 0) {
        DEBUG("[LoRa] ADC init error\n");
        return 0;
    }
    
    DEBUG("[LoRa] ADC initialized\n");
    
    uint16_t vdd;
    /* 2000 mV min voltage */
    /* 50 mV per bit resolution */
    vdd = (adc_sample(ADC_LINE(ADC_VREF_INDEX), ADC_RES_12BIT) - 2000)/50;
    
    if (adc_init(ADC_LINE(ADC_TEMPERATURE_INDEX)) < 0) {
        return ((uint8_t)vdd & 0x1F);
    }
    int temp;
    temp = adc_sample(ADC_LINE(ADC_TEMPERATURE_INDEX), ADC_RES_12BIT);
    temp = 40 + ((temp + 105) / 200); /* -40 is the minimim, 20 deg C per bit */
    
    DEBUG("[LoRa] V = %d, T = %d\n", vdd, temp);
    
    return (uint8_t)((vdd & 0x1F) | ((temp & 0x7) << 5));
#else
    return 0;
#endif
}

static int send_frame(ls_ed_t *ls, ls_type_t type, uint8_t *buf, size_t buflen)
{
    assert(ls != NULL);
    DEBUG("[LoRa] sending frame\n");

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
        DEBUG("[LoRa] FIFO error\n");
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

    DEBUG("[LoRa] rx windows closed\n");

    /* Remove windows */
    lptimer_remove(&ls->_internal.rx_window1);
    lptimer_remove(&ls->_internal.rx_window2);

    /* Put SX127X into sleep */
    ls_ed_sleep(ls);

    /* Schedule next transmission */
    schedule_tx(ls);
}

static bool ack_recv(ls_ed_t *ls, ls_frame_t  *frame) {
    if (frame->header.dev_addr != ls->_internal.dev_addr) {
        DEBUG("[LoRa] address mismatch\n");
    	return false;
    }

	/* Pop frame from uplink queue */
	ls_frame_fifo_pop(&ls->_internal.uplink_queue, NULL);

	/* Advance frame ID */
	ls->_internal.last_fid++;

	/* Reset number of retries */
	ls->_internal.num_retr = 0;

	puts("[LoRa] confirmation received");

	ls->_internal.confirmation_required = false;

	/* Remove timeout timer */
	lptimer_remove(&ls->_internal.conf_ack_expired);

	/* Close RX window only if we haven't pending frames (regular app. data acknowledge received) */
	bool close_rx_window = frame->header.type == LS_DL_ACK;
	return close_rx_window;
}

static void data_recv(ls_ed_t *ls, ls_frame_t *frame) {
    if (frame->header.dev_addr != ls->_internal.dev_addr) {
        DEBUG("[LoRa] address mismatch\n");
    	return;
    }
    
    DEBUG("[LoRa] data received\n");

	/* Notify application about the data */
	if (ls->appdata_received_cb != NULL) {
		/* Send acknowledge if app. data wasn't sent already */
		if (ls->appdata_received_cb(frame->payload.data, frame->payload.len)) {
            DEBUG("[LoRa] sending ACK\n");
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
        case LS_DL_TIME_ACK:
            snprintf(debug_frame_type, 20, "LS_DL_TIME_ACK");
            break;
        default:
            snprintf(debug_frame_type, 20, "UNKNOWN");
            break;
    }
    DEBUG("[LoRa] downlink frame received, type %s\n", debug_frame_type);
#endif

    if ((frame->header.type == LS_DL_ACK_W_DATA) ||
        (frame->header.type == LS_DL_ACK) ||
        (frame->header.type == LS_DL) || 
        (frame->header.type == LS_DL_TIME_ACK)) {
        if (!ls->settings.no_join && !ls->_internal.is_joined) {
            DEBUG("[LoRa] not joined\n");
            return false;
        }

        if (frame->header.dev_addr != ls->_internal.dev_addr) {
            DEBUG("[LoRa] address mismatch\n");
            return false;
        }

        if (!ls_validate_frame_mic(ls->settings.crypto.mic_key, frame)) {
            DEBUG("[LoRa] invalid MIC\n");
            return false;
        }
    } else {
        if (!ls_validate_frame_mic(ls->settings.crypto.join_key, frame)) {
            DEBUG("[LoRa] invalid MIC\n");
            return false;
        }
    }
    
    switch (frame->header.type) {
    	case LS_DL_BROADCAST: { /* Downlink broadcast message */
    		/* Validate and decipher incoming broadcast message */
            DEBUG("[LoRa] broadcast message\n");
            
            ls_decrypt_frame_payload(ls->settings.crypto.join_key, frame);

            /* Notify application code about incoming data */
            if (ls->broadcast_appdata_received_cb != NULL) {
                DEBUG("[LoRa] notify application\n");
            	return ls->broadcast_appdata_received_cb(frame->payload.data, frame->payload.len);
            }
            return false;
    	}
    	break;

    	case LS_DL_ACK_W_DATA: /* Acknowledge with additional data */
            /* Must be joined to the network first */
            DEBUG("[LoRa] ack with data received\n");

            /* Remove app data we've got ACK for from FIFO */
            if (!appdata_fifo_empty(&ls->_internal.appdata_fifo)) {
                DEBUG("[LoRa] remove FIFO entry\n");
                appdata_fifo_pop(&ls->_internal.appdata_fifo, NULL);
            }

            DEBUG("[LoRa] decrypting payload\n");
            ls_decrypt_frame_payload(ls->settings.crypto.aes_key, frame);

            bool close_rx_window = ack_recv(ls, frame);
            data_recv(ls, frame);

            return close_rx_window;

    		break;

        case LS_DL_ACK:                             /* Downlink frame acknowledge for confirmed messages */
            DEBUG("[LoRa] ack received\n");

            /* Remove app data we've got ACK for from FIFO */
            if (!appdata_fifo_empty(&ls->_internal.appdata_fifo)) {
                DEBUG("[LoRa] remove FIFO entry\n");
                appdata_fifo_pop(&ls->_internal.appdata_fifo, NULL);
            }

            return ack_recv(ls, frame);

        case LS_DL:         /* Downlink frame */
            DEBUG("[LoRa] donwlink frame received\n");
            DEBUG("[LoRa] decrypting payload\n");
            ls_decrypt_frame_payload(ls->settings.crypto.aes_key, frame);

            data_recv(ls, frame);
            return true;

        case LS_DL_JOIN_ACK: { /* Downlink join acknowledge */
            DEBUG("[LoRa] join ACK received\n");
        	/* Joins are disabled */
        	if (ls->settings.no_join) {
                DEBUG("[LoRa] OTA disabled\n");
        		return false;
            }

            if (frame->payload.len != sizeof(ls_join_ack_t)) {
                DEBUG("[LoRa] incorrect payload length\n");
                return false;
            }

            DEBUG("[LoRa] decrypting payload\n");
            ls_decrypt_frame_payload(ls->settings.crypto.join_key, frame);

            ls_join_ack_t ack = { 0 };
            memcpy(&ack, frame->payload.data, sizeof(ls_join_ack_t));

            /* This join ack is not for us */
            if (ack.dev_id != ls->settings.node_id) {
                DEBUG("[LoRa] join address mismatch\n");
                return false;
            }

            /* Setup device address */
            ls->_internal.dev_addr = ack.addr;

            /* This join ack for us, derive encryption keys and save */
            ls_derive_keys(ls->_internal.last_nonce, ack.app_nonce, ack.addr, ls->settings.crypto.mic_key, ls->settings.crypto.aes_key);

            /* Remove timeout timer */
            DEBUG("[LoRa] remove join timeout timer\n");
            lptimer_remove(&ls->_internal.join_req_expired);

            /* Make device joined */
            ls->_internal.is_joined = true;

            /* Notify application code via callback */
            DEBUG("[LoRa] notify application\n");
            if (ls->joined_cb != NULL) {
                ls->joined_cb();
            }

            /* Check for queued data to send after join */
            appdata_fifo_t *fifo = &ls->_internal.appdata_fifo;
            DEBUG("[LoRa] checking FIFO after join\n");

    		while(!appdata_fifo_empty(fifo)) {
    			appdata_fifo_entry_t e;

    			if (!appdata_fifo_pop(fifo, &e)) {
    				break;
    			}

    			DEBUG("[LoRa] sending delayed app. data [fid: %d, size: %d]\n", e.id, e.size);

    			ls_ed_send_app_data(ls, e.data, e.size, e.is_confirmed, e.is_with_ack, true);
    		}
            
            DEBUG("[LoRa] done\n");

            return true;
        }

        case LS_DL_INVITE: {
            /* Individual join invitation for class C devices */
            DEBUG("[LoRa] join invitation receinved\n");
            
        	if (ls->settings.class != LS_ED_CLASS_C) {
                /* Only for class C */
                DEBUG("[LoRa] not class C device\n");
        		return false;
            }

        	/* Validate and decrypt frame */
            if (frame->payload.len != sizeof(ls_invite_t)) {
                DEBUG("[LoRa] invalid payload length\n");
                return false;
            }

            DEBUG("[LoRa] decrypting payload\n");
            ls_decrypt_frame_payload(ls->settings.crypto.join_key, frame);

            /* Check device ID */
            ls_invite_t ack;
            memcpy(&ack, frame->payload.data, sizeof(ls_invite_t));

            /* This is not for us */
            if (ack.dev_id != ls->settings.node_id) {
                DEBUG("[LoRa] invitation address mismatch\n");
                return false;
            }

            puts("[LoRa] invited to join, rejoining..."); // XXX: debug

            /* Stop rejoin timeout */
            DEBUG("[LoRa] remove join timeout timer\n");
            lptimer_remove(&ls->_internal.join_req_expired);

			anticollision_delay();

            /* Proceed to join procedure as requested */
            DEBUG("[LoRa] join\n");
            ls_ed_join(ls);
        	return false;
        }
        
        case LS_DL_TIME_ACK: {
            ls_decrypt_frame_payload(ls->settings.crypto.aes_key, frame);

            ls_time_req_ack_t ack;
            memcpy(&ack, frame->payload.data, sizeof(ls_time_req_ack_t));

            if (ls->time_req_ack_cb != NULL) {
                ls->time_req_ack_cb(ack.gate_time);
            }

            return true;
        }

        default:
            /* Not interested in frames from other devices */
            return false;
    }
}

static void sx127x_handler(netdev_t *dev, netdev_event_t event)
{
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;
        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;
        if (msg_send(&msg, isr_pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.");
        }
        return;
    }

    switch (event) {
        case NETDEV_EVENT_RX_COMPLETE: {
            int len;
            netdev_lora_rx_info_t  packet_info;
            uint8_t message[LS_FRAME_SIZE];
    
            len = dev->driver->recv(dev, NULL, 0, 0);
            if (len < 0) {
                printf("RX: bad message, aborting\n");
                break;
            }
            
            dev->driver->recv(dev, message, len, &packet_info);
            
            printf("RX: %d bytes, | RSSI: %d dBm | SNR: %d dBm\n", (int)len,
                    packet_info.rssi, (int)packet_info.snr);

            DEBUG("[LoRa] state = IDLE\n");
            p_ls->state = LS_ED_IDLE;

            /* Save RSSI value of received frame */
            p_ls->_internal.last_rssi = packet_info.rssi;

            ls_frame_t *frame = (ls_frame_t *) message;

            /* Check frame format */
            if (ls_validate_frame(message, len)) {
                DEBUG("[LoRa] data valid\n");
                /* Process new frame */
                if (frame_recv(p_ls, frame)) {
                    DEBUG("[LoRa] frame received\n");
                    /* Class A devices closes RX window after each received packet */
                    if (p_ls->settings.class == LS_ED_CLASS_A) {
                        DEBUG("[LoRa] class A close RX window\n");
						close_rx_windows(p_ls);
					}
				} else {
                    lptimer_remove(&p_ls->_internal.rx_window1);
                    lptimer_remove(&p_ls->_internal.rx_window2);
                    
                    if (p_ls->_internal.num_reopened++ < LS_ED_RX_NUM_REOPEN) {
                        DEBUG("[LoRa] first RX window reopened\n");

                        /* Reopen RX window */
                        open_rx_windows(p_ls);
                    } else {
                        DEBUG("[LoRa] forcing RX window expiring\n");

                        p_ls->_internal.num_reopened = 0;

                        /* Notify timeouts thread about RX window expiration */
                        msg_send(&msg_rx1, p_ls->_internal.tim_thread_pid);
                    }
                }
            }
            else {
                DEBUG("[LoRa] malformed data discarded\n");
            }

            break;
        }
        case NETDEV_EVENT_CRC_ERROR:
            puts("[LoRa] RX CRC failed");
            DEBUG("[LoRa] state = IDLE\n");
            p_ls->state = LS_ED_IDLE;
            break;

        case NETDEV_EVENT_TX_COMPLETE:
            puts("[LoRa] transmission done.");

            /* Open RX windows after each transmitted packet */
            open_rx_windows(p_ls);

            break;

        case NETDEV_EVENT_RX_TIMEOUT:
            puts("[LoRa] RX timeout");
            DEBUG("[LoRa] state = IDLE\n");
            p_ls->state = LS_ED_IDLE;
            close_rx_windows(p_ls);
            ls_ed_sleep(p_ls);
            break;

        case NETDEV_EVENT_TX_TIMEOUT:
            /* this should not happen, re-init SX127X here */
            puts("[LoRa] TX timeout");
            p_ls->_internal.device->driver->init(p_ls->_internal.device);
            ls_ed_sleep(p_ls);

            break;
            
        case NETDEV_EVENT_CAD_DONE:
            DEBUG("[LoRa] CAD done\n");
            p_ls->_internal.last_cad_success = false;
            ls_ed_sleep(p_ls);
            break;
            
        case NETDEV_EVENT_CAD_DETECTED:
            DEBUG("[LoRa] CAD detected\n");
            p_ls->_internal.last_cad_success = true;
            ls_ed_sleep(p_ls);
            break;
            
        case NETDEV_EVENT_RX_STARTED:
            puts("[LoRa] header received, switch to RX state");
            p_ls->state = LS_ED_LISTENING;
            break;

        default:
            printf("[LoRa] received event #%d\n", (int) event);
            ls_ed_sleep(p_ls);
            break;
    }
}

void *isr_thread(void *arg)
{
    (void)arg;
    
    static msg_t _msg_q[SX127X_LORA_MSG_QUEUE];
    msg_init_queue(_msg_q, SX127X_LORA_MSG_QUEUE);

    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == MSG_TYPE_ISR) {
            netdev_t *dev = msg.content.ptr;
            dev->driver->isr(dev);
        }
        else {
            puts("[LoRa] unexpected msg type");
        }
    }
}

#if ENABLE_DEBUG
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
#endif

/**
 * Uplink frame queue handler thread body.
 */
static void *uq_handler(void *arg)
{
    assert(arg != NULL);

    DEBUG("[LoRa] uplink frame queue handler thread started\n");

    ls_ed_t *ls = (ls_ed_t *) arg;
    msg_init_queue(ls->_internal.uq_msg_queue, sizeof(ls->_internal.uq_msg_queue));
    msg_t msg;

    while (1) {
        msg_receive(&msg);
        DEBUG("[LoRa] message received\n");
        
        if (ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
            ls->state = LS_ED_IDLE;
            DEBUG("[LoRa] FIFO is empty\n");
            continue;
        }

        /* Get frame from queue top */
        ls_frame_t *f;
        ls_frame_t frame;
        if (!ls_frame_fifo_peek(&ls->_internal.uplink_queue, &frame)) {
            DEBUG("[LoRa] error getting frame from FIFO\n");
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
            lptimer_set_msg(&ls->_internal.conf_ack_expired, 1000*LS_ACK_TIMEOUT, &msg_ack_timeout, ls->_internal.tim_thread_pid);
        }

        ls->state = LS_ED_TRANSMITTING;

        DEBUG("[LoRa] reconfigure transceiver\n");
        /* Configure to sleep */
        uint8_t state = NETOPT_STATE_SLEEP;
        ls->_internal.device->driver->set(ls->_internal.device, NETOPT_STATE, &state, sizeof(uint8_t));

        /* Configure SX127X to TX */
        configure_sx127x(ls);

        size_t header_size = sizeof(ls_header_t) + sizeof(ls_payload_len_t);
        size_t payload_size = 0;

        /* Apply cryptography procedures */
        if (f->header.type != LS_UL_JOIN_REQ) {
            DEBUG("[LoRa] encrypt regular frame\n");
            ls_encrypt_frame(ls->settings.crypto.mic_key, ls->settings.crypto.aes_key, f, &payload_size);
        }
        else {
            DEBUG("[LoRa] encrypt join request\n");
            ls_encrypt_frame(ls->settings.crypto.join_key, ls->settings.crypto.join_key, f, &payload_size);
        }
        /* Listen Before Talk with LoRa CAD support */
        /* delays between CAD requests */
        int delay_ms = 5 + ((100 + 10*ls->settings.dr) >> ls->settings.dr);
        
        DEBUG("[LoRa] checking channel activity every %d ms\n", delay_ms);
        
        int cad_tries = 0;
        for (int k = 0; k < 10; k++) {
            ls->_internal.last_cad_success = 0;

            uint8_t state = NETOPT_STATE_CAD;
            ls->_internal.device->driver->set(ls->_internal.device, NETOPT_STATE, &state, sizeof(uint8_t));

            //xtimer_spin(xtimer_ticks_from_usec(delay_ms * 1000));
            lptimer_sleep(delay_ms);
            if (ls->_internal.last_cad_success) {
                DEBUG("[LoRa] channel activity detected\n");
                
                /* send anyway if we tried too many times */
                cad_tries++;
                if (cad_tries > 5) {
                    break;
                }
                
                /* otherwise restart CAD after a pause */
                if ((f->header.type == LS_UL_ACK) || (f->header.type == LS_UL_UNC_ACK)) {
                    lptimer_sleep(500);
                } else {
                    lptimer_sleep(3000);
                }
                /* xtimer_spin(xtimer_ticks_from_usec(delay_ms * 1000 * 5)); */
                k = 0;
            }
        }
        DEBUG("[LoRa] sending data to transceiver\n");

#if ENABLE_DEBUG
        char type_str[10] = {};
        get_type_str(f->header.type, type_str);
        printf(">mhdr=0x%02X, mic=0x%04X, addr=0x%02X, <%s> fid=0x%02X (%d bytes) [%d left]\n", (unsigned int) f->header.mhdr,
               (unsigned int) f->header.mic, (unsigned int) f->header.dev_addr,
               type_str,
               (unsigned int) f->header.fid, header_size + payload_size,
			   ls_frame_fifo_size(&ls->_internal.uplink_queue));
#endif
        /* Configure for TX */
        configure_sx127x(ls);
        DEBUG("[LoRa] transceiver configured\n");
        
        /* Send frame into LoRa PHY */
        iolist_t data = {
            .iol_base = f,
            .iol_len = header_size + payload_size,
        };
        
        if (ls->_internal.device->driver->send(ls->_internal.device, &data) < 0) {
            puts("[LoRa] cannot send, device busy");
        }
        
        DEBUG("[LoRa] data sent\n");
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
    
    isr_pid = thread_create(isr_stack, sizeof(isr_stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, isr_thread, NULL,
                              "SX127x handler thread");

    if (isr_pid <= KERNEL_PID_UNDEF) {
        puts("ls_init: creation of SX127X ISR thread failed");
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
                DEBUG("[LoRa] RX1 window expired\n");

                if (ls->settings.class == LS_ED_CLASS_A) {
                    /* Use default settings for the transceiver in second RX window */
                    ls->_internal.use_rx_window_2_settings = true;

                    /* Enter reception mode */
                    DEBUG("[LoRa] class A, enter reception mode\n");
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
                            DEBUG("[LoRa] schedule current frame transmission\n");
                    		schedule_tx(ls);
                        } else {
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
                puts("[LoRa] RX2 window expired"); // XXX: debug

                /* Clear the default settings flag */
                ls->_internal.use_rx_window_2_settings = false;

                /* Transmit next frame from queue */
                if (!ls_frame_fifo_empty(&ls->_internal.uplink_queue)) {
                	/* If current frame in a head of a queue doesn't awaiting confirmation, schedule it for sending
                	 * Otherwise, current frame will be retransmitted after confirmation timeout
                	 */
                	if (!ls->_internal.confirmation_required) {
                        DEBUG("[LoRa] schedule current frame transmission\n");
                		schedule_tx(ls);
                    } else {
                		DEBUG("[LoRa] awaiting confirmation");
                    }
                } else {
					/* Put transceiver into sleep with low power mode */
                    DEBUG("[LoRa] put transciever to sleep\n");
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
                    DEBUG("[LoRa] stop retransmitting\n");
                    ls->_internal.num_retr = 0;

                    if (ls->appdata_send_failed_cb != NULL) {
                        ls->appdata_send_failed_cb();
                    }
                }
                else {
                    /* Do a retransmission */
                    DEBUG("[LoRa] do a retransmission\n");
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
    DEBUG("[LoRa] creating LS timeouts thread\n");

    kernel_pid_t pid_tim = thread_create(ls->_internal.tim_thread_stack, sizeof(ls->_internal.tim_thread_stack), THREAD_PRIORITY_MAIN - 2,
                                         THREAD_CREATE_STACKTEST, tim_handler, ls,
                                         "LS timeouts thread");

    if (pid_tim <= KERNEL_PID_UNDEF) {
        DEBUG("[LoRa] fail\n");
        return false;
    } else {
        DEBUG("[LoRa] success\n");
    }

    ls->_internal.tim_thread_pid = pid_tim;

    return true;
}

int ls_ed_init(ls_ed_t *ls)
{
    assert(ls != NULL);
    assert(ls->_internal.device != NULL);
    
    p_ls = ls;

    p_ls->_internal.last_fid = 0;
    p_ls->_internal.num_retr = 0;
    p_ls->_internal.is_joined = false;

    if (!p_ls->settings.no_join) {
    	p_ls->_internal.dev_addr = LS_ADDR_UNDEFINED;
    }

    mutex_init(&p_ls->_internal.curr_frame_mutex);
    memset(&p_ls->status, 0, sizeof(ls_device_status_t));

    /* Initialize appdata queue */
    appdata_fifo_init(&p_ls->_internal.appdata_fifo);

    /* Initialize uplink frame queue */
    ls_frame_fifo_init(&p_ls->_internal.uplink_queue);

    /* Start threads */
    if (!create_uq_handler_thread(p_ls)) {
        p_ls->state = LS_ED_FAULT;
        return -LS_INIT_E_SX127X_THREAD;
    }

    if (!create_tim_handler_thread(p_ls)) {
        p_ls->state = LS_ED_FAULT;
        return -LS_INIT_E_TIM_THREAD;
    }

    msg_rx1.content.value = LS_ED_RX1_EXPIRED;
    msg_rx2.content.value = LS_ED_RX2_EXPIRED;
    msg_join_timeout.content.value = LS_ED_JOIN_REQ_EXPIRED;
    msg_ack_timeout.content.value = LS_ED_APPDATA_ACK_EXPIRED;

    DEBUG("[LoRa] init SX127X\n");
    /* Initialize the transceiver */
    if (p_ls->_internal.device->driver->init(p_ls->_internal.device) < 0) {
        return -LS_INIT_E_SX127X_DEVICE;
    }
    
    /* Setup event callback and stack state as it's argument */
    p_ls->_internal.device->event_callback = sx127x_handler;

    ls_ed_sleep(p_ls);

    return LS_OK;
}

void ls_ed_poweroff(ls_ed_t *ls) {
    DEBUG("[LoRa] set transceiver to sleep\n");
    uint8_t state = NETOPT_STATE_SLEEP;
    ls->_internal.device->driver->set(ls->_internal.device, NETOPT_STATE, &state, sizeof(uint8_t));
}

int ls_ed_send_app_data(ls_ed_t *ls, uint8_t *buf, size_t buflen, bool confirmed, bool with_ack, bool delayed)
{
    assert(ls != NULL);
    assert(buf != NULL);

    
    /* Store current app. data in FIFO buffer
     * Data will be removed on receiving ACK from gate */
    if (!delayed && (confirmed || (!ls->settings.no_join && !ls->_internal.is_joined))) {
        DEBUG("[LoRa] pushing data to FIFO\n");
        appdata_fifo_t *fifo = &ls->_internal.appdata_fifo;

        /* Last data has priority, so we can pop oldest item from the queue if it's full */
        if (appdata_fifo_full(fifo)) {
            appdata_fifo_pop(fifo, NULL);
        }

        appdata_fifo_push(fifo, buf, buflen, ls->_internal.last_fid, confirmed, with_ack);
        
        /* Not joined to the network, delay appdata frame until device is joined */
        if (!ls->settings.no_join && !ls->_internal.is_joined) {
            DEBUG("[LoRa] data delayed until node is joined\n");
            return -LS_SEND_E_NOT_JOINED;
        }
        
        if (appdata_fifo_size(fifo) > 1) {
            DEBUG("[LoRa] FIFO is not empty, postpone new data\n");
            
            appdata_fifo_entry_t e;
            appdata_fifo_peek(fifo, &e);
            DEBUG("[LoRa] send oldest data instead [fid: %d, size: %d]\n", e.id, e.size);
            buf = e.data;
            buflen = e.size;
            confirmed = e.is_confirmed;
            with_ack = e.is_with_ack;
        }
    }

    ls->_internal.confirmation_required = false;

    ls_type_t type = LS_UL_UNC;
    if (with_ack) {
        type = LS_UL_UNC_ACK;
    } else {
        if (confirmed) {
            type = LS_UL_CONF;
        }
    }

    DEBUG("[LoRa] send_frame\n");
    int res = send_frame(ls, type, buf, buflen);
    if (res < 0) {
        return res;
    }

    return LS_OK;
}

void ls_ed_unjoin(ls_ed_t *ls)
{
    DEBUG("[LoRa] unjoin node\n");
    
	if (ls->settings.no_join) {
        DEBUG("[LoRa] OTA disabled\n");
		return;
    }

    /* Clear uplink queue */
    ls_frame_fifo_clear(&ls->_internal.uplink_queue);
    ls->_internal.confirmation_required = false;

	/* Stop timers */
    lptimer_remove(&ls->_internal.join_req_expired);
    lptimer_remove(&ls->_internal.conf_ack_expired);
    lptimer_remove(&ls->_internal.rx_window1);
    lptimer_remove(&ls->_internal.rx_window2);

    /* Forget network address */
    ls->_internal.dev_addr = LS_ADDR_UNDEFINED;

    /* Forget session cryptographic keys */
    memset(ls->settings.crypto.aes_key, 0, AES_KEY_SIZE);
    memset(ls->settings.crypto.mic_key, 0, AES_KEY_SIZE);

    /* Mark as not joined */
    ls->_internal.is_joined = false;
    
    DEBUG("[LoRa] node unjoined\n");
}

int ls_ed_join(ls_ed_t *ls)
{
    assert(ls != NULL);
    DEBUG("[LoRa] joining\n");

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
        req.dev_nonce = sx127x_random((sx127x_t *)ls->_internal.device);
    } while (req.dev_nonce == 0);

    ls->_internal.last_nonce = req.dev_nonce;

    /* Reset frame ID */
    ls->_internal.last_fid = 0;

    /* Send request */
    DEBUG("[LoRa] send join request\n");
    send_frame(ls, LS_UL_JOIN_REQ, (uint8_t *) &req, sizeof(ls_join_req_t));

    /* Launch timeout timer */
    DEBUG("[LoRa] set join timeout timer\n");
    lptimer_set_msg(&ls->_internal.join_req_expired, 1000*LS_JOIN_TIMEOUT, &msg_join_timeout, ls->_internal.tim_thread_pid);

    return LS_OK;
}

void ls_ed_sleep(ls_ed_t *ls)
{
    assert(ls != NULL);
    
    DEBUG("[LoRa] put transceiver to sleep\n");

    if (ls->settings.class == LS_ED_CLASS_A) {
        ls->state = LS_ED_SLEEP;
        uint8_t state = NETOPT_STATE_SLEEP;
        ls->_internal.device->driver->set(ls->_internal.device, NETOPT_STATE, &state, sizeof(uint8_t));
    } else {
        DEBUG("[LoRa] ignore sleep, not a Class A\n");
    }
}

int ls_ed_req_time(ls_ed_t *ls)
{
    assert(ls != NULL);

    /* Must be joined or statically activated */
    if (!ls->settings.no_join && !ls->_internal.is_joined) {
    	return LS_SEND_E_NOT_JOINED;
    }

    /* Send time request */
    int res = send_frame(ls, LS_UL_TIME_REQ, NULL, 0);
    if (res < 0) {
        return res;
    }

    return LS_OK;
}

#endif /* !defined(UNWDS_MAC_LORAWAN */

#ifdef __cplusplus
}
#endif
