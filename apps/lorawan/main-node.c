/*
 * Copyright (C) 2016-2019 Unwired Devices LLC <info@unwds.com>

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
 * @ingroup     apps
 * @brief
 * @{
 * @file        main-node.c
 * @brief       LoRaWAN node device based on GNRC LoRaWAN stack
 * @author      Evgeniy Ponomarev
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "thread.h"
#include "periph/pm.h"
#include "pm_layered.h"
#include "periph/rtc.h"
#include "periph/gpio.h"
#include "periph/adc.h"
#include "periph/wdg.h"
#include "lptimer.h"
#include "random.h"
#include "fmt.h"
#include "byteorder.h"
#include "mutex.h"

#include "net/gnrc/netapi.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/pktbuf.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/netif/lorawan_base.h"

#include "sx127x_internal.h"
#include "sx127x_params.h"
#include "sx127x_netdev.h"

#include "ls-settings.h"
#include "ls-config.h"
#include "ls-init-device.h"
#include "ls-frame-fifo.h"

#include "board.h"

#include "unwds-common.h"
#include "unwds-gpio.h"

#include "umdk-ids.h"

#include "main.h"
#include "utils.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define LORAWAN_SENDNEXT_DELAY_MS   10000U
#define LORAWAN_MIN_TX_DELAY_MS     5000U

#define LORAWAN_PORT_DEFAULT    2

/**
 * @brief   Define stack parameters for the MAC layer thread
 */
#define SX127X_STACKSIZE           (THREAD_STACKSIZE_DEFAULT)
#define SX127X_PRIO                (GNRC_NETIF_PRIO)

static sx127x_t sx127x_dev;
static char sx127x_stack[SX127X_STACKSIZE];

typedef enum {
    NODE_MSG_JOIN,
    NODE_MSG_DATA,
} node_message_types_t;

bool lorawan_busy = false;

static msg_t msg_join = { .type = NODE_MSG_JOIN };
static msg_t msg_data = { .type = NODE_MSG_DATA };
static kernel_pid_t sender_pid;
static lptimer_t join_retry_timer;
static lptimer_t data_send_timer;

static kernel_pid_t main_thread_pid;

static char sender_stack[2048];

static gnrc_netif_t *ls;
static ls_frame_fifo_t  fifo_uplink_queue; /* packets to be transmitted */
static ls_frame_fifo_t  fifo_backwater;  /* failed packets */
static mutex_t curr_frame_mutex;

static bool lora_joined = false;
static uint8_t current_join_retries = 0;
static uint8_t uplinks_failed = 0;
static uint32_t lora_frm_cnt = 0;
static uint32_t last_tx_time = 0;

static bool appdata_received(uint8_t *buf, size_t buflen, uint8_t fport);
static void unwds_callback(module_data_t *buf);

/* Receiver */
static void _netreg_cb(uint16_t cmd, gnrc_pktsnip_t *pkt, void *ctx)
{
    (void) cmd;

    uint8_t port = *(uint8_t *)ctx;
    printf("[LoRa] Received %d bytes on port %d\n", pkt->size, port);
    appdata_received(pkt->data, pkt->size, port);

    gnrc_pktbuf_release(pkt);
}

gnrc_netreg_entry_cbd_t _cbd = {
    .cb = _netreg_cb,
    .ctx = NULL
};

/* GNRC_NETREG_DEMUX_CTX_ALL ? */
static gnrc_netreg_entry_t _entry = GNRC_NETREG_ENTRY_INIT_CB(LORAWAN_PORT_DEFAULT, &_cbd);

void radio_init(void)
{
    sx127x_params_t sx127x_params;

    sx127x_params.nss_pin = SX127X_SPI_NSS;
    sx127x_params.spi = SX127X_SPI;

    sx127x_params.dio0_pin = SX127X_DIO0;
    sx127x_params.dio1_pin = SX127X_DIO1;
    sx127x_params.dio2_pin = SX127X_DIO2;
    sx127x_params.dio3_pin = SX127X_DIO3;
    sx127x_params.dio4_pin = SX127X_DIO4;
    sx127x_params.dio5_pin = SX127X_DIO5;
    sx127x_params.reset_pin = SX127X_RESET;

    sx127x_params.rfswitch_pin = SX127X_RFSWITCH;
    sx127x_params.rfswitch_active_level = SX127X_GET_RFSWITCH_ACTIVE_LEVEL();

    sx127x_setup(&sx127x_dev, &sx127x_params);
    ls = gnrc_netif_lorawan_create(sx127x_stack, SX127X_STACKSIZE, SX127X_PRIO,
                                   "sx127x", (netdev_t *)&sx127x_dev);

    gnrc_netreg_register(GNRC_NETTYPE_LORAWAN, &_entry);

    printf("[LoRa] LoRaWAN Network Interface created, PID %d\n", ls->dev_pid);
}

static void ls_setup(gnrc_netif_t *ls)
{
    kernel_pid_t interface = ls->dev_pid;

    uint8_t region = LORA_REGION;
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_REGION, 0, (void *)&region, sizeof(region)) < 0) {
        puts("[LoRa] Unable to set Region");
    }

    uint64_t id = config_get_nodeid();
    uint8_t deveui[LORAMAC_DEVEUI_LEN];
    memcpy(deveui, &id, LORAMAC_DEVEUI_LEN);
    byteorder_swap(deveui, LORAMAC_DEVEUI_LEN);
    if (gnrc_netapi_set(interface, NETOPT_ADDRESS_LONG, 0, deveui, LORAMAC_DEVEUI_LEN) < 0) {
        puts("[LoRa] Unable to set DevEUI");
    }

    id = config_get_appid();
    uint8_t appeui[LORAMAC_APPEUI_LEN];
    memcpy(appeui, &id, LORAMAC_APPEUI_LEN);
    byteorder_swap(appeui, LORAMAC_APPEUI_LEN);
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_APPEUI, 0, appeui, LORAMAC_APPEUI_LEN) < 0) {
        puts("[LoRa] Unable to set AppEUI");
    }

    /* set AppKey for OTAA */
    uint8_t appkey[LORAMAC_APPKEY_LEN];
    memcpy(appkey, config_get_appkey(), LORAMAC_APPKEY_LEN);
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_APPKEY, 0, appkey, LORAMAC_APPKEY_LEN) < 0) {
        puts("[LoRa] Unable to set AppKey");
    }

    /* set AppSKey for ABP */
    uint8_t appskey[LORAMAC_APPSKEY_LEN];
    memcpy(appskey, config_get_appskey(), LORAMAC_APPSKEY_LEN);
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_APPSKEY, 0, appskey, LORAMAC_APPSKEY_LEN) < 0) {
        puts("[LoRa] Unable to set AppSKey");
    }

    /* set NwkSKey for ABP */
    uint8_t nwkskey[LORAMAC_NWKSKEY_LEN];
    memcpy(nwkskey, config_get_nwkskey(), LORAMAC_NWKSKEY_LEN);
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_NWKSKEY, 0, nwkskey, LORAMAC_NWKSKEY_LEN) < 0) {
        puts("[LoRa] Unable to set NwkSKey");
    }

    /* set device address for ABP */
    uint32_t devaddr_u32 = config_get_devnonce();
    /* on Little Endian system, we have to reverse byte order for DevAddr */
    byteorder_swap((void *)&devaddr_u32, sizeof(devaddr_u32));
    if (gnrc_netapi_set(interface, NETOPT_ADDRESS, 0, (void *)&devaddr_u32, sizeof(devaddr_u32)) < 0) {
        puts("[LoRa] Unable to set DevAddr");
    }

    /* disable ADR by default, to be enable on per-packet basis */
    netopt_enable_t adr = (unwds_get_node_settings().adr)? NETOPT_ENABLE:NETOPT_DISABLE;
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_ADR, 0, (void *)&adr, sizeof(netopt_enable_t)) < 0) {
        puts("[LoRa] Unable to set ADR");
    }

    uint8_t dr = unwds_get_node_settings().dr;
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_DR, 0, (void *)&dr, sizeof(dr)) < 0) {
        puts("[LoRa] Unable to set Data Rate");
    }
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_RX2_DR, 0, (void *)&dr, sizeof(dr)) < 0) {
        puts("[LoRa] Unable to set RX2 Data Rate");
    }

    uint8_t tx_power = 0; /* 0 for maximum power */
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_TX_POWER, 0, (void *)&tx_power, sizeof(tx_power)) < 0) {
        puts("[LoRa] Unable to set TX power");
    }

    /*
    semtech_loramac_set_class(ls, unwds_get_node_settings().nodeclass);
    */

    netopt_enable_t otaa = (unwds_get_node_settings().no_join)? (NETOPT_DISABLE):(NETOPT_ENABLE);
    if (gnrc_netapi_set(interface, NETOPT_OTAA, 0, (void *)&otaa, sizeof(netopt_enable_t)) < 0) {
        puts("[LoRa] Unable to set ACK");
    }

    netopt_enable_t ack = (unwds_get_node_settings().confirmation)? (NETOPT_ENABLE):(NETOPT_DISABLE);
    if (gnrc_netapi_set(interface, NETOPT_ACK_REQ, 0, (void *)&ack, sizeof(netopt_enable_t)) < 0) {
        puts("[LoRa] Unable to set ACK");
    }

    uint8_t port = 2; /* default TX port */
    if (gnrc_netapi_set(interface, NETOPT_LORAWAN_TX_PORT, 0, (void *)&port, sizeof(port)) < 0) {
        puts("[LoRa] Unable to set TX port");
    }

    /* initialize FIFO for uplink packets */
    ls_frame_fifo_init(&fifo_uplink_queue);

    /* initialize FIFO for failed packets */
    ls_frame_fifo_init(&fifo_backwater);

    puts("[LoRa] LoRaWAN MAC ready");
}

static void *sender_thread(void *arg) {
    (void)arg;

    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    puts("[LoRa] Sender thread started");

    last_tx_time = lptimer_now_msec();

    while (1) {
        msg_receive(&msg);

        int interface = ls->dev_pid; /* LoRaWAN network interface */
        int res;

        /* minimum interval between transmissions */
        uint32_t tx_delay = 0;
        uint32_t now = lptimer_now_msec();

        if ((last_tx_time < now) && (last_tx_time + LORAWAN_MIN_TX_DELAY_MS > now)) {
            tx_delay = last_tx_time + LORAWAN_MIN_TX_DELAY_MS - now;
            printf("[LoRa] TX delayed by %lu ms\n", tx_delay);
            lptimer_sleep(tx_delay);
        }

        if (msg.type == NODE_MSG_DATA) {
            if (ls_frame_fifo_empty(&fifo_uplink_queue)) {
                puts("[LoRa] FIFO is empty");
                continue;
            }

            /* Get frame from FIFO */
            ls_frame_t frame;
            if (!ls_frame_fifo_peek(&fifo_uplink_queue, &frame)) {
                puts("[LoRa] FIFO error");
                continue;
            }

            if (!lora_joined) {
                puts("[LoRa] Packet delayed: not joined");

                if (current_join_retries == 0) {
                    puts("[LoRa] Attempting to rejoin");
                    msg_send(&msg_join, sender_pid);
                } else {
                    puts("[LoRa] Waiting for the node to join");
                }
                frame.retr_cnt = 0;

                /* move frame to backwaters */
                ls_frame_fifo_pop(&fifo_uplink_queue, NULL);
                if (ls_frame_fifo_full(&fifo_backwater)) {
                    ls_frame_fifo_pop(&fifo_backwater, NULL);
                }
                ls_frame_fifo_push(&fifo_backwater, &frame);
                continue;
            }

            if (frame.retr_cnt) {
                /* retransmissions should have the same frame counter as original package */
                gnrc_netapi_set(interface, NETOPT_LORAWAN_FRAMECOUNTER, 0, (void *)&lora_frm_cnt, sizeof(lora_frm_cnt));
                puts("[LoRa] Packet retransmission");
            } else {
                uint32_t u32;
                res = gnrc_netapi_get(interface, NETOPT_LORAWAN_FRAMECOUNTER, 0, &u32, sizeof(u32));
                if (res >= 0) {
                    lora_frm_cnt = u32;
                }
            }

            gnrc_pktsnip_t *pkt;

            pkt = gnrc_pktbuf_add(NULL, frame.data, frame.length, GNRC_NETTYPE_UNDEF);

            /* register for returned packet status */
            if (gnrc_neterr_reg(pkt) != 0) {
                puts("[LoRa] Can't register for error reporting");
                continue;
            }

            uint8_t port = frame.fport;  /* LoRaWAN FPort */
            gnrc_netapi_set(interface, NETOPT_LORAWAN_TX_PORT, 0, &port, sizeof(port));
            gnrc_netapi_send(interface, pkt);

            blink_led(LED0_PIN);

            msg_t _timeout_msg;
            _timeout_msg.type = GNRC_NETERR_MSG_TYPE;
            _timeout_msg.content.value = 111; /* Some error */
            lptimer_t _timeout_timer;
            lptimer_set_msg(&_timeout_timer, 10000, &_timeout_msg, sender_pid);

            /* wait for packet status and check */
            msg_t msg;
            do {
                msg_receive(&msg);
            } while (msg.type != GNRC_NETERR_MSG_TYPE);

            lptimer_remove(&_timeout_timer);

            if (msg.content.value == 111) {
                puts("[LoRa] GNRC timeout");
            }

            if (msg.content.value != GNRC_NETERR_SUCCESS) {
                puts("[LoRa] Error sending data");
                
                if (++frame.retr_cnt > unwds_get_node_settings().max_retr) {
                    /* probably packet was received, but downlink ACK was lost          */
                    /* no reason to try further, brocaar doesn't send ACK twice anyway  */
                    /* let's store packet in backwaters just in case                    */
                    frame.retr_cnt = 0;
                    ls_frame_fifo_push(&fifo_backwater, &frame);
                    /* discard packet from uplink queue */
                    ls_frame_fifo_pop(&fifo_uplink_queue, NULL);
                    puts("[LoRa] Packet moved to backwaters");

                    if (ls_frame_fifo_full(&fifo_backwater)) {
                        /* backwaters FIFO is full, time to rejoin */
                        lora_joined = false;
                        current_join_retries = 0;
                        frame.retr_cnt = 0;
                        msg_send(&msg_join, sender_pid);
                        puts("[LoRa] Too many uplinks failed, rejoining");
                    } else {
                        /* set lower datarate */
                        uint8_t dr;
                        gnrc_netapi_get(interface, NETOPT_LORAWAN_DR, 0, (void *)&dr, sizeof(dr));
                        if (dr > 0) {
                            dr -= 1;
                            gnrc_netapi_set(interface, NETOPT_LORAWAN_DR, 0, (void *)&dr, sizeof(dr));
                            puts("[LoRa] Datarate lowered");
                        }
                    }
                } else {
                    /* update retr_cnt value in FIFO */
                    ls_frame_fifo_replace(&fifo_uplink_queue, &frame);

                    /* TX power back to max after several retries */
                    if (frame.retr_cnt == unwds_get_node_settings().max_retr/2) {
                        uint8_t tx_power = 0;
                        gnrc_netapi_set(interface, NETOPT_LORAWAN_TX_POWER, 0, (void *)&tx_power, sizeof(tx_power));
                        puts("[LoRa] TX power set to maximum");
                    }
                }
                last_tx_time = lptimer_now_msec();
            }
            else {
                puts("[LoRa] Data successfully sent");
                /* LoRa link obviously ok, clear backwaters FIFO */
                ls_frame_fifo_clear(&fifo_backwater);
                /* remove transmitted frame from FIFO */
                ls_frame_fifo_pop(&fifo_uplink_queue, NULL);
                last_tx_time = lptimer_now_msec();
            }

            if (!ls_frame_fifo_empty(&fifo_uplink_queue)) {
                int pktleft = ls_frame_fifo_size(&fifo_uplink_queue);
                unsigned int delay = random_uint32_range(LORAWAN_SENDNEXT_DELAY_MS - LORAWAN_SENDNEXT_DELAY_MS/4, LORAWAN_SENDNEXT_DELAY_MS + LORAWAN_SENDNEXT_DELAY_MS/4);
                printf("[LoRa] %d packet%s in the queue, sending next in %d s\n", pktleft, (pktleft > 1)?"s":"", delay/1000);
                lptimer_set_msg(&data_send_timer, delay, &msg_data, sender_pid);
            } else {
                lptimer_remove(&data_send_timer);
            }
        }

        if (msg.type == NODE_MSG_JOIN) {
            /* limit max delay between attempts to 1 hour */
            if (current_join_retries < 120) {
                current_join_retries++;
            }

            blink_led(LED0_PIN);

            if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A) {
                printf("[LoRa] Joining (%d of %d)\n", current_join_retries, unwds_get_node_settings().max_retr + 1);
            } else {
                puts("[LoRa] Joining");
            }

            lorawan_joinerr_reg();

            netopt_enable_t join = NETOPT_ENABLE;
            if (gnrc_netapi_set(ls->dev_pid, NETOPT_LINK_CONNECTED, 0, (void *)&join, sizeof(netopt_enable_t)) < 0) {
                puts("[LoRa] Unable to join");
                continue;
            }

            msg_t _timeout_msg;
            _timeout_msg.type = GNRC_NETERR_MSG_TYPE;
            _timeout_msg.content.value = 111; /* Some error */
            lptimer_t _timeout_timer;
            lptimer_set_msg(&_timeout_timer, 10000, &_timeout_msg, sender_pid);

            msg_t msg;
            do {
                msg_receive(&msg);
            } while (msg.type != GNRC_NETERR_MSG_TYPE);

            if (msg.content.value == 111) {
                puts("[LoRa] GNRC timeout");
            }

            lptimer_remove(&_timeout_timer);

            if (msg.content.value == GNRC_NETERR_SUCCESS) {
                /* joined */
                uplinks_failed = 0;
                current_join_retries = 0;
                lora_joined = true;
                puts("[LoRa] Successfully joined");
                last_tx_time = lptimer_now_msec();

                /* move packets from backwaters back to uplink */
                if (!ls_frame_fifo_empty(&fifo_backwater)) {
                    if (ls_frame_fifo_full(&fifo_uplink_queue)) {
                        /* uplink already filled full with new packets */
                        ls_frame_fifo_clear(&fifo_backwater);
                    } else {
                        /* move backwaters to uplink queue */
                        memcpy((void *)&fifo_uplink_queue, (void *)&fifo_backwater, sizeof(ls_frame_fifo_t));
                        ls_frame_fifo_clear(&fifo_backwater);
                    }
                }

                /* transmitting initial packet with module data for Class C devices */
                if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_C) {
                    module_data_t data = {};

                    /* first byte */
                    data.data[0] = UNWDS_LORAWAN_SYSTEM_MODULE_ID;
                    data.length++;

                    /* second byte - device class and settings */
                    /* bits 0-1: device class */
                    switch (unwds_get_node_settings().nodeclass) {
                        case (LS_ED_CLASS_A):
                            break;
                        case (LS_ED_CLASS_B):
                            data.data[1] = 1 << 0;
                            break;
                        case (LS_ED_CLASS_C):
                            data.data[1] = 1 << 1;
                            break;
                        default:
                            break;
                    }

                    /* bit 2: ADR */
                    if (unwds_get_node_settings().adr) {
                        data.data[1] |= 1 << 2;
                    }

                    /* bit 3: CNF */
                    if (unwds_get_node_settings().confirmation) {
                        data.data[1] |= 1 << 3;
                    }

                    /* bit 7: FPort usage for module addressing */
                    data.data[1] |= 1 << 7;

                    data.length++;

                    unwds_callback(&data);
                }
            } else {
                /* not joined */
                last_tx_time = lptimer_now_msec();

                if ((current_join_retries > unwds_get_node_settings().max_retr) &&
                    (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A)) {
                    /* class A node: go to sleep */
                    puts("[LoRa] Join failed. Maximum join retries exceeded, stopping");
                    current_join_retries = 0;
                } else {
                    /* Pseudorandom delay for collision avoidance */
                    unsigned int delay = random_uint32_range(30000 + (current_join_retries - 1)*60000, 90000 + (current_join_retries - 1)*60000);
                    printf("[LoRa] Join failed, try again in %d s\n", delay/1000);
                    lptimer_set_msg(&join_retry_timer, delay, &msg_join, sender_pid);
                }
            }
        }
    }
    return NULL;
}

static bool appdata_received(uint8_t *buf, size_t buflen, uint8_t fport)
{
#if ENABLE_DEBUG
    char hex[100] = {};
    bytes_to_hex(buf, buflen, hex, false);
    printf("[LoRa] Received data: \"%s\"\n", hex);
#endif

    blink_led(LED0_PIN);

#if defined LORAWAN_DONT_USE_FPORT
    (void)fport;

    unwds_module_id_t modid = buf[0];

    module_data_t cmd;
    /* Save command data */
    memcpy(cmd.data, buf + 1, buflen - 1);
    cmd.length = buflen - 1;
#else
    unwds_module_id_t modid = fport;

    module_data_t cmd;
    /* Save command data */
    memcpy(cmd.data, buf, buflen);
    cmd.length = buflen;
#endif

    /* Send command to the module */
    module_data_t reply = {};

    /* Send app. data */
    int result = unwds_send_to_module(modid, &cmd, &reply);

    if (result == UNWDS_MODULE_NOT_FOUND) {
        /* No module with specified ID present */
        reply.as_ack = true;
        reply.length = 2;
        reply.data[0] = UNWDS_MODULE_NOT_FOUND;
        reply.data[1] = modid;
    }

    if (result != UNWDS_MODULE_NO_DATA) {
        /* wait a bit */
        lptimer_sleep(3000);
        unwds_callback(&reply);
    }

    /* Don't allow to send app. data ACK by the network.
     * The ACK will be sent either by the callback with the actual app. data or
     * with the command response itself */
    return false;
}

int ls_set_cmd(int argc, char **argv)
{
    if (argc != 3) {
        puts("usage: set <key> <value>");
        puts("keys:");
        if (unwds_get_node_settings().no_join)
        	puts("\taddr <address> -- sets predefined device address for statically personalized devices");

        puts("\totaa <0/1> -- select between OTAA and ABP");
//        puts("\tch <ch> -- sets device channel in selected region");
        puts("\tdr <0-6> -- sets default data rate [0 - slowest, 3 - average, 6 - fastest]");
        puts("\tmaxretr <0-5> -- sets maximum number of retransmissions of confirmed app. data [2 is recommended]");
        puts("\tclass <A/C> -- sets device class");
        puts("\tadr <0/1> -- enable or disable ADR");
        puts("\tcnf <0/1> -- enable or disable messages confirmation");
    }

    char *key = argv[1];
    char *value = argv[2];

    if (strcmp(key, "otaa") == 0) {
        int v = atoi(value);
        if (v) {
            unwds_set_nojoin(false);
        } else {
            unwds_set_nojoin(true);
        }
    }

    if (strcmp(key, "cnf") == 0) {
        int v = atoi(value);
        if (v) {
            unwds_set_cnf(true);
        } else {
            unwds_set_cnf(false);
        }
    }

    if (strcmp(key, "maxretr") == 0) {
        int v = atoi(value);
        if (v > 5) {
            v = 5;
        }
        unwds_set_max_retr(v);
    }

    if (strcmp(key, "adr") == 0) {
        int v = atoi(value);
        unwds_set_adr(v);
    }

    if (strcmp(key, "class") == 0) {
        char v = value[0];

        if (v != 'A' && v != 'C') {
            puts("set сlass: A or C");
            return 1;
        }

        if (v == 'A') {
            unwds_set_class(LS_ED_CLASS_A);
        }
        /*
        else if (v == 'B') {
            unwds_set_class(LS_ED_CLASS_B);
        }
        */
        else if (v == 'C') {
            unwds_set_class(LS_ED_CLASS_C);
        }
    }

    return 0;
}

static void print_config(void)
{
    puts("[ node configuration ]");

    uint64_t eui64 = config_get_nodeid();
    uint64_t appid = config_get_appid();

    printf("OTAA = %s\n", (unwds_get_node_settings().no_join) ? "no" : "yes");

#if DISPLAY_JOINKEY_2BYTES
    uint8_t *key;
    if (unwds_get_node_settings().no_join) {
        key = config_get_appskey();
        printf("AppsKey = 0x....%02X%02X\n", key[14], key[15]);

        key = config_get_nwkskey();
        printf("NwksKey = 0x....%02X%02X\n", key[14], key[15]);
    } else {
        key = config_get_appkey();
        printf("AppKey = 0x....%02X%02X\n", key[14], key[15]);
    }
#endif

    if (unwds_get_node_settings().no_join) {
    	printf("DevAddr = 0x%08X\n", (unsigned)config_get_devnonce());
    }

    printf("DevEUI = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    printf("AppEUI = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));

    switch (LORA_REGION) {
        case 0:
            puts("REGION = EU868");
            break;
        case 1:
            puts("REGION = RU864");
            break;
        case 2:
            puts("REGION = KZ865");
            break;
        default:
            printf("REGION = <unknown> (%d)\n", LORA_REGION);
            break;
    }

    printf("DATARATE = %d\n", unwds_get_node_settings().dr);

    printf("ADR = %s\n", (unwds_get_node_settings().adr)?  "yes" : "no");

    printf("CONFIRMED = %s\n", (unwds_get_node_settings().confirmation) ? "yes" : "no");

    char nodeclass = 'A'; // unwds_get_node_settings().nodeclass == LS_ED_CLASS_A
    if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_B) {
        nodeclass = 'B';
    }
    else if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_C) {
        nodeclass = 'C';
    }
    printf("CLASS = %c\n", nodeclass);

    printf("MAXRETR = %d\n", unwds_get_node_settings().max_retr);

    puts("[ enabled modules ]");
    unwds_list_modules(unwds_get_node_settings().enabled_mods, true);
}

static int ls_printc_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    print_config();

    return 0;
}

int ls_cmd_cmd(int argc, char **argv)
{
    if (argc < 2) {
        puts("Usage: cmd <modid> <cmdhex>");
        return 1;
    }

    uint8_t modid = atoi(argv[1]);
    if (!unwds_is_module_exists(modid)) {
        printf("[LORa] Module ID %d not found\n", modid);
        return 1;
    }

    module_data_t cmd = {};

    int len = strlen(argv[2]);
    if (len % 2 != 0) {
        puts("[LoRa] Invalid hex number");
        return 1;
    }

    if (len / 2 > UNWDS_MAX_DATA_LEN) {
        printf("[LoRa] Command too long (%d > %d bytes)\n", len / 2, UNWDS_MAX_DATA_LEN);
        return 1;
    }

    hex_to_bytes(argv[2], cmd.data, false);
    cmd.length = len / 2;

    /* No RSSI from console commands */
    cmd.rssi = 0;

    module_data_t reply = {};
    bool res = unwds_send_to_module(modid, &cmd, &reply);
    char replystr[2 * UNWDS_MAX_DATA_LEN] = {};
    bytes_to_hex(reply.data, reply.length, replystr, false);

    if (res) {
        printf("[ok] Reply: %s\n", replystr);
    }
    else {
        printf("[fail] Reply: %s\n", replystr);
    }

    return 0;
}

static int ls_listmodules_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    puts("[ available modules ]");
    unwds_list_modules(unwds_get_node_settings().enabled_mods, false);

    return 0;
}

static int ls_module_cmd(int argc, char **argv)
{
    if (argc < 3) {
        puts("Usage: mod <name> <enable|disable>. Example: mod adc enable");
        return 1;
    }

    int modid = 0;

    if (is_number(argv[1])) {
        modid = atoi(argv[1]);
    } else {
        modid = unwds_modid_by_name(argv[1]);
    }

    if (modid < 0) {
        printf("[LoRa] Module %s not found\n", argv[1]);
        return 1;
    }

    if (!unwds_is_module_exists(modid)) {
        printf("LoRa] Module %d not found\n", modid);
        return 1;
    }

    bool modenable = false;
    if (is_number(argv[2])) {
        modenable = atoi(argv[2]);
    } else {
        if (strcmp(argv[2], "enable") == 0) {
            modenable = true;
        } else {
            if (strcmp(argv[2], "disable") != 0) {
                printf("[LoRa] Unknown command: %s\n", argv[2]);
                return 1;
            }
        }
    }

    unwds_set_module(modid, modenable);

    return 0;
}

static int ls_safe_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;

#if defined RTC_REGBACKUP_BOOTMODE
    uint32_t bootmode = UNWDS_BOOT_SAFE_MODE;
    rtc_save_backup(bootmode, RTC_REGBACKUP_BOOTMODE);
#endif
    puts("Rebooting in safe mode");
    NVIC_SystemReset();
    return 0;
}

static int ls_join_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;

    msg_send(&msg_join, sender_pid);
    return 0;
}

shell_command_t shell_commands[UNWDS_SHELL_COMMANDS_MAX] = {
    { "set", "<config> <value> -- set value for the configuration entry", ls_set_cmd },
    { "lscfg", "-- print out current configuration", ls_printc_cmd },
    { "lsmod", "-- list available modules", ls_listmodules_cmd },
    { "mod", "<name> <enable|disable>	-- disable or enable selected module", ls_module_cmd },
    { "cmd", "<modid> <cmdhex> -- send command to another UNWDS device", ls_cmd_cmd },
    { "safe", " -- reboot in safe mode", ls_safe_cmd },
    { "join", " -- join now", ls_join_cmd },
    { NULL, NULL, NULL },
};

static void unwds_callback(module_data_t *buf)
{
    /* get MCU temperature and supply voltage */
    cpu_update_status();
    int8_t temperature = INT8_MIN;
    uint8_t voltage = 0;

    if (cpu_status.temp.core_temp != INT16_MIN) {
        temperature = cpu_status.temp.core_temp;
    }
    if (cpu_status.voltage.vdd != INT16_MIN) {
        voltage = cpu_status.voltage.vdd/50;
    }
    printf("[LoRa] MCU is %d C, battery is %d mV\n", temperature, voltage * 50);
    convert_to_be_sam((void *)&temperature, 1);

    mutex_lock(&curr_frame_mutex);

    ls_frame_t frame;

    /* this is a new frame */
    frame.retr_cnt = 0;

    /* move module ID to FPort */
    frame.fport = buf->data[0];
    memcpy(frame.data, &buf->data[1], buf->length - 1);

    /* 1 byte moved to FPort, 2 bytes to be added */
    frame.length = buf->length + 1;

    if (frame.length > 32) {
        printf("[LoRa] Payload too big (%d > 32)\n", frame.length);
        return;
    }

    printf("[LoRa] Payload %d bytes\n", frame.length);

    frame.data[frame.length - 2] = temperature;
    frame.data[frame.length - 1] = voltage;

#if ENABLE_DEBUG
    for (int k = 0; k < frame.length; k++) {
        printf("%02X ", frame.data[k]);
    }
    printf("\n");
#endif

    /* push frame to FIFO */
    if (ls_frame_fifo_full(&fifo_uplink_queue)) {
        DEBUG("[LoRa] remove oldest frame from FIFO\n");
        ls_frame_fifo_pop(&fifo_uplink_queue, NULL);
    }

    if (!ls_frame_fifo_push(&fifo_uplink_queue, &frame)) {
    	mutex_unlock(&curr_frame_mutex);
        DEBUG("[LoRa] FIFO error\n");
        return;
    }

    /* send data */
    msg_send(&msg_data, sender_pid);

    mutex_unlock(&curr_frame_mutex);
}

static int unwds_init(void) {
    radio_init();
    ls_setup(ls);

    return 0;
}

static void unwds_join(void) {
    msg_send(&msg_join, sender_pid);
}

static void unwds_sleep(void) {
    /* TBD */
    /*
    semtech_loramac_set_class(&ls, LS_ED_CLASS_A);
    */
}

void init_normal(shell_command_t *commands)
{
    /* should always be 2 */
    main_thread_pid = thread_getpid();

    bool cfg_valid = unwds_config_load();
    print_config();

    if (!cfg_valid) {
        puts("[!] Device is not configured yet. Type \"help\" to see list of possible configuration commands.");
        puts("[!] Configure the node and type \"reboot\" to reboot and apply settings.");
    } else {
        sender_pid = thread_create(sender_stack, sizeof(sender_stack), THREAD_PRIORITY_MAIN - 2,
                                   THREAD_CREATE_STACKTEST, sender_thread, NULL,  "LoRa sender thread");

        unwds_device_init(unwds_callback, unwds_init, unwds_join, unwds_sleep);
    }

    /* Add our commands to shell */
    int i = 0;
    do {
        i++;
    } while (commands[i].name);

    int k = 0;
    do {
        k++;
    } while (shell_commands[k].name);

    assert(i + k < UNWDS_SHELL_COMMANDS_MAX - 1);

    memcpy((void *)&commands[i], (void *)shell_commands, k*sizeof(shell_commands[i]));
}

#ifdef __cplusplus
}
#endif
