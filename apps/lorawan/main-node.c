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
 * @file        main-node.c
 * @brief       LoRaLAN node device
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

#include "net/lora.h"
#include "net/netdev.h"
#include "net/loramac.h"
#include "semtech_loramac.h"
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

typedef enum {
    NODE_MSG_JOIN,
    NODE_MSG_SEND,
} node_message_types_t;

static msg_t msg_join = { .type = NODE_MSG_JOIN };
static msg_t msg_data = { .type = NODE_MSG_SEND };
static kernel_pid_t sender_pid;
static kernel_pid_t receiver_pid;
static lptimer_t join_retry_timer;
static lptimer_t send_timer;

static kernel_pid_t main_thread_pid;

static char sender_stack[2048];
static char receiver_stack[2048];

static semtech_loramac_t ls;
static ls_frame_fifo_t  fifo_lorapacket;
static mutex_t curr_frame_mutex;

static uint8_t current_join_retries = 0;
static uint8_t uplinks_failed = 0;
static uint32_t lora_frm_cnt = 0;
static uint32_t last_tx_time = 0;

static bool appdata_received(uint8_t *buf, size_t buflen, uint8_t fport);
static void unwds_callback(module_data_t *buf);

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
    
    if (semtech_loramac_init(&ls, &sx127x_params) == 0) {
        puts("[LoRa] LoRaMAC successfully initialized");
    } else {
        puts("[LoRa] LoRaMAC initialization failed");
    }
}

static int node_join(semtech_loramac_t *ls) {
    /* limit max delay between attempts to 1 hour */
    if (current_join_retries < 120) {
        current_join_retries++;
    }
    
    blink_led(LED0_PIN);
    
    if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A) {
        printf("[LoRa] joining, attempt %d / %d\n", current_join_retries, unwds_get_node_settings().max_retr + 1);
    } else {
        puts("[LoRa] joining");
    }
    
    uint8_t join_type = (unwds_get_node_settings().no_join)? LORAMAC_JOIN_ABP: LORAMAC_JOIN_OTAA;

    return (semtech_loramac_join(ls, join_type));
}

static void *sender_thread(void *arg) {
    semtech_loramac_t *ls = (semtech_loramac_t *)arg;
    
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);
    
    puts("[LoRa] sender thread started");
    
    last_tx_time = lptimer_now().ticks32;
    
    while (1) {
        msg_receive(&msg);

        int res;
        
        /* minimum interval between transmissions */
        uint32_t tx_delay = 0;
        uint32_t now = lptimer_now().ticks32;

        if (last_tx_time + LORAWAN_MIN_TX_DELAY_MS > now) {
            tx_delay = last_tx_time + LORAWAN_MIN_TX_DELAY_MS - now;
            printf("[LoRa] delaying TX by %lu ms\n", tx_delay);
            lptimer_sleep(tx_delay);
        }
        
        if (msg.type == NODE_MSG_SEND) {
            if (ls_frame_fifo_empty(&fifo_lorapacket)) {
                puts("[LoRa] FIFO empty, nothing to send");
                continue;
            }
            
            /* Get frame from FIFO */
            ls_frame_t frame;
            if (!ls_frame_fifo_peek(&fifo_lorapacket, &frame)) {
                printf("[LoRa] error getting frame from FIFO\n");
                continue;
            }

            if (frame.retransmit) {
                /* retransmissions should have the same frame counter as original package */
                semtech_loramac_set_uplink_counter(ls, lora_frm_cnt);
                puts("[LoRa] packet retransmission");
            } else {
                lora_frm_cnt = semtech_loramac_get_uplink_counter(ls);
                puts("[LoRa] sending new packet");
            }
            
            res = semtech_loramac_send(ls, frame.data, frame.length);

            switch (res) {
                case SEMTECH_LORAMAC_BUSY:
                    puts("[LoRa] MAC already busy");
                    frame.retransmit = true;
                    ls_frame_fifo_replace(&fifo_lorapacket, &frame);
                    break;
                case SEMTECH_LORAMAC_DUTYCYCLE_RESTRICTED:
                    puts("[LoRa] TX duty cycle restricted");
                    frame.retransmit = true;
                    ls_frame_fifo_replace(&fifo_lorapacket, &frame);
                    break;
                case SEMTECH_LORAMAC_NO_FREE_CHANNEL:
                    puts("[LoRa] LBT no free channels");
                    frame.retransmit = true;
                    ls_frame_fifo_replace(&fifo_lorapacket, &frame);
                    break;
                case SEMTECH_LORAMAC_NOT_JOINED: {
                    puts("[LoRa] not joined to the network");

                    if (current_join_retries == 0) {
                        puts("[LoRa] attempting to rejoin");
                        msg_send(&msg_join, sender_pid);
                    } else {
                        puts("[LoRa] waiting for the node to join");
                    }
                    frame.retransmit = true;
                    ls_frame_fifo_replace(&fifo_lorapacket, &frame);
                    break;
                }
                case SEMTECH_LORAMAC_TX_OK:
                    puts("[LoRa] TX is in progress");
                    break;
                case SEMTECH_LORAMAC_TX_DONE:
                    puts("[LoRa] TX done");
                    
                    /* remove transmitted frame from FIFO */
                    ls_frame_fifo_pop(&fifo_lorapacket, NULL);
                    break;
                case SEMTECH_LORAMAC_TX_CNF_FAILED:
                    puts("[LoRa] uplink confirmation failed");
                    uplinks_failed++;
                    
                    if (uplinks_failed > unwds_get_node_settings().max_retr) {
                        puts("[LoRa] too many uplinks failed, rejoining");
                        current_join_retries = 0;
                        uplinks_failed = 0;
                        msg_send(&msg_join, sender_pid);
                        frame.retransmit = false;
                    } else {
                        frame.retransmit = true;
                    }
                    ls_frame_fifo_replace(&fifo_lorapacket, &frame);
                    break;
                default:
                    /* fix if ever happened */
                    printf("[LoRa] send: unknown response %d\n", res);
                    /* remove frame from FIFO */
                    ls_frame_fifo_pop(&fifo_lorapacket, NULL);
                    break;
            }
            
            if (!ls_frame_fifo_empty(&fifo_lorapacket)) {
                printf("[LoRa] queue not empty, sending next packet in %d sec\n", LORAWAN_SENDNEXT_DELAY_MS/1000);
                lptimer_set_msg(&send_timer, LORAWAN_SENDNEXT_DELAY_MS, &msg_data, sender_pid);
            } else {
                lptimer_remove(&send_timer);
            }
        }
        
        if (msg.type == NODE_MSG_JOIN) {
            res = node_join(ls);

            switch (res) {
            case SEMTECH_LORAMAC_JOIN_SUCCEEDED: {
                current_join_retries = 0;
                puts("[LoRa] successfully joined to the network");
                
                /* transmitting a packet with module data */
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
                break;
            }
            case SEMTECH_LORAMAC_BUSY:
            case SEMTECH_LORAMAC_NOT_JOINED:
            case SEMTECH_LORAMAC_JOIN_FAILED:
            case SEMTECH_LORAMAC_DUTYCYCLE_RESTRICTED:
            {
                printf("[LoRa] join failed: code %d\n", res);
                if ((current_join_retries > unwds_get_node_settings().max_retr) &&
                    (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A)) {
                    /* class A node: go to sleep */
                    puts("[LoRa] maximum join retries exceeded, stopping");
                    current_join_retries = 0;
                } else {
                    puts("[LoRa] join request timed out, resending");
                    
                    /* Pseudorandom delay for collision avoidance */
                    unsigned int delay = random_uint32_range(30000 + (current_join_retries - 1)*60000, 90000 + (current_join_retries - 1)*60000);
                    printf("[LoRa] random delay %d s\n", delay/1000);
                    lptimer_set_msg(&join_retry_timer, delay, &msg_join, sender_pid);
                }
                break;
            }
            default:
                printf("[LoRa] join request: unknown response %d\n", res);
                /* Pseudorandom delay for collision avoidance */
                unsigned int delay = random_uint32_range(30000 + (current_join_retries - 1)*60000, 90000 + (current_join_retries - 1)*60000);
                printf("[LoRa] random delay %d s\n", delay/1000);
                lptimer_set_msg(&join_retry_timer, delay, &msg_join, sender_pid);
                break;
            }
        }
        last_tx_time = lptimer_now().ticks32;
    }
    return NULL;
}

static void *receiver_thread(void *arg) {
    semtech_loramac_t *ls = (semtech_loramac_t *)arg;
    
    puts("[LoRa] receiver thread started");
    
    while (1) {
        int res = semtech_loramac_recv(ls);
        switch (res) {
            case SEMTECH_LORAMAC_RX_DATA: {
                printf("[LoRa] data received: %d bytes, port %d, RSSI %d, DR %d\n",
                        ls->rx_data.payload_len,
                        ls->rx_data.port,
                        ls->rx_data.rssi,
                        ls->rx_data.datarate);
#if ENABLE_DEBUG
                printf("[LoRa] hex data: ");
                for (int l = 0; l < ls->rx_data.payload_len; l++) {
                    printf("%02X ", ls->rx_data.payload[l]);
                }
                printf("\n");
#endif
                appdata_received(ls->rx_data.payload, ls->rx_data.payload_len, ls->rx_data.port);
                break;
            }
            case SEMTECH_LORAMAC_RX_LINK_CHECK: {
                printf("[LoRa] link check: margin %d, gateways %d\n",
                        ls->link_chk.demod_margin,
                        ls->link_chk.nb_gateways);
                break;
            }
            case SEMTECH_LORAMAC_RX_CONFIRMED: {
                puts("[LoRa] ack received");
                break;
            }
            default:
                printf("[LoRa] unknown LoRaMAC response %d\n", res);
                break;
        }
    }
    return NULL;
}

static bool appdata_received(uint8_t *buf, size_t buflen, uint8_t fport)
{
#if ENABLE_DEBUG
    char hex[100] = {};
    bytes_to_hex(buf, buflen, hex, false);
    printf("[LoRa] received data: \"%s\"\n", hex);
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

static void ls_setup(semtech_loramac_t *ls)
{
    uint64_t id = config_get_nodeid();
    uint8_t deveui[LORAMAC_DEVEUI_LEN];
    memcpy(deveui, &id, LORAMAC_DEVEUI_LEN);
    byteorder_swap(deveui, LORAMAC_DEVEUI_LEN);
    semtech_loramac_set_deveui(ls, deveui);
    
    id = config_get_appid();
    uint8_t appeui[LORAMAC_APPEUI_LEN];
    memcpy(appeui, &id, LORAMAC_APPEUI_LEN);
    byteorder_swap(appeui, LORAMAC_APPEUI_LEN);
    semtech_loramac_set_appeui(ls, appeui);
    
    /* set AppKey for OTAA */
    uint8_t appkey[LORAMAC_APPKEY_LEN];
    memcpy(appkey, config_get_appkey(), LORAMAC_APPKEY_LEN);
    semtech_loramac_set_appkey(ls, appkey);

    /* set AppSKey for ABP */
    uint8_t appskey[LORAMAC_APPSKEY_LEN];
    memcpy(appskey, config_get_appskey(), LORAMAC_APPKEY_LEN);
    semtech_loramac_set_appskey(ls, appskey);
    
    /* set NwkSKey for ABP */
    uint8_t nwkskey[LORAMAC_NWKSKEY_LEN];
    memcpy(nwkskey, config_get_nwkskey(), LORAMAC_APPKEY_LEN);
    semtech_loramac_set_nwkskey(ls, nwkskey);
    
    /* set device address for ABP */
    uint32_t devaddr_u32 = config_get_devnonce();
    /* on Little Endian system, we have to reverse byte order for DevAddr */
    byteorder_swap((void *)&devaddr_u32, sizeof(devaddr_u32));
    semtech_loramac_set_devaddr(ls, (void *)&devaddr_u32);
    
    //semtech_loramac_set_netid(ls, 0xAB130C);

    semtech_loramac_set_dr(ls, unwds_get_node_settings().dr);
    
    semtech_loramac_set_adr(ls, unwds_get_node_settings().adr);
    semtech_loramac_set_class(ls, unwds_get_node_settings().nodeclass);
    
    /* Retries will be handled by application */
    /* semtech_loramac_set_retries(ls, unwds_get_node_settings().max_retr);*/ 
    semtech_loramac_set_retries(ls, 0);
    
    if (unwds_get_node_settings().confirmation) {
        semtech_loramac_set_tx_mode(ls, LORAMAC_TX_CNF);   /* confirmed packets */
    } else {
        semtech_loramac_set_tx_mode(ls, LORAMAC_TX_UNCNF); /* unconfirmed packets */
    }

    semtech_loramac_set_tx_port(ls, LORAMAC_DEFAULT_TX_PORT); /* port 2 */
    
    /* initialize FIFO for uplink packets */
    ls_frame_fifo_init(&fifo_lorapacket);
    
    puts("[LoRa] LoRaMAC values set");
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
    
    printf("REGION = %s\n", LORA_REGION);

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
        printf("cmd: module with ID %d does not exists\n", modid);
        return 1;
    }

    module_data_t cmd = {};

    int len = strlen(argv[2]);
    if (len % 2 != 0) {
        puts("cmd: invalid hex number");
        return 1;
    }

    if (len / 2 > UNWDS_MAX_DATA_LEN) {
        printf("cmd: command too long. Maximum is %d bytes\n", UNWDS_MAX_DATA_LEN);
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
        printf("mod: module %s does not exist\n", argv[1]);
        return 1;
    }
    
    if (!unwds_is_module_exists(modid)) {
        printf("mod: module with ID %d does not exist\n", modid);
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
                printf("mod: unknown command: %s\n", argv[2]);
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
        printf("[LoRa] MCU temperature is %d C\n", temperature);
    }
    if (cpu_status.voltage.vdd != INT16_MIN) {
        voltage = cpu_status.voltage.vdd/50;
        printf("[LoRa] Battery voltage %d mV\n", voltage * 50);
    }
    convert_to_be_sam((void *)&temperature, 1);
    
    mutex_lock(&curr_frame_mutex);
    
    ls_frame_t frame;
    
    /* this is a new frame */
    frame.retransmit = false;

    /* move module ID to FPort */
    frame.fport = buf->data[0];
    memcpy(frame.data, &buf->data[1], buf->length - 1);

    /* 1 byte moved to FPort, 2 bytes to be added */
    frame.length = buf->length + 1;
    
    if (frame.length > 32) {
        printf("[LoRa] payload too big (%d bytes)\n", frame.length);
        return;
    }
    
    printf("[LoRa] payload size %d bytes\n", frame.length);

    frame.data[frame.length - 2] = temperature;
    frame.data[frame.length - 1] = voltage;
    
#if ENABLE_DEBUG
    for (int k = 0; k < frame.length; k++) {
        printf("%02X ", frame.data[k]);
    }
    printf("\n");
#endif
    
    /* push frame to FIFO */
    if (ls_frame_fifo_full(&fifo_lorapacket)) {
        DEBUG("[LoRa] remove oldest frame from FIFO\n");
        ls_frame_fifo_pop(&fifo_lorapacket, NULL);
    }
    
    if (!ls_frame_fifo_push(&fifo_lorapacket, &frame)) {
    	mutex_unlock(&curr_frame_mutex);
        DEBUG("[LoRa] FIFO error\n");
        return;
    }
    
    /* send data */
    msg_send(&msg_data, sender_pid);
    
    mutex_unlock(&curr_frame_mutex);

    blink_led(LED0_PIN);
}

static int unwds_init(void) {
    radio_init();
    ls_setup(&ls);
    
    return 0;
}

static void unwds_join(void) {
    msg_send(&msg_join, sender_pid);
}

static void unwds_sleep(void) {
    semtech_loramac_set_class(&ls, LS_ED_CLASS_A);
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
                                   THREAD_CREATE_STACKTEST, sender_thread, &ls,  "LoRa sender thread");
                                   
        receiver_pid = thread_create(receiver_stack, sizeof(receiver_stack), THREAD_PRIORITY_MAIN - 2,
                                   THREAD_CREATE_STACKTEST, receiver_thread, &ls,  "LoRa receiver thread");

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
