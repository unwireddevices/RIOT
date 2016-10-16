/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file
 * @brief
 * @author      Evgeniy Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "xtimer.h"
#include "lpm.h"
#include "periph/rtc.h"
#include "random.h"
#include "ringbuffer.h"
#include "periph/uart.h"

#include "board.h"

#include "sx1276.h"

#include "ls-mac-types.h"
#include "ls-crypto.h"
#include "ls-gate.h"

#include "gate-commands.h"
#include "pending-fifo.h"

#include "main.h"
#include "config.h"
#include "utils.h"
typedef struct {
    bool is_valid;

    ls_channel_t channel;
    ls_datarate_t dr;
} node_role_settings_t;

static node_role_settings_t node_settings;
static bool dr_set = false, channel_set = false;

static sx1276_t sx1276;
static ls_gate_t ls;

static ls_gate_channel_t channels[1] = {
    { LS_DR6, 0, { &sx1276, &ls } },        /* DR3, channel 2 */
};

/* UART interaction */
#define UART_BUFSIZE        (255U)
#define EOL '\r'

static char rx_mem[UART_BUFSIZE];
static ringbuffer_t rx_buf;

static kernel_pid_t reader_pid;
static char reader_stack[1024 + 2 * 1024];

static kernel_pid_t writer_pid;
static char writer_stack[1024];

static uart_t uart = GATE_COMM_UART;

static gc_pending_fifo_t fifo;

static void rx_cb(void *arg, uint8_t data)
{
    ringbuffer_add_one(&rx_buf, data);

    if (data == EOL) {
        msg_t msg;
        msg_send(&msg, reader_pid);
    }
}

static void *writer(void *arg)
{
    msg_t msg;
    msg_t msg_queue[8];

    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);

        char buf[GC_MAX_REPLY_LEN];
        while (!gc_pending_fifo_empty(&fifo)) {
            if (gc_pending_fifo_pop(&fifo, buf)) {
                uart_write(uart, (uint8_t *) buf, strlen(buf));
            }
        }
    }

    return NULL;
}

static void *reader(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    char buf[255] = { '\0' };

    while (1) {
        msg_receive(&msg);

        char c;
        int i = 0;
        do {
            c = ringbuffer_get_one(&rx_buf);
            buf[i++] = c;
        } while (c != EOL);

        /* Strip the string just in case that there's a garbage after EOL */
        buf[i] = '\0';

        /* Parse received command */
        gc_parse_command(&ls, writer_pid, &fifo, buf);
    }

    /* this should never be reached */
    return NULL;
}

static void uart_gate_init(void)
{
    ringbuffer_init(&rx_buf, rx_mem, UART_BUFSIZE);
    gc_pending_fifo_init(&fifo);

    /* start the reader thread */
    reader_pid = thread_create(reader_stack, sizeof(reader_stack), THREAD_PRIORITY_MAIN - 1, 0, reader, NULL, "uart reader");

    /* start the writer thread */
    writer_pid = thread_create(writer_stack, sizeof(writer_stack), THREAD_PRIORITY_MAIN - 1, 0, writer, NULL, "uart writer");

    if (uart_init(uart, 115200, rx_cb, (void *) uart) == -1) {
        puts("uart_gate_init: failed to initialize uart #1");
    }
}

static void radio_init(void)
{
    sx1276.nss_pin = SX1276_SPI_NSS;
    sx1276.spi = SX1276_SPI;

    sx1276.dio0_pin = SX1276_DIO0;
    sx1276.dio1_pin = SX1276_DIO1;
    sx1276.dio2_pin = SX1276_DIO2;
    sx1276.dio3_pin = SX1276_DIO3;

    sx1276.dio4_pin = (gpio_t) NULL;
    sx1276.dio5_pin = (gpio_t) NULL;
    sx1276.reset_pin = (gpio_t) SX1276_RESET;

    sx1276_settings_t settings;
    settings.channel = RF_FREQUENCY;
    settings.modem = MODEM_LORA;
    settings.state = RF_IDLE;

    sx1276.settings = settings;

    puts("init_radio: sx1276 initialization done");
}

static int ls_list_cmd(int argc, char **argv);

static void node_kicked_cb(ls_gate_node_t *node)
{
    printf("ls-gate: node 0x%08X%08X kicked for long silence\n", (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    char str[18] = {};

    sprintf(str, "%c%08X%08X\n", REPLY_KICK, (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    gc_pending_fifo_push(&fifo, str);
}

static uint32_t node_joined_cb(ls_gate_node_t *node)
{
    printf("gate: node with ID 0x%08X%08X joined to the network with address 0x%08X\n",
           (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF),
           (unsigned int) node->addr);

    /* Notify the gate */
    char str[128] = { '\0' };
    sprintf(str, "%c%08X%08X%u\n", REPLY_JOIN, (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF), (unsigned int) node->node_class);

    gc_pending_fifo_push(&fifo, str);

    /* Return random app nonce */
    return sx1276_random(&sx1276);
}

static bool accept_node_join_cb(uint64_t dev_id, uint64_t app_id)
{
    return true; /* Stub */
}

void app_data_received_cb(ls_gate_node_t *node, ls_gate_channel_t *ch, uint8_t *buf, size_t bufsize)
{
    char hex[255] = {};

    bytes_to_hex(buf, bufsize, hex, false);

    printf("ls-gate: data from 0x%08X%08X: %s\n", (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF), hex);

    char str[200] = { };
    char msg[180] = { };
    memcpy(msg, hex, sizeof(msg));
    msg[sizeof(msg) - 1] = '\0';

    sprintf(str, "%c%08X%08X%s\n", REPLY_IND, (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF), msg);

    gc_pending_fifo_push(&fifo, str);
}

void app_data_ack_cb(ls_gate_node_t *node, ls_gate_channel_t *ch)
{
    printf("ls-gate: data acknowledged from 0x%08X%08X\n", (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    char str[18] = {};

    sprintf(str, "%c%08X%08X\n", REPLY_ACK, (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    gc_pending_fifo_push(&fifo, str);
}

void link_ok_cb(ls_gate_node_t *node, ls_gate_channel_t *ch)
{
    printf("ls-gate: link of with 0x%08X%08X\n", (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    char str[18] = {};
    sprintf(str, "%c%08X%08X\n", REPLY_LNKCHK, (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    gc_pending_fifo_push(&fifo, str);
}

static void ls_setup(ls_gate_t *ls)
{
    ls->settings.gate_id = config_get_nodeid();
    ls->settings.join_key = config_get_joinkey();

    if (node_settings.is_valid) {
        channels[0].ch = node_settings.channel;
        channels[0].dr = node_settings.dr;
    }

    ls->channels = channels;
    ls->num_channels = 1;

    ls->accept_node_join_cb = accept_node_join_cb;
    ls->node_joined_cb = node_joined_cb;
    ls->node_kicked_cb = node_kicked_cb;
    ls->app_data_received_cb = app_data_received_cb;

    ls->link_ok_cb = link_ok_cb;
    ls->app_data_ack_cb = app_data_ack_cb;
}

static int ls_set_cmd(int argc, char **argv)
{
    if (argc != 3) {
        puts("usage: get <key> <value>");
        puts("keys:");
        puts("\tdr <0-6> -- sets device data rate [0 - slowest, 3 - average, 6 - fastest]");
        puts("\tch <0-2> -- sets device channel");

        return 1;
    }

    char *key = argv[1];
    char *value = argv[2];

    if (strcmp(key, "dr") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (v > 6) {
            puts("set dr: datarate value must be from 0 to 6");
            return 1;
        }

        node_settings.dr = (ls_datarate_t) v;
        dr_set = true;
    }
    else if (strcmp(key, "ch") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (v > 2) {
            puts("set ch: channel value must be from 0 to 2");
            return 1;
        }

        node_settings.channel = (ls_channel_t) v;
        channel_set = true;
    }
    else {
        printf("set: unknown key %s\n", key);
        return 1;
    }

    node_settings.is_valid = channel_set && dr_set;

    if (node_settings.is_valid) {
        channels[0].ch = node_settings.channel;
        channels[0].dr = node_settings.dr;
    }

    return 0;
}

static int ls_list_cmd(int argc, char **argv)
{
    ls_gate_devices_t *devs = &ls.devices;

    printf("Total devices: %d\n", (unsigned int) devs->num_nodes);
    printf("num.\t|\taddr.\t\t|\tnode id.\t\t|\tapp id.\t\t\t|\tlast seen\n");

    for (int i = 0; i < LS_GATE_MAX_NODES; i++) {
        if (!devs->nodes_free_list[i]) {
            printf("%02d.\t|\t0x%08X\t|\t0x%08X%08X\t|\t0x%08X%08X\t|\t%d sec. ago\n", (unsigned int) (i + 1),
                   (unsigned int) devs->nodes[i].addr,
                   (unsigned int) (devs->nodes[i].node_id >> 32), (unsigned int) (devs->nodes[i].node_id & 0xFFFFFFFF),
                   (unsigned int) (devs->nodes[i].app_id >> 32), (unsigned int) (devs->nodes[i].app_id & 0xFFFFFFFF),
                   (unsigned int) ((ls._internal.ping_count - devs->nodes[i].last_seen) * LS_PING_TIMEOUT_S));
        }
    }

    return 0;
}

static void print_config(void)
{
    puts("[ gate configuration ]");

    uint64_t eui64 = config_get_nodeid();
    uint64_t appid = config_get_appid();

    printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    printf("APPID64 = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));

    if (channel_set || node_settings.is_valid) {
        printf("CHANNEL = %d\n", node_settings.channel);
    }
    else {
        puts("CHANNEL = <not set>");
    }

    if (dr_set || node_settings.is_valid) {
        printf("DATARATE = %d\n", node_settings.dr);
    }
    else {
        puts("DATARATE = <not set>");
    }
}

static int ls_printc_cmd(int argc, char **argv)
{
    print_config();

    return 0;
}

static int ls_save_cmd(int argc, char **argv)
{
    if (!node_settings.is_valid) {
        puts("[error] You must setup following settings via `set` command(s):");
        if (!dr_set) {
            puts("\tset dr");
        }
        if (!channel_set) {
            puts("\tset ch");
        }

        return 1;
    }

    puts("[*] Saving configuration...");
    if (!config_write_role_block((uint8_t *) &node_settings, sizeof(node_role_settings_t))) {
        puts("[error] Unable to save configuration");
    }

    puts("[done] Configuration saved. Type \"reboot\" to apply changes.");

    return 0;
}

static int ls_clear_nvram(int argc, char **argv)
{

    if (clear_nvram()) {
        puts("[ok] Settings cleared");
        puts("Type \"reboot\" to define new configuration");
    }
    else {
        puts("[error] Unable to clear NVRAM");
    }

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "set", "<config> <value> -- sets up value for the config entry", ls_set_cmd },
    { "listconfig", "-- prints out current configuration", ls_printc_cmd },
    { "save", "-- saves current settings in NVRAM", ls_save_cmd },

	{ "clear", "-- clears settings stored in NVRAM", ls_clear_nvram },

    { "list", "-- prints list of connected devices", ls_list_cmd },

    { NULL, NULL, NULL }
};

static bool load_config(void)
{
    if (!config_read_role_block((uint8_t *) &node_settings, sizeof(node_role_settings_t))) {
        puts("[node] Unable to load role specific configuration");

        return false;
    }

    return node_settings.is_valid;
}

void init_gate(shell_command_t **commands)
{
    if (load_config()) {
        print_config();

        puts("[ok] Configuration seems valid, initializing LoRa gate...");
        uart_gate_init();
        radio_init();
        sx1276_init(&sx1276);
        ls_setup(&ls);
        ls_gate_init(&ls);

        blink_led();
    }
    else {
        print_config();

        puts("[!] This gate is not configured yet. Type \"help\" to see list of possible configuration commands.");
        puts("[!] Configure this node via \"set\" commands and type \"reboot\" to reboot and apply settings.");
    }

    /* Set our commands for shell */
    memcpy(commands, shell_commands, sizeof(shell_commands));
}


#ifdef __cplusplus
}
#endif
