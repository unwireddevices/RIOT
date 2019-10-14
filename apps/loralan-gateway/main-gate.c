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
 * @file        main-gate.c
 * @brief       LoRaLAN gateway device
 * @author      Evgeniy Ponomarev
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"

#include "thread.h"
#include "random.h"
#include "periph/rtc.h"
#include "periph/wdg.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "ringbuffer.h"
#include "lptimer.h"
#include "utils.h"
#include "shell.h"
#include "shell_commands.h"

#include "net/lora.h"
#include "net/netdev.h"
#include "sx127x_internal.h"
#include "sx127x_params.h"
#include "sx127x_netdev.h"

#include "board.h"

#include "ls-settings.h"
#include "ls-config.h"
#include "ls-regions.h"
#include "ls-mac-types.h"
#include "ls-crypto.h"
#include "ls-gate.h"

#include "gate-commands.h"
#include "pending-fifo.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define IWDG_PRESCALER  (5)
#define IWDG_RELOAD     (0x0FFF)
#define IWDG_TIMEOUT    (1000*((((IWDG_RELOAD) * (1 << (IWDG_PRESCALER + 2))) / 56000) - 3))

static lptimer_t iwdg_timer;

static sx127x_t sx127x;
static netdev_t netdev;
static ls_gate_t ls;

static ls_gate_channel_t channels[1] = {
    { 
        .dr = LS_DR6,
        .frequency = 0,
        .last_rssi = 0,
        .state = LS_GATE_CHANNEL_STATE_IDLE,
        ._internal = {
                .device = &netdev,
                .gate = &ls
        }
    },        /* DR, frequency, rssi, state, sx127x & LS instance */
};

/* UART interaction */
#define UART_BUFSIZE        (255U)
#define EOL '\r'

static char rx_mem[UART_BUFSIZE];
static ringbuffer_t rx_buf;

static kernel_pid_t gate_reader_pid;
static char reader_stack[1024 + 2 * 1024];

static kernel_pid_t writer_pid;
static char writer_stack[1024];

static uart_t uart = GATE_COMM_UART;

static gc_pending_fifo_t fifo;

static void rx_cb(void *arg, uint8_t data)
{
    (void)arg;
    
    ringbuffer_add_one(&rx_buf, data);

    if (data == EOL) {
        msg_t msg;
        msg_send(&msg, gate_reader_pid);
    }
}

static void *writer(void *arg)
{
    (void)arg;
    
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
    gate_reader_pid = thread_create(reader_stack, sizeof(reader_stack), THREAD_PRIORITY_MAIN - 1, 0, reader, NULL, "uart reader");

    /* start the writer thread */
    writer_pid = thread_create(writer_stack, sizeof(writer_stack), THREAD_PRIORITY_MAIN - 1, 0, writer, NULL, "uart writer");

    if (uart_init(uart, 115200, rx_cb, (void *) uart) == -1) {
        puts("uart_gate_init: failed to initialize uart #1");
    }
}

static void radio_init(void)
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

    sx127x_radio_settings_t settings;
    settings.channel = RF_FREQUENCY;
    settings.modem = SX127X_MODEM_LORA;
    settings.state = SX127X_RF_IDLE;

    sx127x.settings = settings;
    memcpy(&sx127x.params, &sx127x_params, sizeof(sx127x_params));
    
    memcpy((void *)&netdev, (void *)&sx127x, sizeof(sx127x));
    netdev.driver = &sx127x_driver;

    puts("init_radio: sx127x initialization done");
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
    printf("gate: node 0x%08X%08X joined to the network, local address is 0x%08X\n",
           (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF),
           (unsigned int) node->addr);

    /* Notify the gate */
    char str[128] = { '\0' };
    sprintf(str, "%c%08X%08X%u\n", REPLY_JOIN, (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF), (unsigned int) node->node_class);

    gc_pending_fifo_push(&fifo, str);

    /* Return random app nonce */
    return sx127x_random(&sx127x);
}

static bool accept_node_join_cb(uint64_t dev_id, uint64_t app_id)
{
    (void)dev_id;
    (void)app_id;
    
    return true; /* Stub */
}

void app_data_received_cb(ls_gate_node_t *node, ls_gate_channel_t *ch, uint8_t *buf, size_t bufsize, uint8_t status)
{
    char hex[GC_MAX_REPLY_LEN - 19] = {};
    if (bufsize > sizeof(hex))
    	bufsize = sizeof(hex);

    int16_t rssi = ch->last_rssi;

    char buf_rssi[5] = {};
    bytes_to_hex((uint8_t *) &rssi, 2, buf_rssi, true);
    
    char buf_status[5]  = {};
    bytes_to_hex(&status, 1, buf_status, true);

    bytes_to_hex(buf, bufsize, hex, false);
    printf("Data: %u bytes, 0x%s\n", bufsize, hex);

    char str[GC_MAX_REPLY_LEN] = { };
    sprintf(str, "%c%08X%08X%s%s%s\n", REPLY_IND,
    		(unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF),
			buf_rssi,
            buf_status,
			hex);

    gc_pending_fifo_push(&fifo, str);
}

void app_data_ack_cb(ls_gate_node_t *node, ls_gate_channel_t *ch)
{
    (void)ch;
    
    printf("ls-gate: data acknowledged from 0x%08X%08X\n", (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    char str[18] = {};

    sprintf(str, "%c%08X%08X\n", REPLY_ACK, (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    gc_pending_fifo_push(&fifo, str);
}

static void pending_frames_req_cb(ls_gate_node_t *node) {
	printf("ls-gate: requesting next pending frame for 0x%08X%08X\n", (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    char str[18] = {};
    sprintf(str, "%c%08X%08X\n", REPLY_PENDING_REQ,
    		(unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));
    gc_pending_fifo_push(&fifo, str);
}

static void ls_setup(ls_gate_t *ls)
{
    ls->settings.gate_id = config_get_nodeid();
    ls->settings.join_key = config_get_appkey();

    channels[0].frequency = regions[unwds_get_node_settings().region_index].channels[unwds_get_node_settings().channel];
    channels[0].dr = unwds_get_node_settings().dr;

    ls->channels = channels;
    ls->num_channels = 1;

    ls->accept_node_join_cb = accept_node_join_cb;
    ls->node_joined_cb = node_joined_cb;
    ls->node_kicked_cb = node_kicked_cb;
    ls->app_data_received_cb = app_data_received_cb;

    ls->app_data_ack_cb = app_data_ack_cb;

    ls->pending_frames_req = pending_frames_req_cb;
}

static int ls_set_cmd(int argc, char **argv)
{
    if (argc != 3) {
        puts("usage: get <key> <value>");
        puts("keys:");
        puts("\tdr <0-6> -- sets device data rate [0 - slowest, 3 - average, 6 - fastest]");
        printf("\tregion <0-%d> -- sets device region\n", LS_UNI_NUM_REGIONS - 1);
        puts("\tch <ch> -- sets device channel for selected region");

        return 1;
    }

    char *key = argv[1];
    char *value = argv[2];

    int v;
    
    if (strcmp(key, "dr") == 0) {
        v = strtol(value, NULL, 10);

        if (v > 6) {
            puts("ls-gate: datarate value must be from 0 to 6");
            return 1;
        } else {
            printf("ls-gate: datarate set to %d\n", v);
        }

        ls.channels[0].dr = (ls_datarate_t) v;
        unwds_set_dr(v);
    }
    else if (strcmp(key, "region") == 0) {
        v = strtol(value, NULL, 10);

        if (v > LS_UNI_NUM_REGIONS - 1) {
            printf("ls-gate: region value must be from 0 to %d\n", LS_UNI_NUM_REGIONS - 1);
            return 1;
        } else {
            printf("ls-gate: region set to %d\n", v);
        }

        unwds_set_region(v);
    }
    else if (strcmp(key, "ch") == 0) {
        v = strtol(value, NULL, 10);

        if (v > regions[unwds_get_node_settings().region_index].num_channels - 1) {
            printf("set ch: channel value must be from 0 to %d for this region\n", regions[unwds_get_node_settings().region_index].num_channels - 1);
            return 1;
        }

        unwds_set_channel(v);
    }
    else {
        printf("set: unknown key %s\n", key);
        return 1;
    }

    ls.channels[0].frequency = regions[unwds_get_node_settings().region_index].channels[unwds_get_node_settings().channel];

    return 0;
}

static int ls_list_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
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

/*
static void print_regions(void) {
	puts("[ available regions ]");

	int i;
	for (i = 0; i < LS_UNI_NUM_REGIONS; i++) {
		printf("%d. %s [", i, regions[i].region);
		int j;
		for (j = 0; j < regions[i].num_channels; j++) {
			printf("%d", (unsigned) regions[i].channels[j]);

			if (j + 1 != regions[i].num_channels)
				printf(", ");
		}

		puts("]");
	}
}
*/

static void print_config(void)
{
    puts("[ gate configuration ]");

    uint64_t eui64 = config_get_nodeid();
    uint64_t appid = config_get_appid();

    if (DISPLAY_JOINKEY_2BYTES) {
        uint8_t *key = config_get_appkey();
        printf("JOINKEY = 0x....%01X%01X\n", key[14], key[15]);
    }

    printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    printf("APPID64 = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));

    printf("REGION = %s\n", regions[unwds_get_node_settings().region_index].region);
    printf("CHANNEL = %d [%d]\n", unwds_get_node_settings().channel, (unsigned) regions[unwds_get_node_settings().region_index].channels[unwds_get_node_settings().channel]);

    printf("DATARATE = %d\n", unwds_get_node_settings().dr);
}

static int ls_printc_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    print_config();

    return 0;
}

static int add_cmd(int argc, char **argv) {
	if (argc != 6) {
		puts("usage: add <nodeid> <appid> <addr> <devnonce> <channel>");
		return - 1;
	}

	uint64_t nodeid = 0;
	if (!hex_to_bytes(argv[1], (uint8_t *) &nodeid, true)) {
		return -1;
	}

	uint64_t appid = 0;
	if (!hex_to_bytes(argv[2], (uint8_t *) &appid, true)) {
		return -1;
	}

	ls_addr_t addr = 0;
	if (!hex_to_bytes(argv[3], (uint8_t *) &addr, true)) {
		return -1;
	}

	uint32_t dev_nonce = 0;
	if (!hex_to_bytes(argv[4], (uint8_t *) &dev_nonce, true)) {
		return -1;
	}

	uint8_t channel = atoi(argv[5]);

	puts("Adding device:");
	printf("nodeid = 0x%08X%08X\n",
					(unsigned int) (nodeid >> 32),
					(unsigned int) (nodeid & 0xFFFFFFFF));
	printf("appid = 0x%08X%08X\n",
					(unsigned int) (appid >> 32),
					(unsigned int) (appid & 0xFFFFFFFF));

	printf("address = 0x%08X\n", (unsigned int) addr);
	printf("nonce = 0x%08X\n", (unsigned int) dev_nonce);
	printf("ch = 0x%02X\n", (unsigned int) channel);

	/* Kick previous device if present */
	if (ls_devlist_is_in_network(&ls.devices, addr)) {
		ls_devlist_remove_device(&ls.devices, addr);
	}

	/* Add device with specified nonce and address */
	ls_gate_node_t *node = ls_devlist_add_by_addr(&ls.devices, addr, nodeid, appid, dev_nonce, &ls.channels[channel]);
	if (node == NULL)
		return -1;

	return 0;
}

static int kick_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    
	return -1;
}

static void iwdg_reset (void *arg) {
    (void)arg;
    
    wdg_reload();
    lptimer_set(&iwdg_timer, IWDG_TIMEOUT);
    DEBUG("Watchdog reset\n");
    return;
}

shell_command_t shell_commands[UNWDS_SHELL_COMMANDS_MAX] = {
    { "set", "<config> <value> -- sets up value for the config entry", ls_set_cmd },
    { "listconfig", "-- prints out current configuration", ls_printc_cmd },
    { "list", "-- prints list of connected devices", ls_list_cmd },
	{ "add", "<nodeid> <appid> <addr> <devnonce> <channel> -- adds node to the list", add_cmd },
	{ "kick", "<addr> -- kicks node from the list by its address", kick_cmd},
    { NULL, NULL, NULL }
};

static void watchdog_start(void) {
    iwdg_timer.callback = iwdg_reset;
    lptimer_set(&iwdg_timer, IWDG_TIMEOUT);
    
	/* IWDG period is 18 seconds minimum, 28 seconds typical */
    wdg_set_reload(28);

    /* Start watchdog */
    wdg_reload();
    wdg_enable();

	puts("[!] Watchdog timer is enabled. Use `connect` button on reset to disable watchdog timer");
}

static bool is_connect_button_pressed(void) {
    if (!gpio_init(UNWD_CONNECT_BTN, GPIO_IN_PU)) {
		if (!gpio_read(UNWD_CONNECT_BTN)) {
			return true;
		}
	}
	else {
		puts("Error initializing Connect button");
	}

	return false;
}

void init_normal(shell_command_t *commands)
{
    if (!unwds_config_load()) {
        puts("[!] Gate is not configured yet. Type \"help\" to see list of possible configuration commands.");
        puts("[!] Configure the node and type \"reboot\" to reboot and apply settings.");

        print_config();
    } else {
        print_config();
        puts("[ok] Configuration seems valid, initializing LoRa gate...");

        if (!is_connect_button_pressed())
        	watchdog_start();
        else
        	puts("[!] Watchdog timer is suppressed by `connect` button");

        uart_gate_init();
        radio_init();
        
        ls_setup(&ls);
        if (ls_gate_init(&ls) != LS_GATE_OK) {
            puts("ls: error initializing gateway");
            gpio_set(LED0_PIN);
            lptimer_sleep(5000);
            NVIC_SystemReset();
        }
        
        unwds_setup_nvram_config(UNWDS_CONFIG_BASE_ADDR, UNWDS_CONFIG_BLOCK_SIZE_BYTES);

        blink_led(LED0_PIN);
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
