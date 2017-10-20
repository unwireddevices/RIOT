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
#include "periph/uart.h"
#include "periph/wdg.h"

#include "random.h"
#include "ringbuffer.h"
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

#include "ls-regions.h"

static node_role_settings_t node_settings;
static bool dr_set = false, channel_set = false;

static sx1276_t sx1276;
static ls_gate_t ls;

static ls_gate_channel_t channels[1] = {
    { LS_DR6, 0, 0, LS_GATE_CHANNEL_STATE_IDLE, { &sx1276, &ls } },        /* DR, frequency, rssi, state, sx1276 & LS instance */
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
    settings.modem = SX1276_MODEM_LORA;
    settings.state = SX1276_RF_IDLE;

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
    printf("[recv] %d bytes: %s | rssi: %d\n", bufsize, hex, rssi);

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
    printf("ls-gate: data acknowledged from 0x%08X%08X\n", (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    char str[18] = {};

    sprintf(str, "%c%08X%08X\n", REPLY_ACK, (unsigned int) (node->node_id >> 32), (unsigned int) (node->node_id & 0xFFFFFFFF));

    gc_pending_fifo_push(&fifo, str);
}

#ifdef GATE_USE_WATCHDOG
static void keepalive_cb(void) {
	xtimer_usleep(100); /* XXX: watchdog timer don't want to reset without a small delay before */
	wdg_reload();
}
#endif

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
    ls->settings.join_key = config_get_joinkey();

#ifdef GATE_USE_WATCHDOG
    ls->settings.keepalive_period_ms = 1000; /* 1 second */
#else
    ls->settings.keepalive_period_ms = 0;
#endif

    if (node_settings.is_valid) {
        channels[0].frequency = regions[node_settings.region_index].channels[node_settings.channel];
        channels[0].dr = node_settings.dr;
    }

    ls->channels = channels;
    ls->num_channels = 1;

    ls->accept_node_join_cb = accept_node_join_cb;
    ls->node_joined_cb = node_joined_cb;
    ls->node_kicked_cb = node_kicked_cb;
    ls->app_data_received_cb = app_data_received_cb;

    ls->app_data_ack_cb = app_data_ack_cb;

#ifdef GATE_USE_WATCHDOG
    ls->keepalive_cb = keepalive_cb;
#else
    ls->keepalive_cb = NULL;
#endif

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

    if (strcmp(key, "dr") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (v > 6) {
            puts("set dr: datarate value must be from 0 to 6");
            return 1;
        }

        node_settings.dr = (ls_datarate_t) v;
        dr_set = true;
    }
    else if (strcmp(key, "region") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (v > LS_UNI_NUM_REGIONS - 1) {
            printf("set region: region value must be from 0 to %d\n", LS_UNI_NUM_REGIONS - 1);
            return 1;
        }

        node_settings.region_index = v;
        node_settings.region_not_set = false;
    }
    else if (strcmp(key, "ch") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (node_settings.region_not_set) {
        	puts("set ch: set region first");
        	return 1;
        }


        if (v > regions[node_settings.region_index].num_channels - 1) {
            printf("set ch: channel value must be from 0 to %d\n", regions[node_settings.region_index].num_channels - 1);
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
        channels[0].frequency = regions[node_settings.region_index].channels[node_settings.channel];
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

static void print_config(void)
{
	if (node_settings.region_not_set) {
		puts("[!] Region is not set yet");
		print_regions();
	}

    puts("[ gate configuration ]");

    uint64_t eui64 = config_get_nodeid();
    uint64_t appid = config_get_appid();

    printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    printf("APPID64 = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));

    if (!node_settings.region_not_set) {
    	printf("REGION = %s\n", regions[node_settings.region_index].region);

        if (channel_set || node_settings.is_valid) {
            printf("CHANNEL = %d\n", (unsigned) regions[node_settings.region_index].channels[node_settings.channel]);
        }
        else {
            puts("CHANNEL = <not set>");
        }
    } else {
    	puts("REGION = <not set>");
    	puts("CHANNEL = <set region first>");
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
	if (node_settings.region_not_set) {
		puts("[error] You must setup region via `set region` command");
		return 1;
	}

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
	if (argc != 2) {
		puts("usage: clear <all | joinkey>");
		puts("\tall -- clears all data in NVRAM including device EUI64");
		puts("\tjoinkey -- clears network encryption key stored in NVRAM");

		return 1;
	}

	char *arg = argv[1];
	if (strcmp(arg, "all") == 0) {
		if (!clear_nvram()) {
			puts("[error] Unable to clear NVRAM");
		}
	} else if (strcmp(arg, "joinkey") == 0) {
		puts("clear joinkey: not supported yet");
		//config_clear_joinkey();
	}

	puts("[ok] Settings cleared");
	puts("Type \"reboot\" to define new configuration");

    return 0;
}

static int ls_goto_bootloader(int argc, char **argv)
{
    rtc_save_backup(0xB00710AD, 0);
    NVIC_SystemReset();
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
	return -1;
}

static const shell_command_t shell_commands[] = {
    { "set", "<config> <value> -- sets up value for the config entry", ls_set_cmd },
    { "listconfig", "-- prints out current configuration", ls_printc_cmd },
    { "save", "-- saves current settings in NVRAM", ls_save_cmd },

	{ "clear", "<all | joinkey> clears settings in NVRAM", ls_clear_nvram },
    { "update", "-- jumps to UART bootloader", ls_goto_bootloader},

    { "list", "-- prints list of connected devices", ls_list_cmd },
	{ "add", "<nodeid> <appid> <addr> <devnonce> <channel> -- adds node to the list", add_cmd },
	{ "kick", "<addr> -- kicks node from the list by its address", kick_cmd},
    { NULL, NULL, NULL }
};

#ifdef GATE_USE_WATCHDOG
static void watchdog_start(void) {
	/* Set watchdog to about 3.5 seconds */
    wdg_set_prescaler(0x03);
    wdg_set_reload((uint16_t) 0x0FFF);

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
	else
	{
		puts("Error initializing Connect button");
	}

	return false;
}
#endif

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

#ifdef GATE_USE_WATCHDOG
        if (!is_connect_button_pressed())
        	watchdog_start();
        else
        	puts("[!] Watchdog timer is suppressed by `connect` button");
#endif

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
