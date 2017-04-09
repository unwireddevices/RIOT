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
 * @author      Cr0s
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "lpm.h"
#include "arch/lpm_arch.h"
#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "xtimer.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "random.h"

#include "sx1276.h"
#include "board.h"

#include "main.h"
#include "config.h"
#include "utils.h"

#define LPTP_MAX_PAYLOAD_SIZE (255 - 1 - 1 - 2 - 1)
#define LPTP_HEADER 0x42

typedef struct {
	uint8_t size : 8;
	uint8_t data[LPTP_MAX_PAYLOAD_SIZE];
} lptp_frame_payload_t;

typedef struct {
	uint8_t header;		/* LPTP header */
	uint8_t port;		/* Message port */

	uint16_t rfu;		/* Reserved */
} lptp_frame_header_t;

typedef struct {
	lptp_frame_header_t header;
	lptp_frame_payload_t payload;
} lptp_frame_t;



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

#define UART_BUFSIZE 220
#define UART UART_DEV(1)

typedef struct {
    bool is_valid;

    uint32_t freq;
    uint8_t dr;

    uint8_t port;

    uint32_t baudrate;
    uint32_t symbol_timeout_ms;
} ptp_settings_t;

static ptp_settings_t node_settings;

static sx1276_t sx1276;

static uint8_t rxbuf[UART_BUFSIZE] = {};

static volatile uint8_t num_bytes_received;

static kernel_pid_t writer_pid;
static char writer_stack[THREAD_STACKSIZE_MAIN + 2048];

static msg_t send_msg;

static xtimer_t send_timer;

static void sx1276_handler(void *dev, sx1276_event_type_t event_type);

static void send_frame(uint8_t *buf, size_t size) {
	lptp_frame_t frame = {};

	frame.header.header = LPTP_HEADER;
	frame.header.port = node_settings.port;

	frame.payload.size = size;
	memcpy(frame.payload.data, buf, size);

	sx1276_send(&sx1276, (uint8_t *) &frame, sizeof(lptp_frame_header_t) + 1 + frame.payload.size);
}

static void *writer(void *arg) {
    msg_t msg;
    msg_t msg_queue[128];
    msg_init_queue(msg_queue, 128);

    while (1) {
        msg_receive(&msg);

        /* Received payload, send it */
        if (msg.content.value == send_msg.content.value) {
			send_frame(rxbuf, num_bytes_received);
        	num_bytes_received = 0;
        }
    }

	return NULL;
}

static void rx_cb(void *arg, uint8_t data)
{
	/* Buffer overflow */
	if (num_bytes_received == UART_BUFSIZE) {
		/* Send all data on overflow */
		msg_send(&send_msg, writer_pid);
		return;
	}

	rxbuf[num_bytes_received++] = data;

	/* Schedule sending after timeout */
	xtimer_set_msg(&send_timer, 1e3 * node_settings.symbol_timeout_ms, &send_msg, writer_pid);
}

static void ptp_uart_init(void)
{
    /* Initialize the UART */
    if (uart_init(UART, node_settings.baudrate, rx_cb, NULL)) {
        return;
    }

    send_msg.content.value = 0;

	/* Create handler thread */
	writer_pid = thread_create(writer_stack, sizeof(writer_stack), THREAD_PRIORITY_MAIN - 1, 0, writer, NULL, "uart writer thread");
}

static void radio_init(void)
{
    sx1276.nss_pin = SX1276_SPI_NSS;
    sx1276.spi = SX1276_SPI;

    sx1276.dio0_pin = SX1276_DIO0;
    sx1276.dio1_pin = SX1276_DIO1;
    sx1276.dio2_pin = SX1276_DIO2;
    sx1276.dio3_pin = SX1276_DIO3;

    sx1276.dio4_pin = GPIO_UNDEF;
    sx1276.dio5_pin = GPIO_UNDEF;
    sx1276.reset_pin = (gpio_t) SX1276_RESET;

    sx1276_settings_t settings;
    settings.channel = RF_FREQUENCY;
    settings.modem = SX1276_MODEM_LORA;
    settings.state = SX1276_RF_IDLE;

    sx1276.settings = settings;

    sx1276.sx1276_event_cb = sx1276_handler;
    sx1276.callback_arg = NULL;

    sx1276_init(&sx1276);

    puts("init_radio: sx1276 initialization done");
}

static void configure_sx1276(void)
{
    /* Choose data rate */
    uint8_t *datarate = datarate_table[node_settings.dr];

    /* Setup channel (frequency) */
    sx1276_set_channel(&sx1276, node_settings.freq);

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

    sx1276_configure_lora(&sx1276, &settings);
}

static void enter_rx(void)
{
    configure_sx1276();
    sx1276_set_rx(&sx1276, sx1276.settings.lora.rx_timeout);
}

static bool validate_frame(lptp_frame_t *frame) {
	/* Check header */
	if (frame->header.header != LPTP_HEADER)
		return false;

	if (frame->header.port != node_settings.port)
		return false;

	// TODO: check HMAC

	return true;
}

static void frame_recv(lptp_frame_t *frame) {
	if (!frame->payload.size)
		return;

	uart_write(UART, frame->payload.data, frame->payload.size);
}

static void sx1276_handler(void *dev, sx1276_event_type_t event_type)
{
	(void) dev;

	switch (event_type) {
		case SX1276_RX_DONE:
			printf("[rx] %u bytes, | RSSI: %d\n", sx1276._internal.last_packet.size, sx1276._internal.last_packet.rssi_value);

			lptp_frame_t *frame = (lptp_frame_t *) sx1276._internal.last_packet.content;

			/* Check frame format */
			if (validate_frame(frame)) {
				frame_recv(frame);
			}
			else {
				printf("[rx] Frame discarded. Header: 0x%02x, port: %d\n", frame->header.header, frame->header.port);
			}
			break;

		case SX1276_RX_ERROR_CRC:
			puts("sx1276: RX CRC failed");
			sx1276_set_sleep(&sx1276);
			break;

		case SX1276_TX_DONE:
			puts("sx1276: transmission done.");

			/* Switch to reception */
			enter_rx();

			break;

		case SX1276_RX_TIMEOUT:
			puts("sx1276: RX timeout");
			sx1276_set_sleep(&sx1276);
			break;

		case SX1276_TX_TIMEOUT:
			puts("sx1276: TX timeout");
			sx1276_set_sleep(&sx1276);

			break;

		default:
			printf("sx1276: received event #%d\n", (int) event_type);
			sx1276_set_sleep(&sx1276);
			break;
	}
}

static int cmd_set(int argc, char **argv)
{
    if (argc != 3) {
        puts("usage: get <key> <value>");
        puts("keys:");
        puts("\tdr <0-6> -- sets device data rate [0 - slowest, 3 - average, 6 - fastest]");
        puts("\tfreq <frequency> -- sets LoRa frequency [Hz]");
        puts("\tport <0-255> -- sets port to transmit and receive data");

        puts("");

        puts("\tbaudrate <baudrate> -- sets data UART baud rate [bauds]");
        puts("\tsymbto <timeout> -- sets maximum pause time in data stream in UART before sending already received data [ms]");

    }

    char *key = argv[1];
    char *value = argv[2];

    if (strcmp(key, "dr") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (v > 6) {
            puts("set dr: datarate value must be from 0 to 6");
            return 1;
        }

        node_settings.dr = v;
    }
    else if (strcmp(key, "freq") == 0) {
        uint32_t v = strtol(value, NULL, 10);

        if (v < 433000000 || v > 900000000) {
            puts("set freq: value must be from 433000000 to 900000000");
			puts("To comply with Russian regulations, please set frequency in the range of 868900000 to 869100000");
            return 1;
        }

        node_settings.freq = v;
    }
    else if (strcmp(key, "port") == 0) {
        uint32_t v = strtol(value, NULL, 10);

        node_settings.port = v;
    }
    else if (strcmp(key, "baudrate") == 0) {
        uint32_t v = strtol(value, NULL, 10);

        node_settings.baudrate = v;
    }
    else if (strcmp(key, "symbto") == 0) {
        uint32_t v = strtol(value, NULL, 10);

        node_settings.symbol_timeout_ms = v;
    }

    return 0;
}

static void print_config(void)
{
    puts("[ node configuration ]");

    uint64_t eui64 = config_get_nodeid();

    printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));

    printf("FREQUENCY = %u [Hz]\n", (unsigned int) node_settings.freq);
    printf("PORT = %d\n", node_settings.port);
    printf("DATARATE = %d\n", node_settings.dr);

    puts("");
    printf("BAUDRATE = %u\n", (unsigned int) node_settings.baudrate);
    printf("SYMBTIMEOUT = %u\n", (unsigned int) node_settings.symbol_timeout_ms);
}

static int cmd_printc(int argc, char **argv) {
	print_config();

	return 0;
}

static int cmd_clear_nvram(int argc, char **argv)
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

static int cmd_save(int argc, char **argv)
{

    puts("[*] Saving configuration...");
    node_settings.is_valid = true;
    if (!config_write_role_block((uint8_t *) &node_settings, sizeof(ptp_settings_t))) {
        puts("[error] Unable to save configuration");
    }

    puts("[done] Configuration saved. Type \"reboot\" to apply changes.");

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "set", "<config> <value> -- sets up value for the config entry", cmd_set },
    { "lscfg", "-- prints out current configuration", cmd_printc },

    { "save", "-- saves current configuration", cmd_save },

    { "clear", "-- clears all data in NVRAM", cmd_clear_nvram },

    { NULL, NULL, NULL }
};

static bool load_config(void)
{
    if (!config_read_role_block((uint8_t *) &node_settings, sizeof(ptp_settings_t))) {
        puts("[node] Unable to load configuration");

        return false;
    }

    if (node_settings.baudrate == 0)
    	node_settings.baudrate = 115200;

    return node_settings.is_valid;
}

void init_node(shell_command_t **commands)
{
    puts("[node] Initializing...");

    /* Set our commands for shell */
    memcpy(commands, shell_commands, sizeof(shell_commands));

    if (!load_config()) {
        puts("[!] This device is not configured yet. Type \"help\" to see list of possible configuration commands.");
        puts("[!] Configure this node and type \"reboot\" to reboot and apply settings.");

        print_config();
    }
    else {
        print_config();

        ptp_uart_init();
        radio_init();
        configure_sx1276();
        enter_rx();
        blink_led();
    }
}

#ifdef __cplusplus
}
#endif
