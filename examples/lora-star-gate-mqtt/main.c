/*
 * Copyright (C) 2016 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Default application that shows a functionality of LoRa-Star gateway
 *
 * @author      Eugene Ponomarev
 *
 * @}
 */

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

#include "ls-mac-types.h"
#include "ls-crypto.h"
#include "ls-gate.h"

#include "gate-commands.h"
#include "pending-fifo.h"

sx1276_t sx1276;
ls_gate_t ls;

static uint8_t join_key[LS_MIC_KEY_LEN] = { 0xCA, 0xFE, 0xBA, 0xBE, 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED, 0xCA, 0xFE, 0xDE, 0xAD, 0xBE, 0xEF };

ls_gate_channel_t channels[1] = {
		{ LS_DR6, 0, { &sx1276, &ls } },	/* DR3, channel 2 */
};

/* UART interaction */
#define UART_BUFSIZE        (255U)
#define EOL '\n'

static char rx_mem[UART_BUFSIZE];
static ringbuffer_t rx_buf;

static kernel_pid_t reader_pid;
static char reader_stack[THREAD_STACKSIZE_MAIN + 1024];

static kernel_pid_t writer_pid;
static char writer_stack[THREAD_STACKSIZE_MAIN];

static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN];

static char payload[255];
static ls_addr_t send_addr;


static uart_t uart = UART_DEV(1);

static gc_pending_fifo_t fifo;

static void rx_cb(void *arg, uint8_t data)
{
    ringbuffer_add_one(&rx_buf, data);

    if (data == EOL) {
        msg_t msg;
        msg_send(&msg, reader_pid);
    }
}

static void *sender(void *arg)
{
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);

        ls_gate_send_to(&ls, send_addr, (uint8_t *) payload, strlen(payload) + 1);
    }

    return NULL;
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
        		printf("Sending reply: %s\n", buf);

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

void uart_gate_init(void) {
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

void print_logo(void)
{
    puts("                                                .@                           @  ");
    puts("                                                                             @  ");
    puts("  @@@           %@@,     &@**%@. .#    ./   .#  .@   #@*.   *@@@@@,    @#%.%%@  ");
    puts("  @@@           %@@,    @#    .&  .# ..@.  .#*  .@  /#    .@.    ,%   @.    .@  ");
    puts("  @@@           %@@,    @*    .@  .&,,&  @.#%   .@  %*    .@&&&&&&&*  @      @  ");
    puts("  @@@           %@@,    @*    .@   .@@   .@%    .@  %*    ,@          *@,   ,@, ");
    puts("  @@@           %@@,    *.     *                .#  ,.      %@&&@#     **,*.@   ");
    puts("  @@@           %@@,															  ");
    puts("  @@@   .,,,,,,,...            %@@@%   %     .# *%   .#@@@@,    *@@@@,    @@@*  ");
    puts("  @@@   @@@@@@@@@@@@@@&.     %&     &%  @    @  .@  &*        .@.    ,@  @      ");
    puts("  @@@   @@@     /.. *@@@@.   @&&&&&&&&  ,&  @.  .@  @         %@&&&&&&&*  #@(   ");
    puts("  &@@*  @@@     @@@   (@@@   @#          (@@/   .@  @.        ,@             @  ");
    puts("   @@@. @@@    @@@#    #@@%   .@@&@@.     /*    .@   /@@&@@(    %@&&@#   &@&@*  ");
    puts("    @@% @@@ @@@@@.     .@@@                                                     ");
    puts("        @@@ ####/#####/ @@@ ##################################################  ");
    puts("        @@@            *@@&                                                     ");
    puts("        @@@            @@@,                                                     ");
    puts("        @@@          *@@@#                                                      ");
    puts("        @@@,...,,#&@@@@@                                                        ");
    puts("        @@@@@@@@@@@%,                                                           ");
    puts("                                                                                ");
    puts("                                                                                ");
    puts("                                                                                ");
    puts("");
}

void blink_led(void)
{
    volatile int i;

    LED0_OFF;

    for (i = 0; i < 5; i++) {
        LED0_TOGGLE;
        xtimer_usleep(50000);

        LED0_TOGGLE;
        xtimer_usleep(50000);
    }

    LED0_OFF;
}

void radio_init(void)
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

void node_kicked_cb (ls_gate_node_t *node) {
	printf("gate: node with ID 0x%08X and address 0x%08X kicked from the network due to long silence\n",
			(unsigned int) node->node_id,
			(unsigned int) node->addr);

	/* Notify the gate */
	char str[32] = { '\0' };
	sprintf(str, "%c|%u\n", REPLY_KICK, (unsigned int) node->node_id);

	gc_pending_fifo_push(&fifo, str);
}

uint32_t node_joined_cb (ls_gate_node_t *node) {
	printf("gate: node with ID 0x%08X joined to the network with address 0x%08X\n",
			(unsigned int) node->node_id,
			(unsigned int) node->addr);

	/* Notify the gate */
	char str[128] = { '\0' };
	sprintf(str, "%c|%u|%u\n", REPLY_JOIN, (unsigned int) node->node_id, (unsigned int) node->node_class);

	gc_pending_fifo_push(&fifo, str);

	/* Return random app nonce */
	return sx1276_random(&sx1276);
}

bool accept_node_join_cb(uint64_t dev_id, uint64_t app_id) {
	return true; /* Stub */
}

void app_data_received_cb (ls_gate_node_t *node, ls_gate_channel_t *ch, uint8_t *buf, size_t bufsize) {
	printf("data from 0x%08X: \"%s\"\n", (unsigned int) node->addr, buf);

	char str[160] = { '\0' };
	char msg[128] = { '\0' };
	memcpy(msg, buf, sizeof(msg));
	msg[sizeof(msg) - 1] = '\0';

	sprintf(str, "%c|%u|%s\n", REPLY_IND, (unsigned int) node->node_id, msg);

	gc_pending_fifo_push(&fifo, str);
}

void app_data_ack_cb (ls_gate_node_t *node, ls_gate_channel_t *ch) {
	printf("ls-gate: data acknowledged from 0x%08X\n", (unsigned int) node->addr);

	if (node->num_pending)
		node->num_pending--;

	/* Notify the gate */
	char str[64] = { '\0' };
	sprintf(str, "%c|%u\n", REPLY_ACK, (unsigned int) node->node_id);
	gc_pending_fifo_push(&fifo, str);
}

void link_ok_cb (ls_gate_node_t *node, ls_gate_channel_t *ch) {
	printf("ls-gate: link ok with 0x%08X\n", (unsigned int) node->addr);

	/* Notify the gate */
	char str[64] = { '\0' };
	sprintf(str, "%c|%u\n", REPLY_LNKCHK, (unsigned int) node->node_id);
	gc_pending_fifo_push(&fifo, str);
}


void ls_setup(ls_gate_t *ls)
{
    ls->settings.gate_id = 0xFEEDBEEF;
    ls->settings.join_key = join_key;

    ls->channels = channels;
    ls->num_channels = 1;

    ls->accept_node_join_cb = accept_node_join_cb;
    ls->node_joined_cb = node_joined_cb;
    ls->node_kicked_cb = node_kicked_cb;
    ls->app_data_received_cb = app_data_received_cb;

    ls->link_ok_cb = link_ok_cb;
    ls->app_data_ack_cb = app_data_ack_cb;
}

int ls_get_cmd(int argc, char **argv) {
	if (argc != 2) {
		puts("usage: get <key>");
		puts("keys:");
		puts("\taddr -- returns device address assigned by the gate or manually");
		puts("\tnodeid -- returns unique device ID in hex");
		puts("\tappid -- returns device application ID in hex");
		puts("\tdr -- returns device data rate");
	}

    return 0;
}

int ls_set_cmd(int argc, char **argv) {
	if (argc != 3) {
		puts("usage: get <key> <value>");
		puts("keys:");
		puts("\taddr <value> -- sets device address assigned by the gate or manually");
		puts("\tnodeid <0xABC> -- sets unique device ID in hex");
		puts("\tappid <0xDEF> -- sets device application ID in hex");
		puts("\tdr <0-7> -- sets device data rate");
	}

    return 0;
}

int ls_list_cmd(int argc, char **argv) {
	ls_gate_devices_t *devs = &ls.devices;

	printf("Total devices: %d\n", (unsigned int) devs->num_nodes);
	printf("num.\t|\taddr.\t\t|\tnode id.\t\t|\tapp id.\t\t\t|\tlast seen\n");

	for (int i = 0; i < LS_GATE_MAX_NODES; i++) {
		if (!devs->nodes_free_list[i]) {
			printf("%02d.\t|\t0x%08X\t|\t0x%016X\t|\t0x%016X\t|\t%d sec. ago\n", (unsigned int) (i + 1),
					(unsigned int) devs->nodes[i].addr,
					(unsigned int) devs->nodes[i].node_id,
					(unsigned int) devs->nodes[i].app_id,
					(unsigned int) ((ls._internal.ping_count - devs->nodes[i].last_seen) * LS_PING_TIMEOUT_S));
		}
	}

	return 0;
}

int ls_sendn_cmd(int argc, char **argv) {
	if (argc != 3) {
		puts("usage: sendn <nodeid> <payload>");

		return 1;
	}

	uint64_t nodeid = atoi(argv[1]);

	ls_gate_node_t *node = ls_devlist_get_by_nodeid(&ls.devices, nodeid);
	if (node == NULL) {
		puts("send: device with specified nodeid not found");
	}

	send_addr = node->addr;
	memset(payload, 0, sizeof(payload));
	memcpy(payload, argv[2], strlen(argv[2]));

	msg_t msg;
	msg_send(&msg, sender_pid);

	return 0;
}

int ls_senda_cmd(int argc, char **argv) {
	if (argc != 3) {
		puts("usage: senda <addr> <payload>");

		return 1;
	}

	send_addr = atoi(argv[1]);
	memset(payload, 0, sizeof(payload));
	memcpy(payload, argv[2], strlen(argv[2]));

	msg_t msg;
	msg_send(&msg, sender_pid);

	return 0;
}

static const shell_command_t shell_commands[] = {
	{ "set", "<config> <value> -- sets up value for the config entry", ls_set_cmd },
	{ "get", "<config> -- gets value for the config entry", ls_get_cmd },

	{ "senda", "<addr> <payload> -- sends the payload by specified address", ls_senda_cmd },
	{ "sendn", "<nodeid> <payload> -- sends the payload by specified address", ls_sendn_cmd },

	{ "list", "-- prints list of connected devices", ls_list_cmd },

    { NULL, NULL, NULL }
};

int main(void)
{
    print_logo();
    xtimer_init();

    radio_init();
    sx1276_init(&sx1276);

    ls_setup(&ls);
    ls_gate_init(&ls);

    sender_pid = thread_create(sender_stack, sizeof(sender_stack), THREAD_PRIORITY_MAIN - 1, 0, sender, NULL, "sender thread");
    uart_gate_init();

    blink_led();

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
