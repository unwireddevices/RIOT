/*
 * Copyright (C) 2016 Cr0s
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for SX1276 modem driver
 *
 * @author      Cr0s
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../driver_sx1276_relay_network/common.h"
#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "xtimer.h"
#include "lpm.h"
#include "periph/rtc.h"
#include "random.h"

#include "board.h"
#include "lrn.h"
#include "lrn_routing.h"
#include "lrn_crypto.h"
#include "lrn_frame_fifo.h"
#include "sx1276_regs_lora.h"
#include "sx1276_regs_fsk.h"

static lrn_t lrn;

static int count;

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

void frame_recv_cb (lrn_addr_t from, uint8_t* buf, uint8_t buflen) {
	printf("Frame received from 0x%02X (%u bytes): ", from, buflen);
	uint8_t i;
	for(i = 0; i < buflen; i++) {
		printf("%02X ", buf[i]);
	}

	printf("\nCount: %d\n", ++count);
}

void joined_cb(uint8_t numnodes) {
	printf("joined to the network with %d nodes\n", (unsigned int) numnodes);

	LED0_ON;
}

void init_lrn(void)
{
    lrn._sx1276 = &sx1276;
    lrn.data_recv_cb = frame_recv_cb;
    lrn.cmd_recv_cb = frame_recv_cb;
    lrn.joined_cb = joined_cb;

    lrn.datarate_rx = DR6;
    lrn.datarate_tx = DR6; // XXX

    uint8_t key[16] = { 0xAB, 0xFE, 0x06, 0x43, 0x42, 0x11, 0xDE, 0xEE, 0x12, 0x54, 0xD0, 0xAA, 0x81, 0x00, 0xA1, 0xFF };
    memcpy(lrn.crypto_key, key, 16);


    lrn_init(&lrn);
}

void init_radio(void)
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

int lrn_set_addr(int argc, char **argv)
{
	if (argc < 2) {
		puts("lrn_set_addr: no address specified");

		return -1;
	}

	lrn.dev_addr = (lrn_addr_t) atoi(argv[1]);

    return 0;
}

int lrn_send_cmd(int argc, char **argv)
{
    if (argc < 3) {
        return -1;
    }

    lrn_addr_t dest = atoi(argv[1]);

    uint8_t *buf = (uint8_t *) argv[2];
    uint8_t buflen = strlen(argv[2]) + 1;

    lrn_send(&lrn, lrn.dev_addr, dest, LRN_TYPE_CMD, count++, buf, buflen);

    return 0;
}

int lrn_listen_cmd(int argc, char **argv)
{
	lrn_listen(&lrn);

	puts("lrn_listen: device is listening");
    return 0;
}

int lrn_sleep_cmd(int argc, char **argv)
{
	lrn_sleep(&lrn);

	puts("lrn_sleep: device is sleeping");
    return 0;
}

int lrn_join_cmd(int argc, char **argv) {
    lrn_join(&lrn);

    return 0;
}

static const shell_command_t shell_commands[] = {
	{ "lrn_join", "joins to the network", lrn_join_cmd },
    { "lrn_set_addr", "<addr> - sets up a device address", lrn_set_addr },
    { "lrn_send", "<to> <payload> - sends a frame to the device with address <to>", lrn_send_cmd },
    { "lrn_listen", "setup node into listener/relay mode", lrn_listen_cmd, },
	{ "lrn_sleep", "setup node into sleep mode", lrn_sleep_cmd, },

    { NULL, NULL, NULL }
};

int main(void)
{
    print_logo();
    xtimer_init();
    init_radio();
    init_lrn();
    blink_led();

    // XXX: choose one of these
//#define RX_TEST_GATE
//#define RX_TEST_2
//#define RX_TEST_3
//#define TX_TEST_JOIN
#define TX_TEST_MAX

#ifdef TX_TEST_MAX
    lrn_routing_table_entry_t routing_table[10] = {
    		{ 0x02, DR6, false },
			{ 0x04, DR3, false },
    };

    lrn.routing_table = routing_table;
    lrn.routing_table_size = 2;

    lrn.joined = true;
    lrn.dev_addr = 0x04;

    lrn_listen(&lrn);

    uint8_t buf[LRN_PAYLOAD_SIZE_MAX] = { 0xAA, 0x00, };
    buf[LRN_PAYLOAD_SIZE_MAX - 1] = 0xAB;

    for (int i = 0; i < LRN_PAYLOAD_SIZE_MAX; i++)
    	buf[i] = i;

    for (int i = 0; i < LRN_PAYLOAD_SIZE_MAX; i++) {
    	lrn_send(&lrn, lrn.dev_addr, 0x02, LRN_TYPE_CMD, count++, buf, i);

    	xtimer_usleep(1e6 * 5);
    }
#endif
#ifdef TX_TEST_JOIN
    puts("lrn test join");

    lrn.dev_addr = 0x04;

    lrn_listen(&lrn);

    while(!lrn.joined) {
    	lrn_join(&lrn);
    	xtimer_usleep(1e6 * 30);
    }
#endif

#ifdef RX_TEST_GATE
    puts("Gate mode");

    lrn_routing_table_entry_t routing_table[10] = {
    		{ 0x01, DR6, true },
    		{ 0x02, DR6, false },
			{ 0x04, DR3, false },
    		{ 0x03, DR6, false },
    };

    lrn.routing_table = routing_table;
    lrn.routing_table_size = 4;

    lrn.joined = true;
    lrn.is_gateway = true;
    lrn.dev_addr = 0x01;

    lrn_listen(&lrn);
#endif

#ifdef RX_TEST_2
    puts("Rx test 2");

    lrn_routing_table_entry_t routing_table[10] = {
    		//{ 0x01, DR6, true },
    		{ 0x02, DR6, false },
			{ 0x04, DR3, false },
    		//{ 0x03, DR6, false },
    };

    lrn.routing_table = routing_table;
    lrn.routing_table_size = 2;

    lrn.joined = true;
    lrn.dev_addr = 0x02;
    lrn_listen(&lrn);
#endif

#ifdef RX_TEST_3
    puts("RX test 3");

    lrn_routing_table_entry_t routing_table[10] = {
    		{ 0x01, DR6, true },
    		{ 0x02, DR6, false },
			{ 0x04, DR3, false },
    		{ 0x03, DR6, false },
    };

    lrn.routing_table = routing_table;
    lrn.routing_table_size = 4;

    lrn.joined = true;
    lrn.dev_addr = 0x03;
    lrn_listen(&lrn);
#endif

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
