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

#include "board.h"

#include "ls-mac-types.h"
#include "ls-crypto.h"
#include "ls-gate.h"

#include "sx1276.h"

sx1276_t sx1276;
ls_gate_t ls;

static uint8_t join_key[LS_MIC_KEY_LEN] = { 0xCA, 0xFE, 0xBA, 0xBE, 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED, 0xCA, 0xFE, 0xDE, 0xAD, 0xBE, 0xEF };

ls_gate_channel_t channels[1] = {
		{ LS_DR6, 0, { &sx1276, &ls } },	/* DR3, channel 2 */
};

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

int ls_list_cmd(int argc, char **argv);

void node_kicked_cb (ls_gate_node_t *node) {
	printf("gate: node with ID 0x%08X and address 0x%08X kicked from the network due to long silence\n",
			(unsigned int) node->node_id,
			(unsigned int) node->addr);

	ls_list_cmd(0, NULL);
}

uint32_t node_joined_cb (ls_gate_node_t *node) {
	printf("gate: node with ID 0x%08X joined to the network with address 0x%08X\n",
			(unsigned int) node->node_id,
			(unsigned int) node->addr);

	/* Return random app nonce */
	return sx1276_random(&sx1276);
}

bool accept_node_join_cb(uint64_t dev_id, uint64_t app_id) {
	return true; /* Stub */
}

void app_data_received_cb (ls_addr_t devaddr, ls_channel_t ch, uint8_t *buf, size_t bufsize) {
	printf("data from 0x%08X: \"%s\"\n", (unsigned int) devaddr, buf);
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

static const shell_command_t shell_commands[] = {
	{ "set", "<config> <value> -- sets up value for the config entry", ls_set_cmd },
	{ "get", "<config> -- gets value for the config entry", ls_get_cmd },

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
    blink_led();

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
