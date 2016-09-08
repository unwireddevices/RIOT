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
 * @brief       Default application that shows a lot of functionality of RIOT
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
#include "periph/gpio.h"
#include "random.h"

#include "sx1276.h"
#include "board.h"

#include "ls-end-device.h"
#include "unwds-common.h"
#include "unwds-gpio.h"


static uint8_t join_key[AES_KEY_SIZE] = { 0xCA, 0xFE, 0xBA, 0xBE, 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED, 0xCA, 0xFE, 0xDE, 0xAD, 0xBE, 0xEF };

static sx1276_t sx1276;
static ls_ed_t ls;

static unsigned int join_retr_count;

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

void lnkchk_timeout_cb(void) {
	puts("lnkchk: check failed, link is unavailable");

	ls_ed_join(&ls);
}

void link_good_cb(void) {
	puts("lnkchk: link good");

	blink_led();
}

void joined_timeout_cb(void) {
	puts("ls: join request timed out, resenting");

    xtimer_usleep(1e6 * 2 * ++join_retr_count);
	ls_ed_join(&ls);
}


void joined_cb(void) {
	puts("ls-ed: successfully joined to the network");

	ls.status.batt_level = 0xAA;
	ls_ed_lnkchk(&ls);

	blink_led();
}

void appdata_send_failed_cb(void) {
	puts("ls-ed: application data confirmation timeout. Checking link...");

	ls_ed_lnkchk(&ls);
}

void appdata_received_cb(uint8_t *buf, size_t buflen) {
	printf("ls-ed: received data: \"%s\"\n", buf);

	char reply[UNWDS_MAX_REPLY_LEN];
	memset(reply, 0, sizeof(reply));

	unwds_command((char *) buf, reply);

    int res = ls_ed_send_app_data(&ls, (uint8_t *) reply, strlen(reply) + 1, true);
    if (res < 0)
    	printf("sendc: error #%d\n", res);

    blink_led();
}

void ls_setup(ls_ed_t *ls)
{
	ls->settings.class = LS_ED_CLASS_B;

	ls->settings.dr = LS_DR3;
	ls->settings.channel = 0;

    ls->settings.app_id = 0xFEEDF00D;
    ls->settings.node_id = 1;

    memcpy(ls->settings.crypto.join_key, join_key, LS_MIC_KEY_LEN);

    ls->join_timeout_cb = joined_timeout_cb;
    ls->joined_cb = joined_cb;

    ls->link_good_cb = link_good_cb;
    ls->lnkchk_timeout_cb = lnkchk_timeout_cb;

    ls->appdata_send_failed_cb = appdata_send_failed_cb;
    ls->settings.max_retr = 5;									/* Maximum number of confirmed data retransmissions */

    ls->appdata_received_cb = appdata_received_cb;

    ls->settings.lnkchk_failed_action = LS_ED_REJOIN;
    ls->settings.lnkchk_period_s = 255;

    ls->_internal.sx1276 = &sx1276;
}

int convert(const char *hex_str, unsigned char *byte_array, int byte_array_max)
{
    int hex_str_len = strlen(hex_str);
    int i = 0, j = 0;

    // The output array size is half the hex_str length (rounded up)
    int byte_array_size = (hex_str_len+1)/2;

    if (byte_array_size > byte_array_max)
    {
        // Too big for the output array
        return -1;
    }

    if (hex_str_len % 2 == 1)
    {
        // hex_str is an odd length, so assume an implicit "0" prefix
        if (sscanf(&(hex_str[0]), "%1hhx", &(byte_array[0])) != 1)
        {
            return -1;
        }

        i = j = 1;
    }

    for (; i < hex_str_len; i+=2, j++)
    {
    	/**
    	 * TODO: check this
    	 */
        if (sscanf(&hex_str[i], "%2hhx", &(byte_array[j])) != 1)
        {
            return -1;
        }
    }

    return byte_array_size;
}

int ls_get_cmd(int argc, char **argv) {
	if (argc != 2) {
		puts("usage: get <key>");
		puts("keys:");
		puts("\taddr -- returns device address assigned by the gate or manually");
		puts("\tnodeid -- returns unique device ID in hex");
		puts("\tappid -- returns device application ID in hex");
		puts("\tdr -- returns device data rate");
		puts("\tch -- sets device working channel");
	}


    return 0;
}

int ls_set_cmd(int argc, char **argv) {
	if (argc != 3) {
		puts("usage: get <key> <value>");
		puts("keys:");
		puts("\tnodeid <feedbeefcafebabe> -- sets unique device ID in hex (must be 16 hex digits)");
		puts("\tappid <feedbeefcafebabe> -- sets device application ID in hex (must be 16 hex digits)");
		puts("\tjoinkey <feedbeefcafebabefeedbeefcafebabe> -- sets device join key in hex (must be 32 hex digits)");
		puts("\tdr <0-7> -- sets device data rate");
		puts("\tch <0-2> -- sets device channel");
	}

	char* key = argv[1];
	char* value = argv[2];

	if (strcmp(key, "nodeid") == 0) {
		/*if (strlen(argv[2]) != 16) {
			puts("set nodeid: id length is not enough");
			return -1;
		}


		uint8_t bytes[8];
		convert(argv[2], bytes, 8);

		uint64_t v;
		for (int i = 0; i < 8; i++) {
			v |= bytes[i] << (56 - 8 * i);
		}*/

		ls.settings.node_id = strtol(argv[2], NULL, 16);
	} else if (strcmp(key, "appid") == 0) {
		/*joif (argc - 2 != 8) {
			puts("set nodeid: id length is not enough");
			return -1;
		}

		uint64_t v;
		for (int i = 0; i < 8; i++) {
			v |= strtol(argv[2 + i], NULL, 16) << (56 - 8 * i);
		}*/

		ls.settings.node_id = strtol(argv[2], NULL, 16);
	} else if (strcmp(key, "joinkey") == 0) {
		if (argc - 2 != 16) {
			puts("set joinkey: key length is not enough");
			return -1;
		}

		for (int i = 0; i < 16; i++) {
			ls.settings.crypto.join_key[i] = (uint8_t) strtol(argv[2 + i], NULL, 16);
		}
	} else if (strcmp(key, "dr") == 0) {
		uint8_t v = strtol(value, NULL, 10);

		if (v > 7) {
			puts("set dr: datarate value must be from 0 to 7");
		}

		ls.settings.dr = (ls_datarate_t) v;
	} else if (strcmp(key, "ch") == 0) {
		uint8_t v = strtol(value, NULL, 10);

		if (v > 7) {
			puts("set ch: channel value must be from 0 to 2");
		}

		ls.settings.channel = (ls_channel_t) v;
	}

    return 0;
}

int ls_join_cmd(int argc, char **argv) {
	join_retr_count = 0;

	puts("join: sending join request...");
    ls_ed_join(&ls);

    return 0;
}

int ls_unjoin_cmd(int argc, char **argv) {
	join_retr_count = 0;

	puts("unjoin: leaving network...");
    ls_ed_unjoin(&ls);

    return 0;
}

int ls_sendu_cmd(int argc, char **argv) {
	if (!ls._internal.is_joined) {
		puts("sendu: not joined to the network");
		return -1;
	}

    if (argc != 2) {
    	puts("send: payload not specified");
    }

    size_t buflen = strlen(argv[1]) + 1; /* Don't forget about \0 at the end of a string */

    int res = ls_ed_send_app_data(&ls, (uint8_t *) argv[1], buflen, false);
    if (res < 0)
    	printf("sendu: error #%d\n", res);

    return 0;
}

int ls_sendc_cmd(int argc, char **argv) {
	if (!ls._internal.is_joined) {
		puts("sendc: not joined to the network");
		return -1;
	}

    if (argc != 2) {
    	puts("send: payload not specified");
    }

    size_t buflen = strlen(argv[1]) + 1; /* Don't forget about \0 at the end of a string */

    int res = ls_ed_send_app_data(&ls, (uint8_t *) argv[1], buflen, true);
    if (res < 0)
    	printf("sendc: error #%d\n", res);

    return 0;
}

int ls_lnkchk_cmd(int argc, char **argv) {
	if (!ls._internal.is_joined) {
		puts("lnchk: not joined to the network");
		return -1;
	}

	ls_ed_lnkchk(&ls);

	return 0;
}

int store_cmd(int argc, char **argv) {

	return 0;
}

int load_cmd(int argc, char **argv) {

	return 0;
}

static const shell_command_t shell_commands[] = {
	{ "set", "<config> <value> -- sets up value for the config entry", ls_set_cmd },
	{ "get", "<config> -- gets value for the config entry", ls_get_cmd },

	//{ "store", "-- saves current configuration to the EEPROM", store_cmd },
	//{ "load", "-- loads current configuration from the EEPROM", load_cmd },

	{ "join", "joins to the network", ls_join_cmd },
	{ "leave", "leaves the network", ls_unjoin_cmd },
	{ "lnkchk", "link check", ls_lnkchk_cmd },

	{ "sendu", "<payload> -- sends the UNconfirmed message payload to the gateway", ls_sendu_cmd },
	{ "sendc", "<payload> -- sends the message payload to the gateway with confirmation", ls_sendc_cmd },

    { NULL, NULL, NULL }
};

#define pulses_

#ifndef pulses
static void unwds_callback(char *buf) {
    size_t buflen = strlen(buf) + 1; /* Don't forget about \0 at the end of a string */

    int res = ls_ed_send_app_data(&ls, (uint8_t *) buf, buflen, true);
    if (res < 0)
    	printf("send: error #%d\n", res);

    blink_led();
}
#endif

#ifdef pulses
void cb(void *arg) {
	(void) arg;

	int i = 0;
	int n = 1280;

	gpio_init(UNWD_GPIO_1, GPIO_OUT);
	gpio_clear(UNWD_GPIO_1);

	int j;
	for (j = 0; j < 2; j++) {
		xtimer_usleep(1e3 * 54);

		for (i = 0; i < n; i++) {
			gpio_set(UNWD_GPIO_1);

			xtimer_usleep(1);

			gpio_clear(UNWD_GPIO_1);

			xtimer_usleep(1);
		}
	}
}
#endif
int main(void)
{
    print_logo();
    xtimer_init();
#ifdef pulses
    gpio_init_int(UNWD_GPIO_4, GPIO_IN, GPIO_RISING, cb, NULL);
    gpio_init(UNWD_GPIO_1, GPIO_OUT);
#endif
#ifndef pulses
    rtc_init();
    radio_init();

    ls_setup(&ls);
    ls_ed_init(&ls);

    unwds_init(unwds_callback);

    ls.settings.ability = unwds_get_ability();

    xtimer_usleep(1e6 * 1);
    ls_ed_join(&ls);

    blink_led();
#endif
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
