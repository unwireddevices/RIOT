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
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "xtimer.h"
#include "cpu.h"

#include "shell.h"
#include "main.h"
#include "utils.h"
#include "config.h"

static uint8_t joinkey[16];
static bool joinkey_set = false;


static int set_cmd(int argc, char **argv)
{
    if (argc < 3) {
        puts("set joinkey <32 hex digits> -- sets the join (network) encryption key. Must be shared between all nodes in the same network");
        puts("\tExample: set joinkey aabbccddeeff00112233445566778899");

        return 1;
    }

	char *type = argv[1];
    char *arg = argv[2];
	
    if (strcmp(type, "joinkey") == 0) {
        if (strlen(arg) != 32) {
            puts("[error] There must be 32 hexadecimal digits in lower case");
            return 1;
        }

        char s[32] = {};

        if (!hex_to_bytes(arg, joinkey, false)) {
            puts("[error] Pardon me, but that's not a hex number!");
            return 1;
        }
		
		bool all_zero = true;
		for (int i=0; i<16; i++) {
			if (joinkey[i] != 0) {
				all_zero = false;
			}
		}
		
		if (all_zero) {
			puts("[error] Oh, I forgot to mention — a lot of 0's wouldn't do.\nThank you for understanding.");
			return -1;
		}

        bytes_to_hex(joinkey, 16, s, false);
        joinkey_set = true;

        printf("[ok] That's a nice key, thank you!\n \
Don't forget to save it and write it down for future use, as there's no way to get it back from the programmed LoRa modem.\n \
JOINKEY = %s\n", s);
    }


    return 0;
}

static int save_cmd(int argc, char **argv)
{
	if (!joinkey_set) {
		puts("[error] I'm deeply sorry, but you have provided no key.");
		return -1;
	}
	
	if (argc == 1) {
		puts("[!] Saving security key...");
		
		if (config_write_main_block(config_get_appid(), joinkey)) {
			puts("[ok] Security key was written. Rebooting.");

			/* Reboot */
			NVIC_SystemReset();
		} else {
			puts("[error] An error occurred trying to save the key");
		}
	}

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "set", "joinkey <value> -- set up security key", set_cmd },

    { "save", "Save the configuration to non-volatile memory", save_cmd },

    { NULL, NULL, NULL }
};

void init_no_key(shell_command_t **commands)
{
    /* Set our commands for shell */
    memcpy(commands, shell_commands, sizeof(shell_commands));

    blink_led();

    puts("[unk] I'm afraid, there's no security key. Could you please set a new one with 'set joinkey NNNNNNNNNNNNNNNN' command?");
}

#ifdef __cplusplus
}
#endif
