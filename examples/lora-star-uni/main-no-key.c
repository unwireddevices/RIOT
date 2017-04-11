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

#include "main.h"
#include "utils.h"
#include "ls-config.h"

static uint8_t joinkey[16];
static bool joinkey_set = false;

static uint32_t devnonce;
static bool devnonce_set = false;

static int set_cmd(int argc, char **argv)
{
    if (argc < 3) {
        puts("set joinkey <32 hex digits> -- sets the join (network) encryption key. Must be shared between all nodes in the same network");
        puts("set devnonce <8 gex digits> -- sets device nonce (random number is preferred). Must be shared between gate and this node");
        puts("\tExample: set joinkey aabbccddeeff00112233445566778899");
        puts("\tExample: set devnonce aabbccdd");

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
    } else if (strcmp(type, "devnonce") == 0) {
        if (strlen(arg) != 8) {
            puts("[error] There must be 8 hexadecimal digits in lower case");
            return 1;
        }

        uint32_t d = 0;

        if (!hex_to_bytesn(arg, 4, (uint8_t *) &d, true)) {
            puts("[error] Pardon me, but that's not a hex number!");
            return 1;
        }

        printf("[ok] That's a nice nonce, thank you!\n \
Don't forget to save it and write it down for future use, as there's no way to get it back from the programmed LoRa modem.\n \
DEVNONCE = %s\n", arg);

        devnonce_set = true;
        devnonce = d;
    }

    return 0;
}

static int save_cmd(int argc, char **argv)
{
	if (!joinkey_set || !devnonce) {
		puts("[error] I'm deeply sorry, but you have provided no key or device nonce");
		return -1;
	}
	
	if (argc == 1) {
		puts("[!] Saving...");
		
		if (config_write_main_block(config_get_appid(), joinkey, devnonce)) {
			puts("[ok] Data was written. Rebooting.");

			/* Reboot */
			NVIC_SystemReset();
		} else {
			puts("[error] An error occurred trying to save the data");
		}
	}

    return 0;
}

static const shell_command_t shell_commands_nokey[] = {
    { "set", "joinkey|devnonce <value> -- set up network key or device nonce", set_cmd },

    { "save", "Save the configuration to non-volatile memory", save_cmd },

    { NULL, NULL, NULL }
};

void init_no_key(shell_command_t **commands)
{
    /* Set our commands for shell */
    memcpy(commands, shell_commands_nokey, sizeof(shell_commands_nokey));

    blink_led();

    puts("[unk] I'm afraid, there's no security key. Could you please set a new one with 'set joinkey NNNNNNNNNNNNNNNN' command?");
}

#ifdef __cplusplus
}
#endif
