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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "xtimer.h"
#include "cpu.h"

#include "main.h"
#include "utils.h"
#include "ls-config.h"

static uint64_t appid = 0;

static uint8_t joinkey[16] = {
};

bool joinkey_set = false;

uint32_t devnonce = 0;
bool devnonce_set = false;

static void print_appid64(void)
{
    if (appid) {
        printf("APPID64 = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));
    }
    else {
        puts("APPID64 = <not set>");
    }
}

static void print_joinkey(void)
{
    if (joinkey_set) {
        char s[32] = {};
        bytes_to_hex(joinkey, 16, s, false);
        printf("JOINKEY = %s\n", s);
    }
    else {
        puts("JOINKEY = <not set>");
    }
}

static void print_devnonce(void)
{
    if (devnonce_set) {
        printf("DEVNONCE = 0x%08X\n", (unsigned int) devnonce);
    }
    else {
        puts("DEVNONCE = <not set>");
    }
}

static void print_config(void)
{
    puts("[config] Current configuration:");

    print_appid64();
    print_joinkey();
    print_devnonce();
}

static int unk_set_cmd(int argc, char **argv)
{
    if (argc < 3) {
        puts("set appid64 <16 hex digits> -- sets application ID");
        puts("\tExample: set appid64 00000000000011ff");

        puts("");

        puts("set joinkey <32 hex digits> -- sets the join (network) encryption key. Must be shared between all nodes in same network");
        puts("\tExample: set joinkey aabbccddeeff00112233445566778899");

        return 1;
    }

    char *type = argv[1];
    char *arg = argv[2];

    if (strcmp(type, "appid64") == 0) {
        uint64_t id = 0;

        if (strlen(arg) != 16) {
            puts("[error] There must be 16 hexadecimal digits in lower case as APP64 ID");
            return 1;
        }

        if (!hex_to_bytes(arg, (uint8_t *) &id, true)) {
            puts("[error] Invalid number format specified");
            return 1;
        }

        printf("[ok] APPID64 = 0x%08x%08x\n", (unsigned int) (id >> 32), (unsigned int) (id & 0xFFFFFFFF));
        appid = id;
    }
    else if (strcmp(type, "joinkey") == 0) {
        if (strlen(arg) != 32) {
            puts("[error] There must be 32 hexadecimal digits in lower case as JOIN KEY");
            return 1;
        }

        if (!hex_to_bytes(arg, joinkey, false)) {
            puts("[error] Invalid format specified");
            return 1;
        }

        joinkey_set = true;

        printf("[ok] JOINKEY = %s\n", arg);
    } if (strcmp(type, "devnonce") == 0) {
        if (strlen(arg) != 8) {
            puts("[error] There must be 8 hexadecimal digits in lower case");
            return 1;
        }

        uint32_t d = 0;

        if (!hex_to_bytesn(arg, 8, (uint8_t *) &d, true)) {
            puts("[error] Pardon me, but that's not a hex number!");
            return 1;
        }

        printf("[ok] That's a nice value, thank you!\n \
Don't forget to save it and write it down for future use, as there's no way to get it back from the programmed LoRa modem.\n \
DEVNONCE = %s\n", arg);

        devnonce_set = true;
        devnonce = d;
    }

    print_config();

    return 0;
}

int unk_get_cmd(int argc, char **argv)
{
    if (argc < 2) {
        puts("get appid64 -- gets sets application ID");
        puts("get joinkey -- gets the join (network) encryption key");
        puts("get devnonce -- gets the device nonce value");
    }

    char *type = argv[1];

    if (strcmp(type, "appid64") == 0) {
    	print_appid64();
    }
    else if (strcmp(type, "joinkey") == 0) {
    	print_joinkey();
    } else if (strcmp(type, "devnonce") == 0) {
    	print_devnonce();
    } else {
    	puts("[error] Unknown get parameter");
    }

    return 0;
}

int unk_save_cmd(int argc, char **argv)
{
	if (argc == 1) {
		puts("Current configuration:");
		print_config();

		puts("[!] Saving current configuration...");

		if (config_write_main_block(appid, joinkey, devnonce)) {
			puts("[ok] Configuration is written. Rebooting...");

			/* Reboot */
			NVIC_SystemReset();
		} else {
			puts("[error] An error occurred when saving the configuration");
		}
	}

    return 0;
}

static const shell_command_t shell_commands_nocfg[] = {
    { "set", "<config> <value> -- sets up value for the config entry", unk_set_cmd },
    { "get", "<config> -- gets value for the config entry", unk_get_cmd },

    { "save", "Saves the configuration in nvram", unk_save_cmd },

    { NULL, NULL, NULL }
};

void init_no_cfg(shell_command_t **commands)
{
    /* Set our commands for shell */
    memcpy(commands, shell_commands_nocfg, sizeof(shell_commands_nocfg));

    blink_led();

    puts("[unk] Please configure this device. Type \"help\" for list of possible commands.");
    print_config();
}

#ifdef __cplusplus
}
#endif
