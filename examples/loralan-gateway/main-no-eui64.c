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

#include "shell.h"
#include "main.h"
#include "utils.h"
#include "config.h"

static uint64_t eui64 = 0;

static void print_eui64(void)
{
    if (eui64) {
        printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    }
    else {
        puts("EUI64 = <not set>");
    }
}


static void print_config(void)
{
    puts("[config] Current configuration:");
    print_eui64();
}

static int set_cmd(int argc, char **argv)
{
    if (argc < 3) {
        puts("set eui64 <16 hex digits> -- sets device EUI64 (permanently after save!)");
        puts("\tExample: set eui64 00000000000011ff");

        return 1;
    }

    char *type = argv[1];
    char *arg = argv[2];

    if (strcmp(type, "eui64") == 0) {
        uint64_t id = 0;

        if (strlen(arg) != 16) {
            puts("[error] There must be 16 hexadecimal digits in lower case as EUI64 ID");
            return 1;
        }

        if (!hex_to_bytes(arg, (uint8_t *) &id, true)) {
            puts("[error] Invalid number format specified");
            return 1;
        }

        printf("[ok] EUI64 = 0x%08x%08x\n", (unsigned int) (id >> 32), (unsigned int) (id & 0xFFFFFFFF));
        eui64 = id;
    }

    print_config();

    return 0;
}

static int get_cmd(int argc, char **argv)
{
    if (argc < 2) {
        puts("get eui64 -- gets device EUI64");
    }

    char *type = argv[1];

    if (strcmp(type, "eui64") == 0) {
    	print_eui64();
    }

    return 0;
}

static int save_cmd(int argc, char **argv)
{
	if (argc == 1) {
		puts("Current configuration:");
		print_config();

		puts("");
        
        puts("[!] Saving current configuration...");
        
        if (write_eui64_nvram(eui64)) {
            puts("[ok] Configuration was written. Rebooting.");
            NVIC_SystemReset();
        } else {
            puts("[error] An error occurred when saving the configuration");
        }
    }

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "set", "<config> <value> -- sets up value for the config entry", set_cmd },
    { "get", "<config> -- gets value for the config entry", get_cmd },

    { "save", "Saves configuration in nvram", save_cmd },

    { NULL, NULL, NULL }
};

void init_no_eui64(shell_command_t **commands)
{
    /* Set our commands for shell */
    memcpy(commands, shell_commands, sizeof(shell_commands));

    blink_led();

    puts("[unk] Please configure this device. Type \"help\" for list of possible commands.");
    print_config();
}

#ifdef __cplusplus
}
#endif
