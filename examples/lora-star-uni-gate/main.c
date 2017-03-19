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

#include "eeprom.h"

#include "board.h"

#include "ls-mac-types.h"
#include "sx1276.h"

#include "main.h"
#include "config.h"

static nvram_t nvram;

void print_logo(void)
{
    puts("*****************************************");
    puts("Unwired Range firmware by Unwired Devices");
    puts("www.unwds.com - info@unwds.com");
#ifdef NO_RIOT_BANNER
    puts("powered by RIOT - www.riot-os.org");
#endif
    puts("*****************************************");
    printf("Version: %s (%s %s)\n", FIRMWARE_VERSION, __DATE__, __TIME__);
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

static shell_command_t shell_commands[10] = {};

static void init_role(config_role_t role)
{
    switch (role) {
        case ROLE_GATE:
            init_gate((shell_command_t **) &shell_commands);
            break;

        case ROLE_NO_EUI64:
            init_no_eui64((shell_command_t **) &shell_commands);
            break;

        default:
        case ROLE_NO_CFG:
            init_no_cfg((shell_command_t **) &shell_commands);

            break;
    }
}

int main(void)
{
    /* Gate never sleeps */
    lpm_prevent_sleep = 1;

    print_logo();
    xtimer_init();

    nvram_eeprom_init(&nvram);

    /* Check EUI64 */
    if (!load_eui64_nvram(&nvram)) {
        puts("[config] No EUI64 defined for this device. Please provide EUI64 and reboot to apply changes.");
    }

    /* It's first launch or config memory is corrupted */
    if (!load_config_nvram(&nvram)) {
        puts("[config] No valid configuration found in NVRAM. It's either first launch or NVRAM content is corrupted.");
        puts("[config] Please select role for this device and provide APPID64 and JOINKEY for this device.");

        config_reset_nvram(&nvram);
    }
    else {
        puts("[config] Configuration loaded from NVRAM");
    }

    init_role(config_get_role());

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
