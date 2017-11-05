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

#include "lpm.h"
#include "arch/lpm_arch.h"
#include "thread.h"
#include "xtimer.h"
#include "lpm.h"
#include "periph/rtc.h"
#include "random.h"
#include "cpu.h"

#include "rtctimers.h"
#include "rtctimers-millis.h"

#include "eeprom.h"

#include "board.h"

#include "ls-mac-types.h"
#include "ls-config.h"
#include "sx1276.h"

#include "main.h"

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
    char cpu_model[20];
    switch (get_cpu_category()) {
        case 1:
            snprintf(cpu_model, 20, "STM32L151CB");
            break;
        case 2:
            snprintf(cpu_model, 20, "STM32L151CB-A");
            break;
        case 3:
            snprintf(cpu_model, 20, "STM32L151CC");
            break;
    }
    printf("%s %lu MHz (%s clock)\n", cpu_model,
                                      cpu_clock_global/1000000,
                                      cpu_clock_source);
    printf("%lu KB RAM, %lu KB flash, %lu KB EEPROM\n\n", get_cpu_ram_size()/1024,
                                                          get_cpu_flash_size()/1024,
                                                          get_cpu_eeprom_size()/1024);
}

void blink_led(void)
{
    int i;
    LED0_OFF;
    for (i = 0; i < 4; i++) {
        LED0_TOGGLE;
        rtctimers_millis_sleep(50);        
    }
    LED0_OFF;
}

static shell_command_t shell_commands_node[UNWDS_SHELL_COMMANDS_MAX] = {};

static void init_role(config_role_t role) {
	switch (role) {
	case ROLE_NODE:
		init_node((shell_command_t **) &shell_commands_node);
		break;

	case ROLE_NO_EUI64:
		init_no_eui64((shell_command_t **) &shell_commands_node);
		break;
		
	case ROLE_EMPTY_KEY:
		init_no_key((shell_command_t **) &shell_commands_node);
		break;

	default:
	case ROLE_NO_CFG:
		init_no_cfg((shell_command_t **) &shell_commands_node);

		break;
	}
}

int main(void)
{
	lpm_arch_init();
	
    /* disable sleep and frequency switching for now */
	lpm_prevent_sleep = 1;
    lpm_prevent_switch = 1;
    
    print_logo();
    xtimer_init();
    rtctimers_init();
    rtctimers_millis_init();

    nvram_eeprom_init(&nvram);

    /* Check EUI64 */
    if (!load_eui64_nvram(&nvram)) {
    	puts("[config] No EUI64 defined for this device. Please provide EUI64 and reboot to apply changes.");
    } else {
        if (!load_config_nvram(&nvram)) {
            /* It's first launch or config memory is corrupted */
            puts("[config] No valid configuration found in NVRAM. It's either first launch or NVRAM content is corrupted.");
            puts("[config] Could you please provide APPID64 and JOINKEY for this device?");

            config_reset_nvram(&nvram);
        } else {
            puts("[config] Configuration loaded from NVRAM");
        }
    }

    init_role(config_get_role());

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands_node, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
