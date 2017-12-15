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

#include "cpu.h"
#include "eeprom.h"
#include "shell.h"
#include "utils.h"
#include "ls-config.h"

#include "lpm.h"
#include "arch/lpm_arch.h"
#include "periph/rtc.h"
#include "random.h"
#include "cpu.h"
#include "xtimer.h"
#include "rtctimers.h"
#include "rtctimers-millis.h"
#include "unwds-common.h"

#include "ls-settings.h"
#include "ls-init-device.h"
#include "ls-mac.h"

static nvram_t nvram;

static void init_common(shell_command_t *commands);
static void init_config(shell_command_t *commands);
static int init_clear_nvram(int argc, char **argv);
static int init_save_cmd(int argc, char **argv);
static int init_update_cmd(int argc, char **argv);
static void print_config(void);

static uint64_t eui64 = 0;
static uint64_t appid = 0;
static uint8_t joinkey[16] = {};
static uint32_t devnonce = 0;

/**
 * Data rates table.
 */
const uint8_t datarate_table[7][3] = {
    { SX1276_SF12, SX1276_BW_125_KHZ, SX1276_CR_4_5 },       /* DR0 */
    { SX1276_SF11, SX1276_BW_125_KHZ, SX1276_CR_4_5 },       /* DR1 */
    { SX1276_SF10, SX1276_BW_125_KHZ, SX1276_CR_4_5 },       /* DR2 */
    { SX1276_SF9, SX1276_BW_125_KHZ, SX1276_CR_4_5 },        /* DR3 */
    { SX1276_SF8, SX1276_BW_125_KHZ, SX1276_CR_4_5 },        /* DR4 */
    { SX1276_SF7, SX1276_BW_125_KHZ, SX1276_CR_4_5 },        /* DR5 */
    { SX1276_SF7, SX1276_BW_250_KHZ, SX1276_CR_4_5 },        /* DR6 */
};

void ls_setup_sx1276(sx1276_t *dev, ls_datarate_t dr, uint32_t frequency) {
    /* Choose data rate */
    const uint8_t *datarate = datarate_table[dr];
    
    /* Setup transceiver settings according to datarate */
    sx1276_lora_settings_t settings;

    settings.datarate = datarate[0];
    settings.bandwidth = datarate[1];
    settings.coderate = datarate[2];

    settings.crc_on = true;
    settings.freq_hop_on = false;
    settings.hop_period = 0;
    settings.implicit_header = false;
    settings.iq_inverted = false;
    settings.low_datarate_optimize = false;
    settings.payload_len = 0;
    settings.power = TX_OUTPUT_POWER;
    settings.preamble_len = LORA_PREAMBLE_LENGTH;
    settings.rx_continuous = true;
    settings.tx_timeout = 1e6 * 30; // 30 sec
    settings.rx_timeout = LORA_SYMBOL_TIMEOUT;

    sx1276_configure_lora(dev, &settings);

    /* Setup channel */
    sx1276_set_channel(dev, frequency);
}

void init_role(shell_command_t *commands) {
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
            puts("[config] Could you please provide APPID64, DEVNONCE and JOINKEY for this device?");

            config_reset_nvram(&nvram);
        } else {
            puts("[config] Configuration loaded from NVRAM");
        }
    }
    
    
	switch (config_get_role()) {
        case ROLE_NORMAL:
            init_common(commands);
            break;

        case ROLE_NO_EUI64:
        case ROLE_EMPTY_KEY:
        case ROLE_NO_CFG:
            init_config(commands);

            break;
    }
    
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(commands, line_buf, SHELL_DEFAULT_BUFSIZE);
}

static void print_help(void) {
    switch (config_get_role()) {
        case ROLE_NO_EUI64:
            puts("set eui64 <16 hex symbols> -- sets device EUI64 (permanently after save!)");
            puts("\tExample: set eui64 00000000000011ff");
            break;
            
        case ROLE_EMPTY_KEY:
            puts("set joinkey <32 hex symbols> -- sets network encryption key. Must be shared between all nodes in the same network");
            puts("\tExample: set joinkey aabbccddeeff00112233445566778899");
            devnonce = config_get_devnonce();
            appid = config_get_appid();
            eui64 = config_get_nodeid();
            break;
            
        case ROLE_NO_CFG:
            puts("set appid64 <16 hex symbols> -- sets application ID");
            puts("\tExample: set appid64 00000000000011ff");
            puts("");
            puts("set joinkey <32 hex symbols> -- sets network encryption key. Must be shared between all nodes in the same network");
            puts("\tExample: set joinkey aabbccddeeff00112233445566778899");
            puts("");
            puts("set devnonce <8 hex symbols> -- sets session encryption key for no-join devices");
            puts("\tExample: set devnonce aabbccdd");
            eui64 = config_get_nodeid();
            break;
        default:
            puts("Unknown mode");
    }
}

static shell_command_t shell_commands_common[3] = {
    { "save", "-- saves current configuration", init_save_cmd },
    { "clear", "<all|key|modules> -- clear settings stored in NVRAM", init_clear_nvram },
    { "update", " -- reboot in bootloader mode", init_update_cmd },
};

static void init_common(shell_command_t *commands) {
    puts("[device] Initializing...");
    memcpy(commands, shell_commands_common, sizeof(shell_commands_common));
    init_normal(commands);
}

static int set_cmd(int argc, char **argv)
{
    if (argc < 3) {
        print_help();
        return 1;
    }

    char *type = argv[1];
    char *arg = argv[2];

    if ((strcmp(type, "appid64") == 0) && (config_get_role() == ROLE_NO_CFG)) {
        uint64_t id = 0;

        if (strlen(arg) != 16) {
            puts("[error] AppID must be 64 bits (16 hex symbols) long");
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
            puts("[error] Joinkey must be 128 bits (32 hex symbols) long");
            return 1;
        }

        if (!hex_to_bytes(arg, joinkey, false)) {
            puts("[error] Invalid format specified");
            return 1;
        }

        printf("[ok] JOINKEY = %s\n", arg);
    } 
    else if ((strcmp(type, "devnonce") == 0) && (config_get_role() == ROLE_NO_CFG)) {
        if (strlen(arg) != 8) {
            puts("[error] Nonce must be 32 bits (8 hex symbols) long");
            return 1;
        }

        uint32_t d = 0;

        if (!hex_to_bytesn(arg, 8, (uint8_t *) &d, true)) {
            puts("[error] Invalid format specified");
            return 1;
        }

        printf("[ok] DEVNONCE = %s\n", arg);

        devnonce = d;
    }
    else if ((strcmp(type, "eui64") == 0) && (config_get_role() == ROLE_NO_EUI64)) {
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
    } else {
        puts("[error] Unknown command");
    }
    
    print_config();
    
    puts("Settings can be changed by calling 'set' command again");
    puts("Invoke 'save' command when finished");

    return 0;
}

static int get_cmd(int argc, char **argv)
{
    print_config();
    
    return 0;
}

static int save_cmd(int argc, char **argv)
{
	if (argc == 1) {
		puts("Current configuration:");
		print_config();

		puts("[!] Saving current configuration...");

        bool status = false;
        
        switch (config_get_role()) {
            case ROLE_EMPTY_KEY:
                /* Set joinkey */
                status = config_write_main_block(appid, joinkey, devnonce);
                break;
            case ROLE_NO_EUI64:
                /* Set EUI64 */
                status = write_eui64_nvram(eui64);
                break;
            case ROLE_NO_CFG:
                /* Set appID, joinkey and nonce */
                status = config_write_main_block(appid, joinkey, devnonce);
                break;
            default:
                break;
        }
        
        if (status) {
            puts("Configuration saved, rebooting");
            NVIC_SystemReset();
        } else {
            puts("[!] Error saving configuration");
        }
    }

    return 0;
}

static const shell_command_t shell_commands_cfg[] = {
    { "set", "<config> <value> -- set device configuration values", set_cmd },
    { "get", "-- print current configuration", get_cmd },
    { "save", "-- save configuration to EEPROM", save_cmd },

    { NULL, NULL, NULL }
};

static void init_config(shell_command_t *commands)
{
    /* Set our commands for shell */
    memcpy(commands, shell_commands_cfg, sizeof(shell_commands_cfg));

    blink_led(LED_GREEN);

    print_help();
    print_config();
}

static void print_config(void)
{
    puts("[config] Current configuration:");
    
    char s[32] = {};
    
    printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    
    bytes_to_hex(joinkey, 16, s, false);
    printf("JOINKEY = %s\n", s);
    
    printf("DEVNONCE = 0x%08X\n", (unsigned int) devnonce);
    
    printf("APPID64 = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));
}

static int init_clear_nvram(int argc, char **argv)
{
    if (argc < 2) {
        puts("Usage: clear <all|key|modules> -- clear all NVRAM contents or just the security key.");
        return 1;
    }

    char *key = argv[1];

    if (strcmp(key, "all") == 0) {
        puts("Clearing EEPROM, please wait");
        if (clear_nvram()) {
            puts("[ok] Settings cleared, rebooting");
            NVIC_SystemReset();
        }
        else {
            puts("[error] Unable to clear NVRAM");
        }
    }
    else if (strcmp(key, "key") == 0) {
        uint8_t joinkey_zero[16];
        memset(joinkey_zero, 0, 16);
        if (config_write_main_block(config_get_appid(), joinkey_zero, 0)) {
            puts("[ok] Security key and device nonce was zeroed. Rebooting.");
            NVIC_SystemReset();
        }
        else {
            puts("[error] An error occurred trying to save the key");
        }
    }
    else if (strcmp(key, "modules") == 0) {
        puts("Please wait a minute while I'm cleaning up here...");
        if (clear_nvram_modules(0)) {
            puts("[ok] Module settings cleared, let me reboot this device now");
            NVIC_SystemReset();
        }
        else {
            puts("[error] Unable to clear NVRAM");
        }
    }

    return 0;
}

static int init_save_cmd(int argc, char **argv) {
    (void) argc;
    (void) argv;
    
    puts("[*] Saving configuration...");
    
    if (!unwds_config_save()) {
        puts("[error] Unable to save configuration");
    }

    puts("[done] Configuration saved. Type \"reboot\" to apply changes.");
    
    return 0;
}

static int init_update_cmd(int argc, char **argv) {
    (void) argc;
    (void) argv;
    
    puts("[*] Rebooting to UART bootloader...");
    rtc_save_backup(0xB00710AD, 0);
    
    NVIC_SystemReset();
    
    return 0;
}