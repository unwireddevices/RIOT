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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "lpm.h"
#include "arch/lpm_arch.h"
#include "thread.h"
#include "periph/rtc.h"
#include "periph/gpio.h"
#include "random.h"

#include "sx1276.h"
#include "board.h"

#include "ls-end-device.h"
#include "unwds-common.h"
#include "ls-settings.h"
#include "unwds-gpio.h"
#include "ls-config.h"

#include "main.h"
#include "utils.h"

#include "ls-regions.h"
#include "periph/wdg.h"
#include "rtctimers.h"

#define DISPLAY_JOINKEY_2BYTES 1
#define DISPLAY_DEVNONCE_BYTE 1

#define ENABLE_DEBUG (0)
#include "debug.h"

static rtctimer_t iwdg_timer;

static sx1276_t sx1276;
static ls_ed_t ls;

static uint8_t current_join_retries = 0;

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
    settings.modem = SX1276_MODEM_LORA;
    settings.state = SX1276_RF_IDLE;

    sx1276.settings = settings;

    puts("init_radio: sx1276 initialization done");
}

void joined_timeout_cb(void)
{
	if (unwds_get_node_settings().no_join)
		return;

    if ((current_join_retries >= unwds_get_node_settings().max_retr) && (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A)) {
        /* class A node: go to sleep */
        puts("ls-ed: maximum join retries exceeded, stopping");
    } else {
        puts("ls: join request timed out, resending");
        
        /* Pseudorandom delay for collision avoidance */
        unsigned int delay = random_uint32_range(5 + current_join_retries*30, 30 + current_join_retries*30);
        printf("ls-ed: random delay %d s\n", (unsigned int) (delay));
        
        rtctimers_sleep(delay);
        
        /* limit max delay between attempts to 1 hour */
        if (current_join_retries < 120) {
            current_join_retries++;
        }
        
        if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A) {
            printf("ls-ed: rejoining, attempt %d / %d\n", current_join_retries, unwds_get_node_settings().max_retr);
        } else {
            puts("ls-ed: rejoining");
        }
        
        ls_ed_join(&ls);
    }
}

void joined_cb(void)
{
	current_join_retries = 0;

    puts("ls-ed: successfully joined to the network");
    blink_led();
}

void appdata_send_failed_cb(void)
{
	if (!unwds_get_node_settings().no_join) {
		puts("ls-ed: application data confirmation timeout. Rejoining...");
//		joined_timeout_cb();
        ls_ed_join(&ls);
	} else
		puts("ls-ed: failed to send confirmed application data");
}

static bool appdata_received_cb(uint8_t *buf, size_t buflen)
{
    char hex[100] = {};

    bytes_to_hex(buf, buflen, hex, false);

    printf("ls-ed: received data: \"%s\"\n", hex);
    blink_led();

    if (buflen < 2) {
        return true;
    }

    unwds_module_id_t modid = buf[0];

    module_data_t cmd;
    /* Save command data */
    memcpy(cmd.data, buf + 1, buflen - 1);
    cmd.length = buflen - 1;

    /* Save RSSI value */
    cmd.rssi = ls._internal.last_rssi;

    /* Send command to the module */
    module_data_t reply = {};
    bool has_app_data = unwds_send_to_module(modid, &cmd, &reply);

    /* Send app. data */
    if (has_app_data) {
		int res = ls_ed_send_app_data(&ls, reply.data, reply.length, true, true);
		if (res < 0) {
			printf("send: error #%d\n", res);
		}
    }

    /* Don't allow to send app. data ACK by the network.
     * The ACK will be sent either by the callback with the actual app. data or
     * with the command response itself */
    return false;
}

static bool broadcast_appdata_received_cb(uint8_t *buf, size_t buflen) {
    char hex[100] = {};

    bytes_to_hex(buf, buflen, hex, false);

    printf("ls-ed: received broadcast data: \"%s\"\n", hex);
    blink_led();

    if (buflen < 2) {
        return true;
    }

    unwds_module_id_t modid = buf[0];

    module_data_t cmd;
    /* Save command data */
    memcpy(cmd.data, buf + 1, buflen - 1);
    cmd.length = buflen - 1;

    /* Save RSSI value */
    cmd.rssi = ls._internal.last_rssi;

    unwds_send_broadcast(modid, &cmd, NULL);

    return false;
}

static void ls_setup(ls_ed_t *ls)
{
    ls->settings.class = unwds_get_node_settings().nodeclass;

    ls->settings.dr = unwds_get_node_settings().dr;
    ls->settings.channel = unwds_get_node_settings().channel;

    ls->settings.channels_table = regions[unwds_get_node_settings().region_index].channels;
    ls->settings.channels_table_size = regions[unwds_get_node_settings().region_index].num_channels;

    ls->settings.app_id = config_get_appid();
    ls->settings.node_id = config_get_nodeid();

    ls->settings.no_join = unwds_get_node_settings().no_join;
    if (unwds_get_node_settings().no_join) {
    	ls_derive_keys(config_get_devnonce(), 0, unwds_get_node_settings().dev_addr, ls->settings.crypto.mic_key, ls->settings.crypto.aes_key);
    	ls->_internal.dev_addr = unwds_get_node_settings().dev_addr;
    } else {
    	memcpy(ls->settings.crypto.join_key, config_get_joinkey(), LS_MIC_KEY_LEN);
    }
    ls->join_timeout_cb = joined_timeout_cb;
    ls->joined_cb = joined_cb;

    ls->appdata_send_failed_cb = appdata_send_failed_cb;
    ls->settings.max_retr = unwds_get_node_settings().max_retr;     /* Maximum number of confirmed data retransmissions */

    ls->appdata_received_cb = appdata_received_cb;
    ls->broadcast_appdata_received_cb = broadcast_appdata_received_cb;

    ls->_internal.sx1276 = &sx1276;
}

int ls_set_cmd(int argc, char **argv)
{
    if (argc != 3) {
        puts("usage: set <key> <value>");
        puts("keys:");
        if (unwds_get_node_settings().no_join)
        	puts("\taddr <address> -- sets predefined device address for statically personalized devices");

        puts("\tnojoin <0/1> -- selecting wether device is statically personalized or not");
        printf("\tregion <0-%d> -- sets channels region\n", LS_UNI_NUM_REGIONS - 1);
        puts("\tch <ch> -- sets device channel in selected region");
        puts("\tdr <0-6> -- sets device data rate [0 - slowest, 3 - average, 6 - fastest]");
        puts("\tmaxretr <0-255> -- sets maximum number of retransmissions of confirmed app. data [5 is recommended]");
        puts("\tclass <A/B/C> -- sets device class");
    }

    char *key = argv[1];
    char *value = argv[2];

    if (strcmp(key, "dr") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (v > 6) {
            puts("set dr: datarate value must be from 0 to 6");
            return 1;
        }

        ls.settings.dr = (ls_datarate_t) v;
    }
    else if (strcmp(key, "ch") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (v > regions[unwds_get_node_settings().region_index].num_channels - 1) {
            printf("set ch: channel value must be from 0 to %d\n", regions[unwds_get_node_settings().region_index].num_channels - 1);
            return 1;
        }

        ls.settings.channel = (ls_channel_t) v;
    }
    else if (strcmp(key, "region") == 0) {
        uint8_t v = strtol(value, NULL, 10);

        if (v > LS_UNI_NUM_REGIONS) {
            printf("set region: region value must be from 0 to %d\n", LS_UNI_NUM_REGIONS - 1);
            return 1;
        }
        unwds_set_region(v, false);
    }
    else if (strcmp(key, "maxretr") == 0) {
        uint8_t v = strtol(value, NULL, 10);
        ls.settings.max_retr = v;
    }
    else if (strcmp(key, "class") == 0) {
        char v = value[0];

        if (v != 'A' && v != 'B' && v != 'C') {
            puts("set сlass: A, B or C");
            return 1;
        }

        if (v == 'A') {
            ls.settings.class = LS_ED_CLASS_A;
        }
        else if (v == 'B') {
            ls.settings.class = LS_ED_CLASS_B;
        }
        else if (v == 'C') {
            ls.settings.class = LS_ED_CLASS_C;
        }
    }
    else if (strcmp(key, "nojoin") == 0) {
    	char v = value[0];

    	ls.settings.no_join = (v == '1');
    }
    else if (strcmp(key, "addr") == 0) {
        if (strlen(value) != 8) {
            puts("[error] There must be 8 hexadecimal digits in lower case");
            return 1;
        }

        uint32_t addr = 0;

        if (!hex_to_bytesn(value, 8, (uint8_t *) &addr, true)) {
            puts("[error] Pardon me, but that's not a hex number!");
            return 1;
        }

        ls._internal.dev_addr = addr;
    }

    unwds_set_nojoin(ls.settings.no_join);
    if (unwds_get_node_settings().no_join)
    	unwds_set_addr(ls._internal.dev_addr);

    unwds_set_channel(ls.settings.channel);
    unwds_set_dr(ls.settings.dr);
    unwds_set_max_retr(ls.settings.max_retr);
    unwds_set_class(ls.settings.class);

    return 0;
}

static void print_regions(void)
{
    puts("[ available regions ]");

    int i;
    for (i = 0; i < LS_UNI_NUM_REGIONS; i++) {
        printf("%d. %s [", i, regions[i].region);
        int j;
        for (j = 0; j < regions[i].num_channels; j++) {
            printf("%d", (unsigned) regions[i].channels[j]);

            if (j + 1 != regions[i].num_channels) {
                printf(", ");
            }
        }

        puts("]");
    }
}

static void print_config(void)
{
    puts("[ node configuration ]");

    uint64_t eui64 = config_get_nodeid();
    uint64_t appid = config_get_appid();

    printf("NOJOIN = %s\n", (unwds_get_node_settings().no_join) ? "yes" : "no");

    if (!unwds_get_node_settings().no_join && DISPLAY_JOINKEY_2BYTES) {
        uint8_t *key = config_get_joinkey();
        printf("JOINKEY = 0x....%01X%01X\n", key[14], key[15]);
    }

    if (unwds_get_node_settings().no_join && DISPLAY_DEVNONCE_BYTE) {
    	uint8_t devnonce = config_get_devnonce();
    	printf("DEVNONCE = 0x...%01X\n", devnonce & 0x0F);
    }

    if (unwds_get_node_settings().no_join)
    	printf("ADDR = 0x%08X\n", (unsigned int) unwds_get_node_settings().dev_addr);

    printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    printf("APPID64 = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));

    printf("REGION = %s\n", regions[unwds_get_node_settings().region_index].region);
    printf("CHANNEL = %d [%d]\n", unwds_get_node_settings().channel, (unsigned) regions[unwds_get_node_settings().region_index].channels[unwds_get_node_settings().channel]);

    printf("DATARATE = %d\n", unwds_get_node_settings().dr);

    char nodeclass = 'A'; // unwds_get_node_settings().nodeclass == LS_ED_CLASS_A
    if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_B) {
        nodeclass = 'B';
    }
    else if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_C) {
        nodeclass = 'C';
    }
    printf("CLASS = %c\n", nodeclass);

    printf("MAXRETR = %d\n", unwds_get_node_settings().max_retr);

    puts("[ enabled modules ]");
    unwds_list_modules(unwds_get_node_settings().enabled_mods, true);
}

static int ls_printc_cmd(int argc, char **argv)
{
    print_config();

    return 0;
}

int ls_cmd_cmd(int argc, char **argv)
{
    if (argc < 2) {
        puts("Usage: cmd <modid> <cmdhex>");
        return 1;
    }

    uint8_t modid = atoi(argv[1]);
    if (!unwds_is_module_exists(modid)) {
        puts("cmd: module with specified id is not exists");
        return 1;
    }

    module_data_t cmd = {};

    int len = strlen(argv[2]);
    if (len % 2 != 0) {
        puts("cmd: command in hex must have an even length. Example: a0b100cc (4 bytes)");
        return 1;
    }

    if (len / 2 > UNWDS_MAX_DATA_LEN) {
        printf("cmd: command in hex is too long. Maximum is %d bytes long\n", UNWDS_MAX_DATA_LEN);
        return 1;
    }

    hex_to_bytes(argv[2], cmd.data, false);
    cmd.length = len / 2;

    /* No RSSI from console commands */
    cmd.rssi = 0;

    module_data_t reply = {};
    bool res = unwds_send_to_module(modid, &cmd, &reply);
    char replystr[2 * UNWDS_MAX_DATA_LEN] = {};
    bytes_to_hex(reply.data, reply.length, replystr, false);

    if (res) {
        printf("[ok] Reply: %s\n", replystr);
    }
    else {
        printf("[fail] Reply: %s\n", replystr);
    }

    return 0;
}

static int ls_listmodules_cmd(int argc, char **argv)
{
    puts("[ available modules ]");
    unwds_list_modules(unwds_get_node_settings().enabled_mods, false);

    return 0;
}

static int ls_module_cmd(int argc, char **argv)
{
    if (argc < 3) {
        puts("Usage: mod <name> <enable|disable>. Example: mod adc enable");
        return 1;
    }

    uint8_t modid = 0;
    
    if (is_number(argv[1])) {
        modid = atoi(argv[1]);
    } else {
        modid = unwds_modid_by_name(argv[1]);
    }
    
    if (modid < 0) {
        puts("mod: module with specified name doesn't exist");
        return 1;
    }
    
    if (!unwds_is_module_exists(modid)) {
        puts("mod: module with specified id doesn't exist");
        return 1;
    }

    bool modenable = false;
    if (is_number(argv[2])) {
        modenable = atoi(argv[2]);
    } else {
        if (strcmp(argv[2], "enable") == 0) {
            modenable = true;
        } else {
            if (strcmp(argv[2], "disable") != 0) {
                printf("mod: unknown command: %s\n", argv[2]);
                return 1;
            }
        }
    }
    
    unwds_set_module(modid, modenable);
        
    return 0;
}

static int ls_clear_nvram(int argc, char **argv)
{
    if (argc < 2) {
        puts("Usage: clear <all|key|modules> -- clear all NVRAM contents or just the security key.");
        return 1;
    }

    char *key = argv[1];

    if (strcmp(key, "all") == 0) {
        puts("Please wait a minute while I'm cleaning up here...");
        if (clear_nvram()) {
            puts("[ok] System settings cleared, let me reboot this device now");
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

static int print_regions_cmd(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    print_regions();

    return 0;
}

static int ls_save_cmd(int argc, char **argv) {
    (void) argc;
    (void) argv;
    
    puts("[*] Saving configuration...");
    
    if (!unwds_config_save()) {
        puts("[error] Unable to save configuration");
    }

    puts("[done] Configuration saved. Type \"reboot\" to apply changes.");
    
    return 0;
}

shell_command_t shell_commands[UNWDS_SHELL_COMMANDS_MAX] = {
    { "set", "<config> <value> -- set value for the configuration entry", ls_set_cmd },

    { "lscfg", "-- print out current configuration", ls_printc_cmd },

    { "lsregion", "-- list available regions", print_regions_cmd },

    { "lsmod", "-- list available modules", ls_listmodules_cmd },

    { "mod", "<name> <enable|disable>	-- disable or enable selected module", ls_module_cmd },

    { "save", "-- saves current configuration", ls_save_cmd },

    { "clear", "<all|key|modules> -- clear settings stored in NVRAM", ls_clear_nvram },

    { "cmd", "<modid> <cmdhex> -- send command to another UNWDS devices", ls_cmd_cmd },

    { NULL, NULL, NULL }
};

static void unwds_callback(module_data_t *buf)
{
    int res = ls_ed_send_app_data(&ls, buf->data, buf->length, true, buf->as_ack);

    if (res < 0) {
        if (res == -LS_SEND_E_FQ_OVERFLOW) {
            puts("[error] Uplink queue overflowed!");
        }
        if (res == -LS_SEND_E_NOT_JOINED) {
            puts("[error] Cannot send app. data: not joined to the network");

            /* Try to join to the network */
            current_join_retries = 0;
            ls_ed_join(&ls);
        }
        else {
            printf("send: error #%d\n", res);
        }
    }

    blink_led();
}

static bool is_connect_button_pressed(void)
{
	if (!UNWD_USE_CONNECT_BTN) {
		return false;
	}
	
    if (!gpio_init(UNWD_CONNECT_BTN, GPIO_IN_PU)) {
        if (!gpio_read(UNWD_CONNECT_BTN)) {
            return true;
        }
    }
    else {
        puts("Error initializing Connect button\n");
    }

    return false;
}

static void iwdg_reset (void *arg) {
    wdg_reload();
    rtctimers_set(&iwdg_timer, 15);
    DEBUG("Watchdog reset\n");
    return;
}

void init_node(shell_command_t **commands)
{
    puts("[node] Initializing...");

    /* fill the rest of shell_commands array with NULLs */
    int i = 0;
    bool fillzeros = false;
    for (i = 0; i < UNWDS_SHELL_COMMANDS_MAX; i++) {
        if (shell_commands[i].name == NULL) {
            fillzeros = true;
        }
        if (fillzeros) {
            shell_commands[i].name = NULL;
            shell_commands[i].desc = NULL;
            shell_commands[i].handler = NULL;
        }
    }

    rtctimers_init();

    if (!unwds_config_load()) {
        puts("[!] Device is not configured yet. Type \"help\" to see list of possible configuration commands.");
        puts("[!] Configure the node and type \"reboot\" to reboot and apply settings.");

        print_config();
    }
    else {
        print_config();

        radio_init();
        ls_setup(&ls);
        ls_ed_init(&ls);

        unwds_set_enabled(unwds_get_node_settings().enabled_mods);
        memcpy(ls.settings.ability, unwds_get_node_settings().enabled_mods, sizeof(ls.settings.ability));
        ls.settings.class = unwds_get_node_settings().nodeclass;

        unwds_setup_nvram_config(config_get_nvram(), UNWDS_CONFIG_BASE_ADDR, UNWDS_CONFIG_BLOCK_SIZE_BYTES);

        if (is_connect_button_pressed()) {
            puts("[!] Entering Safe Mode, all modules disabled, class C.");
            blink_led();
            blink_led();
            blink_led();
        }
        else {
            unwds_init_modules(unwds_callback);
            
            /* reset IWDG timer every 15 seconds */
            /* NB: unwired-module MUST NOT need more than 3 seconds to finish its job */
            
            iwdg_timer.callback = iwdg_reset;
            rtctimers_set(&iwdg_timer, 15);
            
            /* IWDG period is 18 seconds minimum, 28 seconds typical */
            wdg_set_prescaler(6);
            wdg_set_reload((uint16_t) 0x0FFF);
            wdg_reload();
            wdg_enable();
            
            /* allow CPU frequency switching */
            lpm_prevent_switch = 0;
            
            /* enable sleep for Class A devices only */        
            if (ls.settings.class == LS_ED_CLASS_A) {
                lpm_prevent_sleep = 0;
            }
            
            rtctimers_sleep(1);
            blink_led();
        }

        if (!unwds_get_node_settings().no_join) {
        	ls_ed_join(&ls);
        }
    }
    /* Set our commands for shell */
    memcpy(commands, shell_commands, sizeof(shell_commands));
}

#ifdef __cplusplus
}
#endif
