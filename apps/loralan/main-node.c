/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/**
 * @defgroup
 * @ingroup
 * @brief
 * @{
 * @file        main-node.c
 * @brief       LoRaLAN node device
 * @author      Evgeniy Ponomarev
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "thread.h"
#include "periph/pm.h"
#include "pm_layered.h"
#include "periph/rtc.h"
#include "periph/gpio.h"
#include "random.h"

#include "net/lora.h"
#include "net/netdev.h"
#include "sx127x_internal.h"
#include "sx127x_params.h"
#include "sx127x_netdev.h"

#include "board.h"

#include "unwds-common.h"
#include "unwds-gpio.h"
#include "ls-settings.h"
#include "ls-end-device.h"
#include "ls-init-device.h"
#include "ls-config.h"

#include "main.h"
#include "utils.h"

#include "ls-regions.h"
#include "lptimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

static sx127x_t sx127x;
static netdev_t *netdev;

static ls_ed_t ls;

static uint8_t current_join_retries = 0;

void radio_init(void)
{
    sx127x_params_t sx127x_params;
    
    sx127x_params.nss_pin = SX127X_SPI_NSS;
    sx127x_params.spi = SX127X_SPI;

    sx127x_params.dio0_pin = SX127X_DIO0;
    sx127x_params.dio1_pin = SX127X_DIO1;
    sx127x_params.dio2_pin = SX127X_DIO2;
    sx127x_params.dio3_pin = SX127X_DIO3;
    sx127x_params.dio4_pin = SX127X_DIO4;
    sx127x_params.dio5_pin = SX127X_DIO5;
    sx127x_params.reset_pin = SX127X_RESET;
   
    sx127x_params.rfswitch_pin = SX127X_RFSWITCH;
    sx127x_params.rfswitch_active_level = SX127X_GET_RFSWITCH_ACTIVE_LEVEL();

    sx127x_radio_settings_t settings;
    settings.channel = RF_FREQUENCY;
    settings.modem = SX127X_MODEM_LORA;
    settings.state = SX127X_RF_IDLE;

    sx127x.settings = settings;
    memcpy(&sx127x.params, &sx127x_params, sizeof(sx127x_params));
    
    netdev = (netdev_t*)&sx127x;
    netdev->driver = &sx127x_driver;

    puts("init_radio: sx127x initialization done");
}

static void node_join(ls_ed_t *ls) {
    /* limit max delay between attempts to 1 hour */
    if (current_join_retries < 120) {
        current_join_retries++;
    }
    ls_ed_join(ls);
}
void joined_timeout_cb(void)
{
	if (unwds_get_node_settings().no_join)
		return;

    if ((current_join_retries >= unwds_get_node_settings().max_retr + 1) && (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A)) {
        /* class A node: go to sleep */
        puts("[LoRa] maximum join retries exceeded, stopping");
        current_join_retries = 0;
    } else {
        puts("ls: join request timed out, resending");
        
        /* Pseudorandom delay for collision avoidance */
        unsigned int delay = random_uint32_range(5000 + (current_join_retries - 1)*30000, 30000 + (current_join_retries - 1)*30000);
        printf("[LoRa] random delay %d ms\n", (unsigned int) (delay));
        
        lptimer_sleep(delay);
        
        /* limit max delay between attempts to 1 hour */
        if (current_join_retries < 120) {
            current_join_retries++;
        }
        
        if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A) {
            printf("[LoRa] joining, attempt %d / %d\n", current_join_retries, unwds_get_node_settings().max_retr);
        } else {
            puts("[LoRa] joining");
        }
        
        node_join(&ls);
    }
}

void joined_cb(void)
{
	current_join_retries = 0;

    puts("[LoRa] successfully joined to the network");
    blink_led(LED0_PIN);
    
    /* Synchronize time if necessary */
    /*
    if (unwds_get_node_settings().req_time) {
        ls_ed_req_time(&ls);
    }
    */
}

static void time_req_ack_cb(time_t time) {
    puts("ls: received time");
	struct tm *t;
	t = localtime(&time);

	printf("ls: received new time from the gate: %04d-%02d-%02d %02d:%02d:%02d\n", t->tm_year + 1900, t->tm_mon + 1,
			t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

	/* Setup new time as system time for all the timers */
	/* rtctimers_millis_set_timebase(t); */
}

void appdata_send_failed_cb(void)
{
	if (!unwds_get_node_settings().no_join) {
		puts("[LoRa] rejoining");
//		joined_timeout_cb();
        node_join(&ls);
	} else
		puts("[LoRa] failed to send confirmed application data");
}

static bool appdata_received_cb(uint8_t *buf, size_t buflen)
{
    char hex[100] = {};

    bytes_to_hex(buf, buflen, hex, false);

    printf("[LoRa] received data: \"%s\"\n", hex);
    blink_led(LED0_PIN);

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

    /* Send app. data */
    int result = unwds_send_to_module(modid, &cmd, &reply);
    
    if (result == UNWDS_MODULE_NOT_FOUND) {
        /* No module with specified ID present */
        reply.as_ack = true;
        reply.length = 2;
        reply.data[0] = UNWDS_MODULE_NOT_FOUND;
        reply.data[1] = modid;
    }
    
    if (result != UNWDS_MODULE_NO_DATA) {
        int res = ls_ed_send_app_data(&ls, reply.data, reply.length, true, true, false);
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

    printf("[LoRa] received broadcast data: \"%s\"\n", hex);
    blink_led(LED0_PIN);

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
    	memcpy(ls->settings.crypto.join_key, config_get_appkey(), LS_MIC_KEY_LEN);
    }
    ls->join_timeout_cb = joined_timeout_cb;
    ls->joined_cb = joined_cb;
    
    ls->settings.confirmation = unwds_get_node_settings().confirmation;

    ls->appdata_send_failed_cb = appdata_send_failed_cb;
    ls->settings.max_retr = unwds_get_node_settings().max_retr;     /* Maximum number of confirmed data retransmissions */

    ls->appdata_received_cb = appdata_received_cb;
    ls->broadcast_appdata_received_cb = broadcast_appdata_received_cb;
    
    ls->time_req_ack_cb = time_req_ack_cb;

    ls->_internal.device = netdev;
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
        puts("\time -- request network time");
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
        unwds_set_region(v);
    }
    else if (strcmp(key, "maxretr") == 0) {
        uint8_t v = strtol(value, NULL, 10);
        ls.settings.max_retr = v;
    }
    else if (strcmp(key, "class") == 0) {
        char v = value[0];

        if (v != 'A' && v != 'B' && v != 'C') {
            puts("set class: A, B or C");
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
    else if (strcmp(key, "nojoin") == 0) {
    	char v = value[0];

    	ls.settings.req_time = (v == '1');
    }
    else if (strcmp(key, "addr") == 0) {
        if (strlen(value) != 8) {
            puts("[error] There must be 8 hexadecimal digits in lower case");
            return 1;
        }

        uint32_t addr = 0;

        if (!hex_to_bytesn(value, 8, (uint8_t *) &addr, true)) {
            puts("[error] Invalid hex number!");
            return 1;
        }

        ls._internal.dev_addr = addr;
    }

    unwds_set_nojoin(ls.settings.no_join);
    if (unwds_get_node_settings().no_join) {
    	unwds_set_addr(ls._internal.dev_addr);
    }

    unwds_set_channel(ls.settings.channel);
    unwds_set_dr(ls.settings.dr);
    unwds_set_max_retr(ls.settings.max_retr);
    unwds_set_class(ls.settings.class);

    return 0;
}

static void print_regions(void)
{
    puts("[ available regions ]");

    uint32_t i;
    for (i = 0; i < LS_UNI_NUM_REGIONS; i++) {
        printf("%" PRIu32 ". %s [", i, regions[i].region);
        uint32_t j;
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
        uint8_t *key = config_get_appkey();
        printf("JOINKEY = 0x....%01X%01X\n", key[14], key[15]);
    }

    if (unwds_get_node_settings().no_join && DISPLAY_DEVNONCE_BYTE) {
    	uint8_t devnonce = config_get_devnonce();
    	printf("DEVNONCE = 0x...%01X\n", devnonce & 0x0F);
    }

    if (unwds_get_node_settings().no_join) {
    	printf("ADDR = 0x%08X\n", (unsigned int) unwds_get_node_settings().dev_addr);
    }

    printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    printf("APPID64 = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));

    printf("REGION = %s\n", regions[unwds_get_node_settings().region_index].region);
    printf("CHANNEL = %d [%d]\n", unwds_get_node_settings().channel, (unsigned) regions[unwds_get_node_settings().region_index].channels[unwds_get_node_settings().channel]);

    printf("DATARATE = %d\n", unwds_get_node_settings().dr);
    
    printf("CONFIRMED = %s\n", (unwds_get_node_settings().confirmation) ? "yes" : "no");
    
    printf("TIME SYNC = %s\n", (unwds_get_node_settings().req_time) ? "yes" : "no");

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
    (void)argc;
    (void)argv;
    
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
        printf("cmd: module with ID %d does not exists\n", modid);
        return 1;
    }

    module_data_t cmd = {};

    int len = strlen(argv[2]);
    if (len % 2 != 0) {
        puts("cmd: invalid hex number");
        return 1;
    }

    if (len / 2 > UNWDS_MAX_DATA_LEN) {
        printf("cmd: command too long. Maximum is %d bytes\n", UNWDS_MAX_DATA_LEN);
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
    (void)argc;
    (void)argv;
    
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

    int modid = 0;
    
    if (is_number(argv[1])) {
        modid = atoi(argv[1]);
    } else {
        modid = unwds_modid_by_name(argv[1]);
    }
    
    if (modid < 0) {
        printf("mod: module %s does not exist\n", argv[1]);
        return 1;
    }
    
    if (!unwds_is_module_exists(modid)) {
        printf("mod: module with ID %d does not exist\n", modid);
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

static int print_regions_cmd(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    print_regions();

    return 0;
}

static int ls_safe_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;

#if defined RTC_REGBACKUP_BOOTMODE
    uint32_t bootmode = UNWDS_BOOT_SAFE_MODE;
    rtc_save_backup(bootmode, RTC_REGBACKUP_BOOTMODE);
#endif
    puts("Rebooting in safe mode");
    NVIC_SystemReset();
    return 0;
}

static int ls_join_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    
    node_join(&ls);
    return 0;
}

static int ls_get_time(int argc, char **argv) {
    (void)argc;
    (void)argv;
    
    ls_ed_req_time(&ls);
    return 0;
}

shell_command_t shell_commands[UNWDS_SHELL_COMMANDS_MAX] = {
    { "set", "<config> <value> -- set value for the configuration entry", ls_set_cmd },
    { "lscfg", "-- print out current configuration", ls_printc_cmd },
    { "lsregion", "-- list available regions", print_regions_cmd },
    { "lsmod", "-- list available modules", ls_listmodules_cmd },
    { "mod", "<name> <enable|disable>	-- disable or enable selected module", ls_module_cmd },
    { "cmd", "<modid> <cmdhex> -- send command to another UNWDS device", ls_cmd_cmd },
    { "safe", " -- reboot in safe mode", ls_safe_cmd },
    { "join", " -- join now", ls_join_cmd },
    { "time", " -- get network time", ls_get_time },
    { NULL, NULL, NULL },
};

static void unwds_callback(module_data_t *buf)
{
    int res = ls_ed_send_app_data(&ls, buf->data, buf->length, true, buf->as_ack, false);

    if (res < 0) {
        if (res == -LS_SEND_E_FQ_OVERFLOW) {
            puts("[error] Uplink queue overflowed!");
        }
        if (res == -LS_SEND_E_NOT_JOINED) {
            puts("[error] Cannot send app. data: not joined to the network");

            /* Try to join to the network */
            if (current_join_retries == 0) {
                puts("[info] Attempting to rejoin");
                node_join(&ls);
            } else {
                puts("[info] Waiting for the node to join");
            }
        }
        else {
            printf("send: error #%d\n", res);
        }
    }

    blink_led(LED0_PIN);
}

static int unwds_init(void) {
    radio_init();
    ls_setup(&ls);
    
    if (ls_ed_init(&ls) != LS_OK) {
        return -1;
    }
    
    return 0;
}

static void unwds_join(void) {
    node_join(&ls);
}

static void unwds_sleep(void) {
    /* put SX127x to sleep */
    ls_ed_sleep(&ls);
}

void init_normal(shell_command_t *commands)
{
    bool cfg_valid = unwds_config_load();
    print_config();
    
    if (!cfg_valid) {
        puts("[!] Device is not configured yet. Type \"help\" to see list of possible configuration commands.");
        puts("[!] Configure the node and type \"reboot\" to reboot and apply settings.");
    } else {
        unwds_device_init(unwds_callback, unwds_init, unwds_join, unwds_sleep);
    }

    /* Add our commands to shell */
    int i = 0;
    do {
        i++;
    } while (commands[i].name);
    
    int k = 0;
    do {
        k++;
    } while (shell_commands[k].name);
    
    assert(i + k < UNWDS_SHELL_COMMANDS_MAX - 1);
    
    memcpy((void *)&commands[i], (void *)shell_commands, k*sizeof(shell_commands[i]));
}

#ifdef __cplusplus
}
#endif
