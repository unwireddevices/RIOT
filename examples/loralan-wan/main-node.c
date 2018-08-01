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

#include "ls-settings.h"
#include "ls-config.h"

#include "board.h"

#include "unwds-common.h"
#include "unwds-gpio.h"

#include "main.h"
#include "utils.h"

#include "periph/wdg.h"
#include "rtctimers-millis.h"

#include "net/loramac.h"
#include "semtech_loramac.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

static rtctimers_millis_t iwdg_timer;
static rtctimers_millis_t pm_enable_timer;

static msg_t msg_join;
static kernel_pid_t sender_pid;
static rtctimers_millis_t send_retry_timer;

static kernel_pid_t receiver_pid;

static sx127x_t sx127x;
static semtech_loramac_t ls;

static uint8_t current_join_retries = 0;

static bool appdata_received(uint8_t *buf, size_t buflen);

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
    
    semtech_loramac_init(&ls, &sx127x_params);

    puts("init_radio: loramac initialization done");
}

static int node_join(semtech_loramac_t *ls) {
    /* limit max delay between attempts to 1 hour */
    if (current_join_retries < 120) {
        current_join_retries++;
    }
    
    if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A) {
        printf("[LoRa] joining, attempt %d / %d\n", current_join_retries + 1, unwds_get_node_settings().max_retr + 1);
    } else {
        puts("[LoRa] joining");
    }
    
    return (semtech_loramac_join(ls, LORAMAC_JOIN_OTAA));
}

static void *sender_thread(void *arg) {
    semtech_loramac_t *ls = (semtech_loramac_t *) arg;
    
    msg_t msg;
    
    while (1) {
        msg_receive(&msg);
        
        int res = node_join(ls);
        
        switch (res) {
        case SEMTECH_LORAMAC_JOIN_SUCCEEDED: {
            current_join_retries = 0;
            puts("[LoRa] successfully joined to the network");
            blink_led(LED_GREEN);
            break;
        }
        case SEMTECH_LORAMAC_JOIN_FAILED:
        case SEMTECH_LORAMAC_BUSY: {
            if ((current_join_retries >= unwds_get_node_settings().max_retr + 1) &&
                (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A)) {
                /* class A node: go to sleep */
                puts("[LoRa] maximum join retries exceeded, stopping");
                current_join_retries = 0;
            } else {
                puts("ls: join request timed out, resending");
                
                /* Pseudorandom delay for collision avoidance */
                unsigned int delay = random_uint32_range(5000 + (current_join_retries - 1)*30000, 30000 + (current_join_retries - 1)*30000);
                printf("[LoRa] random delay %d ms\n", (unsigned int) (delay));
                
                /* limit max delay between attempts to 1 hour */
                if (current_join_retries < 120) {
                    current_join_retries++;
                }
                
                rtctimers_millis_set_msg(&send_retry_timer, delay, &msg_join, sender_pid);
      
                break;
            }
        }
        default:
            break;
    }
    }
    return NULL;
}

static void *receiver_thread(void *arg) {
    semtech_loramac_t *ls = (semtech_loramac_t *) arg;
    
    while (1) {
        int res = semtech_loramac_recv(ls);
        
        switch (res) {
            case SEMTECH_LORAMAC_TX_DONE:
                puts("[LoRa] TX completed, no data received");
                break;
            case SEMTECH_LORAMAC_DATA_RECEIVED:
                appdata_received(ls->rx_data.payload, ls->rx_data.payload_len);
                break;
            case SEMTECH_LORAMAC_TX_CNF_FAILED:
                puts("[LoRa] uplink message ACK failed");
                break;
            default:
                puts("[LoRa] unknown response");
                break;
        }
    }
    
    return NULL;
}

/*
void appdata_send_failed_cb(void)
{
	if (!unwds_get_node_settings().no_join) {
		puts("[LoRa] rejoining");
        msg_send(&msg_join, sender_pid);
	} else
		puts("[LoRa] failed to send confirmed application data");
}
*/

static bool appdata_received(uint8_t *buf, size_t buflen)
{
    char hex[100] = {};

    bytes_to_hex(buf, buflen, hex, false);

    printf("[LoRa] received data: \"%s\"\n", hex);
    blink_led(LED_GREEN);

    if (buflen < 2) {
        return true;
    }

    unwds_module_id_t modid = buf[0];

    module_data_t cmd;
    /* Save command data */
    memcpy(cmd.data, buf + 1, buflen - 1);
    cmd.length = buflen - 1;

    /* Save RSSI value */
    /* cmd.rssi = ls._internal.last_rssi; */

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
/*
        int res = ls_ed_send_app_data(&ls, reply.data, reply.length, true, true, false);
        if (res < 0) {
            printf("send: error #%d\n", res);
        }
*/
    }

    /* Don't allow to send app. data ACK by the network.
     * The ACK will be sent either by the callback with the actual app. data or
     * with the command response itself */
    return false;
}

static void ls_setup(semtech_loramac_t *ls)
{
    // ls->settings.max_retr = unwds_get_node_settings().max_retr;     /* Maximum number of confirmed data retransmissions */

    uint64_t id = config_get_nodeid();
    semtech_loramac_set_deveui(ls, (uint8_t *)&id);
    
    id = config_get_appid();
    semtech_loramac_set_appeui(ls, (uint8_t *)&id);
    
    semtech_loramac_set_appkey(ls, config_get_joinkey());
    
    semtech_loramac_set_dr(ls, LORAMAC_DR_0);
    semtech_loramac_set_class(ls, unwds_get_node_settings().nodeclass);
    
    char sender_stack[2048];
    sender_pid = thread_create(sender_stack, sizeof(sender_stack), THREAD_PRIORITY_MAIN - 2,
                               THREAD_CREATE_STACKTEST, sender_thread, ls,  "LoRa sender thread");
        
    char receiver_stack[2048];
    receiver_pid = thread_create(receiver_stack, sizeof(receiver_stack), THREAD_PRIORITY_MAIN - 2,
                               THREAD_CREATE_STACKTEST, receiver_thread, ls,  "LoRa receiver thread");
}

int ls_set_cmd(int argc, char **argv)
{
    if (argc != 3) {
        puts("usage: set <key> <value>");
        puts("keys:");
        if (unwds_get_node_settings().no_join)
        	puts("\taddr <address> -- sets predefined device address for statically personalized devices");

        puts("\tnojoin <0/1> -- selecting wether device is statically personalized or not");
        puts("\tch <ch> -- sets device channel in selected region");
        puts("\tdr <0-6> -- sets device data rate [0 - slowest, 3 - average, 6 - fastest]");
        puts("\tmaxretr <0-255> -- sets maximum number of retransmissions of confirmed app. data [5 is recommended]");
        puts("\tclass <A/B/C> -- sets device class");
    }

    (void)argv;

    return 0;
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

    if (unwds_get_node_settings().no_join) {
    	printf("ADDR = 0x%08X\n", (unsigned int) unwds_get_node_settings().dev_addr);
    }

    printf("EUI64 = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    printf("APPID64 = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));

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

static int ls_safe_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;

    uint32_t bootmode = UNWDS_BOOT_SAFE_MODE;
    rtc_save_backup(bootmode, RTC_REGBACKUP_BOOTMODE);
    puts("Rebooting in safe mode");
    NVIC_SystemReset();
    return 0;
}

static int ls_join_cmd(int argc, char **argv) {
    (void)argc;
    (void)argv;
    
    msg_send(&msg_join, sender_pid);
    return 0;
}

shell_command_t shell_commands[UNWDS_SHELL_COMMANDS_MAX] = {
    { "set", "<config> <value> -- set value for the configuration entry", ls_set_cmd },
    { "lscfg", "-- print out current configuration", ls_printc_cmd },
    { "lsmod", "-- list available modules", ls_listmodules_cmd },
    { "mod", "<name> <enable|disable>	-- disable or enable selected module", ls_module_cmd },
    { "cmd", "<modid> <cmdhex> -- send command to another UNWDS device", ls_cmd_cmd },
    { "safe", " -- reboot in safe mode", ls_safe_cmd },
    { "join", " -- join now", ls_join_cmd },
    { NULL, NULL, NULL },
};

static void unwds_callback(module_data_t *buf)
{
    int res = semtech_loramac_send(&ls, buf->data, buf->length);

    switch (res) {
        case SEMTECH_LORAMAC_BUSY:
            puts("[error] MAC already busy");
        case SEMTECH_LORAMAC_NOT_JOINED: {
            puts("[error] Not joined to the network");

            /* Try to join to the network */
            if (current_join_retries == 0) {
                puts("[info] Attempting to rejoin");
                msg_send(&msg_join, sender_pid);
            } else {
                puts("[info] Waiting for the node to join");
            }
            break;
        }
        case SEMTECH_LORAMAC_TX_SCHEDULED:
            puts("[info] TX scheduled");
            break;
        default:
            puts("[warning] Unknown response");
            break;
    }

    blink_led(LED_GREEN);
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
    (void)arg;
    
    wdg_reload();
    rtctimers_millis_set(&iwdg_timer, 15000);
    DEBUG("Watchdog reset\n");
    return;
}

static void ls_enable_sleep (void *arg) {
    (void)arg;
    pm_unblock(PM_SLEEP);
    puts("Low-power sleep mode active");
    return;
}

void init_normal(shell_command_t *commands)
{
    if (!unwds_config_load()) {
        puts("[!] Device is not configured yet. Type \"help\" to see list of possible configuration commands.");
        puts("[!] Configure the node and type \"reboot\" to reboot and apply settings.");

        print_config();
    }
    else {
        print_config();

        radio_init();
        ls_setup(&ls);

        unwds_set_enabled(unwds_get_node_settings().enabled_mods);
        //memcpy(ls.settings.ability, unwds_get_node_settings().enabled_mods, sizeof(ls.settings.ability));
        //ls.settings.class = unwds_get_node_settings().nodeclass;

        unwds_setup_nvram_config(config_get_nvram(), UNWDS_CONFIG_BASE_ADDR, UNWDS_CONFIG_BLOCK_SIZE_BYTES);

        uint32_t bootmode = rtc_restore_backup(RTC_REGBACKUP_BOOTLOADER);
        
        if (is_connect_button_pressed() || (bootmode == UNWDS_BOOT_SAFE_MODE)) {
            uint32_t bootmode = UNWDS_BOOT_NORMAL_MODE;
            rtc_save_backup(bootmode, RTC_REGBACKUP_BOOTMODE);
            
            puts("[!] Entering Safe Mode, all modules disabled, class C.");
            blink_led(LED_GREEN);
            blink_led(LED_GREEN);
            blink_led(LED_GREEN);
        }
        else {
            unwds_init_modules(unwds_callback);
            
            /* reset IWDG timer every 15 seconds */
            /* NB: unwired-module MUST NOT need more than 3 seconds to finish its job */
            
            iwdg_timer.callback = iwdg_reset;
            rtctimers_millis_set(&iwdg_timer, 15000);
            
            /* IWDG period is 18 seconds minimum, 28 seconds typical */
            wdg_set_prescaler(6);
            wdg_set_reload(0x0FFF);
            wdg_reload();
            wdg_enable();

            /* enable sleep for Class A devices only */      
            if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A) {
                pm_enable_timer.callback = ls_enable_sleep;
                rtctimers_millis_set(&pm_enable_timer, 15000);
            }
            
            blink_led(LED_GREEN);
        }

        if (!unwds_get_node_settings().no_join) {
        	msg_send(&msg_join, sender_pid);
        }
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
