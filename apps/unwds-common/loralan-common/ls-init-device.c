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
 * @file        ls-init-device.c
 * @brief       Common LoRaLAN device initialization functions
 * @author      Oleg Artamonov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "cpu.h"
#include "periph/eeprom.h"
#include "shell.h"
#include "utils.h"

#include "sx127x.h"
#include "net/lora.h"
#include "periph/pm.h"
#include "pm_layered.h"
#include "periph/rtc.h"
#include "periph/wdg.h"
#include "random.h"
#include "cpu.h"
#include "luid.h"
#include "xtimer.h"
#include "lptimer.h"
#include "unwds-common.h"
#include "umdk-ids.h"
#include "umdk-modules.h"

#include "ls-settings.h"
#include "ls-config.h"
#include "ls-init-device.h"

#define ENABLE_DEBUG    0
#include "debug.h"

#define UNWD_IWDG_THREAD_SIZE   (256)

static void init_common(shell_command_t *commands);
static void init_config(shell_command_t *commands);
static int init_clear_nvram(int argc, char **argv);
static int init_save_cmd(int argc, char **argv);
static int init_update_cmd(int argc, char **argv);
static void print_config(void);

static uint64_t eui64 = 0;
static uint64_t appid = 0;

static uint8_t appkey[16] = {};
static uint8_t appskey[16] = {};
static uint8_t nwkskey[16] = {};

static uint32_t devnonce = 0;

static kernel_pid_t iwdg_pid;

static lptimer_t delayed_setup_timer;

/**
 * Data rates table.
 */
const uint8_t datarate_table[7][3] = {
    { LORA_SF12, LORA_BW_125_KHZ, LORA_CR_4_5 },       /* DR0 */
    { LORA_SF11, LORA_BW_125_KHZ, LORA_CR_4_5 },       /* DR1 */
    { LORA_SF10, LORA_BW_125_KHZ, LORA_CR_4_5 },       /* DR2 */
    { LORA_SF9, LORA_BW_125_KHZ, LORA_CR_4_5 },        /* DR3 */
    { LORA_SF8, LORA_BW_125_KHZ, LORA_CR_4_5 },        /* DR4 */
    { LORA_SF7, LORA_BW_125_KHZ, LORA_CR_4_5 },        /* DR5 */
    { LORA_SF7, LORA_BW_250_KHZ, LORA_CR_4_5 },        /* DR6 */
};

void ls_setup_sx127x(netdev_t *dev, ls_datarate_t dr, uint32_t frequency) {    
    const netopt_enable_t enable = true;
    const netopt_enable_t disable = false;

    /* Choose data rate */
    const uint8_t *datarate = datarate_table[dr];
    dev->driver->set(dev, NETOPT_SPREADING_FACTOR, &datarate[0], sizeof(uint8_t));
    dev->driver->set(dev, NETOPT_BANDWIDTH, &datarate[1], sizeof(uint8_t));
    dev->driver->set(dev, NETOPT_CODING_RATE, &datarate[2], sizeof(uint8_t));
    
    uint8_t hop_period = 0;
    dev->driver->set(dev, NETOPT_CHANNEL_HOP_PERIOD, &hop_period, sizeof(uint8_t));
    dev->driver->set(dev, NETOPT_CHANNEL_HOP, &disable, sizeof(disable));
    dev->driver->set(dev, NETOPT_SINGLE_RECEIVE, &disable, sizeof(disable));
    dev->driver->set(dev, NETOPT_INTEGRITY_CHECK, &enable, sizeof(enable));
    dev->driver->set(dev, NETOPT_FIXED_HEADER, &disable, sizeof(disable));
    dev->driver->set(dev, NETOPT_IQ_INVERT, &disable, sizeof(disable));
    
    int16_t power = TX_OUTPUT_POWER;
    dev->driver->set(dev, NETOPT_TX_POWER, &power, sizeof(int16_t));
    
    uint16_t preamble_len = LORA_PREAMBLE_LENGTH;
    dev->driver->set(dev, NETOPT_PREAMBLE_LENGTH, &preamble_len, sizeof(uint8_t));
    
    uint32_t tx_timeout = 30000;
    dev->driver->set(dev, NETOPT_TX_TIMEOUT, &tx_timeout, sizeof(uint8_t));
    
    uint32_t rx_timeout = 0;
    dev->driver->set(dev, NETOPT_RX_TIMEOUT, &rx_timeout, sizeof(uint8_t));

    /* Setup channel */
    dev->driver->set(dev, NETOPT_CHANNEL, &frequency, sizeof(uint32_t));
}

void init_role(shell_command_t *commands) {
    pm_init();
    /* all power modes are blocked by default */
    /* unblock PM_IDLE here, PM_SLEEP to be unlocked later */
    pm_unblock(PM_IDLE);
    
    print_logo();
    
    /* Initialize random number generator with CPUID-derived value */
    uint32_t prng_seed;
    luid_base(&prng_seed, 4);
    random_init(prng_seed);

    /* Check EUI64 */
    if (!load_eui64_nvram()) {
    	puts("[config] No EUI64 defined for this device. Please provide EUI64 and reboot to apply changes.");
    } else {
        if (!load_config_nvram()) {
            /* It's first launch or config memory is corrupted */
            puts("[config] No valid configuration found in NVRAM. It's either first launch or NVRAM content is corrupted.");
            puts("[config] Could you please provide APPID64, DEVNONCE and JOINKEY for this device?");

            config_reset_nvram();
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
            puts("set deveui <16 hex symbols> -- sets DevEUI");
            puts("\tExample: set deveui 00000000000011ff");
            break;
            
        case ROLE_EMPTY_KEY:
        case ROLE_NO_CFG:
            puts("set appeui <16 hex symbols> -- sets AppEUI");
            puts("set appkey <32 hex symbols> -- sets OTAA network encryption key");
            
            #if defined(UNWDS_MAC_LORAWAN)
            puts("set appskey <32 hex symbols> -- sets ABP AppSkey encryption key");
            puts("set nwkskey <32 hex symbols> -- sets ABP NwkSkey encryption key");
            #endif
            
            #if defined(UNWDS_MAC_LORAWAN)
            puts("set devaddr <8 hex symbols> -- sets network device address DevAddr");
            #else
            puts("set devnonce <8 hex symbols> -- sets session encryption key for no-join devices");
            #endif

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
    
    /* somehow auto_init doesn't work, at least on nRF52, so init xtimer here */
    xtimer_init();
    
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

    if (strcmp(type, "appeui") == 0) {
        uint64_t id = 0;

        if (strlen(arg) != 16) {
            puts("[error] AppEUI must be 64 bits (16 hex symbols) long");
            return 1;
        }

        if (!hex_to_bytes(arg, (uint8_t *) &id, true)) {
            puts("[error] Invalid number format specified");
            return 1;
        }

        printf("[ok] AppEUI = 0x%08x%08x\n", (unsigned int) (id >> 32), (unsigned int) (id & 0xFFFFFFFF));
        appid = id;
    }
    else if (strcmp(type, "appkey") == 0) {
        if (strlen(arg) != 32) {
            puts("[error] AppKey must be 128 bits (32 hex symbols) long");
            return 1;
        }

        if (!hex_to_bytes(arg, appkey, false)) {
            puts("[error] Invalid format specified");
            return 1;
        }

        printf("[ok] AppKey = %s\n", arg);
    }
#if defined(UNWDS_MAC_LORAWAN)
    else if (strcmp(type, "appskey") == 0) {
        if (strlen(arg) != 32) {
            puts("[error] AppsKey must be 128 bits (32 hex symbols) long");
            return 1;
        }

        if (!hex_to_bytes(arg, appskey, false)) {
            puts("[error] Invalid format specified");
            return 1;
        }

        config_set_appskey(appskey);
        printf("[ok] AppsKey = %s\n", arg);
    }
    else if (strcmp(type, "nwkskey") == 0) {
        if (strlen(arg) != 32) {
            puts("[error] AppKey must be 128 bits (32 hex symbols) long");
            return 1;
        }

        if (!hex_to_bytes(arg, nwkskey, false)) {
            puts("[error] Invalid format specified");
            return 1;
        }

        config_set_nwkskey(nwkskey);
        printf("[ok] NwksKey = %s\n", arg);
    }
    else if (strcmp(type, "devaddr") == 0) {
        if (strlen(arg) != 8) {
            puts("[error] DevAddr must be 32 bits (8 hex symbols) long");
            return 1;
        }

        uint32_t d = 0;

        if (!hex_to_bytesn(arg, 8, (uint8_t *) &d, true)) {
            puts("[error] Invalid format specified");
            return 1;
        }

        printf("[ok] DevAddr = %s\n", arg);

        devnonce = d;
    }
#else
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
#endif
    else if ((strcmp(type, "deveui") == 0) && (config_get_role() == ROLE_NO_EUI64)) {
        uint64_t id = 0;

        if (strlen(arg) != 16) {
            puts("[error] There must be 16 hexadecimal digits in lower case as DevEUI");
            return 1;
        }

        if (!hex_to_bytes(arg, (uint8_t *) &id, true)) {
            puts("[error] Invalid number format specified");
            return 1;
        }

        printf("[ok] DevEUI = 0x%08x%08x\n", (unsigned int) (id >> 32), (unsigned int) (id & 0xFFFFFFFF));
        eui64 = id;
    } else {
        puts("[error] Unknown command");
    }
    
    /* print_config(); */
    
    puts("Settings can be changed by calling 'set' command again");
    puts("Invoke 'save' command when finished");

    return 0;
}

static int get_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
    print_config();
    
    return 0;
}

static int save_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    
	if (argc == 1) {
		puts("Current configuration:");
		print_config();

		puts("[!] Saving current configuration...");

        bool status = false;
        
        switch (config_get_role()) {
            case ROLE_NO_EUI64:
                /* Set EUI64 */
                status = write_eui64_nvram(eui64);
                break;
            case ROLE_EMPTY_KEY:
            case ROLE_NO_CFG:
                /* Set appID, appkey and nonce */
                status = config_write_main_block(appid, appkey, appskey, nwkskey, devnonce);
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
    { "save", "-- save configuration to NVRAM", save_cmd },

    { NULL, NULL, NULL }
};

static void init_config(shell_command_t *commands)
{
    /* Set our commands for shell */
    memcpy(commands, shell_commands_cfg, sizeof(shell_commands_cfg));

    blink_led(LED0_PIN);
    
    if (config_get_role() != ROLE_NO_EUI64) {
        eui64 = config_get_nodeid();
        appid = config_get_appid();
        devnonce = config_get_devnonce();
        
        memcpy(appkey, config_get_appkey(), sizeof(appkey));

        if ((config_get_role() == ROLE_NO_CFG) || (config_get_role() == ROLE_EMPTY_KEY)) {
            /* Generate ABP keys */
            uint32_t rand = 0;
            for (uint32_t i = 0; i < 16; i += sizeof(rand)) {
                rand = random_uint32();
                memcpy(appskey + i, (void *)&rand, sizeof(rand));
                
                rand = random_uint32();
                memcpy(nwkskey + i, (void *)&rand, sizeof(rand));
            }
        } else {
            memcpy(appskey, config_get_appskey(), sizeof(appskey));
            memcpy(nwkskey, config_get_nwkskey(), sizeof(nwkskey));
        }

        print_config();
    }
    
    print_help();
}

static void print_config(void)
{
    puts("[config] Current configuration:");
    
    char s[33] = {};
    
    printf("DevEUI = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));

    if (config_get_role() != ROLE_NO_EUI64) {
        printf("AppEUI = 0x%08x%08x\n", (unsigned int) (appid >> 32), (unsigned int) (appid & 0xFFFFFFFF));
        
        #if defined(UNWDS_MAC_LORAWAN)
        printf("DevAddr = 0x%08X\n", (unsigned int) devnonce);
        #else
        printf("DEVNONCE = 0x%08X\n", (unsigned int) devnonce);
        #endif
        
        bytes_to_hex(appkey, 16, s, false);
        printf("AppKey = %s\n", s);
        
        #if defined(UNWDS_MAC_LORAWAN)
        bytes_to_hex(appskey, 16, s, false);
        printf("AppsKey = %s\n", s);
        
        bytes_to_hex(nwkskey, 16, s, false);
        printf("NwksKey = %s\n", s);
        #endif
    }
}

static int init_clear_nvram(int argc, char **argv)
{
    if (argc < 2) {
        puts("Usage: clear <all|key|modules> -- clear all NVRAM contents or just the security key.");
        return 1;
    }

    char *key = argv[1];

    if (strcmp(key, "all") == 0) {
        puts("Clearing NVRAM, please wait");
        if (clear_nvram()) {
            puts("[ok] Settings cleared, rebooting");
            NVIC_SystemReset();
        }
        else {
            puts("[error] Unable to clear NVRAM");
        }
    }
    else if (strcmp(key, "key") == 0) {
        uint8_t appkey_zero[16];
        memset(appkey_zero, 0, 16);
        if (config_write_main_block(config_get_appid(), appkey_zero, appskey, nwkskey, devnonce)) {
            puts("[ok] Security key was zeroed. Rebooting.");
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
    
#if defined(RTC_REGBACKUP_BOOTLOADER)
    puts("[*] Rebooting to UART bootloader...");
    rtc_save_backup(RTC_REGBACKUP_BOOTLOADER_VALUE, RTC_REGBACKUP_BOOTLOADER);
#endif
    
    NVIC_SystemReset();
    
    return 0;
}

static  uint32_t            connect_btn_last_press = 0;
static  bool                board_is_off = false;
static  lptimer_t           sys_on_off;

static void system_on_off (void *arg) {
    if (arg) {
        gpio_clear(LED1_PIN);
        gpio_clear(LED0_PIN);
        /* stop all activity */
        lptimer_remove_all();
        /* restart watchdog timer */
        pm_unblock(PM_SLEEP);
        void (*board_sleep)(void) = arg;
        board_sleep();
        puts("*** SYSTEM HALTED BY USER ***");
    } else {
        gpio_clear(LED0_PIN);
        puts("*** REBOOTING SYSTEM ***");
        pm_reboot();
    }
}

static bool is_connect_button_pressed(void)
{
#if defined(UNWD_CONNECT_POL)
    uint8_t state;
    if (UNWD_CONNECT_POL) {
        state = GPIO_IN_PD;
    } else {
        state = GPIO_IN_PU;
    }
    
    if (!gpio_init(UNWD_CONNECT_BTN, state)) {
        if (gpio_read(UNWD_CONNECT_BTN) == UNWD_CONNECT_POL) {
            return true;
        }
    }
#else
    if (!gpio_init(UNWD_CONNECT_BTN, GPIO_IN_PU)) {
        if (gpio_read(UNWD_CONNECT_BTN) == 0) {
            return true;
        }
    }
#endif

    return false;
}

static void connect_btn_pressed (void *arg) {
    uint32_t ms_now = lptimer_now_msec();
    
    /* debounce */
    if ((connect_btn_last_press != 0) &&
        (ms_now < connect_btn_last_press + 100) &&
        (ms_now >= connect_btn_last_press)) {
        return;
    }
    
    /* button released */
#if defined(UNWD_CONNECT_POL)
    if ((UNWD_CONNECT_BTN != GPIO_UNDEF) && (gpio_read(UNWD_CONNECT_BTN)) != UNWD_CONNECT_POL) {
#else
    if ((UNWD_CONNECT_BTN != GPIO_UNDEF) && (gpio_read(UNWD_CONNECT_BTN)) != 0) {
#endif
        if (((ms_now > connect_btn_last_press) && ((ms_now - connect_btn_last_press) > 1000)) || 
            ((ms_now + (UINT32_MAX - connect_btn_last_press)) > 1000)) {
            /* long press */
            sys_on_off.callback = system_on_off;
            
            if (board_is_off) {
                gpio_set(LED0_PIN);
                sys_on_off.arg = NULL;
                lptimer_set(&sys_on_off, 1000);
            } else {            
                board_is_off = true;
                gpio_set(LED1_PIN);
                sys_on_off.arg = arg;
                lptimer_set(&sys_on_off, 1000);
            }
        } else {
            /* short press */
            /* invoke send command for all modules enabled */
            /* must not be executed inside IRQ */
            /*
            int i = 0;
            uint32_t *enabled_mods = unwds_get_node_settings().enabled_mods;
            while (modules[i].init_cb != NULL && modules[i].cmd_cb != NULL) {
                bool enabled = (enabled_mods[modules[i].module_id / 32] & (1 << (modules[i].module_id % 32)));

                if (!enabled) {
                    i++;
                    continue;
                }
                
                int argc = 2;
                char arg[UNWDS_MAX_MODULE_NAME];
                char *argv[2] = { arg, "send\0" };
                memcpy(arg, modules[i].name, UNWDS_MAX_MODULE_NAME);
                DEBUG("Executing: %s %s\n", argv[0], argv[1]);
                shell_call(argc, argv);

                i++;
            }
            */
        }
    } else {
        /* button pressed */
    }
    
    connect_btn_last_press = ms_now;
}

static void *iwdg_reset (void *arg) {
    (void)arg;
    
    while (1) {
        lptimer_sleep(15000UL);
        wdg_reload();
    }
    return NULL;
}

static void ls_delayed_setup (void *arg) {
    if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_A) {
        pm_unblock(PM_SLEEP);
        puts("Low-power sleep mode active");
    }
#if defined(UNWD_CONNECT_POL)
    uint8_t state;
    if (UNWD_CONNECT_POL) {
        state = GPIO_IN_PD;
    } else {
        state = GPIO_IN_PU;
    }
    if (!gpio_init_int(UNWD_CONNECT_BTN, state, GPIO_BOTH, connect_btn_pressed, arg)) {
#else
    if (!gpio_init_int(UNWD_CONNECT_BTN, GPIO_IN_PU, GPIO_BOTH, connect_btn_pressed, arg)) {
#endif
        puts("Safe/Connect button active");
}
    
    return;
}

void unwds_device_init(void *unwds_callback, void *unwds_init, void *unwds_join, void *unwds_sleep) {
    int (*board_init)(void) = unwds_init;
    
    if (board_init() != 0) {        
        puts("ls: error initializing device");
        gpio_set(LED0_PIN);
        lptimer_sleep(5000);
        NVIC_SystemReset();
    }

    unwds_set_enabled(unwds_get_node_settings().enabled_mods);
    
    //memcpy(ls.settings.ability, unwds_get_node_settings().enabled_mods, sizeof(ls.settings.ability));
    //ls.settings.class = unwds_get_node_settings().nodeclass;

    unwds_setup_nvram_config(UNWDS_CONFIG_BASE_ADDR, UNWDS_CONFIG_BLOCK_SIZE_BYTES);

#if defined(RTC_REGBACKUP_BOOTLOADER)
    uint32_t bootmode = rtc_restore_backup(RTC_REGBACKUP_BOOTLOADER);
#else
    uint32_t bootmode = UNWDS_BOOT_NORMAL_MODE;
#endif
    
    if (is_connect_button_pressed() || (bootmode == UNWDS_BOOT_SAFE_MODE)) {
        bootmode = UNWDS_BOOT_NORMAL_MODE;
#if defined(RTC_REGBACKUP_BOOTMODE)
        rtc_save_backup(bootmode, RTC_REGBACKUP_BOOTMODE);
#endif
        
        puts("[!] Entering Safe Mode, all modules disabled, class C.");
        blink_led(LED0_PIN);
        blink_led(LED0_PIN);
        blink_led(LED0_PIN);
    }
    else {
        /* IWDG timer reset thread */
        char *stack = (char *) allocate_stack(UNWD_IWDG_THREAD_SIZE);
        iwdg_pid = thread_create(stack, UNWD_IWDG_THREAD_SIZE, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, iwdg_reset, NULL, "IWDG reset");

        /* IWDG period is 18 seconds minimum, 28 seconds typical */
        wdg_set_reload(28);
        wdg_reload();
        wdg_enable();

        unwds_init_modules(unwds_callback);
        
        /* delayed startup */
        delayed_setup_timer.callback = ls_delayed_setup;
        delayed_setup_timer.arg = unwds_sleep;
        lptimer_set(&delayed_setup_timer, 15000);
        
        blink_led(LED0_PIN);
    }

/* even in ABP mode, LoRaWAN must perform join procedure */
#if !defined(UNWDS_MAC_LORAWAN)
    if (!unwds_get_node_settings().no_join) {
        void (*board_join)(void) = unwds_join;
        board_join();
    }
#else
    void (*board_join)(void) = unwds_join;
    board_join();
#endif

}

#ifdef __cplusplus
}
#endif