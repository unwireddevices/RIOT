/*
 * Copyright (C) 2016-2019 Unwired Devices LLC <info@unwds.com>

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
 * @brief       GSM modem node device
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
#include "periph/adc.h"
#include "periph/wdg.h"
#include "base64.h"
#include "random.h"
#include "lptimer.h"
#include "fmt.h"
#include "byteorder.h"

#include "ls-settings.h"
#include "ls-config.h"
#include "ls-init-device.h"

#include "board.h"

#include "unwds-common.h"
#include "unwds-gpio.h"

#include "umdk-ids.h"

#include "main.h"
#include "utils.h"

#include "simcom.h"

#include "MQTTPacket.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

typedef enum {
    NODE_MSG_JOIN,
    NODE_MSG_SEND,
    NODE_MSG_RETX,
} node_message_types_t;

#define ATMODEM_MQTT_PORT_DEFAULT   1883
#define ATMODEM_MQTT_ADDR_DEFAULT   "127.0.0.1"
#define ATMODEM_MQTT_USER_DEFAULT   "test"
#define ATMODEM_MQTT_PASS_DEFAULT   "password"
#define ATMODEM_MQTT_TOPIC_DEFAULT  "atmodem"

static struct {
    int32_t     id;
	uint16_t    upload_period_minutes;
    uint16_t    mqtt_port;
    char        mqtt_address[100];
    char        mqtt_user[50];
    char        mqtt_password[50];
    char        mqtt_topic[50];
} atmodem_mqtt_config;

static uint8_t mqtt_buf[1000];
static char mqtt_payload[1000];

typedef struct {
    uint8_t *buffer;
    uint32_t length;
} node_data_t;

#define AT_DEV_BUF_SIZE         (256)
static char at_dev_buf[AT_DEV_BUF_SIZE];

#define AT_DEV_RESP_SIZE        (1024)
static char at_dev_resp[AT_DEV_RESP_SIZE];

static simcom_dev_t simcom_dev;

static node_data_t node_data;

static msg_t msg_data = { .type = NODE_MSG_SEND };
static kernel_pid_t sender_pid;
static kernel_pid_t receiver_pid;

static char sender_stack[1024];
static char receiver_stack[1024];
/*
static bool appdata_received(uint8_t *buf, size_t buflen, uint8_t fport);
*/
static void unwds_callback(module_data_t *buf);

static void *sender_thread(void *arg) {
    (void) arg;
    
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);
    
    puts("[GSM] sender thread started");
    
    while (1) {
        msg_receive(&msg);

        int res;
        
        printf("[GSM] Powering up modem\n");
        
        if (msg.type == NODE_MSG_SEND) {
            node_data_t *ptr = msg.content.ptr;

            res = simcom_init(&simcom_dev, SIMCOM_UART, SIMCOM_BAUDRATE, at_dev_buf, AT_DEV_BUF_SIZE, at_dev_resp, AT_DEV_RESP_SIZE);
            if (res != SIMCOM_OK) {
                printf("[GSM] Error initializing modem\n");
                continue;
            } else {
                printf("[GSM] Modem initialized\n");
            }

            res = simcom_start_internet(&simcom_dev, 30, NULL);
            if (res != SIMCOM_OK) {
                printf("[GSM] Error setting up modem\n");
                simcom_power_off(&simcom_dev);
                continue;
            } else {
                printf("[GSM] Modem ready\n");
            }

            /*
            switch (res) {
                default:
                    DEBUG("[GSM] send: unknown response %d\n", res);
                    break;
            }
            */
            
            int sockfd = simcom_socket(&simcom_dev);
            
            if (sockfd < SIMCOM_OK) {
                DEBUG("[GSM] Socket open error: %i\n", sockfd);
                simcom_power_off(&simcom_dev);
                continue;
            } else {
                printf("[GSM] Socket ready\n");
            }
            
            char tcp_port[10];
            snprintf(tcp_port, 10, "%d", atmodem_mqtt_config.mqtt_port);
            
            res = simcom_connect(&simcom_dev, 
                                  sockfd, 
                                  atmodem_mqtt_config.mqtt_address, 
                                  tcp_port, 
                                  "TCP");
            if (res < SIMCOM_OK) {
                DEBUG("[GSM] Connection error: %i\n", res);
                simcom_power_off(&simcom_dev);
                continue;
            }  else {
                printf("[GSM] Modem connected\n");
            }
            
            puts("Building MQTT packet");

            /* build MQTT packet */
            size_t buflen = sizeof(mqtt_buf);
            
            MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
            MQTTString topicString = MQTTString_initializer;
            
            data.keepAliveInterval = 20;
            data.cleansession = 1;
            data.MQTTVersion = 4;
            
            /* encode mqtt_payload to Base64 */
            base64_encode(ptr->buffer, ptr->length, mqtt_buf, &buflen);
            mqtt_buf[buflen] = 0;
            
            /* restore buffer size */
            buflen = sizeof(mqtt_buf);
            
            uint64_t id = config_get_nodeid();

            cpu_update_status();
            int8_t temperature = INT8_MIN;
            int16_t voltage = 0;
            
            if (cpu_status.temp.core_temp != INT16_MIN) {
                temperature = cpu_status.temp.core_temp;
                printf("MCU temperature is %d C\n", temperature);
            }
            if (cpu_status.voltage.vdd != INT16_MIN) {
                voltage = cpu_status.voltage.vdd;
                printf("MCU voltage %d mV\n", voltage);
            }
            
            simcom_csq_resp_t simcom_csq_resp = { 0 };
            simcom_get_signal_quality_report(&simcom_dev, &simcom_csq_resp);
            int rssi = simcom_csq_resp.rssi;
            
            snprintf(mqtt_payload, sizeof(mqtt_payload), "{"      \
                    "\"id\": \"%08lx%08lx\","           \
                    "\"rssi\": \"%d\","                 \
                    "\"temperature\": \"%d\","          \
                    "\"battery\": \"%d\","              \
                    "\"mqtt_payload\": \"%s\""               \
                    "}",                                \
                    (uint32_t)(id >> 32),               \
                    (uint32_t)(id & 0xFFFFFFFF), rssi,  \
                    temperature, voltage, mqtt_buf);
                
            int payloadlen = strlen(mqtt_payload);                
            
            int len = 0;
            
            snprintf(data.clientID.cstring, 6, "%ld", atmodem_mqtt_config.id);
            
            memcpy(data.username.cstring, atmodem_mqtt_config.mqtt_user, strlen(atmodem_mqtt_config.mqtt_user) + 1);
            memcpy(data.password.cstring, atmodem_mqtt_config.mqtt_password, strlen(atmodem_mqtt_config.mqtt_password) + 1);
            
            len = MQTTSerialize_connect((unsigned char *)mqtt_buf, buflen, &data);

            memcpy(topicString.cstring, atmodem_mqtt_config.mqtt_topic, strlen(atmodem_mqtt_config.mqtt_topic) + 1);
            
            len += MQTTSerialize_publish((unsigned char *)(mqtt_buf + len), buflen - len, 0, 0, 0, 0, topicString, (unsigned char *)mqtt_payload, payloadlen);
            len += MQTTSerialize_disconnect((unsigned char *)(mqtt_buf + len), buflen - len);
            
            printf("MQTT: JSON %d bytes, total message %d bytes\n", payloadlen, len);
            
            printf("%s\n", mqtt_payload);

            res = simcom_send(&simcom_dev, sockfd, mqtt_buf, len);
            
            res = simcom_close(&simcom_dev, sockfd);
            if (res != SIMCOM_OK) {
                DEBUG("[GSM] Socket close error: %i\n", res);
            }
        }
    }
    return NULL;
}

static void *receiver_thread(void *arg) {
    (void) arg;
    
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    puts("[GSM] receiver thread started");
    
    while (1) {
        msg_receive(&msg);
        /*
        uint8_t server_resp[200];
        
        int res = simcom_receive(&simcom_dev, sockfd, server_resp, sizeof(server_resp));
        switch (res) {
            default:
                printf("[GSM] unknown LoRaMAC response %d\n", res);
                break;
        }
        */
    }
    return NULL;
}

#if 0
static bool appdata_received(uint8_t *buf, size_t buflen)
{
#if ENABLE_DEBUG
    char hex[100] = {};
    bytes_to_hex(buf, buflen, hex, false);
    printf("[GSM] received data: \"%s\"\n", hex);
#endif

    blink_led(LED0_PIN);

    module_data_t cmd;
    /* Save command data */
    memcpy(cmd.data, buf, buflen);
    cmd.length = buflen;

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
        /* wait a bit */
        lptimer_sleep(3000);
        unwds_callback(&reply);
    }
    /* Don't allow to send app. data ACK by the network.
     * The ACK will be sent either by the callback with the actual app. data or
     * with the command response itself */
    return false;
}
#endif

int ls_set_cmd(int argc, char **argv)
{
    if (argc != 3) {
        puts("usage: set <key> <value>");
        puts("keys:");
        /* TBD */
    }

    (void) argv;
    (void) argc;

    char *key = argv[1];
    char *value = argv[2];
    
    if (strcmp(key, "class") == 0) {
        char v = value[0];

        if (v != 'A' && v != 'C') {
            puts("set сlass: A or C");
            return 1;
        }

        if (v == 'A') {
            unwds_set_class(LS_ED_CLASS_A);
        }
        /*
        else if (v == 'B') {
            unwds_set_class(LS_ED_CLASS_B);
        }
        */
        else if (v == 'C') {
            unwds_set_class(LS_ED_CLASS_C);
        }
    }

    return 0;
}

static void print_config(void)
{
    puts("[ node configuration ]");

    uint64_t eui64 = config_get_nodeid();

    printf("DevEUI = 0x%08x%08x\n", (unsigned int) (eui64 >> 32), (unsigned int) (eui64 & 0xFFFFFFFF));
    
    char nodeclass = 'A'; // unwds_get_node_settings().nodeclass == LS_ED_CLASS_A
    if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_B) {
        nodeclass = 'B';
    }
    else if (unwds_get_node_settings().nodeclass == LS_ED_CLASS_C) {
        nodeclass = 'C';
    }
    printf("CLASS = %c\n", nodeclass);

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

#if defined RTC_REGBACKUP_BOOTMODE
    uint32_t bootmode = UNWDS_BOOT_SAFE_MODE;
    rtc_save_backup(bootmode, RTC_REGBACKUP_BOOTMODE);
    puts("Rebooting in safe mode");
    NVIC_SystemReset();
#else
    puts("Safe mode not supported");
#endif
    return 0;
}

static char str_send_buf[200] = { 0 };

static int ls_send_cmd(int argc, char **argv) {
    (void) argv;
    uint32_t len = 0;
    
    if (argc > 1) {
        strcpy(str_send_buf, argv[1]);
        len = strlen(argv[1]) + 1;
        for (int i = 2; i < argc; i++) {
            if (strlen(argv[i]) + 1 + len > 200) {
                puts("String too long, truncating");
                break;
            }
            strcat(str_send_buf, " ");
            strcat(str_send_buf, argv[i]);
            len += 1 + strlen(argv[i]);
        }
        node_data.buffer = (uint8_t *)str_send_buf;
        node_data.length = strlen(str_send_buf) + 1;
        msg_data.content.ptr = &node_data;
        
        /* send data */
        msg_send(&msg_data, sender_pid);
        
        blink_led(LED0_PIN);
    }
    return 0;
}

shell_command_t shell_commands[UNWDS_SHELL_COMMANDS_MAX] = {
    { "set", "<config> <value> -- set value for the configuration entry", ls_set_cmd },
    { "lscfg", "-- print out current configuration", ls_printc_cmd },
    { "lsmod", "-- list available modules", ls_listmodules_cmd },
    { "mod", "<name> <enable|disable>	-- disable or enable selected module", ls_module_cmd },
    { "cmd", "<modid> <cmdhex> -- send command to another UNWDS device", ls_cmd_cmd },
    { "safe", " -- reboot in safe mode", ls_safe_cmd },
    { "send", " -- send string over GSM", ls_send_cmd },
    { NULL, NULL, NULL },
};

static void unwds_callback(module_data_t *buf)
{
    printf("[GSM] payload size %d bytes\n", buf->length);
    
#if ENABLE_DEBUG
    for (int k = 0; k < buf->length; k++) {
        printf("%02X ", buf->data[k]);
    }
    printf("\n");
#endif

    node_data.buffer = buf->data;
    node_data.length = buf->length;
    msg_data.content.ptr = &node_data;
    
    /* send data */
    msg_send(&msg_data, sender_pid);

    blink_led(LED0_PIN);
}

static int unwds_init(void) {
    /* nothing to do yet */

    return 0;
}

static void unwds_join(void) {
    /* nothing to do yet */
}

static void unwds_sleep(void) {
    /* nothing to do yet */
}

void init_normal(shell_command_t *commands)
{
    simcom_dev.power_en_pin    = SIMCOM_DCDC_PIN;
    simcom_dev.power_act_level = SIMCOM_DCDC_LEVEL;
    simcom_dev.gsm_en_pin      = SIMCOM_ENABLE_PIN;
    simcom_dev.gsm_act_level   = SIMCOM_ENABLE_LEVEL;
    
    atmodem_mqtt_config.mqtt_port = ATMODEM_MQTT_PORT_DEFAULT;
    strcpy(atmodem_mqtt_config.mqtt_address, ATMODEM_MQTT_ADDR_DEFAULT);
    strcpy(atmodem_mqtt_config.mqtt_user, ATMODEM_MQTT_USER_DEFAULT);
    strcpy(atmodem_mqtt_config.mqtt_password, ATMODEM_MQTT_PASS_DEFAULT);
    strcpy(atmodem_mqtt_config.mqtt_topic, ATMODEM_MQTT_TOPIC_DEFAULT);

    bool cfg_valid = unwds_config_load();
    print_config();
    
    if (!cfg_valid) {
        puts("[!] Device is not configured yet. Type \"help\" to see list of possible configuration commands.");
        puts("[!] Configure the node and type \"reboot\" to reboot and apply settings.");
    } else {
        sender_pid = thread_create(sender_stack, sizeof(sender_stack), THREAD_PRIORITY_MAIN - 2,
                                   THREAD_CREATE_STACKTEST, sender_thread, NULL,  "GSM sender thread");
                                   
        receiver_pid = thread_create(receiver_stack, sizeof(receiver_stack), THREAD_PRIORITY_MAIN - 2,
                                   THREAD_CREATE_STACKTEST, receiver_thread, NULL,  "GSM receiver thread");

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
