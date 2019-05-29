/*
 * Copyright (C) 2019 Unwired Devices LLC <info@unwds.com>

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
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       3G modem support for the Tele2Med devices
 * @author      Oleg Manchenko
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <random.h>
#include "xtimer.h"
#include "string.h"
#include "thread.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "rtctimers-millis.h"
#include "at.h"
#include "base64.h"
#include "od.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define ATMODEM_DEV_BUF_SIZE        (2048)      /* The size of the buffer for all incoming data from modem */
#define ATMODEM_DEV_RESP_SIZE       (1024)      /* The size of the buffer to answer the command from the modem */

#define ATMODEM_READY               (0)
#define ATMODEM_NOT_INITIALIZED     (-1)        /* Modem is not initialized */
#define ATMODEM_NOT_ANSWERING       (-2)        /* If there is no modem, then sim_status.modem = ATMODEM_NOT_ANSWERING and display N/A on the screen. */
#define ATMODEM_WAIT_SMS_TIMEOUT    (400)       /* SMS waiting time with Internet settings in seconds */

#define ATMODEM_BAUDRATE            (115200)    /* UART baudrate for modem*/
#define ATMODEM_MAX_TIMEOUT         (1000000)   /* Maximum time waiting for a response */ 
#define ATMODEM_TIME_ON             (500)       /* The time of active low level impulse of PWRKEY pin to power on module. Min: 50ms, typ: 100ms */
#define ATMODEM_TIME_ON_UART        (3000)      /* The time from power-on issue to UART port ready. Min: 3s, max: 5s */
#define ATMODEM_CMD_TIMEOUT         (10)        /* Command timeout 10ms */

#define ATMODEM_NEW_INTERNET_SETTINGS_OK              (0)
#define ATMODEM_WAITING_SMS_INTERNET_SETTINGS_TIMEOUT (-1)
#define ATMODEM_ERROR_NEW_INTERNET_SETTINGS           (-2)

static at_dev_t at_dev;                         /* Struct for AT device */
static char at_dev_buf[ATMODEM_DEV_BUF_SIZE];   /* Buffer for incoming data */
static char at_dev_resp[ATMODEM_DEV_RESP_SIZE]; /* Buffer for outgoing commands */

static gpio_t atmodem_enable_pin;
static gpio_t atmodem_power_pin;

/* Our own server address and port */
static char server[]                        = "176.15.4.218:777";

/* Servers for connection test */
static char test_ping_own_server[16]        = "176.15.4.218";       /* Own server address */
static char test_ping_google_server[16]     = "8.8.8.8";            /* Google DNS server address */
static char test_ping_k_root_dns_server[16] = "193.0.14.129";       /* k.root DNS server address */

typedef struct {
    int8_t modem;                   /* Modem initialization status */
	bool is_inserted;               /* Is SIM card inserted? */
    bool is_pin;                    /* Is there a PIN code? */
    bool is_internet;               /* Did you manage to connect to the Internet? */
    bool is_server_connect;         /* Is there a connection to the server? */
    bool is_new_internet_settings;  /* New settings for the Internet are obtained from SMS */
    uint8_t network_signal;         /* Signal level in percent. If 0% failed to register */
    uint32_t hni;                   /* Home Network Identity is the combination of the MCC and the MNC */
    char operator[32];              /* Operator name */
    char apn[32];                   /* Access Point Name */
    char username[32];              /* Username */
    char password[32];              /* Password */
} sim_status_t;

static sim_status_t sim_status = { .modem = ATMODEM_NOT_INITIALIZED }; /* Network Test Result */

/* Ping function */
static bool atmodem_ping(char *ip_addr) {
    /* Create AT command */
    char cmd_with_ping[50]; 
    snprintf(cmd_with_ping, 128, "AT+CIPPING=%s,1,32,100,64", ip_addr);

    /* Try ping */
    for(uint8_t i = 0; i < 3; i++) {
        // int res = at_send_cmd_wait_ok(&at_dev, cmd_with_ping, 7000000);    

        /* Now ping is checked by timeout */ 
        /* TODO: When the Echo Request timeout expires (no reply received on time), */
        /* the response will contains <replyTime> setting to 600 and <ttl> setting to 255 */
        /* Example error resp: +CIPPING: 1,"176.15.4.218",600,255*/ 

        int res = at_send_cmd_get_resp(&at_dev, cmd_with_ping, at_dev_resp, ATMODEM_DEV_RESP_SIZE, 7000000);
        if (res > 0) {
            if (strcmp(at_dev_resp, "ERROR") == 0) {
                /* Ping ERROR */
                DEBUG("[ATMODEM] Ping %s ERROR\n", ip_addr);
                return false;
            } else {
                /* Ping OK */
                DEBUG("[ATMODEM] Ping: %s\n", at_dev_resp);
                break;
            }
        } else {
            /* Ping ERROR */
            DEBUG("[ATMODEM] Ping %i ERROR\n", res);
        }

        // if (res == 0) {
        //     break;
        // }

        if (i == 2) {
            DEBUG("[ATMODEM] %s ERROR\n", cmd_with_ping);
            return false;
        }

        DEBUG("[ATMODEM] %s try %i \n", cmd_with_ping, i+2);

        /* 500 ms delay */
        rtctimers_millis_sleep(500);
    }
    return true;
}

/* Get settings for selected SIM card */
bool atmodem_get_internet_settings(void) {
    /* If the settings are already set from SMS, then exit the function */
    if (sim_status.is_new_internet_settings) {
        DEBUG("[ATMODEM] Set internet settings from SMS command\n");
        return true;
    }

    /* Get standart settings */
    switch (sim_status.hni) {
        // +COPN: "25001","MTS RUS"
        // +COPN: "25002","MegaFon RUS"
        // +COPN: "25003","ROSTELECOM"
        // +COPN: "25004","SIBCHALLENGE RUS"
        // +COPN: "25005","ROSTELECOM"
        // +COPN: "25007","RUS 07, RUS SMARTS"
        // +COPN: "25011","Yota"
        // +COPN: "25012","ROSTELECOM"
        // +COPN: "25013","RUS Kuban-GSM"
        // +COPN: "25015","RUS15, RUS SMARTS"
        // +COPN: "25016","NTC"
        // +COPN: "25017","ROSTELECOM"
        // +COPN: "25020","Tele2 RU"
        // +COPN: "25028","VOICE"
        // +COPN: "25035","MOTIV"
        // +COPN: "25038","ROSTELECOM"
        // +COPN: "25039","ROSTELECOM"
        // +COPN: "25092","Primetelefone RUS"
        // +COPN: "25099","Beeline"

        /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ (˜˜˜) */
        case 25001:
            snprintf(sim_status.apn, 32, "internet.mts.ru");   /* Access Point Name */
            snprintf(sim_status.username, 32, "mts");          /* Username */
            snprintf(sim_status.password, 32, "mts");          /* Password */
            break;

        /* ˜˜˜ ˜˜˜˜˜˜˜˜˜ (˜˜˜˜˜˜˜) */
        case 25002:
            snprintf(sim_status.apn, 32, "internet");      /* Access Point Name */
            memset(sim_status.username, 0x00, 32);         /* Username */ 
            memset(sim_status.password, 0x00, 32);         /* Password */
            break;

        // /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜ (˜˜˜) */
        // case 25003:
        //     /*  */
        //     break;

        // /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜ (˜˜˜) */
        // case 25005:
        //     /*  */
        //     break;

        // /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜ (˜˜˜˜ ˜˜˜˜) */
        // case 25006:
        //     /*  */
        //     break;

        // /* ˜˜˜ ˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ (˜˜˜˜˜) */
        // case 25007:
        //     /*  */
        //     break;

        // /* ˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ (˜˜˜˜˜˜ ˜˜˜˜˜˜˜) */
        // case 25008:
        //     /* apn: "vtk" ˜˜˜ "internet" */
        //     break;

        // /* ˜˜˜ ˜˜˜˜˜ ˜˜˜˜ (˜˜˜˜ ˜˜˜˜) */
        // case 25009:
        //     /*  */
        //     break; 

        // /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜ (DTC) */
        // case 25010:
        //     /*  */
        //     break;

        /* ˜˜˜ ˜˜˜˜˜˜˜˜ (YOTA) */
        case 25011:
            snprintf(sim_status.apn, 32, "internet.yota");     /* Access Point Name */
            memset(sim_status.username, 0x00, 32);             /* Username */
            memset(sim_status.password, 0x00, 32);             /* Password */   
            break;

        // /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜ (KUGSM) */
        // case 25012:
        //     /*  */
        //     break;

        // /* ˜˜˜ ˜˜˜˜˜˜˜-GSM˜ (KUGSM) */
        // case 25013:
        //     /*  */
        //     break;

        /* ˜˜˜ ˜˜˜˜˜˜˜˜˜ (˜˜˜˜˜˜˜) */
        case 25014:
            snprintf(sim_status.apn, 32, "internet");  /* Access Point Name */
            memset(sim_status.username, 0x00, 32);     /* Username (gdata ˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜) */ 
            memset(sim_status.password, 0x00, 32);     /* Password (gdata ˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜) */
            break;
        // /* ˜˜˜ ˜˜˜˜˜˜? (˜˜˜˜˜˜-˜˜˜, ˜˜˜˜˜˜-˜˜˜˜˜˜˜˜˜) */
        // case 25015:
        //     /*  */
        //     break;

        // /* ˜˜˜ ˜Miatel˜ (MIATEL) */
        // case 25016:
        //     /*  */
        //     break;

        // /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ (Utel) */
        // case 25017:
        //     /*  */
        //     break;

        /* Tele2 AB (Tele2) */
        case 25020:
            snprintf(sim_status.apn, 32, "internet.tele2.ru");     /* Access Point Name */
            memset(sim_status.username, 0x00, 32);                 /* Username */
            memset(sim_status.password, 0x00, 32);                 /* Password */  
            break;

        // /* ˜˜˜˜˜˜˜ (˜˜˜˜˜˜˜) */
        // case 25023:
        //     /*  */
        //     break;

        // /* ˜˜˜ ˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ (˜˜˜˜˜) */
        // case 25027:
        //     /*  */
        //     break;

        /* ˜˜˜ ˜˜˜˜˜˜˜-˜˜˜˜˜˜˜˜˜˜˜˜ (˜˜˜˜˜˜) */
        case 25028:
            snprintf(sim_status.apn, 32, "internet.beeline.ru");   /* Access Point Name */
            snprintf(sim_status.username, 32, "beeline");          /* Username */
            snprintf(sim_status.password, 32, "beeline");          /* Password */
            break;

        // /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜-2000˜ (˜˜˜˜˜) */
        // case 25035:
        //     /*  */
        //     break;

        // /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜ (˜˜˜˜˜˜-GSM) */
        // case 25038:
        //     /*  */
        //     break;

        /* ˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ (Utel) (˜˜˜˜˜˜ ROSTELECOM) */
        case 25039:
            snprintf(sim_status.apn, 32, "internet.rt.ru");    /* Access Point Name */
            memset(sim_status.username, 0x00, 32);             /* Username */
            memset(sim_status.password, 0x00, 32);             /* Password */ 
            break;

        // /* OJSC ˜Multiregional TransitTelecom˜ (MTT ˜˜˜˜˜˜) */
        // case 25042:
        //     /*  */
        //     break;

        /* ˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ (Tinkoff) */
        case 25062:
            snprintf(sim_status.apn, 32, "m.tinkoff");     /* Access Point Name */
            memset(sim_status.username, 0x00, 32);         /* Username */
            memset(sim_status.password, 0x00, 32);         /* Password */  
            break;

        /* ˜˜˜ ˜˜˜˜˜˜˜-˜˜˜˜˜˜˜˜˜˜˜˜ (˜˜˜˜˜˜) */
        case 25099:
            snprintf(sim_status.apn, 32, "internet.beeline.ru");   /* Access Point Name */
            snprintf(sim_status.username, 32, "beeline");          /* Username */
            snprintf(sim_status.password, 32, "beeline");          /* Password */
            break;    

        /* Unknown operator */
        default: 
            memset(sim_status.apn,      0x00, 32);     /* Access Point Name */
            memset(sim_status.username, 0x00, 32);     /* Username */
            memset(sim_status.password, 0x00, 32);     /* Password */
            return false;   
            break;
    } 
    return true;
}

/*---------------------------------------------------------------------------*/
/* Check the connection of the selected SIM card */
bool atmodem_test_connection(void) {
    /* Is the modem initialized? */
    if (sim_status.modem < 0) {
        return false;
    }

    /* Cycle counter */
    uint8_t counter = 0;

    int res;

	/* SIM card test */
    DEBUG("[ATMODEM] Test connection\n");

    /* Switch on Detecting SIM Card */ 
    at_send_cmd_get_resp(&at_dev, "AT+CSDT=1", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);

	/* Is SIM card inserted? */
    at_send_cmd_get_resp(&at_dev, "AT+CSMINS?", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
    if (strcmp("+CSMINS: 0,1", at_dev_resp) != 0) {
        DEBUG("[ATMODEM] SIM card is not inserted\n");
        sim_status.is_inserted = false;

        return false;
    }
    sim_status.is_inserted = true;

    /* Is there a PIN code? */
    counter = 0;
    while(1) {
        if (counter >= 5) {
            DEBUG("[ATMODEM] SIM card has PIN code\n");
            sim_status.is_pin = true;
            return false;
        }

        res = at_send_cmd_get_resp(&at_dev, "AT+CPIN?", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
        if (res >= 0) {
            if(strcmp("+CPIN: READY", at_dev_resp) == 0)
            {
                DEBUG("[ATMODEM] SIM card ready\n");
                sim_status.is_pin = false;
                break;
            }
        }

        rtctimers_millis_sleep(100);
        counter++;
    }

    /* Have you registered in the cellular network? */
    counter = 0;
    while(1) {
        if (counter >= 30) {
            DEBUG("[ATMODEM] Registration failed\n");
            sim_status.network_signal = 0;
            return false;
        }

        res = at_send_cmd_get_resp(&at_dev, "AT+CREG?", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
        if (res >= 0) {
            if(strcmp("+CREG: 0,1", at_dev_resp) == 0)
            {
                DEBUG("[ATMODEM] Registration accepted\n");
                break;
            }
        }

        rtctimers_millis_sleep(1000);
        counter++;
    }    

    /* Reject Incoming Call */
    res = at_send_cmd_wait_ok(&at_dev, "AT+GSMBUSY=1", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+GSMBUSY=1 ERROR\n");
    }

	/* Signal strength */ 
    at_send_cmd_get_resp(&at_dev, "AT+CSQ", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);

    uint8_t signal_quality;
    sscanf(at_dev_resp,"%*s %hhi", &signal_quality);

    if (signal_quality == 0) {
        sim_status.network_signal = 115;
    } else if (signal_quality == 1) {
        sim_status.network_signal = 111;
    } else if ((signal_quality > 1) && (signal_quality <= 30)) {
        sim_status.network_signal = 110 - ((signal_quality - 2) * 2);
    } else if (signal_quality == 31) {
        sim_status.network_signal = 52;
    } else 
        sim_status.network_signal = 115;

    DEBUG("[ATMODEM] Signal: -%idBm\n", sim_status.network_signal);

	/* Which operator name? */
    at_send_cmd_get_resp(&at_dev, "AT+COPS?", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
    sscanf(at_dev_resp, "%*s %*i,%*i,\"%[^\"]s,%*i", sim_status.operator); 
    DEBUG("[ATMODEM] Operator: %s\n", sim_status.operator);

    /* Is internet? */
    /* Check attach status */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CGACT?", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+CGACT? ERROR\n");
        sim_status.is_internet = false;
        return false;
    }

    /* Attach to the network */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CGATT=1", 7000000);  /* Max wait for Attach: 7s */
    if (res != 0) {
        DEBUG("[ATMODEM] AT+CGATT=1 ERROR\n");
        sim_status.is_internet = false;
        return false;
    }

    /* Get Home Network Identity is the combination of the MCC and the MNC */
    res = at_send_cmd_get_resp(&at_dev, "AT+CIMI", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
    if (res > 0) {
        DEBUG("[ATMODEM] IMSI: %s\n", at_dev_resp);
        char hni[6];
        snprintf(hni, 6, "%s", at_dev_resp);
        sim_status.hni = atoi(hni);
        DEBUG("[ATMODEM] HNI: %ld\n", sim_status.hni);

        /* Search for Internet settings */
        if(!atmodem_get_internet_settings()) { 
            /* No internet settings found */
            DEBUG("[ATMODEM] No internet settings found\n");
            sim_status.is_internet = false;
            return false;
        }
    } else {
        /* IMSI number not received */
        DEBUG("[ATMODEM] IMSI number not received\n");
        sim_status.is_internet = false;
        return false;
    }
    
    /* Get internet settings */
    char cmd_with_settings_for_internet[128];
    snprintf(cmd_with_settings_for_internet, 128, "AT+CSTT=\"%s\",\"%s\",\"%s\"", sim_status.apn, 
                                                                                  sim_status.username, 
                                                                                  sim_status.password);
    
    /* Print internet settings */
    DEBUG("[ATMODEM] Internet settings: APN=\"%s\", Username=\"%s\", Password=\"%s\"\n", sim_status.apn, 
                                                                                        sim_status.username, 
                                                                                        sim_status.password);

    /* Start task and set the APN */
    res = at_send_cmd_wait_ok(&at_dev, cmd_with_settings_for_internet, ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] %s ERROR\n", cmd_with_settings_for_internet);
        sim_status.is_internet = false;
        return false;
    }

    /* Bring up the wireless connection with wait for bringup */
    for(uint8_t i = 0; i < 5; i++) {
        res = at_send_cmd_wait_ok(&at_dev, "AT+CIICR", 6000000);    /* Max wait for bringup: 6s */ 

        if (res == 0) {
            break;
        }
        if (i == 4) {
            DEBUG("[ATMODEM] AT+CIICR ERROR\n");
            sim_status.is_internet = false;
            return false;
        }

        DEBUG("[ATMODEM] AT+CIICR try %i \n", i+2);

        /* 500 ms delay */
        rtctimers_millis_sleep(500);
    }

    /* Get local IP address */
    res = at_send_cmd_get_resp(&at_dev, "AT+CIFSR", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
    if (res > 0) {
        DEBUG("[ATMODEM] Local IP address: %s\n", at_dev_resp);
    } else {
        /*  */
        DEBUG("[ATMODEM] AT+CIFSR %i ERROR\n", res);
        sim_status.is_internet = false;
        return false;
    }

    /* Test internet */
    /* Ping 3 server for test internet connection */
    if (atmodem_ping(test_ping_google_server)) {
        sim_status.is_internet = true;
    } else if (atmodem_ping(test_ping_own_server)) {
        sim_status.is_internet = true;
    } else if (atmodem_ping(test_ping_k_root_dns_server)) {
        sim_status.is_internet = true;
    } else {
        sim_status.is_internet = false;
        DEBUG("[ATMODEM] Internet ERROR\n");
        return false;
    }
    
    DEBUG("[ATMODEM] Internet OK\n");
    return true;
}

/*---------------------------------------------------------------------------*/
/* Processing incoming SMS commands */
void atmodem_get_sms_command(void) {
    /* Is the modem initialized? */
    if (sim_status.modem < 0) 
        return;

    DEBUG("[ATMODEM] Get SMS command\n");
    int res;
    
    /* Select SMS Message Format: Text mode */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CMGF=1", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+CMGF=1 ERROR\n");
        return;
    } 

    /* Select TE Character Set: GSM 7 bit default alphabet (3GPP TS 23.038) */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CSCS=\"GSM\"", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+CSCS=\"GSM\" ERROR\n");
        return;
    } 

    /* Requesting unread messages */
    res = at_send_cmd(&at_dev, "AT+CMGL=\"ALL\"", ATMODEM_MAX_TIMEOUT); /* TODO: Replace "AT+CMGL=\"ALL\"" on "AT+CMGL=\"REC UNREAD\"" */
    if (res != 0) {
        DEBUG("[ATMODEM] AT+CMGL=\"ALL\" ERROR\n");
        return;
    }

    /* Find unread SMS */
    do {
        res = at_readline(&at_dev, at_dev_resp, ATMODEM_DEV_RESP_SIZE, false, ATMODEM_MAX_TIMEOUT);
        if (memcmp(at_dev_resp, "+CMGL:", 6) == 0) {
            res = at_readline(&at_dev, at_dev_resp, ATMODEM_DEV_RESP_SIZE, false, ATMODEM_MAX_TIMEOUT);
            if(res >= 0) {
                if((memcmp(at_dev_resp, "cmd", 3) == 0)) {
                    DEBUG("[ATMODEM] SMS with cmd: %s\n", at_dev_resp);

                    char sms_cmd[20];
                    sscanf(at_dev_resp, "%*s %s", sms_cmd); 
                    DEBUG("[ATMODEM] SMS command: %s\n", sms_cmd);

                    if (strcmp(sms_cmd, "internet") == 0) {
                        /* Fucking parsing */
                        sscanf(at_dev_resp, "%*s %*s %s %s %s", sim_status.apn, sim_status.username, sim_status.password); 
                        sscanf(sim_status.apn, "\"%[^\"]s\"", sim_status.apn); 
                        sscanf(sim_status.username, "\"%[^\"]s\"", sim_status.username); 
                        sscanf(sim_status.password, "\"%[^\"]s\"", sim_status.password); 

                        if (strcmp(sim_status.apn, "\"\"") == 0) {
                            memset(sim_status.apn, 0x00, 32);         
                        }

                        if (strcmp(sim_status.username, "\"\"") == 0) {
                            memset(sim_status.username, 0x00, 32);         
                        }

                        if (strcmp(sim_status.password, "\"\"") == 0) {
                            memset(sim_status.password, 0x00, 32);         
                        }

                        sim_status.is_new_internet_settings = true;

                        DEBUG("[ATMODEM] New APN: %s\n", sim_status.apn);
                        DEBUG("[ATMODEM] New username: %s\n", sim_status.username);
                        DEBUG("[ATMODEM] New password: %s\n", sim_status.password);

                        DEBUG("[ATMODEM] New internet settings\n");
                    } else {
                        DEBUG("[ATMODEM] Unknown command\n");
                    }
                } else {
                    DEBUG("[ATMODEM] SMS without cmd: %s\n", at_dev_resp); 
                }
            }
        }
    } while (res >= 0);

    /* Delete all messages from preferred message storage including unread messages */
    res = at_send_cmd_wait_ok(&at_dev, "AT+CMGD=1,4", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+CMGD=1,4 ERROR\n");
        return;
    } 
}

/*---------------------------------------------------------------------------*/
/* Power on for modem */
void atmodem_power_on(void) {
    DEBUG("[ATMODEM] Power on\n");

    /* ATMODEM_POWER_ENABLE to Hi */
    gpio_init(atmodem_power_pin, GPIO_OUT);
    gpio_set(atmodem_power_pin);

    /* 3G_PWR to Hi on 100 ms*/
    gpio_init(atmodem_enable_pin, GPIO_OUT);
    gpio_set(atmodem_enable_pin);

    /* 500ms sleep and clear */
    rtctimers_millis_sleep(ATMODEM_TIME_ON);
    gpio_clear(atmodem_enable_pin);
}

/*---------------------------------------------------------------------------*/
/* Power off for modem */
void atmodem_power_off(void) {
    /* Power off UART for modem */
    uart_poweroff(at_dev.uart);

    /* Power pin to Low */
    gpio_init(atmodem_power_pin, GPIO_OUT);
    gpio_clear(atmodem_power_pin);

    /* Modem is not initialized */
    sim_status.modem = ATMODEM_NOT_INITIALIZED;
    DEBUG("[ATMODEM] Power off\n");
}

/*---------------------------------------------------------------------------*/
/* Modem initialization */
bool atmodem_init(uart_t modem_uart, gpio_t power_pin, gpio_t enable_pin) {
    DEBUG("[ATMODEM] Initialization\n");

    at_dev.uart = modem_uart;
    atmodem_enable_pin = enable_pin;
    atmodem_power_pin = power_pin;

    /* Modem power on */
    atmodem_power_on();
    
    /* We wait while modem is initialized */
    rtctimers_millis_sleep(ATMODEM_TIME_ON_UART);

    int res = at_dev_init(&at_dev, modem_uart, ATMODEM_BAUDRATE, at_dev_buf, ATMODEM_DEV_RESP_SIZE);
    
    sim_status.modem = ATMODEM_READY;
    
    if (!res) {
        for(int i = 0; i < 5; i++) {
            res = at_send_cmd_wait_ok(&at_dev, "AT", ATMODEM_MAX_TIMEOUT);
            if (res >= 0) {
                DEBUG("[ATMODEM] Init OK\n");
                break;
            }
            if (i == 4) {
                /* If there is no modem, then sim_status.modem = ATMODEM_NOT_ANSWERING and display N/A on the screen */
                sim_status.modem = ATMODEM_NOT_ANSWERING;

                DEBUG("[ATMODEM] Not answering\n");
                return false;
            }

            rtctimers_millis_sleep(500);
        }

        return true;
    }
    else {
        DEBUG("[ATMODEM] Init ERROR: %i\n", res);

        /* If there is no modem, then sim_status.modem = ATMODEM_NOT_ANSWERING and display N/A on the screen */
        sim_status.modem = ATMODEM_NOT_ANSWERING;

        return false;
    }
}

/*---------------------------------------------------------------------------*/
/*  */
int8_t atmodem_wait_sms_with_cmd(void) {
    uint16_t counter = 0; 
    while (!sim_status.is_new_internet_settings) {
        if (counter < ATMODEM_WAIT_SMS_TIMEOUT) {
            atmodem_get_sms_command();
            counter++;
            rtctimers_millis_sleep(1000);
        } else {
            DEBUG("[ATMODEM] Waiting for SMS with internet settings timeout expired\n");
            return ATMODEM_WAITING_SMS_INTERNET_SETTINGS_TIMEOUT;
        }
    }
    
    return ATMODEM_ERROR_NEW_INTERNET_SETTINGS;
}

/*---------------------------------------------------------------------------*/
/* Sending data via Modem (HTTP GET) */
bool atmodem_send_httpget(uint8_t *data_for_send, size_t data_size) {
    /* Check whether bearer 1 is open */
    int res = at_send_cmd_get_resp(&at_dev, "AT+SAPBR=2,1", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
    if (res <= 0) {
        DEBUG("[ATMODEM] AT+SAPBR=2,1 ERROR: %i\n", res);
        return false;
    }
    // DEBUG("[ATMODEM] Resp on AT+SAPBR=2,1: %s\n", at_dev_resp); // +SAPBR: 1,3,"0.0.0.0"

    /* Enable bearer 1 */
    for(uint8_t i = 0; i < 5; i++) {
        res = at_send_cmd_wait_ok(&at_dev, "AT+SAPBR=1,1", 6000000); /* Wait untial bearer is activated */

        if (res == 0) {
            break;
        }
        if (i == 4) {
            DEBUG("[ATMODEM] AT+SAPBR=1,1 ERROR\n");
            return false;
        }

        DEBUG("[ATMODEM] AT+SAPBR=1,1 try %i \n", i+2);

        /* 500 ms delay */
        rtctimers_millis_sleep(500);
    }

    /* Initialize HTTP service */
    res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPINIT", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+HTTPINIT ERROR\n");
        return false;
    } 

    /* Preparing data for transmission */
    /* Every 3 source bytes are encoded with 4 characters (an increase of 1/3) + '\0' */
    const uint32_t base64_out_len_max = ((data_size / 3) * 4) + (data_size / 10) + 10;
    size_t base64_out_len = base64_out_len_max;
    uint8_t base64_out[base64_out_len_max]; 

    /* Encode base64 */
    int ret = base64_encode((uint8_t*)data_for_send, data_size, base64_out, &base64_out_len);
    if (ret != BASE64_SUCCESS) {
        DEBUG("BASE64: %i ERROR!!!\n", ret);
        return false;
    }

    /* Past '\0' for end string */
    base64_out[base64_out_len] = '\0';

    /* Creation of command with data */
    char cmd_with_url[128];
    snprintf(cmd_with_url, 128, "AT+HTTPPARA=\"URL\",\"http://%s/?&data=%s\"", server, base64_out);

    DEBUG("[ATMODEM] CMD with URL: %s\n", cmd_with_url); 

    /* Set the HTTP URL */
    res = at_send_cmd_wait_ok(&at_dev, cmd_with_url, ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+HTTPPARA ERROR\n");
        return false;
    } 

    /* Set the context ID */
    res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPPARA=\"CID\",1", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+HTTPPARA ERROR\n");
        return false;
    } 

    /* Try 5 */
    uint8_t tries = 5;
    
    for(uint8_t i = 0; i < tries; i++) {
        /* Set up the HTTP action */
        res = at_send_cmd_get_resp(&at_dev, "AT+HTTPACTION=0", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
        if (res <= 0) {
            DEBUG("[ATMODEM] AT+HTTPACTION=0 ERROR: %i\n", res);
            DEBUG("[ATMODEM] Try: %i of %i\n", i+2, tries);
            continue;
        }

        /* Waiting for the message to go */
        rtctimers_millis_sleep(6000);  /* 6s = 2% error */

        /* Terminate the HTTP service */
        res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPTERM", ATMODEM_MAX_TIMEOUT);
        if (res != 0) {
            DEBUG("[ATMODEM] AT+HTTPTERM ERROR\n");
            DEBUG("[ATMODEM] Try: %i of %i\n", i+2, tries);
            continue;
        } else {
            /* Data sent successfully */
            break;
        }

        if (i == 4) {
            DEBUG("[ATMODEM] Data sent ERROR\n");
            sim_status.is_server_connect = false;

            return false;
        }

        rtctimers_millis_sleep(500);
    }

    DEBUG("[ATMODEM] Data sent successfully\n");
    sim_status.is_server_connect = true;

    return true;
}

/*---------------------------------------------------------------------------*/
/* Sending data via Modem (HTTP POST) */
bool atmodem_send_httppost(char *url, uint8_t *data_for_send, size_t data_size) {
    /* Reset flag */
    sim_status.is_server_connect = false;

    /* Check whether bearer 1 is open */
    int res = at_send_cmd_get_resp(&at_dev, "AT+SAPBR=2,1", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
    if (res <= 0) {
        DEBUG("[ATMODEM] AT+SAPBR=2,1 ERROR: %i\n", res);
        return false;
    }
    // DEBUG("[ATMODEM] Resp on AT+SAPBR=2,1: %s\n", at_dev_resp); // +SAPBR: 1,3,"0.0.0.0"

    /* Enable bearer 1 */
    for(uint8_t i = 0; i < 5; i++) {
        res = at_send_cmd_wait_ok(&at_dev, "AT+SAPBR=1,1", 6000000); /* Wait untial bearer is activated */

        if (res == 0) {
            break;
        }
        if (i == 4) {
            DEBUG("[ATMODEM] AT+SAPBR=1,1 ERROR\n");
            return false;
        }

        DEBUG("[ATMODEM] AT+SAPBR=1,1 try %i \n", i+2);

        /* 500 ms delay */
        rtctimers_millis_sleep(500);
    }

    /* Initialize HTTP service */
    res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPINIT", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+HTTPINIT ERROR\n");
        return false;
    } 

    /* Set the context ID */
    res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPPARA=\"CID\",1", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+HTTPPARA ERROR\n");
        return false;
    } 

    /* Creation of command with URL */
    char cmd_with_url[128];
    snprintf(cmd_with_url, 128, "AT+HTTPPARA=\"URL\",\"%s\"", url);
    DEBUG("[ATMODEM] CMD with URL: %s\n", cmd_with_url); 

    /* Set the CONTENT */
    res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPPARA=\"CONTENT\",\"text/plain\"", ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] AT+HTTPPARA=\"CONTENT\",\"text/plain\" ERROR\n");
        return false;
    } 

    /* Set the HTTP URL */
    res = at_send_cmd_wait_ok(&at_dev, cmd_with_url, ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] %s ERROR\n", cmd_with_url);
        return false;
    } 

    /* Creation of command with data len */
    char cmd_http_data[30];
    snprintf(cmd_http_data, 30, "AT+HTTPDATA=%i,10000", data_size); /* Timeout 10s */

    /* Send command with data len */
    res = at_send_cmd(&at_dev, cmd_http_data, ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] %s ERROR\n", cmd_http_data);
        return false;
    } 

    /* Load data in modem */
    DEBUG("[ATMODEM] Send data:\n");
    od_hex_dump(data_for_send, data_size, OD_WIDTH_DEFAULT);
    at_send_bytes(&at_dev, (char*)data_for_send, data_size);
    // at_send_bytes(&at_dev, AT_SEND_EOL, 1);  // Command may not be sent

    rtctimers_millis_sleep(1000);

    //AT+HTTPACTION=1
    // res = at_send_cmd_get_resp(&at_dev, "AT+HTTPACTION=1", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
    // if (res <= 0) {
    //     DEBUG("[ATMODEM] AT+HTTPACTION=0 ERROR: %i\n", res);
    //     return false;
    // }

    // rtctimers_millis_sleep(7000);

    // do {
    //     res = at_readline(&at_dev, at_dev_resp, ATMODEM_DEV_RESP_SIZE, false, ATMODEM_MAX_TIMEOUT);
    //     if (res >= 0) {
    //         // DEBUG("res: %i, line %i: %s\n", res, i, at_dev_resp);
    //         if (memcmp(at_dev_resp, "+HTTPACTION:", 12) == 0) {
    //             DEBUG("[ATMODEM] %s\n", at_dev_resp);
    //             break;
    //         } 
    //     }
    // } while(res >= 0);

    /* Try 5 */
    uint8_t tries = 5;
    
    uint16_t status_code; 

    for(uint8_t i = 0; i < tries; i++) {
        /* Set up the HTTP action */
        res = at_send_cmd_get_resp(&at_dev, "AT+HTTPACTION=1", at_dev_resp, ATMODEM_DEV_RESP_SIZE, ATMODEM_MAX_TIMEOUT);
        if (res <= 0) {
            DEBUG("[ATMODEM] AT+HTTPACTION=1 ERROR: %i\n", res);
            DEBUG("[ATMODEM] Try: %i of %i\n", i+2, tries);
            continue;
        }

        /* Waiting for the message to go */
        rtctimers_millis_sleep(6000);  /* 6s = 2% error */

        do {
            res = at_readline(&at_dev, at_dev_resp, ATMODEM_DEV_RESP_SIZE, false, ATMODEM_MAX_TIMEOUT);
            if (res >= 0) {
                // DEBUG("res: %i, line %i: %s\n", res, i, at_dev_resp);
                if (memcmp(at_dev_resp, "+HTTPACTION:", 12) == 0) {
                    // DEBUG("[ATMODEM] %s\n", at_dev_resp);
                    break;
                } 
            }
        } while(res >= 0);

        if (res < 0) {
            /* No answer (+HTTPACTION) */
            DEBUG("[ATMODEM] No answer (+HTTPACTION) ERROR\n");
            DEBUG("[ATMODEM] Try: %i of %i\n", i+2, tries);
            continue;
        }

        /* Test http code 200 (OK) */
        DEBUG("[ATMODEM] %s\n", at_dev_resp);
        res = sscanf(at_dev_resp,"%*s %*i,%hi,", &status_code); //+HTTPACTION: 1,200,79
        if (res != 1) {
            DEBUG("[ATMODEM] ERROR: %i arg, status_code: %i\n", res, status_code);
            return false;
        }

        if (status_code != 200) {
            DEBUG("[ATMODEM] HTTP Status Codes: %i ERROR\n", status_code);
            return false;
        }

        //AT+HTTPREAD
        res = at_send_cmd_get_lines(&at_dev, "AT+HTTPREAD", at_dev_resp, ATMODEM_DEV_RESP_SIZE, true, ATMODEM_MAX_TIMEOUT);
        if (res < 0) {
            DEBUG("[ATMODEM] AT+HTTPREAD ERROR: %i\n", res);
            DEBUG("[ATMODEM] Try: %i of %i\n", i+2, tries);
            continue;
        }
        DEBUG("[ATMODEM] AT+HTTPREAD resp: \n%s\n", at_dev_resp);

        // /* Terminate the HTTP service */
        // res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPTERM", ATMODEM_MAX_TIMEOUT);
        // if (res != 0) {
        //     DEBUG("[ATMODEM] AT+HTTPTERM ERROR\n");
        //     return false;
        // }
        
        /* Terminate the HTTP service */
        res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPTERM", ATMODEM_MAX_TIMEOUT);
        if (res != 0) {
            DEBUG("[ATMODEM] AT+HTTPTERM ERROR\n");
            DEBUG("[ATMODEM] Try: %i of %i\n", i+2, tries);
            continue;
        } else {
            /* Data sent successfully */
            break;
        }

        if (i == 4) {
            DEBUG("[ATMODEM] Data sent ERROR\n");
            sim_status.is_server_connect = false;

            return false;
        }

        rtctimers_millis_sleep(500);
    }

    // //AT+HTTPREAD
    // res = at_send_cmd_get_lines(&at_dev, "AT+HTTPREAD", at_dev_resp, ATMODEM_DEV_RESP_SIZE, true, ATMODEM_MAX_TIMEOUT);
    // if (res < 0) {
    //     DEBUG("[ATMODEM] AT+HTTPREAD ERROR: %i\n", res);
    //     return false;
    // }
    // DEBUG("[ATMODEM] AT+HTTPREAD resp: \n%s\n", at_dev_resp);

    // /* Terminate the HTTP service */
    // res = at_send_cmd_wait_ok(&at_dev, "AT+HTTPTERM", ATMODEM_MAX_TIMEOUT);
    // if (res != 0) {
    //     DEBUG("[ATMODEM] AT+HTTPTERM ERROR\n");
    //     return false;
    // }

    DEBUG("[ATMODEM] Data sent successfully\n");
    sim_status.is_server_connect = true;

    return true;
}

/*---------------------------------------------------------------------------*/
/* Sending data via Modem (TCP or UDP) */
bool atmodem_send_tcpudp(char    *server_addr, 
                         char    *server_port, 
                         char    *connection_type,
                         uint8_t *data_for_send, 
                         size_t   data_size) {
    /* Start a TCP connection to remote address. Port 80 is TCP */
    char cmd_CIPSTART[60];
    snprintf(cmd_CIPSTART, 60, "AT+CIPSTART=\"%s\",\"%s\",\"%s\"", connection_type, server_addr, server_port);

    int res = at_send_cmd_wait_ok(&at_dev, cmd_CIPSTART, ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] %s ERROR: %i\n", cmd_CIPSTART, res);
        return false;
    } 
    DEBUG("[ATMODEM] %s OK\n", cmd_CIPSTART);

    /* Wait 5 sec for start TCP connection */
    rtctimers_millis_sleep(5000);

    /* CMD with lenth data for send (AT+CIPSEND=data_size) */
    char cmd_CIPSEND[20];
    snprintf(cmd_CIPSEND, 20, "AT+CIPSEND=%i", data_size);
    at_send_cmd(&at_dev, cmd_CIPSEND, ATMODEM_MAX_TIMEOUT);
    // res = at_send_cmd_wait_prompt(&at_dev, cmd_CIPSEND, ATMODEM_MAX_TIMEOUT);
    if (res != 0) {
        DEBUG("[ATMODEM] %s ERROR: %i\n", cmd_CIPSEND, res);
        return false;
    } 
    DEBUG("[ATMODEM] %s OK\n", cmd_CIPSEND);
    rtctimers_millis_sleep(ATMODEM_CMD_TIMEOUT);

    /* Send data */
    DEBUG("[ATMODEM] Send data:\n");
    od_hex_dump(data_for_send, data_size, OD_WIDTH_DEFAULT);
    at_send_bytes(&at_dev, (char*)data_for_send, data_size);
    rtctimers_millis_sleep(2000);

    /* Close connection */
    at_send_cmd(&at_dev, "AT+CIPCLOSE=0", ATMODEM_MAX_TIMEOUT);
    rtctimers_millis_sleep(ATMODEM_CMD_TIMEOUT);
    DEBUG("[ATMODEM] Close TCP or UDP Connection OK\n");

    // res = at_readline(&at_dev, at_dev_resp, ATMODEM_DEV_RESP_SIZE, false, ATMODEM_MAX_TIMEOUT);
    // if (res < 0) {
    //     DEBUG("[ATMODEM] Close TCP or UDP Connection ERROR, no answer\n");
    //     return false;
    // }

    // DEBUG("res: %i, %s\n", res, at_dev_resp);
    // if (memcmp(at_dev_resp, "CLOSE OK", 8) != 0) {
    //     DEBUG("[ATMODEM] Close TCP or UDP Connection ERROR\n");
    //     // return false;
    // } 

    // do {
    //     res = at_readline(&at_dev, at_dev_resp, ATMODEM_DEV_RESP_SIZE, false, ATMODEM_MAX_TIMEOUT);
    //     if (res >= 0) {
    //         DEBUG("res: %i, line %i: %s\n", res, i, at_dev_resp);
    //         if (memcmp(at_dev_resp, "+HTTPACTION:", 12) == 0) {
    //             DEBUG("[ATMODEM] %s\n", at_dev_resp);


    //             break;
    //         } 
    //     }
    // } while(res >= 0);

    
    DEBUG("[ATMODEM] Data sent successfully\n");
    sim_status.is_server_connect = true;

    return true;
}