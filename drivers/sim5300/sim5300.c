/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    SIM5300 driver 
 * @ingroup     drivers
 * @brief       SIM5300 driver 
 *
 * 
 *
 *
 * 
 * 
 * @{
 *
 * @file
 *
 * @brief       SIM5300 driver 
 * @author      Oleg Manchenko <man4enkoos@gmail.com>
 */

// #include <errno.h>
// #include <string.h>

// #include "fmt.h"
// #include "isrpipe.h"
// #include "periph/uart.h"
// #include "xtimer.h"

#include "stdio.h"
#include "string.h"

#include "periph/gpio.h"
#include "periph/uart.h"
#include "byteorder.h"

#include "sim5300.h"

#include "rtctimers-millis.h"


#define ENABLE_DEBUG (1)
#include "debug.h"

#define SIM5300_MAX_TIMEOUT         (1000000)   /* Maximum time waiting for a response */ 
// #define TIME_ON_CHANGE_BAUDRATE (100)       /* Time on change baudrate */


/*---------------------------------------------------------------------------*/
/* AT – ATtention Code */
bool sim5300_send_at(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 
        
    puts("[SIM5300] Send AT");

    /* Send AT */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, "AT", SIM5300_MAX_TIMEOUT);
    if (res == 0) {
        return true;
    } else {
        return false;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CSMINS SIM Inserted Status Reporting */
bool sim5300_get_sim_inserted_status_reporting(sim5300_dev_t         *sim5300_dev,
                                               sim5300_csmins_resp_t *sim5300_csmins_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 

    /* NULL ptr */
    if (sim5300_csmins_resp == NULL) {
        return false;
    }

    /* Get SIM Inserted Status Reporting */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CSMINS?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CSMINS? ERROR");

        return false;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Parse string */
    res = sscanf(sim5300_dev->at_dev_resp, "+CSMINS: %i,%i", &sim5300_csmins_resp->n,
                                                             &sim5300_csmins_resp->sim_inserted);

    /* Check result */
    if (res != 2) {
        puts("[SIM5300] Parse error");

        return false;
    }

    /* Debug output */
    DEBUG("n = %i\n",            sim5300_csmins_resp->n);
    DEBUG("sim_inserted = %i\n", sim5300_csmins_resp->sim_inserted);

    return true;    
}

/*---------------------------------------------------------------------------*/
/* AT+CSMINS SIM Inserted Status Reporting */
bool sim5300_set_sim_inserted_status_reporting(sim5300_dev_t *sim5300_dev, 
                                               uint8_t        n) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 
    
    /* Test range argument */
    if (n > 1) {
        printf("[SIM5300] sim5300_set_sim_inserted_status_reporting() ERROR argument: %i. (range 0-1)\n", n);

        return false;
    }

    /* Create a command to send data */
    char cmd_CSMINSn[12];
    snprintf(cmd_CSMINSn, 12, "AT+CSMINS=%i", n);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, cmd_CSMINSn, SIM5300_MAX_TIMEOUT);

    /* Return result */
    if (res == 0) {
        /* Print result */
        if (n == 0) {
            puts("[SIM5300] Disabled showing an unsolicited event code");
        } else {
            puts("[SIM5300] Enabled showing an unsolicited event code ");
        }

        return true;
    } else {
        puts("[SIM5300] sim5300_set_sim_inserted_status_reporting() ERROR");

        return false;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CPIN Enter PIN */
int8_t sim5300_get_pin_status(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return -1;
    } 

    /* Get alphanumeric string indicating whether some password is required or not. */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CPIN?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CPIN? ERROR");

        return -2;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Parse string */
    sim5300_cpin_resp_t sim5300_cpin_resp;
    if(strcmp("+CPIN: READY", sim5300_dev->at_dev_resp) == 0) {
        puts("[SIM5300] MT is not pending for any password");
        sim5300_cpin_resp = READY;

    } else if (strcmp("+CPIN: SIM PIN", sim5300_dev->at_dev_resp) == 0) {
        puts("[SIM5300] MT is waiting SIM PIN to be given");
        sim5300_cpin_resp = SIM_PIN;

    } else if (strcmp("+CPIN: SIM PUK", sim5300_dev->at_dev_resp) == 0) {
        puts("[SIM5300] MT is waiting for SIM PUK to be given");
        sim5300_cpin_resp = SIM_PUK;

    } else if (strcmp("+CPIN: PH_SIM PIN", sim5300_dev->at_dev_resp) == 0) {
        puts("[SIM5300] ME is waiting for phone to SIM card (antitheft)");
        sim5300_cpin_resp = PH_SIM_PIN;

    } else if (strcmp("+CPIN: PH_SIM PUK", sim5300_dev->at_dev_resp) == 0) {
        puts("[SIM5300] ME is waiting for SIM PUK (antitheft)");
        sim5300_cpin_resp = PH_SIM_PUK;

    } else if (strcmp("+CPIN: SIM PIN2", sim5300_dev->at_dev_resp) == 0) {
        puts("[SIM5300] SIM PIN2");
        sim5300_cpin_resp = SIM_PIN2;

    } else if (strcmp("+CPIN: SIM PUK2", sim5300_dev->at_dev_resp) == 0) {
        puts("[SIM5300] SIM PUK2");
        sim5300_cpin_resp = SIM_PUK2;

    } else if (strcmp("ERROR", sim5300_dev->at_dev_resp) == 0) {
        puts("[SIM5300] SIM ERROR (may not be installed SIM card)");

        return -3;
    } else {
        puts("[SIM5300] Unknown response");

        return -4;
    }

    return sim5300_cpin_resp;
}

/*---------------------------------------------------------------------------*/
/* AT+CREG Network Registration */
bool sim5300_get_network_registration(sim5300_dev_t       *sim5300_dev,
                                      sim5300_creg_resp_t *sim5300_creg_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 

    /* NULL ptr */
    if (sim5300_creg_resp == NULL) {
        return false;
    }

    /* Get Network Registration */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CREG?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CREG? ERROR");

        return false;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Parse string */
    res = sscanf(sim5300_dev->at_dev_resp, "+CREG: %i,%i", &sim5300_creg_resp->n,
                                                           &sim5300_creg_resp->stat);

    /* Check result */
    if (res != 2) {
        puts("[SIM5300] Parse error");

        return false;
    }

    /* Debug output */
    DEBUG("n = %i\n",    sim5300_creg_resp->n);
    DEBUG("stat = %i\n", sim5300_creg_resp->stat);

    return true;    
}

/*---------------------------------------------------------------------------*/
/* AT+GSMBUSY Reject Incoming Call */
int8_t sim5300_get_reject_incoming_call(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return -1;
    } 
    
    /* Send AT command */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+GSMBUSY?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT);
    
    /* Check return code */
    if (res <= 0) {
        puts("[SIM5300] sim5300_get_reject_incoming_call() ERROR: -2");

        return -2;
    }

    /* Parse string */
    int mode;
    res = sscanf(sim5300_dev->at_dev_resp, "+GSMBUSY: %i", &mode);

    /* Check result */
    if (res != 1) {
        puts("[SIM5300] Parse error");

        return -3;
    }

    /* Print result */
    switch (mode) {
        case 0:
            puts("[SIM5300] Enable incoming call");
            break;
        case 1:
            puts("[SIM5300] Forbid all incoming calls");
            break;
        case 2:
            puts("[SIM5300] Forbid incoming voice calls but enable CSD calls");
            break;
        default:
            puts("[SIM5300] Unknow mode");
            
            return -4;
    }

    return mode;
}

/*---------------------------------------------------------------------------*/
/* AT+GSMBUSY Reject Incoming Call */
bool sim5300_set_reject_incoming_call(sim5300_dev_t *sim5300_dev, 
                                      uint8_t        mode) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 
    
    /* Test range argument */
    if (mode > 2) {
        printf("[SIM5300] sim5300_set_reject_incoming_call() ERROR argument: %i. (range 0-2)\n", mode);

        return false;
    }

    /* Create a command to send data */
    char cmd_GSMBUSYn[13];
    snprintf(cmd_GSMBUSYn, 13, "AT+GSMBUSY=%i", mode);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, cmd_GSMBUSYn, SIM5300_MAX_TIMEOUT);

    /* Return result */
    if (res == 0) {
        /* Print result */
        if (mode == 0) {
            puts("[SIM5300] Enable incoming call");
        } else if (mode == 1) {
            puts("[SIM5300] Forbid all incoming calls");
        } else {
            puts("[SIM5300] Forbid incoming voice calls but enable CSD calls");
        }

        return true;
    } else {
        puts("[SIM5300] sim5300_set_reject_incoming_call() ERROR");

        return false;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CSQ Signal Quality Report */
bool sim5300_get_signal_quality_report(sim5300_dev_t      *sim5300_dev,
                                       sim5300_csq_resp_t *sim5300_csq_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 

    /* NULL ptr */
    if (sim5300_csq_resp == NULL) {
        return false;
    }

    /* Get Network Registration */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CSQ", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CSQ ERROR");

        return false;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Parse string */
    int rssi;
    res = sscanf(sim5300_dev->at_dev_resp, "+CSQ: %i,%i", &rssi,
                                                          &sim5300_csq_resp->ber);

    /* Check result */
    if (res != 2) {
        puts("[SIM5300] Parse error");

        return false;
    }

    if (rssi == 0) {
        sim5300_csq_resp->rssi = 115;
    } else if (rssi == 1) {
        sim5300_csq_resp->rssi = 111;
    } else if ((rssi > 1) && (rssi <= 30)) {
        sim5300_csq_resp->rssi = 110 - ((rssi - 2) * 2);
    } else if (rssi == 31) {
        sim5300_csq_resp->rssi = 52;
    } else 
        sim5300_csq_resp->rssi = 115; 

    /* Debug output */
    DEBUG("rssi = %i\n", sim5300_csq_resp->rssi);
    DEBUG("ber = %i\n",  sim5300_csq_resp->ber);

    return true; 
}   

/*---------------------------------------------------------------------------*/
/* AT+COPS Operator Selection */
bool sim5300_get_operator_selection(sim5300_dev_t       *sim5300_dev,
                                    sim5300_cops_resp_t *sim5300_cops_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 

    /* NULL ptr */
    if (sim5300_cops_resp == NULL) {
        return false;
    }

    /* Get SIM Inserted Status Reporting */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+COPS?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+COPS? ERROR");

        return false;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Parse string */ 
    res = sscanf(sim5300_dev->at_dev_resp, "+COPS: %i,%i,\"%[^\"]s\"", &sim5300_cops_resp->mode,
                                                                       &sim5300_cops_resp->format,
                                                                        sim5300_cops_resp->oper);

    /* Check result */
    if (res == 1) {
        /* Debug output */
        DEBUG("mode = %i\n", sim5300_cops_resp->mode);

        return true;
    } else if (res == 3) {
        DEBUG("mode = %i\n", sim5300_cops_resp->mode);
        DEBUG("format = %i\n", sim5300_cops_resp->format);
        DEBUG("oper = %s\n", sim5300_cops_resp->oper);
        // DEBUG("act = %i\n", sim5300_cops_resp->act);

        return true;
    } else {
        puts("[SIM5300] Parse error");

        return false;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CGACT PDP Context Activate or Deactivate */
bool sim5300_set_state_pdp_context(sim5300_dev_t *sim5300_dev,
                                   uint8_t        state,
                                   uint8_t        cid) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 
    
    /* Test range argument */
    if (state > 1) {
        printf("[SIM5300] sim5300_set_sim_inserted_status_reporting() ERROR argument: %i. (range 0-1)\n", state);

        return false;
    }

    /* Create a command to send data */
    char cmd_CGACTn[20];
    if (cid == 0) {
        snprintf(cmd_CGACTn, 20, "AT+CGACT=%i", state);
    } else {
        snprintf(cmd_CGACTn, 20, "AT+CGACT=%i,%i", state, cid); 
    }
    
    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, cmd_CGACTn, 7000000); 

    /* Return result */
    if (res == 0) {
        /* Print result */
        if (state == 0) {
            puts("[SIM5300] Deactivated PDP context");
        } else {
            puts("[SIM5300] Activated PDP context");
        }

        return true;
    } else {
        puts("[SIM5300] sim5300_set_state_pdp_context() ERROR");

        return false;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CIMI Request International Mobile Subscriber Identity */
char *sim5300_get_imsi(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return NULL;
    } 

    /* Send AT command */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CIMI", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT);
    
    /* Return result */
    if (res > 0) {
        printf("[SIM5300] IMSI: %s\n", sim5300_dev->at_dev_resp);

        return sim5300_dev->at_dev_resp;
    } else {
        puts("[SIM5300] sim5300_get_imsi() ERROR");
        return NULL;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CSTT Start Task and Set APN, USER NAME, PASSWORD */
bool sim5300_set_network_settings(sim5300_dev_t *sim5300_dev,
                                  char          *apn,
                                  char          *user,
                                  char          *password) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 

    /* Test arguments */
    if ((apn == NULL) || (user == NULL) || (password == NULL)) {
        puts("Arguments = NULL");
        return false;
    } 

    /* Create a command to send data */
    char cmd_with_settings_for_internet[128];
    snprintf(cmd_with_settings_for_internet, 128, "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, 
                                                                                  user, 
                                                                                  password);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, cmd_with_settings_for_internet, SIM5300_MAX_TIMEOUT);

    /* Return result */
    if (res == 0) {
        /* Print internet settings */
        printf("[SIM5300] Set internet settings: APN=\"%s\", Username=\"%s\", Password=\"%s\"\n", apn, 
                                                                                                  user, 
                                                                                                  password);

        return true;
    } else {
        puts("[SIM5300] sim5300_set_network_settings() ERROR");

        return false;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CIICR Bring Up Wireless Connection with GPRS */
bool sim5300_bring_up_wireless_connection(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, "AT+CIICR", 6000000);

    /* Return result */
    if (res == 0) {
        /* Print result */
        puts("[SIM5300] Bring Up Wireless Connection with GPRS");

        return true;
    } else {
        puts("[SIM5300] sim5300_bring_up_wireless_connection() ERROR");

        return false;
    }
}

/*---------------------------------------------------------------------------*/
/* Communication test between microcontroller and SIM5300 */
bool sim5300_communication_test(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    }     

    /* SIM5300 connection */
    for(int i = 0; i < 5; i++) {
        if (sim5300_send_at(sim5300_dev)) {
            puts("[SIM5300] Connection OK");

            return true;
        }
        if (i == 4) {
            puts("[SIM5300] Not answering");

            return false;
        }

        /* Sleep on 500 ms */
        rtctimers_millis_sleep(500);
    }

    return false;
} 

/*---------------------------------------------------------------------------*/
/* SIM5300 initialization */
bool sim5300_init(sim5300_dev_t *sim5300_dev, 
                  uart_t         uart, 
                  uint32_t       baudrate, 
                  char          *buf, 
                  size_t         bufsize, 
                  char          *at_dev_resp, 
                  uint16_t       at_dev_resp_size) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 
    
    puts("[SIM5300] Initialization...");

    /* Structure initialization */
    sim5300_dev->at_dev.uart = uart;
    sim5300_dev->at_dev_resp = at_dev_resp;
    sim5300_dev->at_dev_resp_size = at_dev_resp_size;

    /* Initialization of UART */
    int res = at_dev_init(&sim5300_dev->at_dev, uart, baudrate, buf, bufsize);
    if (res != 0) {
        printf("[SIM5300] Init ERROR: %i\n", res);

        return false;
    }
    
    /* SIM5300 connection */
    if(!sim5300_communication_test(sim5300_dev)) {
        return false;
        // if(!iridium_find_baudrate(sim5300_dev)) {
        //     return false;
        // }
    }

    puts("[SIM5300] Init OK");
    return true;   
}

/*---------------------------------------------------------------------------*/

