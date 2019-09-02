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
#include "stdlib.h"

#include "periph/gpio.h"
#include "periph/uart.h"
#include "byteorder.h"
#include "od.h"


#include "sim5300.h"

#include "rtctimers-millis.h"


#define ENABLE_DEBUG (1)
#include "debug.h"

#define SIM5300_MAX_TIMEOUT         (1000000)   /* Maximum time waiting for a response */ 
// #define TIME_ON_CHANGE_BAUDRATE (100)       /* Time on change baudrate */


/*---------------------------------------------------------------------------*/
/* AT – ATtention Code */
int sim5300_send_at(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 
        
    puts("[SIM5300] Send AT");

    /* Send AT */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, "AT", SIM5300_MAX_TIMEOUT);
    if (res == 0) {
        return 0;
    } else {
        return -2;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CSMINS SIM Inserted Status Reporting */
int sim5300_get_sim_inserted_status_reporting(sim5300_dev_t         *sim5300_dev,
                                              sim5300_csmins_resp_t *sim5300_csmins_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 

    /* NULL ptr */
    if (sim5300_csmins_resp == NULL) {
        return -2;
    }

    /* Get SIM Inserted Status Reporting */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CSMINS?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CSMINS? ERROR");

        return -3;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Parse string */
    res = sscanf(sim5300_dev->at_dev_resp, "+CSMINS: %i,%i", &sim5300_csmins_resp->n,
                                                             &sim5300_csmins_resp->sim_inserted);

    /* Check result */
    if (res != 2) {
        puts("[SIM5300] Parse error");

        return -4;
    }

    /* Debug output */
    DEBUG("n = %i\n",            sim5300_csmins_resp->n);
    DEBUG("sim_inserted = %i\n", sim5300_csmins_resp->sim_inserted);

    return 0;    
}

/*---------------------------------------------------------------------------*/
/* AT+CSMINS SIM Inserted Status Reporting */
int sim5300_set_sim_inserted_status_reporting(sim5300_dev_t *sim5300_dev, 
                                              uint8_t        n) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 
    
    /* Test range argument */
    if (n > 1) {
        printf("[SIM5300] sim5300_set_sim_inserted_status_reporting() ERROR argument: %i. (range 0-1)\n", n);

        return -2;
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

        return 0;
    } else {
        puts("[SIM5300] sim5300_set_sim_inserted_status_reporting() ERROR");

        return -3;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CPIN Enter PIN */
int sim5300_get_pin_status(sim5300_dev_t *sim5300_dev) {
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
int sim5300_get_network_registration(sim5300_dev_t       *sim5300_dev,
                                     sim5300_creg_resp_t *sim5300_creg_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 

    /* NULL ptr */
    if (sim5300_creg_resp == NULL) {
        return -2;
    }

    /* Get Network Registration */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CREG?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CREG? ERROR");

        return -3;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Parse string */
    res = sscanf(sim5300_dev->at_dev_resp, "+CREG: %i,%i", &sim5300_creg_resp->n,
                                                           &sim5300_creg_resp->stat);

    /* Check result */
    if (res != 2) {
        puts("[SIM5300] Parse error");

        return -4;
    }

    /* Debug output */
    DEBUG("n = %i\n",    sim5300_creg_resp->n);
    DEBUG("stat = %i\n", sim5300_creg_resp->stat);

    return 0;    
}

/*---------------------------------------------------------------------------*/
/* AT+GSMBUSY Reject Incoming Call */
int sim5300_get_reject_incoming_call(sim5300_dev_t *sim5300_dev) {
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
int sim5300_set_reject_incoming_call(sim5300_dev_t *sim5300_dev, 
                                      uint8_t        mode) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 
    
    /* Test range argument */
    if (mode > 2) {
        printf("[SIM5300] sim5300_set_reject_incoming_call() ERROR argument: %i. (range 0-2)\n", mode);

        return -2;
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

        return 0;
    } else {
        puts("[SIM5300] sim5300_set_reject_incoming_call() ERROR");

        return -3;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CSQ Signal Quality Report */
int sim5300_get_signal_quality_report(sim5300_dev_t      *sim5300_dev,
                                      sim5300_csq_resp_t *sim5300_csq_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 

    /* NULL ptr */
    if (sim5300_csq_resp == NULL) {
        return -2;
    }

    /* Get Network Registration */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CSQ", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CSQ ERROR");

        return -3;
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

        return -4;
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

    return 0; 
}   

/*---------------------------------------------------------------------------*/
/* AT+COPS Operator Selection */
int sim5300_get_operator_selection(sim5300_dev_t       *sim5300_dev,
                                   sim5300_cops_resp_t *sim5300_cops_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 

    /* NULL ptr */
    if (sim5300_cops_resp == NULL) {
        return -2;
    }

    /* Get SIM Inserted Status Reporting */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+COPS?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+COPS? ERROR");

        return -3;
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

        return 0;
    } else if (res == 3) {
        DEBUG("mode = %i\n", sim5300_cops_resp->mode);
        DEBUG("format = %i\n", sim5300_cops_resp->format);
        DEBUG("oper = %s\n", sim5300_cops_resp->oper);
        // DEBUG("act = %i\n", sim5300_cops_resp->act);

        return 0;
    } else {
        puts("[SIM5300] Parse error");

        return -4;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CGACT PDP Context Activate or Deactivate */
int sim5300_set_state_pdp_context(sim5300_dev_t *sim5300_dev,
                                  uint8_t        state,
                                  uint8_t        cid) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 
    
    /* Test range argument */
    if (state > 1) {
        printf("[SIM5300] sim5300_set_sim_inserted_status_reporting() ERROR argument: %i. (range 0-1)\n", state);

        return -2;
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

        return 0;
    } else {
        puts("[SIM5300] sim5300_set_state_pdp_context() ERROR");

        return -3;
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
/* Get Home Network Identity (HNI) */
int sim5300_get_hni(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 

    /* Get IMSI */
    char* imsi = sim5300_get_imsi(sim5300_dev);
    if (imsi == NULL) {
        return -2;
    }

    /* Get HNI */
    imsi[5] = 0x00;
    int hni = atoi(imsi);
    
    return hni;
}

/*---------------------------------------------------------------------------*/
/* AT+CGATT Get GPRS service state */
int sim5300_get_gprs_service_state(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 
    
    /* Send AT command */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CGATT?", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT);
    
    /* Check return code */
    if (res <= 0) {
        puts("[SIM5300] sim5300_get_gprs_service() ERROR: -2");

        return -2;
    }

    /* Parse string */
    int state;
    res = sscanf(sim5300_dev->at_dev_resp, "+CGATT: %i", &state);

    /* Check result */
    if (res != 1) {
        puts("[SIM5300] Parse error");

        return -3;
    }

    /* Print result */
    if (state == 1) {
        puts("[SIM5300] GPRS attached");
    } else {
        puts("[SIM5300] GPRS detached");
    }

    return state;
}

/*---------------------------------------------------------------------------*/
/* AT+CGATT Set GPRS service state */
int sim5300_set_gprs_service_state(sim5300_dev_t *sim5300_dev, 
                                   uint8_t        state) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 
    
    /* Test range argument */
    if (state > 1) {
        printf("[SIM5300] sim5300_set_gprs_service_state() ERROR argument: %i. (range 0-1)\n", state);

        return -2;
    }

    /* Create a command to send data */
    char cmd_CGATTn[13];
    snprintf(cmd_CGATTn, 13, "AT+CGATT=%i", state);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, cmd_CGATTn, 7000000);

    /* Return result */
    if (res == 0) {
        /* Print result */
        if (state == 1) {
            puts("[SIM5300] GPRS attached");
        } else {
            puts("[SIM5300] GPRS detached");
        }

        return 0;
    } else {
        puts("[SIM5300] sim5300_set_gprs_service_state() ERROR");

        return -3;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CSTT Start Task and Set APN, USER NAME, PASSWORD */
int sim5300_set_network_settings(sim5300_dev_t *sim5300_dev,
                                 char          *apn,
                                 char          *user,
                                 char          *password) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 

    /* Test arguments */
    if ((apn == NULL) || (user == NULL) || (password == NULL)) {
        puts("Arguments = NULL");

        return -2;
    } 

    /* Create a command to send data */
    char cmd_with_settings_for_internet[128];
    snprintf(cmd_with_settings_for_internet, 128, "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, 
                                                                                  user, 
                                                                                  password);

    /* Sleep on 50 ms */
    rtctimers_millis_sleep(50);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, cmd_with_settings_for_internet, SIM5300_MAX_TIMEOUT);

    /* Return result */
    if (res == 0) {
        /* Print internet settings */
        printf("[SIM5300] Set internet settings: APN=\"%s\", Username=\"%s\", Password=\"%s\"\n", apn, 
                                                                                                  user, 
                                                                                                  password);

        return 0;
    } else {
        puts("[SIM5300] sim5300_set_network_settings() ERROR");

        return -3;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CIICR Bring up wireless connection with GPRS */
int sim5300_bring_up_wireless_connection(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, "AT+CIICR", 6000000);

    /* Return result */
    if (res == 0) {
        /* Print result */
        puts("[SIM5300] Bring up wireless connection with GPRS");

        return 0;
    } else {
        puts("[SIM5300] sim5300_bring_up_wireless_connection() ERROR");

        return -2;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CIFSR Get local IP address */ 
int sim5300_get_local_ip_address(sim5300_dev_t        *sim5300_dev,
                                 sim5300_cifsr_resp_t *sim5300_cifsr_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        
        return -1;
    } 

    /* NULL ptr */
    if (sim5300_cifsr_resp == NULL) {
        return -2;
    }

    /* Get local IP address */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CIFSR", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CIFSR ERROR");

        return -3;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Parse string */
    res = sscanf(sim5300_dev->at_dev_resp, "%i.%i.%i.%i", &sim5300_cifsr_resp->local_ip_address[0],
                                                          &sim5300_cifsr_resp->local_ip_address[1],
                                                          &sim5300_cifsr_resp->local_ip_address[2],
                                                          &sim5300_cifsr_resp->local_ip_address[3]);

    /* Check result */
    if (res != 4) {
        puts("[SIM5300] Parse error");

        return -4;
    }

    /* Print result */
    printf("[SIM5300] Local IP address: %i.%i.%i.%i\n", sim5300_cifsr_resp->local_ip_address[0],
                                                        sim5300_cifsr_resp->local_ip_address[1],
                                                        sim5300_cifsr_resp->local_ip_address[2],
                                                        sim5300_cifsr_resp->local_ip_address[3]);

    return 0;    
}

/*---------------------------------------------------------------------------*/
/* AT+CIPMUX Start up multi-IP connection */
int sim5300_start_up_multi_ip_connection(sim5300_dev_t *sim5300_dev, 
                                         uint8_t        n) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 
    
    /* Test range argument */
    if (n > 1) {
        printf("[SIM5300] sim5300_start_up_multi_ip_connection() ERROR argument: %i. (range 0-1)\n", n);

        return -2;
    }

    /* Create a command to send data */
    char cmd_CIPMUXn[12];
    snprintf(cmd_CIPMUXn, 12, "AT+CIPMUX=%i", n);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, cmd_CIPMUXn, SIM5300_MAX_TIMEOUT);

    /* Return result */
    if (res == 0) {
        /* Print result */
        if (n == 0) {
            puts("[SIM5300] Set single-IP connection");
        } else {
            puts("[SIM5300] Set multi-IP connection");
        }

        return 0;
    } else {
        puts("[SIM5300] sim5300_start_up_multi_ip_connection() ERROR");

        return -3;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CIPCLOSE Close up multi-IP connection */
int sim5300_close_up_multi_ip_connection(sim5300_dev_t *sim5300_dev, 
                                         uint8_t        id,  
                                         uint8_t        n) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 
    
    /* Test id range argument*/
    if (id > 7) {
        printf("[SIM5300] sim5300_close_up_multi_ip_connection() ERROR argument id: %i. (range 0-7)\n", id);

        return -2;
    }

    /* Test n range argument */
    if (n > 1) {
        printf("[SIM5300] sim5300_close_up_multi_ip_connection() ERROR argument: %i. (range 0-1)\n", n);

        return -3;
    }

    /* Create a command to send data */
    char cmd_CIPCLOSEn[17];
    snprintf(cmd_CIPCLOSEn, 17, "AT+CIPCLOSE=%i,%i", id, n);

    /* Send AT command */
    at_drain(&sim5300_dev->at_dev);
    int res = at_send_cmd(&sim5300_dev->at_dev, cmd_CIPCLOSEn, SIM5300_MAX_TIMEOUT);
    if (res != 0) {
        return -4;
    }

    /* Create resp string */
    char resp_on_CIPCLOSE[13];
    snprintf(resp_on_CIPCLOSE, 13, "%i, CLOSE OK", n);

    /* Read string */
    res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
    DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);

    /* Compare resp */
    if (memcmp(resp_on_CIPCLOSE, sim5300_dev->at_dev_resp, 11) != 0) {
        return -5;
    }

    return 0;
}

/*---------------------------------------------------------------------------*/
/* AT+CIPPING PING request */
int sim5300_ping_request(sim5300_dev_t          *sim5300_dev,
                         sim5300_cipping_resp_t  sim5300_cipping_resp[],
                         char                   *address,
                         char                   *retr_num,
                         char                   *datalen, 
                         char                   *timeout,
                         char                   *ttl) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }   

    /* NULL ptr */
    if (sim5300_cipping_resp == NULL) {
        return -2;
    }

    /* NULL ptr */
    if (address == NULL) {
        return -3;
    } 

    /* Calculation of the number of arguments */
    uint8_t num_arg = 1;
    if (retr_num != NULL) {
        num_arg++;
        if (datalen != NULL) {
            num_arg++;
            if (timeout != NULL) {
                num_arg++;
                if (ttl != NULL) {
                    num_arg++;
                }
            }
        }
    }

    /* Create a command to send data */
    char cmd_CIPPING[200];
    switch (num_arg) {
        case 1:
            snprintf(cmd_CIPPING, 200, "AT+CIPPING=%s", address);

            break;
        case 2:
            snprintf(cmd_CIPPING, 200, "AT+CIPPING=%s,%s", address, retr_num);

            break;
        case 3:
            snprintf(cmd_CIPPING, 200, "AT+CIPPING=%s,%s,%s", address, retr_num, datalen);

            break;
        case 4:
            snprintf(cmd_CIPPING, 200, "AT+CIPPING=%s,%s,%s,%s", address, retr_num, datalen, timeout);

            break;
        case 5:
            snprintf(cmd_CIPPING, 200, "AT+CIPPING=%s,%s,%s,%s,%s", address, retr_num, datalen, timeout, ttl);

            break;
        default:
            puts("[SIM5300] Unknow num arg");
            
            return -4;
    }

    /* Debug output */
    DEBUG("num_arg: %i, cmd_CIPPING: %s\n", num_arg, cmd_CIPPING);

    /* Send AT command */
    at_drain(&sim5300_dev->at_dev);
    int res = at_send_cmd(&sim5300_dev->at_dev, cmd_CIPPING, 7000000);
    if (res != 0) {
        return -5;
    }

    /* Sleep on 10000 ms */
    rtctimers_millis_sleep(10000);

    int reply_id_scan;
    int reply_time_scan; 
    int ttl_scan;   

    /* Create format string */
    char format[64];
    snprintf(format, 64, "+CIPPING: %%i,\\\"%s\\\",%%i,%%i", address);
    DEBUG("format: %s\n", format);
    do {
        /* Read string */
        res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
        DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);

        // TODO: if (res) 

        /* TODO: PARSE ERROR BECAUSE IP ADDREESS HAVE: "" */
        /* Parse string */
        res = sscanf(sim5300_dev->at_dev_resp, format, &reply_id_scan, &reply_time_scan, &ttl_scan);

        /* Check result */
        if (res != 3) {
            printf("[SIM5300] Parse error: %i\n", res);

            continue;
        }

        sim5300_cipping_resp[reply_id_scan - 1].reply_time = reply_time_scan;      
        sim5300_cipping_resp[reply_id_scan - 1].ttl = ttl_scan;    

        if (num_arg > 1) {
            DEBUG("atoi(%s) = %i", retr_num, atoi(retr_num));
            if (reply_id_scan == atoi(retr_num)) {
                break;
            }
        }
    } while (res >= 0);

    return 0;
}

/*---------------------------------------------------------------------------*/
/* AT+CIPSTART Start up multi-IP TCP or UDP connection */
int sim5300_multi_ip_up_single_connection(sim5300_dev_t *sim5300_dev,
                                             uint8_t        n,
                                             char          *mode,
                                             char          *address,
                                             char          *port) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }   

    /* Test n */
    if (n > 7) {
        printf("[SIM5300] sim5300_multi_ip_up_single_connection() ERROR argument n: %i. (range 0-7)\n", n);

        return -2;
    }

    /* Test mode */
    if (mode != NULL) {
        if (!((strcmp(mode, "TCP") == 0) || 
              (strcmp(mode, "UDP") == 0))) {
            printf("mode: %s != (TCP || UDP)\n", mode);

            return -3;
        }
    } else {
        puts("mode = NULL");

        return -4;
    }

    /* Test address */
    if (address == NULL) {
        puts("address = NULL");

        return -5;
    }  

    /* Test port */
    if (port == NULL) {
        puts("port = NULL");

        return -6;
    }  

    /* Create a command to send data */
    char cmd_CIPSTART[128];
    snprintf(cmd_CIPSTART, 128, "AT+CIPSTART=%i,\"%s\",\"%s\",\"%s\"", n, mode, address, port);
    
    /* Debug output */
    DEBUG("cmd_CIPSTART: %s\n", cmd_CIPSTART);

    /* Send AT command */
    at_drain(&sim5300_dev->at_dev);
    int res = at_send_cmd(&sim5300_dev->at_dev, cmd_CIPSTART, 5000000);
    if (res != 0) {
        return -7;
    }

    /* Wait 5 sec for start TCP connection */
    rtctimers_millis_sleep(5000);

    /* Read string with OK */
    res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
    DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
    
    /* Check read len string */
    if (res != 2) {
        return -8;
    }

    /* Validation of the answer */
    if (strcmp(sim5300_dev->at_dev_resp, "OK") != 0) {
        return -9;
    }

    /* Read empty string */
    res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
    DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
    if (res != 0) {
        return -10;
    }

    /* Create resp string for compare */
    char connect_ok[15];
    snprintf(connect_ok, 15, "%i, CONNECT OK", n);

    res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
    DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
    
    /* Check read len string */
    if (res != 13) {
        return -11;
    }

    /* Validation of the answer */
    if (strcmp(sim5300_dev->at_dev_resp, connect_ok) != 0) {
        return -12;
    }

    printf("[SIM5300] Start %s connect %i to %s:%s\n", mode, n, address, port);

    return 0;
}

/*---------------------------------------------------------------------------*/
/* AT+CIPRXGET Get data from network manually for multi IP connection */
int sim5300_receive_data_through_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                                     uint8_t        mode,
                                                     uint8_t        n,
                                                     uint8_t       *data_for_receive,
                                                     size_t         data_size) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }  

    /* Test mode */
    if (mode > 4 || mode == 0) {
        printf("[SIM5300] sim5300_receive_data_through_multi_ip_connection() ERROR argument mode: %i. (range 1-4)\n", mode);

        return -2;
    }   

    /* Test n */
    if (n > 7) {
        printf("[SIM5300] sim5300_receive_data_through_multi_ip_connection() ERROR argument n: %i. (range 0-7)\n", n);

        return -3;
    }   

    /* Test data_for_receive on NULL ptr */
    if ((data_for_receive == NULL) && (mode != 1)) {
        puts("data_for_receive = NULL");

        return -4;
    } 

    int res;

    /* Create a command to send data */
    char cmd_CIPRXGET[32];
    switch (mode) {
        case 1:
            snprintf(cmd_CIPRXGET, 32, "AT+CIPRXGET=%i", mode);

            /* Send AT command */
            res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, cmd_CIPRXGET, SIM5300_MAX_TIMEOUT);

            /* Return result */
            if (res != 0) {
                return -5;
            }

            return 0;
        case 2:
            snprintf(cmd_CIPRXGET, 32, "AT+CIPRXGET=%i,%i,%i", mode, n, data_size);

            /* Send AT command */
            at_drain(&sim5300_dev->at_dev);
            res = at_send_cmd(&sim5300_dev->at_dev, cmd_CIPRXGET, SIM5300_MAX_TIMEOUT);
            if (res != 0) {
                return -6;
            }

            /* Sleep on 10 ms */
            rtctimers_millis_sleep(10); 

            res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
            DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
            if (res < 0) {
                return -7;
            }

            /* Parse string */
            int id, req_length, cnf_length;
            res = sscanf(sim5300_dev->at_dev_resp, "+CIPRXGET: 2,%i,%i,%i", &id, &req_length, &cnf_length);
            DEBUG("id = %i\n", id);
            DEBUG("req_length = %i\n", req_length);
            DEBUG("cnf_length = %i\n", cnf_length);

            /* Check result */
            if (res != 3) {
                printf("[SIM5300] Parse error: %i\n", res);

                return -8;
            }

            if (req_length == 0) {
                return 0;
            }

            /* Calculate receive_length */
            size_t receive_length;
            if ((uint32_t)req_length <= data_size) {
                receive_length = req_length;
            } else {
                receive_length = data_size;
            }

            /* Copy received data */
            res = at_recv_bytes(&sim5300_dev->at_dev, (char*)data_for_receive, receive_length, SIM5300_MAX_TIMEOUT);
            if ((uint32_t)res != receive_length) {
                return -9;
            }

            // if ((uint32_t)req_length <= data_size) {
            //     res = at_recv_bytes(&sim5300_dev->at_dev, (char*)data_for_receive, req_length, SIM5300_MAX_TIMEOUT);
            //     if (res != req_length) {
            //         puts("1");

            //         return -10;
            //     }
            // } else {
            //     res = at_recv_bytes(&sim5300_dev->at_dev, (char*)data_for_receive, data_size, SIM5300_MAX_TIMEOUT);
            //     if ((uint32_t)res != data_size) {
            //         puts("2");

            //         return -10;
            //     }
            // }

            /* TODO: Uncomment */
            // /* Read empty string */ 
            // res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
            // DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
            // if (res != 0) {
            //     return -10;
            // }

            /* TODO: Uncomment */
            // /* Read string with OK */
            // res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
            // DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
            // /* Check read len string */
            // if (res != 2) {
            //     return -11;
            // }

            /* TODO: Uncomment */
            // /* Validation of the answer */
            // if (strcmp(sim5300_dev->at_dev_resp, "OK") != 0) {
            //     return -12;
            // }

#if ENABLE_DEBUG == 1
            /* Debug data */
            printf("[SIM5300] Received %i byte:\n", receive_length);
            od_hex_dump(data_for_receive, receive_length, OD_WIDTH_DEFAULT);
#endif

            return receive_length;
        case 3:
            /* TODO */

            return -13;
        case 4:
            /* TODO */

            return -14;
        default:
            puts("[SIM5300] Unknow mode");
            
            return -15;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CIPSEND Send data through TCP or UDP multi IP connection */
int sim5300_send_data_through_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                                  uint8_t        n,
                                                  uint8_t       *data_for_send, 
                                                  size_t         data_size) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }     

    /* Test n */
    if (n > 7) {
        printf("[SIM5300] sim5300_send_data_through_multi_ip_connection() ERROR argument n: %i. (range 0-7)\n", n);

        return -2;
    }  

    /* Test data_for_send on NULL ptr */
    if (data_for_send == NULL) {
        puts("data_for_send = NULL");
        return -3;
    }  

    /* Check data_size */
    if (data_size == 0) {
        return 0;
    }

    /* CMD with lenth data for send (AT+CIPSEND=n,data_size) */
    char cmd_CIPSEND[22];
    snprintf(cmd_CIPSEND, 22, "AT+CIPSEND=%i,%i", n, data_size);
    at_drain(&sim5300_dev->at_dev);
    int res = at_send_cmd(&sim5300_dev->at_dev, cmd_CIPSEND, SIM5300_MAX_TIMEOUT);
    if (res != 0) {
        return -4;
    } 

    /* Sleep on 30 ms */
    rtctimers_millis_sleep(30);

    /* Send data */
    at_send_bytes(&sim5300_dev->at_dev, (char*)data_for_send, data_size);

    // /* Check on valid data */
    // res = at_recv_bytes(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, data_size + 4, 3000000);
    // if ((!(memcmp(sim5300_dev->at_dev_resp, "> ", 2) == 0)) && (!(memcmp((uint8_t*)(sim5300_dev->at_dev_resp) + 2, data_for_send, data_size) == 0))) {
    //     puts("[SIM5300] Data for send don't valid");

    //     return false;
    // } 

    // TODO: TIME ON EXIT on rtctimers_millis_now
    for (uint16_t i = 0; i < 500; i++) {
        res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
        if (res != 10) {
            rtctimers_millis_sleep(10);
            continue;
        } 

        char resp_on_CIPSEND[11];
        snprintf(resp_on_CIPSEND, 11, "%i, SEND OK", n);
        if (memcmp(sim5300_dev->at_dev_resp, resp_on_CIPSEND, 10) != 0) {
            return -5;
        }

#if ENABLE_DEBUG == 1
    /* Debug data */
    printf("[SIM5300] Sent %i byte:\n", data_size);
    od_hex_dump(data_for_send, data_size, OD_WIDTH_DEFAULT);
#endif

        return data_size;
    }

    return -6; 
}

/*---------------------------------------------------------------------------*/
/* Get internet settings from base */
int sim5300_get_internet_settings_from_base(sim5300_dev_t               *sim5300_dev,
                                            uint32_t                     hni,
                                            sim5300_internet_settings_t *sim5300_internet_settings) {
    (void)sim5300_dev;

    printf("[SIM5300] Get internet settings from base for HNI: %lu\n", hni);

    /* Get standart settings */
    switch (hni) {
        // +COPN: "25004","SIBCHALLENGE RUS"
        // +COPN: "25028","VOICE"
        // +COPN: "25092","Primetelefone RUS"

        /* +COPN: "25001","MTS RUS" */
        case 25001:
            snprintf(sim5300_internet_settings->apn, 32, "internet.mts.ru");   /* Access Point Name */
            snprintf(sim5300_internet_settings->username, 32, "mts");          /* Username */
            snprintf(sim5300_internet_settings->password, 32, "mts");          /* Password */
            break;

        /* +COPN: "25002","MegaFon RUS" */
        case 25002:
            snprintf(sim5300_internet_settings->apn, 32, "internet");      /* Access Point Name */
            memset(sim5300_internet_settings->username, 0x00, 32);         /* Username */ 
            memset(sim5300_internet_settings->password, 0x00, 32);         /* Password */
            break;

        // /* +COPN: "25003","ROSTELECOM" */
        // case 25003:
        //     /*  */
        //     break;

        // /* +COPN: "25005","ROSTELECOM" */
        // case 25005:
        //     /*  */
        //     break;

        // /*  */
        // case 25006:
        //     /*  */
        //     break;

        // /* +COPN: "25007","RUS 07, RUS SMARTS" */
        // case 25007:
        //     /*  */
        //     break;

        // /*  */
        // case 25008:
        //     /* apn: "vtk" "internet" */
        //     break;

        // /*  */
        // case 25009:
        //     /*  */
        //     break; 

        // /* DTC */
        // case 25010:
        //     /*  */
        //     break;

        /* +COPN: "25011","Yota" */
        case 25011:
            snprintf(sim5300_internet_settings->apn, 32, "internet.yota");     /* Access Point Name */
            memset(sim5300_internet_settings->username, 0x00, 32);             /* Username */
            memset(sim5300_internet_settings->password, 0x00, 32);             /* Password */   
            break;

        // /* +COPN: "25012","ROSTELECOM" */
        // case 25012:
        //     /*  */
        //     break;

        // /* +COPN: "25013","RUS Kuban-GSM" */
        // case 25013:
        //     /*  */
        //     break;

        /*  */
        case 25014:
            snprintf(sim5300_internet_settings->apn, 32, "internet");  /* Access Point Name */
            memset(sim5300_internet_settings->username, 0x00, 32);     /* Username */ 
            memset(sim5300_internet_settings->password, 0x00, 32);     /* Password */
            break;

        // /* +COPN: "25015","RUS15, RUS SMARTS" */
        // case 25015:
        //     /*  */
        //     break;

        // /* +COPN: "25016","NTC" */
        // case 25016:
        //     /*  */
        //     break;

        // /* +COPN: "25017","ROSTELECOM" */
        // case 25017:
        //     /*  */
        //     break;

        /* Tele2 AB (Tele2) */
        case 25020:
            snprintf(sim5300_internet_settings->apn, 32, "internet.tele2.ru");     /* Access Point Name */
            memset(sim5300_internet_settings->username, 0x00, 32);                 /* Username */
            memset(sim5300_internet_settings->password, 0x00, 32);                 /* Password */  
            break;

        // /*  */
        // case 25023:
        //     /*  */
        //     break;

        // /*  */
        // case 25027:
        //     /*  */
        //     break;

        /*  */
        case 25028:
            snprintf(sim5300_internet_settings->apn, 32, "internet.beeline.ru");   /* Access Point Name */
            snprintf(sim5300_internet_settings->username, 32, "beeline");          /* Username */
            snprintf(sim5300_internet_settings->password, 32, "beeline");          /* Password */
            break;

        // /* +COPN: "25035","MOTIV" */
        // case 25035:
        //     /*  */
        //     break;

        // /* +COPN: "25038","ROSTELECOM" */
        // case 25038:
        //     /*  */
        //     break;

        /* +COPN: "25039","ROSTELECOM" */
        case 25039:
            snprintf(sim5300_internet_settings->apn, 32, "internet.rt.ru");    /* Access Point Name */
            memset(sim5300_internet_settings->username, 0x00, 32);             /* Username */
            memset(sim5300_internet_settings->password, 0x00, 32);             /* Password */ 
            break;

        // /* OJSC Multiregional TransitTelecom (MTT) */
        // case 25042:
        //     /*  */
        //     break;

        /* Tinkoff */
        case 25062:
            snprintf(sim5300_internet_settings->apn, 32, "m.tinkoff");     /* Access Point Name */
            memset(sim5300_internet_settings->username, 0x00, 32);         /* Username */
            memset(sim5300_internet_settings->password, 0x00, 32);         /* Password */  
            break;

        /* +COPN: "25099","Beeline" */
        case 25099:
            snprintf(sim5300_internet_settings->apn, 32, "internet.beeline.ru");   /* Access Point Name */
            snprintf(sim5300_internet_settings->username, 32, "beeline");          /* Username */
            snprintf(sim5300_internet_settings->password, 32, "beeline");          /* Password */
            break;    

        /* Unknown operator */
        default: 
            memset(sim5300_internet_settings->apn,      0x00, 32);     /* Access Point Name */
            memset(sim5300_internet_settings->username, 0x00, 32);     /* Username */
            memset(sim5300_internet_settings->password, 0x00, 32);     /* Password */

            return -1;   
            break;
    } 
    return 0;
}

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_start_internet(sim5300_dev_t               *sim5300_dev,
                           uint8_t                      registration_timeout,
                           sim5300_internet_settings_t *sim5300_internet_settings) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }     

    int res;

    /* Disabled showing an unsolicited event code */
    res = sim5300_set_sim_inserted_status_reporting(sim5300_dev, 0);
    if (res != 0) {
        return -2;
    }

    /* Is SIM card inserted? */
    sim5300_csmins_resp_t sim5300_csmins_resp;
    res = sim5300_get_sim_inserted_status_reporting(sim5300_dev, &sim5300_csmins_resp);
    if (res != 0) {
        return -3;
    }
    if (sim5300_csmins_resp.sim_inserted != 1) {
        return -4;
    }

    /* Is there a PIN code? */
    if (sim5300_get_pin_status(sim5300_dev) != READY) {
        return -5;
    }

    /* Have you registered in the cellular network? */
    /* Cycle counter */
    uint8_t counter = 0;

    /* Response on AT+CREG */
    sim5300_creg_resp_t sim5300_creg_resp;
    while(1) {
        if (counter >= registration_timeout) {
            puts("[SIM5300] Registration failed");
            return -6;
        }

        res = sim5300_get_network_registration(sim5300_dev, &sim5300_creg_resp);
        if (res == 0) {
            if (sim5300_creg_resp.stat == 1) {
                break;
            }
        }

        rtctimers_millis_sleep(1000);
        counter++;
    } 
    puts("[SIM5300] Registration OK");

    /* Reject Incoming Call */
    res = sim5300_set_reject_incoming_call(sim5300_dev, 1);
    if (res != 0) {
        return -7;
    }

    /* Start up multi-IP connection */
    res = sim5300_start_up_multi_ip_connection(sim5300_dev, 1);
    if (res != 0) {
        return -8;
    }

    /* Attach to the network */
    int8_t gprs_state = sim5300_get_gprs_service_state(sim5300_dev);
    if (gprs_state == 0) {
        res = sim5300_set_gprs_service_state(sim5300_dev, 1);
        if (res != 0) {
            return -9;
        }
    } else if (gprs_state != 1) {
        return -10;
    }

    /* Get data from network manually for multi IP connection */
    res = sim5300_receive_data_through_multi_ip_connection(sim5300_dev, 1, 1, NULL, 0);
    if(res != 0) {
        puts("[SIM5300] Set get data from network manually for multi IP connection ERROR");
    } 

    /* Have internet settings? */
    if (sim5300_internet_settings != NULL) {
        /* Start Task and Set APN, USER NAME, PASSWORD */
        res = sim5300_set_network_settings(sim5300_dev, 
                                           sim5300_internet_settings->apn, 
                                           sim5300_internet_settings->username, 
                                           sim5300_internet_settings->password);
        if (res != 0) {
            return -11;
        }
    } else {
        /* Get internet settings from base */
        sim5300_internet_settings_t sim5300_get_internet_settings;
        res = sim5300_get_internet_settings_from_base(sim5300_dev,
                                                      sim5300_get_hni(sim5300_dev),
                                                      &sim5300_get_internet_settings);
        if (res != 0) {
            return -12;
        }

        /* Start Task and Set APN, USER NAME, PASSWORD */
        res = sim5300_set_network_settings(sim5300_dev, 
                                           sim5300_get_internet_settings.apn, 
                                           sim5300_get_internet_settings.username, 
                                           sim5300_get_internet_settings.password);
        if (res != 0) {
            return -13;
        }
    }

    /* Bring Up Wireless Connection with GPRS */
    res = sim5300_bring_up_wireless_connection(sim5300_dev);
    if (res != 0) {
        return -14;
    }

    /* Get local IP address */
    sim5300_cifsr_resp_t sim5300_cifsr_resp;
    res = sim5300_get_local_ip_address(sim5300_dev, &sim5300_cifsr_resp);
    if (res != 0) {
        return -15;
    }

    /* Check local address */
    if ((sim5300_cifsr_resp.local_ip_address[0] == 0) && 
        (sim5300_cifsr_resp.local_ip_address[1] == 0) &&
        (sim5300_cifsr_resp.local_ip_address[2] == 0) &&
        (sim5300_cifsr_resp.local_ip_address[3] == 0)) {
        puts("[SIM5300] Zero IP ERROR");

        return -16;
    }

    /* AT+CDNSCFG="8.8.8.8","8.8.4.4" */

    /* AT+CIPSTATUS */

    /* ping */
    // sim5300_cipping_resp_t sim5300_cipping_resp[3] = {};
    // if (sim5300_ping_request(sim5300_dev,
    //                          sim5300_cipping_resp,
    //                          "8.8.8.8",
    //                          "3",
    //                          "32", 
    //                          "100",
    //                          "64")) {
    //     puts("ping true");

    //     for (uint8_t i = 0; i < 3; i++ ) {
    //         printf("sim5300_cipping_resp[%i].reply_time = %i; sim5300_cipping_resp[%i].ttl = %i\n", i, sim5300_cipping_resp[i].reply_time, i, sim5300_cipping_resp[i].ttl);
    //     }
    // } else {
    //     puts("ping false");

    //     return false;
    // }

    return 0;
}

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_socket(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }   

    /* Search free socket */
    for(uint8_t i = 0; i < 8; i++) {
        if (!sim5300_dev->socketfd[i]) {
            /* Mark as busy */
            sim5300_dev->socketfd[i] = true;

            return i;
        }
    }

    return -2;
}

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_connect(sim5300_dev_t *sim5300_dev,
                    int            sockfd, 
                    char          *address,
                    char          *port,
                    char          *type) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }   

    /* Test sockfd */
    if (sockfd > 7) {
        printf("[SIM5300] sim5300_connect() ERROR argument sockfd: %i. (range 0-7)\n", sockfd);

        return -2;
    }

    /* Test NULL address */
    if (address == NULL) {
        puts("address = NULL");

        return -3;
    }   

    /* Test NULL port */
    if (port == NULL) {
        puts("port = NULL");

        return -4;
    }   

    /* Test NULL type */
    if (type == NULL) {
        puts("type = NULL");

        return -5;
    }   

    int res;

    /* Start up multi-IP TCP or UDP connection */
    res = sim5300_multi_ip_up_single_connection(sim5300_dev, sockfd, type, address, port);
    if (!(res >= 0)) {
        printf("[SIM5300] Error start connection: %i\n", res);

        return -6;
    }

    return 0;
}

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_send(sim5300_dev_t *sim5300_dev,
                 int            sockfd, 
                 uint8_t       *buffer,
                 size_t         buffer_len) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }   

    /* Test sockfd */
    if (sockfd > 7) {
        printf("[SIM5300] sim5300_send() ERROR argument sockfd: %i. (range 0-7)\n", sockfd);

        return -2;
    } 

    /* Test NULL buffer */
    if (buffer == NULL) {
        puts("buffer = NULL");

        return -3;
    }
 
    return sim5300_send_data_through_multi_ip_connection(sim5300_dev, sockfd, buffer, buffer_len);
}

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_receive(sim5300_dev_t *sim5300_dev,
                    int            sockfd, 
                    uint8_t       *buffer,
                    size_t         buffer_len) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }   

    /* Test sockfd */
    if (sockfd > 7) {
        printf("[SIM5300] sim5300_send() ERROR argument sockfd: %i. (range 0-7)\n", sockfd);

        return -2;
    } 

    /* Test NULL buffer */
    if (buffer == NULL) {
        puts("buffer = NULL");

        return -3;
    }   

    return sim5300_receive_data_through_multi_ip_connection(sim5300_dev, 2, sockfd, buffer, buffer_len);
}

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_close(sim5300_dev_t *sim5300_dev,
                  int            sockfd) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }   

    /* Test sockfd */
    if (sockfd > 7) {
        printf("[SIM5300] sim5300_close() ERROR argument sockfd: %i. (range 0-7)\n", sockfd);

        return -2;
    }

    int res; 
    
    /* Close socket */
    res = sim5300_close_up_multi_ip_connection(sim5300_dev, 
                                             sockfd,  
                                             0);

    if(res != 0) {
        printf("[SIM5300] Error close socket %i\n", sockfd); 

        return -3;
    }

    /* Free the socket */
    sim5300_dev->socketfd[sockfd] = false;

    return 0;
}

/*---------------------------------------------------------------------------*/
/* Communication test between microcontroller and SIM5300 */
int sim5300_communication_test(sim5300_dev_t *sim5300_dev) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    }     

    int res; 

    /* SIM5300 connection */
    for(int i = 0; i < 5; i++) {
        res = sim5300_send_at(sim5300_dev);
        if (res == 0) {
            puts("[SIM5300] Connection OK");

            return 0;
        }
        if (i == 4) {
            puts("[SIM5300] Not answering");

            return -2;
        }

        /* Sleep on 500 ms */
        rtctimers_millis_sleep(500);
    }

    return -3;
} 

/*---------------------------------------------------------------------------*/
/* SIM5300 initialization */
int sim5300_init(sim5300_dev_t *sim5300_dev, 
                 uart_t         uart, 
                 uint32_t       baudrate, 
                 char          *buf, 
                 size_t         bufsize, 
                 char          *at_dev_resp, 
                 uint16_t       at_dev_resp_size) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");

        return -1;
    } 

    int res;
    
    puts("[SIM5300] Initialization...");

    /* Structure initialization */
    sim5300_dev->at_dev.uart      = uart;
    sim5300_dev->at_dev_resp      = at_dev_resp;
    sim5300_dev->at_dev_resp_size = at_dev_resp_size;
    for(uint8_t i = 0; i < 8; i++) {
        sim5300_dev->socketfd[i]  = false;
    }

    /* Initialization of UART */
    res = at_dev_init(&sim5300_dev->at_dev, uart, baudrate, buf, bufsize);
    if (res != 0) {
        printf("[SIM5300] Init ERROR: %i\n", res);

        return -2;
    }
    
    /* SIM5300 connection */
    res = sim5300_communication_test(sim5300_dev);
    if(res != 0) {
        return -3;
        // if(!iridium_find_baudrate(sim5300_dev)) {
        //     return false;
        // }
    }

    puts("[SIM5300] Init OK");

    return 0;   
}

/*---------------------------------------------------------------------------*/

