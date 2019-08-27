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
/* AT+CGATT Get GPRS service state */
int8_t sim5300_get_gprs_service_state(sim5300_dev_t *sim5300_dev) {
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
bool sim5300_set_gprs_service_state(sim5300_dev_t *sim5300_dev, 
                                    uint8_t        state) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 
    
    /* Test range argument */
    if (state > 1) {
        printf("[SIM5300] sim5300_set_gprs_service_state() ERROR argument: %i. (range 0-1)\n", state);

        return false;
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

        return true;
    } else {
        puts("[SIM5300] sim5300_set_gprs_service_state() ERROR");

        return false;
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
/* AT+CIFSR Get local IP address */ 
bool sim5300_get_local_ip_address(sim5300_dev_t        *sim5300_dev,
                                  sim5300_cifsr_resp_t *sim5300_cifsr_resp) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 

    /* NULL ptr */
    if (sim5300_cifsr_resp == NULL) {
        return false;
    }

    /* Get local IP address */
    int res = at_send_cmd_get_resp(&sim5300_dev->at_dev, "AT+CIFSR", sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, SIM5300_MAX_TIMEOUT); 
    if (res <= 0) {
        puts("[SIM5300] AT+CIFSR ERROR");

        return false;
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

        return false;
    }

    /* Print result */
    printf("[SIM5300] Local IP address: %i.%i.%i.%i\n", sim5300_cifsr_resp->local_ip_address[0],
                                                        sim5300_cifsr_resp->local_ip_address[1],
                                                        sim5300_cifsr_resp->local_ip_address[2],
                                                        sim5300_cifsr_resp->local_ip_address[3]);

    return true;    
}

/*---------------------------------------------------------------------------*/
/* AT+CIPMUX Start up multi-IP connection */
bool sim5300_start_up_multi_ip_connection(sim5300_dev_t *sim5300_dev, 
                                          uint8_t        n) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    } 
    
    /* Test range argument */
    if (n > 1) {
        printf("[SIM5300] sim5300_start_up_multi_ip_connection() ERROR argument: %i. (range 0-1)\n", n);

        return false;
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

        return true;
    } else {
        puts("[SIM5300] sim5300_start_up_multi_ip_connection() ERROR");

        return false;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CIPPING PING request */
bool sim5300_ping_request(sim5300_dev_t          *sim5300_dev,
                          sim5300_cipping_resp_t  sim5300_cipping_resp[],
                          char                   *address,
                          char                   *retr_num,
                          char                   *datalen, 
                          char                   *timeout,
                          char                   *ttl) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    }   

    /* NULL ptr */
    if (sim5300_cipping_resp == NULL) {
        return false;
    }

    /* NULL ptr */
    if (address == NULL) {
        return false;
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
            
            return false;
    }

    /* Debug output */
    DEBUG("num_arg: %i, cmd_CIPPING: %s\n", num_arg, cmd_CIPPING);

    /* Send AT command */
    at_drain(&sim5300_dev->at_dev);
    int res = at_send_cmd(&sim5300_dev->at_dev, cmd_CIPPING, 7000000);
    if (res != 0) {
        return false;
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

    return true;
}

/*---------------------------------------------------------------------------*/
/* AT+CIPSTART Start up multi-IP TCP or UDP connection */
int8_t sim5300_multi_ip_up_single_connection(sim5300_dev_t *sim5300_dev,
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
    if (strcmp(sim5300_dev->at_dev_resp, "OK") != 0) {
        return -8;
    }

    /* Read empty string */
    res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
    DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
    if (res != 0) {
        return -9;
    }

    /* Create resp string for compare */
    char connect_ok[15];
    snprintf(connect_ok, 15, "%i, CONNECT OK", n);

    res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
    DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
    if (strcmp(sim5300_dev->at_dev_resp, connect_ok) != 0) {
        return -10;
    }

    printf("[SIM5300] Start %s connect %i to %s:%s\n", mode, n, address, port);

    return 0;
}

/*---------------------------------------------------------------------------*/
/* AT+CIPRXGET Get data from network manually for multi IP connection */
bool sim5300_receive_data_through_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                                      uint8_t        mode,
                                                      uint8_t        n,
                                                      uint8_t       *data_for_receive,
                                                      size_t         data_size) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    }  

    /* Test mode */
    if (n > 4 || n == 0) {
        printf("[SIM5300] sim5300_receive_data_through_multi_ip_connection() ERROR argument mode: %i. (range 1-4)\n", mode);

        return false;
    }   

    /* Test n */
    if (n > 7) {
        printf("[SIM5300] sim5300_receive_data_through_multi_ip_connection() ERROR argument n: %i. (range 0-7)\n", n);

        return false;
    }   

    /* Test data_for_receive on NULL ptr */
    if (data_for_receive == NULL) {
        puts("data_for_receive = NULL");
        return false;
    } 

    /* Create a command to send data */
    char cmd_CIPRXGET[32];
    switch (mode) {
        case 1:
            /* TODO */

            return false;
        case 2:
            snprintf(cmd_CIPRXGET, 32, "AT+CIPRXGET=%i,%i,%i", mode, n, data_size);

            /* Send AT command */
            at_drain(&sim5300_dev->at_dev);
            int res = at_send_cmd(&sim5300_dev->at_dev, cmd_CIPRXGET, SIM5300_MAX_TIMEOUT);
            if (res != 0) {
                return false;
            }

            /* Sleep on 10 ms */
            rtctimers_millis_sleep(10);

            do {
                /* Read string */
                res = at_readline(&sim5300_dev->at_dev, sim5300_dev->at_dev_resp, sim5300_dev->at_dev_resp_size, false, SIM5300_MAX_TIMEOUT);
                DEBUG("res = %i, data: %s\n", res, sim5300_dev->at_dev_resp);
            } while (res >= 0);

            // +CIPRXGET: 2,<id>,<reqlength>,<cnflength>[,<IP ADDRESS>:<PORT>] 1234567890...
            
            return true;
        case 3:
            /* TODO */

            return false;
        case 4:
            /* TODO */

            return false;
        default:
            puts("[SIM5300] Unknow mode");
            
            return false;
    }
}

/*---------------------------------------------------------------------------*/
/* AT+CIPSEND Send data through TCP or UDP multi IP connection */
bool sim5300_send_data_through_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                                   uint8_t        n,
                                                   uint8_t       *data_for_send, 
                                                   size_t         data_size) {
    /* Test NULL device */
    if (sim5300_dev == NULL) {
        puts("sim5300_dev = NULL");
        return false;
    }     

    /* Test n */
    if (n > 7) {
        printf("[SIM5300] sim5300_send_data_through_multi_ip_connection() ERROR argument n: %i. (range 0-7)\n", n);

        return false;
    }  

    /* Test data_for_send on NULL ptr */
    if (data_for_send == NULL) {
        puts("data_for_send = NULL");
        return false;
    }  

    /* CMD with lenth data for send (AT+CIPSEND=n,data_size) */
    char cmd_CIPSEND[22];
    snprintf(cmd_CIPSEND, 22, "AT+CIPSEND=%i,%i", n, data_size);
    at_drain(&sim5300_dev->at_dev);
    int res = at_send_cmd(&sim5300_dev->at_dev, cmd_CIPSEND, SIM5300_MAX_TIMEOUT);
    if (res != 0) {
        return false;
    } 

    /* Sleep on 10 ms */
    rtctimers_millis_sleep(10);

    /* Send data */
    puts("[SIM5300] Send data:");
    od_hex_dump(data_for_send, data_size, OD_WIDTH_DEFAULT);
    at_send_bytes(&sim5300_dev->at_dev, (char*)data_for_send, data_size);

    return true;
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

