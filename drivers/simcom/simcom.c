/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    SimCom AT modem driver 
 * @ingroup     drivers
 * @brief       SimCom AT modem driver 
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
 * @brief       SimCom AT modem driver 
 * @author      Oleg Manchenko <man4enkoos@gmail.com>
 */

#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "periph/gpio.h"
#include "periph/uart.h"
#include "byteorder.h"
#include "od.h"


#include "simcom.h"

#include "lptimer.h"


#define ENABLE_DEBUG        (1)
#define ENABLE_DEBUG_DATA   (0)
#include "debug.h"

// #if (ENABLE_DEBUG == 1)
//     #define ENABLE_DEBUG_DATA (1)
// #endif

/*---------------------------------------------------------------------------*/
/*------------------------ START LOW LEVEL FUNCTION -------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * @brief       Power on for SIMCOM
 *
 * @param[in]   simcom_dev         Device to operate on
 * 
 * Power on for SIMCOM
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 */
int simcom_power_on(simcom_dev_t *simcom_dev) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    puts("[SIMCOM] Power on");

    /* DC/DC power on */
    gpio_write(simcom_dev->power_en_pin, simcom_dev->power_act_level);

    /* Enable modem */
    gpio_write(simcom_dev->gsm_en_pin, simcom_dev->gsm_act_level);

    /* 500ms sleep */
    lptimer_usleep(SIMCOM_TIME_ON);

    /* Set or clear GPIO */
    gpio_toggle(simcom_dev->gsm_en_pin);
    
    /* Modem enabled */

    /* We wait while SIMCOM is initialized */
    lptimer_usleep(SIMCOM_TIME_ON_UART);

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Power off for SIMCOM
 *
 * @param[in]   simcom_dev         Device to operate on
 * 
 * Power off for SIMCOM
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 */
int simcom_power_off(simcom_dev_t *simcom_dev) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* Power off UART for modem */
    uart_poweroff(simcom_dev->at_dev.uart);

    /* Disable DC/DC */
    if (simcom_dev->power_act_level) {
        gpio_clear(simcom_dev->power_en_pin);
    } else {
        gpio_set(simcom_dev->power_en_pin);
    }

    puts("[SIMCOM] Power off");

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Send ATtention Code
 *
 * @param[in]   simcom_dev         Device to operate on
 * 
 * AT – ATtention Code
 * This is the prefix for all commands except A/. When entered on its own, the SIMCOM will respond OK.
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int simcom_send_at(simcom_dev_t *simcom_dev) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
        
    puts("[SIMCOM] Send AT");

    /* Send AT */
    int res = at_send_cmd_wait_ok(&simcom_dev->at_dev, "AT", SIMCOM_MAX_TIMEOUT);
    if (res == SIMCOM_OK) {
        return SIMCOM_OK;
    } else {
        return SEND_CMD_WAIT_OK_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get SIM inserted status reporting 
 *
 * @param[in]   simcom_dev             Device to operate on
 * @param[out]  simcom_csmins_resp     Structure with response data
 * 
 * AT+CSMINS – SIM Inserted Status Reporting
 * Get SIM inserted status reporting 
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int simcom_get_sim_inserted_status_reporting(simcom_dev_t         *simcom_dev,
                                              simcom_csmins_resp_t *simcom_csmins_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* NULL ptr */
    if (simcom_csmins_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* Get SIM Inserted Status Reporting */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+CSMINS?", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT); 
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+CSMINS? ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, simcom_dev->at_dev_resp);

    /* Parse string */
    res = sscanf(simcom_dev->at_dev_resp, "+CSMINS: %i,%i", &simcom_csmins_resp->n,
                                                             &simcom_csmins_resp->sim_inserted);

    /* Check result */
    if (res != 2) {
        puts("[SIMCOM] Parse error");

        return PARSE_ERROR;
    }

    /* Debug output */
    DEBUG("n = %i\n",            simcom_csmins_resp->n);
    DEBUG("sim_inserted = %i\n", simcom_csmins_resp->sim_inserted);

    return SIMCOM_OK;    
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set SIM inserted status reporting 
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   n               A numeric parameter to show an unsolicited event code indicating whether the SIM has been inserted or removed.
 * 
 * AT+CSMINS – SIM Inserted Status Reporting
 * Set SIM inserted status reporting 
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int simcom_set_sim_inserted_status_reporting(simcom_dev_t *simcom_dev, 
                                              uint8_t        n) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
    
    /* Test range argument */
    if (n > 1) {
        printf("[SIMCOM] simcom_set_sim_inserted_status_reporting() ERROR argument: %i. (range 0-1)\n", n);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Create a command to send data */
    char cmd_CSMINSn[12];
    snprintf(cmd_CSMINSn, 12, "AT+CSMINS=%i", n);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&simcom_dev->at_dev, cmd_CSMINSn, SIMCOM_MAX_TIMEOUT);

    /* Return result */
    if (res == SIMCOM_OK) {
        /* Print result */
        if (n == 0) {
            puts("[SIMCOM] Disabled showing an unsolicited event code");
        } else {
            puts("[SIMCOM] Enabled showing an unsolicited event code ");
        }

        return SIMCOM_OK;
    } else {
        puts("[SIMCOM] simcom_set_sim_inserted_status_reporting() ERROR");

        return SEND_CMD_WAIT_OK_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get PIN status
 *
 * @param[in]   simcom_dev             Device to operate on
 * @param[out]  simcom_cpin_resp       Structure with response data
 * 
 * AT+CPIN Enter PIN
 * Get PIN status
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     UNKNOWN_RESP            - ERROR: Unknown response
 */
int simcom_get_pin_status(simcom_dev_t       *simcom_dev,
                           simcom_cpin_resp_t *simcom_cpin_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* NULL ptr */
    if (simcom_cpin_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* Get alphanumeric string indicating whether some password is required or not. */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+CPIN?", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT); 
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+CPIN? ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, simcom_dev->at_dev_resp);

    /* Parse string */
    if(strcmp("+CPIN: READY", simcom_dev->at_dev_resp) == 0) {
        puts("[SIMCOM] MT is not pending for any password");
        *simcom_cpin_resp = READY;

    } else if (strcmp("+CPIN: SIM PIN", simcom_dev->at_dev_resp) == 0) {
        puts("[SIMCOM] MT is waiting SIM PIN to be given");
        *simcom_cpin_resp = SIM_PIN;

    } else if (strcmp("+CPIN: SIM PUK", simcom_dev->at_dev_resp) == 0) {
        puts("[SIMCOM] MT is waiting for SIM PUK to be given");
        *simcom_cpin_resp = SIM_PUK;

    } else if (strcmp("+CPIN: PH_SIM PIN", simcom_dev->at_dev_resp) == 0) {
        puts("[SIMCOM] ME is waiting for phone to SIM card (antitheft)");
        *simcom_cpin_resp = PH_SIM_PIN;

    } else if (strcmp("+CPIN: PH_SIM PUK", simcom_dev->at_dev_resp) == 0) {
        puts("[SIMCOM] ME is waiting for SIM PUK (antitheft)");
        *simcom_cpin_resp = PH_SIM_PUK;

    } else if (strcmp("+CPIN: SIM PIN2", simcom_dev->at_dev_resp) == 0) {
        puts("[SIMCOM] SIM PIN2");
        *simcom_cpin_resp = SIM_PIN2;

    } else if (strcmp("+CPIN: SIM PUK2", simcom_dev->at_dev_resp) == 0) {
        puts("[SIMCOM] SIM PUK2");
        *simcom_cpin_resp = SIM_PUK2;

    } else if (strcmp("ERROR", simcom_dev->at_dev_resp) == 0) {
        puts("[SIMCOM] SIM ERROR (may not be installed SIM card)");

        return UNKNOWN_RESP;
    } else {
        puts("[SIMCOM] Unknown response");

        return UNKNOWN_RESP;
    }

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get network registration
 *
 * @param[in]   simcom_dev             Device to operate on
 * @param[out]  simcom_creg_resp       Structure with response data
 * 
 * AT+CREG Network registration
 * Get network registration
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int simcom_get_network_registration(simcom_dev_t       *simcom_dev,
                                     simcom_creg_resp_t *simcom_creg_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* NULL ptr */
    if (simcom_creg_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* Get Network Registration */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+CREG?", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT); 
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+CREG? ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, simcom_dev->at_dev_resp);

    /* Parse string */
    res = sscanf(simcom_dev->at_dev_resp, "+CREG: %i,%i", &simcom_creg_resp->n,
                                                           &simcom_creg_resp->stat);

    /* Check result */
    if (res != 2) {
        puts("[SIMCOM] Parse error");

        return PARSE_ERROR;
    }

    /* Debug output */
    DEBUG("n = %i\n",    simcom_creg_resp->n);
    DEBUG("stat = %i\n", simcom_creg_resp->stat);

    return SIMCOM_OK;    
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get reject incoming call 
 *
 * @param[in]   simcom_dev             Device to operate on
 * @param[out]  simcom_gsmbusy_resp    Structure with response data
 * 
 * AT+GSMBUSY Reject incoming call
 * Get reject incoming call 
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 * @returns     UNKNOWN_RESP            - ERROR: Unknown response
 */
int simcom_get_reject_incoming_call(simcom_dev_t          *simcom_dev,
                                     simcom_gsmbusy_resp_t *simcom_gsmbusy_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* NULL ptr */
    if (simcom_gsmbusy_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }
    
    /* Send AT command */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+GSMBUSY?", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT);
    
    /* Check return code */
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+GSMBUSY? ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Parse string */
    int mode;
    res = sscanf(simcom_dev->at_dev_resp, "+GSMBUSY: %i", &mode);

    /* Check result */
    if (res != 1) {
        puts("[SIMCOM] Parse error");

        return PARSE_ERROR;
    }

    /* Print result */
    switch (mode) {
        case ENABLE_INCOMING_CALL:
            puts("[SIMCOM] Enable incoming call");

            break;
        case FORBID_ALL_INCOMING_CALLS:
            puts("[SIMCOM] Forbid all incoming calls");

            break;
        case FORBID_INCOMING_VOICE_CALLS_BUT_ENABLE_CSD_CALLS:
            puts("[SIMCOM] Forbid incoming voice calls but enable CSD calls");

            break;
        default:
            puts("[SIMCOM] Unknow mode");
            
            return UNKNOWN_RESP;
    }

    /* Copy mode from resp */
    *simcom_gsmbusy_resp = mode;

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set reject incoming call 
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   mode            Mode
 * 
 * AT+GSMBUSY Reject incoming call
 * Set reject incoming call 
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int simcom_set_reject_incoming_call(simcom_dev_t *simcom_dev, 
                                     uint8_t        mode) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
    
    /* Test range argument */
    if (mode > 2) {
        printf("[SIMCOM] simcom_set_reject_incoming_call() ERROR argument: %i. (range 0-2)\n", mode);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Create a command to send data */
    char cmd_GSMBUSYn[13];
    snprintf(cmd_GSMBUSYn, 13, "AT+GSMBUSY=%i", mode);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&simcom_dev->at_dev, cmd_GSMBUSYn, SIMCOM_MAX_TIMEOUT);

    /* Return result */
    if (res == SIMCOM_OK) {
        /* Print result */
        if (mode == 0) {
            puts("[SIMCOM] Enable incoming call");
        } else if (mode == 1) {
            puts("[SIMCOM] Forbid all incoming calls");
        } else {
            puts("[SIMCOM] Forbid incoming voice calls but enable CSD calls");
        }

        return SIMCOM_OK;
    } else {
        puts("[SIMCOM] simcom_set_reject_incoming_call() ERROR");

        return SEND_CMD_WAIT_OK_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get signal quality report
 *
 * @param[in]   simcom_dev             Device to operate on
 * @param[out]  simcom_csq_resp        Structure with response data
 * 
 * AT+CSQ Signal quality report
 * Get signal quality report
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int simcom_get_signal_quality_report(simcom_dev_t      *simcom_dev,
                                      simcom_csq_resp_t *simcom_csq_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* NULL ptr */
    if (simcom_csq_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* Get network registration */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+CSQ", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT); 
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+CSQ ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, simcom_dev->at_dev_resp);

    /* Parse string */
    int rssi;
    res = sscanf(simcom_dev->at_dev_resp, "+CSQ: %i,%i", &rssi,
                                                          &simcom_csq_resp->ber);

    /* Check result */
    if (res != 2) {
        puts("[SIMCOM] Parse error");

        return PARSE_ERROR;
    }

    /* Calculate in -dBm */
    if (rssi == 0) {
        simcom_csq_resp->rssi = 115;
    } else if (rssi == 1) {
        simcom_csq_resp->rssi = 111;
    } else if ((rssi > 1) && (rssi <= 30)) {
        simcom_csq_resp->rssi = 110 - ((rssi - 2) * 2);
    } else if (rssi == 31) {
        simcom_csq_resp->rssi = 52;
    } else 
        simcom_csq_resp->rssi = 115; 

    /* Debug output */
    DEBUG("rssi = %i\n", simcom_csq_resp->rssi);
    DEBUG("ber = %i\n",  simcom_csq_resp->ber);

    return SIMCOM_OK; 
}   

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get operator selection
 *
 * @param[in]   simcom_dev             Device to operate on
 * @param[out]  simcom_cops_resp       Structure with response data
 * 
 * AT+COPS Operator selection
 * Get operator selection
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int simcom_get_operator_selection(simcom_dev_t       *simcom_dev,
                                   simcom_cops_resp_t *simcom_cops_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* NULL ptr */
    if (simcom_cops_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* Get SIM Inserted Status Reporting */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+COPS?", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT); 
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+COPS? ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, simcom_dev->at_dev_resp);

    /* Parse string */ 
    res = sscanf(simcom_dev->at_dev_resp, "+COPS: %i,%i,\"%[^\"]s\"", &simcom_cops_resp->mode,
                                                                       &simcom_cops_resp->format,
                                                                        simcom_cops_resp->oper);

    /* Check result */
    if (res == 1) {
        /* Debug output */
        DEBUG("mode = %i\n", simcom_cops_resp->mode);

        return SIMCOM_OK;
    } else if (res == 3) {
        DEBUG("mode = %i\n", simcom_cops_resp->mode);
        DEBUG("format = %i\n", simcom_cops_resp->format);
        DEBUG("oper = %s\n", simcom_cops_resp->oper);
        // DEBUG("act = %i\n", simcom_cops_resp->act);

        return SIMCOM_OK;
    } else {
        puts("[SIMCOM] Parse error");

        return PARSE_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set PDP context activate or deactivate 
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   state           Indicates the state of PDP context activation
 * @param[in]   cid             A numeric parameter which specifies a particular PDP context definition (see +CGDCONT Command). 
 *                              If the <cid> is omitted, it only affects the first cid.
 * 
 * AT+CGACT PDP context activate or deactivate
 * Set PDP context activate or deactivate 
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int simcom_set_state_pdp_context(simcom_dev_t *simcom_dev,
                                  uint8_t        state,
                                  uint8_t        cid) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
    
    /* Test range argument */
    if (state > 1) {
        printf("[SIMCOM] simcom_set_sim_inserted_status_reporting() ERROR argument: %i. (range 0-1)\n", state);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Create a command to send data */
    char cmd_CGACTn[20];
    if (cid == 0) {
        snprintf(cmd_CGACTn, 20, "AT+CGACT=%i", state);
    } else {
        snprintf(cmd_CGACTn, 20, "AT+CGACT=%i,%i", state, cid); 
    }
    
    /* Send AT command */
    int res = at_send_cmd_wait_ok(&simcom_dev->at_dev, cmd_CGACTn, 7000000); 

    /* Return result */
    if (res == SIMCOM_OK) {
        /* Print result */
        if (state == 0) {
            puts("[SIMCOM] Deactivated PDP context");
        } else {
            puts("[SIMCOM] Activated PDP context");
        }

        return SIMCOM_OK;
    } else {
        puts("[SIMCOM] cmd_CGACTn ERROR");

        return SEND_CMD_WAIT_OK_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get IMSI
 *
 * @param[in]   simcom_dev     Device to operate on
 * 
 * AT+CIMI Request International Mobile Subscriber Identity (IMSI)
 * Get IMSI
 * 
 * @returns     Pointer to a string - OK
 * @returns     NULL                - ERROR
 */
char *simcom_get_imsi(simcom_dev_t *simcom_dev) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return NULL;
    } 

    /* Send AT command */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+CIMI", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT);
    
    /* Return result */
    if (res > SIMCOM_OK) {
        printf("[SIMCOM] IMSI: %s\n", simcom_dev->at_dev_resp);

        return simcom_dev->at_dev_resp;
    } else {
        puts("[SIMCOM] simcom_get_imsi() ERROR");
        return NULL;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get HNI
 *
 * @param[in]   simcom_dev     Device to operate on
 * 
 * Get Home Network Identity (HNI)
 * 
 * @returns     SIMCOM_OK >= 0   - HNI
 * @returns     SIMCOM_DEV_ERROR - ERROR: simcom_dev == NULL
 * @returns     UNDEFINED_ERROR   - ERROR: Undefined error 
 */
int simcom_get_hni(simcom_dev_t *simcom_dev) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* Get IMSI */
    char* imsi = simcom_get_imsi(simcom_dev);
    if (imsi == NULL) {
        return UNDEFINED_ERROR;
    }

    /* Get HNI */
    imsi[5] = 0x00;
    int hni = atoi(imsi);
    
    return hni;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get GPRS service state
 *
 * @param[in]   simcom_dev             Device to operate on
 * @param[out]  simcom_cgatt_resp      Structure with response data
 * 
 * AT+CGATT Get GPRS service state
 * Get GPRS service state
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int simcom_get_gprs_service_state(simcom_dev_t        *simcom_dev,
                                   simcom_cgatt_resp_t *simcom_cgatt_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
    
    /* NULL ptr */
    if (simcom_cgatt_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* Send AT command */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+CGATT?", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT);
    
    /* Check return code */
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+CGATT? ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Parse string */
    res = sscanf(simcom_dev->at_dev_resp, "+CGATT: %i", &simcom_cgatt_resp->state);

    /* Check result */
    if (res != 1) {
        puts("[SIMCOM] Parse error");

        return PARSE_ERROR;
    }

    /* Print result */
    if (simcom_cgatt_resp->state == 1) {
        puts("[SIMCOM] GPRS attached");
    } else {
        puts("[SIMCOM] GPRS detached");
    }

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set GPRS service state
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   state           Indicates the state of PDP context activation
 * 
 * AT+CGATT Set GPRS service state
 * Set GPRS service state
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int simcom_set_gprs_service_state(simcom_dev_t *simcom_dev, 
                                   uint8_t        state) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
    
    /* Test range argument */
    if (state > 1) {
        printf("[SIMCOM] simcom_set_gprs_service_state() ERROR argument: %i. (range 0-1)\n", state);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Create a command to send data */
    char cmd_CGATTn[13];
    snprintf(cmd_CGATTn, 13, "AT+CGATT=%i", state);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&simcom_dev->at_dev, cmd_CGATTn, 7000000);

    /* Return result */
    if (res == SIMCOM_OK) {
        /* Print result */
        if (state == 1) {
            puts("[SIMCOM] GPRS attached");
        } else {
            puts("[SIMCOM] GPRS detached");
        }

        return SIMCOM_OK;
    } else {
        puts("[SIMCOM] cmd_CGATTn ERROR");

        return SEND_CMD_WAIT_OK_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set start task and set APN, USER NAME, PASSWORD
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   apn             A string parameter which indicates the GPRS access point name
 * @param[in]   user            A string parameter which indicates the GPRS user name
 * @param[in]   password        A string parameter which indicates the GPRS password
 * 
 * AT+CSTT Set start task and set APN, USER NAME, PASSWORD
 * Set start task and set APN, USER NAME, PASSWORD
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR    - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int simcom_set_network_settings(simcom_dev_t *simcom_dev,
                                 char          *apn,
                                 char          *user,
                                 char          *password) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* Test arguments */
    if ((apn == NULL) || (user == NULL) || (password == NULL)) {
        puts("Arguments = NULL");

        return ARGUMENT_NULL_ERROR;
    } 

    /* Create a command to send data */
    char cmd_with_settings_for_internet[128];
    snprintf(cmd_with_settings_for_internet, 128, "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, 
                                                                                  user, 
                                                                                  password);

    /* Sleep on 50 ms */
    lptimer_usleep(50);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&simcom_dev->at_dev, cmd_with_settings_for_internet, SIMCOM_MAX_TIMEOUT);

    /* Return result */
    if (res == SIMCOM_OK) {
        /* Print internet settings */
        printf("[SIMCOM] Set internet settings: APN=\"%s\", Username=\"%s\", Password=\"%s\"\n", apn, 
                                                                                                  user, 
                                                                                                  password);

        return SIMCOM_OK;
    } else {
        puts("[SIMCOM] cmd_with_settings_for_internet ERROR");

        return SEND_CMD_WAIT_OK_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Bring up wireless connection with GPRS
 *
 * @param[in]   simcom_dev     Device to operate on
 * 
 * AT+CIICR Bring up wireless connection with GPRS
 * Bring up wireless connection with GPRS
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int simcom_bring_up_wireless_connection(simcom_dev_t *simcom_dev) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&simcom_dev->at_dev, "AT+CIICR", 6000000);

    /* Return result */
    if (res == SIMCOM_OK) {
        /* Print result */
        puts("[SIMCOM] Bring up wireless connection with GPRS");

        return SIMCOM_OK;
    } else {
        puts("[SIMCOM] simcom_bring_up_wireless_connection() ERROR");

        return SEND_CMD_WAIT_OK_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get local IP address
 *
 * @param[in]   simcom_dev             Device to operate on
 * @param[out]  simcom_cifsr_resp      Structure with response data
 * 
 * AT+CIFSR Get local IP address
 * Get local IP address
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int simcom_get_local_ip_address(simcom_dev_t        *simcom_dev,
                                 simcom_cifsr_resp_t *simcom_cifsr_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");
        
        return SIMCOM_DEV_ERROR;
    } 

    /* NULL ptr */
    if (simcom_cifsr_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* Get local IP address */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, "AT+CIFSR", simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT); 
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+CIFSR ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, simcom_dev->at_dev_resp);

    /* Parse string */
    res = sscanf(simcom_dev->at_dev_resp, "%i.%i.%i.%i", &simcom_cifsr_resp->local_ip_address[0],
                                                          &simcom_cifsr_resp->local_ip_address[1],
                                                          &simcom_cifsr_resp->local_ip_address[2],
                                                          &simcom_cifsr_resp->local_ip_address[3]);

    /* Check result */
    if (res != 4) {
        puts("[SIMCOM] Parse error");

        return PARSE_ERROR;
    }

    /* Print result */
    printf("[SIMCOM] Local IP address: %i.%i.%i.%i\n", simcom_cifsr_resp->local_ip_address[0],
                                                        simcom_cifsr_resp->local_ip_address[1],
                                                        simcom_cifsr_resp->local_ip_address[2],
                                                        simcom_cifsr_resp->local_ip_address[3]);

    return SIMCOM_OK;    
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Query current connection status
 *
 * @param[in]   simcom_dev                 Device to operate on
 * @param[in]   n                           A numeric parameter which indicates the connection number
 * @param[out]  simcom_set_cipstatus_resp  Structure with response data
 * 
 * AT+CIPSTATUS Query current connection status
 * Query current connection status
 * 
 * @returns     SIMCOM_OK              - OK
 * @returns     SIMCOM_DEV_ERROR       - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR    - ERROR: Invalid argument value
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int simcom_set_query_current_connection_status(simcom_dev_t                *simcom_dev,
                                                uint8_t                       n,
                                                simcom_set_cipstatus_resp_t *simcom_set_cipstatus_resp) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
    
    /* Test range argument */
    if (n > 7) {
        printf("[SIMCOM] simcom_set_gprs_service_state() ERROR argument: %i. (range 0-7)\n", n);

        return ARGUMENT_RANGE_ERROR;
    }

    /* NULL ptr */
    if (simcom_set_cipstatus_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* Create a command to send data */
    char cmd_CIPSTATUSn[15];
    snprintf(cmd_CIPSTATUSn, 15, "AT+CIPSTATUS=%i", n);    
    
    /* Set query current connection status */
    int res = at_send_cmd_get_resp(&simcom_dev->at_dev, cmd_CIPSTATUSn, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, SIMCOM_MAX_TIMEOUT); 
    if (res <= SIMCOM_OK) {
        puts("[SIMCOM] AT+CIFSR ERROR");

        return SEND_CMD_GET_RESP_ERROR;
    }

    /* Debug output */
    DEBUG("len: %i, data: %s\n", res, simcom_dev->at_dev_resp);

    uint8_t  resp_len = res;
    char    *resp = simcom_dev->at_dev_resp;
    uint8_t  parse_buffer[32];
    uint8_t  parse_len;
    uint8_t  start_parse;
    uint8_t  end_parse;

    if (memcmp(resp, "+CIPSTATUS: ", 12) != 0) {
        puts("[SIMCOM] Parse error");

        return PARSE_ERROR;
    }

    start_parse = 12;
    end_parse   = 0;

    /* Parse n */
    for (uint8_t i = start_parse; i < resp_len; i++) {
        if (resp[i] == ',') {
            if (start_parse == i) {
                return PARSE_ERROR;
            } else {
                end_parse = i;
                parse_len = end_parse - start_parse;

                memcpy(parse_buffer, resp + start_parse, end_parse - start_parse);
                parse_buffer[parse_len] = 0x00;

                res = sscanf((char*)parse_buffer, "%i", &simcom_set_cipstatus_resp->n);
                if (res != 1) {
                    puts("[SIMCOM] Parse error: n");

                    return PARSE_ERROR;
                }

                DEBUG("simcom_set_cipstatus_resp->n: %i\n", simcom_set_cipstatus_resp->n);

                break;
            }
        }
    }

    start_parse = end_parse + 1;

    /* Parse bearer */
    for (uint8_t i = start_parse; i < resp_len; i++) {
        if (resp[i] == ',') {
            if (start_parse == i) {
                end_parse = i;

                simcom_set_cipstatus_resp->bearer = 0;
                DEBUG("simcom_set_cipstatus_resp->bearer: %i\n", simcom_set_cipstatus_resp->bearer);

                break;
            } else {
                end_parse = i;
                parse_len = end_parse - start_parse;

                memcpy(parse_buffer, resp + start_parse, end_parse - start_parse);
                parse_buffer[parse_len] = 0x00;

                res = sscanf((char*)parse_buffer, "%i", &simcom_set_cipstatus_resp->bearer);
                if (res != 1) {
                    puts("[SIMCOM] Parse error: bearer");

                    return PARSE_ERROR;
                }

                printf("simcom_set_cipstatus_resp->bearer: %i\n", simcom_set_cipstatus_resp->bearer);

                break;
            }
        }
    }

    start_parse = end_parse + 1;

    /* Parse type_connection */
    for (uint8_t i = start_parse; i < resp_len; i++) {
        if (resp[i] == ',') {
            if (start_parse == i) {
                puts("[SIMCOM] Parse error: type_connection");

                return PARSE_ERROR;
            } else {
                end_parse = i;
                parse_len = end_parse - start_parse;

                if (parse_len == 2) {
                    memset(simcom_set_cipstatus_resp->type_connection, 0x00, sizeof(simcom_set_cipstatus_resp->type_connection));
                } else if (parse_len == 5) {
                    memcpy(simcom_set_cipstatus_resp->type_connection, resp + start_parse + 1, end_parse - start_parse - 2);
                    simcom_set_cipstatus_resp->type_connection[end_parse - start_parse - 2] = 0x00;
                } else {
                    puts("[SIMCOM] Parse error: type_connection");

                    return PARSE_ERROR;
                }

                DEBUG("simcom_set_cipstatus_resp->type_connection = \"%s\"\n", simcom_set_cipstatus_resp->type_connection);

                break;
            }
        }
    }

    start_parse = end_parse + 1;

    /* Parse ip_address */
    for (uint8_t i = start_parse; i < resp_len; i++) {
        if (resp[i] == ',') {
            if (start_parse == i) {
                puts("[SIMCOM] Parse error: ip_address");

                return PARSE_ERROR;
            } else {
                end_parse = i;
                parse_len = end_parse - start_parse;

                if (parse_len == 2) {
                    memset(simcom_set_cipstatus_resp->ip_address, 0x00, sizeof(simcom_set_cipstatus_resp->ip_address));
                } else {
                    memcpy(simcom_set_cipstatus_resp->ip_address, resp + start_parse + 1, end_parse - start_parse - 2);
                    simcom_set_cipstatus_resp->ip_address[end_parse - start_parse - 2] = 0x00;
                }

                DEBUG("simcom_set_cipstatus_resp->ip_address = \"%s\"\n", simcom_set_cipstatus_resp->ip_address);

                break;
            }
        }
    }

    start_parse = end_parse + 1;

    /* Parse port */
    for (uint8_t i = start_parse; i < resp_len; i++) {
        if (resp[i] == ',') {
            if (start_parse == i) {
                puts("[SIMCOM] Parse error: port");

                return PARSE_ERROR;
            } else {
                end_parse = i;
                parse_len = end_parse - start_parse;

                if (parse_len == 2) {
                    memset(simcom_set_cipstatus_resp->port, 0x00, sizeof(simcom_set_cipstatus_resp->port));
                } else {
                    memcpy(simcom_set_cipstatus_resp->port, resp + start_parse + 1, end_parse - start_parse - 2);
                    simcom_set_cipstatus_resp->port[end_parse - start_parse - 2] = 0x00;
                }

                DEBUG("simcom_set_cipstatus_resp->port = \"%s\"\n", simcom_set_cipstatus_resp->port);

                break;
            }
        }
    }

    start_parse = end_parse + 1;
    end_parse   = resp_len;

    /* Parse port */
    memcpy(simcom_set_cipstatus_resp->client_state, resp + start_parse + 1, end_parse - start_parse - 2);
    simcom_set_cipstatus_resp->client_state[end_parse - start_parse - 2] = 0x00;

    DEBUG("simcom_set_cipstatus_resp->client_state = \"%s\"\n", simcom_set_cipstatus_resp->client_state);

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Start up multi-IP connection
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   n               Mode: single IP connection or multi IP connection
 * 
 * AT+CIPMUX Start up multi-IP connection
 * Start up multi-IP connection
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int simcom_start_up_multi_ip_connection(simcom_dev_t *simcom_dev, 
                                         uint8_t        n) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
    
    /* Test range argument */
    if (n > 1) {
        printf("[SIMCOM] simcom_start_up_multi_ip_connection() ERROR argument: %i. (range 0-1)\n", n);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Create a command to send data */
    char cmd_CIPMUXn[12];
    snprintf(cmd_CIPMUXn, 12, "AT+CIPMUX=%i", n);

    /* Send AT command */
    int res = at_send_cmd_wait_ok(&simcom_dev->at_dev, cmd_CIPMUXn, SIMCOM_MAX_TIMEOUT);

    /* Return result */
    if (res == SIMCOM_OK) {
        /* Print result */
        if (n == 0) {
            puts("[SIMCOM] Set single-IP connection");
        } else {
            puts("[SIMCOM] Set multi-IP connection");
        }

        return SIMCOM_OK;
    } else {
        puts("[SIMCOM] cmd_CIPMUXn ERROR");

        return SEND_CMD_WAIT_OK_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Close up multi-IP connection
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   id              A numeric parameter which indicates the connection number 
 * @param[in]   n               Closing type: slow or quick close 
 * 
 * AT+CIPCLOSE Close up multi-IP connection
 * Close up multi-IP connection
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_ERROR         - ERROR: at_send_cmd() != 0
 * @returns     UNKNOWN_RESP           - ERROR: Unknown response
 */
int simcom_close_up_multi_ip_connection(simcom_dev_t *simcom_dev, 
                                         uint8_t        id,  
                                         uint8_t        n) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 
    
    /* Test id range argument*/
    if (id > 7) {
        printf("[SIMCOM] simcom_close_up_multi_ip_connection() ERROR argument id: %i. (range 0-7)\n", id);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Test n range argument */
    if (n > 1) {
        printf("[SIMCOM] simcom_close_up_multi_ip_connection() ERROR argument: %i. (range 0-1)\n", n);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Create a command to send data */
    char cmd_CIPCLOSEn[17];
    snprintf(cmd_CIPCLOSEn, 17, "AT+CIPCLOSE=%i,%i", id, n);

    /* Send AT command */
    at_drain(&simcom_dev->at_dev);
    int res = at_send_cmd(&simcom_dev->at_dev, cmd_CIPCLOSEn, SIMCOM_MAX_TIMEOUT);
    if (res != SIMCOM_OK) {
        return SEND_CMD_ERROR;
    }

    /* Create resp string */
    char resp_on_CIPCLOSE[13];
    snprintf(resp_on_CIPCLOSE, 13, "%i, CLOSE OK", n);

    /* Read string */
    res = at_readline(&simcom_dev->at_dev, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, false, SIMCOM_MAX_TIMEOUT);
    DEBUG("res = %i, data: %s\n", res, simcom_dev->at_dev_resp);

    /* Check read len string */
    if (res < SIMCOM_OK) {
        return READLINE_ERROR;
    }

    /* Compare resp */
    if (memcmp(resp_on_CIPCLOSE, simcom_dev->at_dev_resp, 11) != 0) {
        return UNKNOWN_RESP;
    }

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/* AT+CIPPING PING request */
int simcom_ping_request(simcom_dev_t          *simcom_dev,
                         simcom_cipping_resp_t  simcom_cipping_resp[],
                         char                   *address,
                         char                   *retr_num,
                         char                   *datalen, 
                         char                   *timeout,
                         char                   *ttl) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }   

    /* NULL ptr */
    if (simcom_cipping_resp == NULL) {
        return ARGUMENT_NULL_ERROR;
    }

    /* NULL ptr */
    if (address == NULL) {
        return ARGUMENT_NULL_ERROR;
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
            puts("[SIMCOM] Unknow num arg");
            
            return UNDEFINED_ERROR;
    }

    /* Debug output */
    DEBUG("num_arg: %i, cmd_CIPPING: %s\n", num_arg, cmd_CIPPING);

    /* Send AT command */
    at_drain(&simcom_dev->at_dev);
    int res = at_send_cmd(&simcom_dev->at_dev, cmd_CIPPING, 7000000);
    if (res != SIMCOM_OK) {
        return SEND_CMD_ERROR;
    }

    /* Sleep on 10000 ms */
    lptimer_usleep(10000);

    int reply_id_scan;
    int reply_time_scan; 
    int ttl_scan;   

    /* Create format string */
    char format[64];
    snprintf(format, 64, "+CIPPING: %%i,\\\"%s\\\",%%i,%%i", address);
    DEBUG("format: %s\n", format);
    do {
        /* Read string */
        res = at_readline(&simcom_dev->at_dev, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, false, SIMCOM_MAX_TIMEOUT);
        DEBUG("res = %i, data: %s\n", res, simcom_dev->at_dev_resp);

        /* Check read len string */
        if (res < SIMCOM_OK) {
            return READLINE_ERROR;
        }

        /* TODO: PARSE ERROR BECAUSE IP ADDREESS HAVE: "" */
        /* Parse string */
        res = sscanf(simcom_dev->at_dev_resp, format, &reply_id_scan, &reply_time_scan, &ttl_scan);

        /* Check result */
        if (res != 3) {
            printf("[SIMCOM] Parse error: %i\n", res);

            continue;
        }

        simcom_cipping_resp[reply_id_scan - 1].reply_time = reply_time_scan;      
        simcom_cipping_resp[reply_id_scan - 1].ttl = ttl_scan;    

        if (num_arg > 1) {
            DEBUG("atoi(%s) = %i", retr_num, atoi(retr_num));
            if (reply_id_scan == atoi(retr_num)) {
                break;
            }
        }
    } while (res >= 0);

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Start up multi-IP TCP or UDP connection
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   n               A numeric parameter which indicates the connection number
 * @param[in]   mode            A string parameter which indicates the connection type: TCP or UDP
 * @param[in]   address         A string parameter which indicates remote server IP address or domain name
 * @param[in]   port            A string parameter which indicates remote server port 
 * 
 * AT+CIPSTART Start up multi-IP TCP or UDP connection
 * Start up multi-IP TCP or UDP connection
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     ARGUMENT_NULL_ERROR    - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_ERROR         - ERROR: at_send_cmd() != 0
 * @returns     READLINE_ERROR         - ERROR: at_readline() < 0
 * @returns     UNKNOWN_RESP           - ERROR: Unknown response
 * @returns     TIMEOUT_EXPIRED        - ERROR: Timeout expired
 */
int simcom_start_up_multi_ip_up_connection(simcom_dev_t *simcom_dev,
                                            uint8_t        n,
                                            char          *mode,
                                            char          *address,
                                            char          *port) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }

    /* Test n */
    if (n > 7) {
        printf("[SIMCOM] simcom_start_up_multi_ip_up_connection() ERROR argument n: %i. (range 0-7)\n", n);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Test mode */
    if (mode != NULL) {
        if (!((strcmp(mode, "TCP") == 0) || 
              (strcmp(mode, "UDP") == 0))) {
            printf("mode: %s != (TCP || UDP)\n", mode);

            return ARGUMENT_RANGE_ERROR;
        }
    } else {
        puts("mode = NULL");

        return ARGUMENT_NULL_ERROR;
    }

    /* Test address */
    if (address == NULL) {
        puts("address = NULL");

        return ARGUMENT_NULL_ERROR;
    }  

    /* Test port */
    if (port == NULL) {
        puts("port = NULL");

        return ARGUMENT_NULL_ERROR;
    }  

    /* Create a command to send data */
    char cmd_CIPSTART[128];
    snprintf(cmd_CIPSTART, 128, "AT+CIPSTART=%i,\"%s\",\"%s\",\"%s\"", n, mode, address, port);
    
    /* Debug output */
    DEBUG("cmd_CIPSTART: %s\n", cmd_CIPSTART);

    /* Send AT command */
    at_drain(&simcom_dev->at_dev);
    int res = at_send_cmd(&simcom_dev->at_dev, cmd_CIPSTART, 15000000);
    if (res != SIMCOM_OK) {
        return SEND_CMD_ERROR;
    }

    /* Read string with OK */
    res = at_readline(&simcom_dev->at_dev, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, false, SIMCOM_MAX_TIMEOUT);
    DEBUG("res = %i, data: %s\n", res, simcom_dev->at_dev_resp);
    
    /* Check read len string */
    if (res != 2) {
        return READLINE_ERROR;
    }

    /* Validation of the answer */
    if (strcmp(simcom_dev->at_dev_resp, "OK") != 0) {
        return UNKNOWN_RESP;
    }

    /* Create resp string for compare */
    char connect_ok[15];
    snprintf(connect_ok, 15, "%i, CONNECT OK", n);

    /* Get time now */
    uint32_t timeout = lptimer_now_msec() + 10000;
    DEBUG("timeout: %lu\n", timeout);

    do {
        DEBUG("time_now: %lu\n", lptimer_now_msec());
        /* Check time */
        if (lptimer_now_msec() > timeout) {
            return TIMEOUT_EXPIRED;
        }

        /* Wait 50 ms for start TCP connection */
        lptimer_usleep(50);

        /* Read string */
        res = at_readline(&simcom_dev->at_dev, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, false, SIMCOM_MAX_TIMEOUT);
        DEBUG("res = %i, data: %s\n", res, simcom_dev->at_dev_resp);

        /* Check read len string */
        if (res == 13) {
            /* Validation of the answer */
            if (strcmp(simcom_dev->at_dev_resp, connect_ok) != 0) {
                return UNKNOWN_RESP;
            } else {
                break;
            }
        }
    } while (true);

    printf("[SIMCOM] Start %s connect %i to %s:%s\n", mode, n, address, port);

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Start up multi-IP TCP or UDP connection
 *
 * @param[in]   simcom_dev         Device to operate on
 * @param[in]   mode                Mode
 * @param[in]   n                   A numeric parameter which indicates the connection number
 * @param[out]  data_for_receive    Pointer to a receive buffer
 * @param[in]   data_size           Received data size
 * 
 * AT+CIPSTART Start up multi-IP TCP or UDP connection
 * Start up multi-IP TCP or UDP connection
 * 
 * @returns     SIMCOM_OK             - OK (mode = 1)
 * @returns     res >= 0               - OK, receive_length (mode = 2)
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     ARGUMENT_NULL_ERROR    - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 * @returns     RECEIVE_MAX_LEN_ERROR  - ERROR: data_size > RECEIVE_MAX_LEN
 * @returns     SEND_CMD_ERROR         - ERROR: at_send_cmd() != 0
 * @returns     READLINE_ERROR         - ERROR: at_readline() < 0
 * @returns     PARSE_ERROR            - ERROR: sscanf() != desired number of variables
 * @returns     NOT_IMPLEMENTED        - ERROR: Not implemented 
 * @returns     UNKNOWN_RESP           - ERROR: Unknown response
 * @returns     UNDEFINED_ERROR        - ERROR: Undefined error 
 */

int simcom_receive_data_through_multi_ip_connection(simcom_dev_t *simcom_dev,
                                                     uint8_t        mode,
                                                     uint8_t        n,
                                                     uint8_t       *data_for_receive,
                                                     size_t         data_size) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }  

    /* Test mode */
    if (mode > 4 || mode == 0) {
        printf("[SIMCOM] simcom_receive_data_through_multi_ip_connection() ERROR argument mode: %i. (range 1-4)\n", mode);

        return ARGUMENT_RANGE_ERROR;
    }   

    /* Test n */
    if (n > 7) {
        printf("[SIMCOM] simcom_receive_data_through_multi_ip_connection() ERROR argument n: %i. (range 0-7)\n", n);

        return ARGUMENT_RANGE_ERROR;
    }   

    /* Test data_for_receive on NULL ptr */
    if ((data_for_receive == NULL) && (mode != 1)) {
        puts("data_for_receive = NULL");

        return ARGUMENT_NULL_ERROR;
    } 

    int res;

    /* Create a command to send data */
    char cmd_CIPRXGET[32];
    switch (mode) {
        case 1:
            snprintf(cmd_CIPRXGET, 32, "AT+CIPRXGET=%i", mode);

            /* Send AT command */
            res = at_send_cmd_wait_ok(&simcom_dev->at_dev, cmd_CIPRXGET, SIMCOM_MAX_TIMEOUT);

            /* Return result */
            if (res != SIMCOM_OK) {
                return SEND_CMD_WAIT_OK_ERROR;
            }

            return SIMCOM_OK;
        case 2:
            /* Check on min len on read */
            if (data_size == 0) {
                return 0;
            }

            /* Check on max len on read */
            if (data_size > RECEIVE_MAX_LEN) {
                return RECEIVE_MAX_LEN_ERROR;
            }

            /* Create command */
            snprintf(cmd_CIPRXGET, 32, "AT+CIPRXGET=%i,%i,%i", mode, n, data_size);

            /* Sleep on 50 ms */
            lptimer_usleep(50);

            /* Send AT command */
            at_drain(&simcom_dev->at_dev);
            res = at_send_cmd(&simcom_dev->at_dev, cmd_CIPRXGET, SIMCOM_MAX_TIMEOUT);
            if (res != 0) {    
                return SEND_CMD_ERROR;
            }

            /* Sleep on 10 ms */
            lptimer_usleep(10); 

            /* Get response */
            res = at_readline(&simcom_dev->at_dev, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, false, SIMCOM_MAX_TIMEOUT);
            DEBUG("res = %i, data: %s\n", res, simcom_dev->at_dev_resp);
            
            /* Check read len string */
            if (res < SIMCOM_OK) {
                return READLINE_ERROR;
            }

            /* Parse string */
            int id, req_length, cnf_length;
            res = sscanf(simcom_dev->at_dev_resp, "+CIPRXGET: 2,%i,%i,%i", &id, &req_length, &cnf_length);
            DEBUG("id = %i\n", id);
            DEBUG("req_length = %i\n", req_length);
            DEBUG("cnf_length = %i\n", cnf_length);

            /* Check result */
            if (res != 3) {
                printf("[SIMCOM] Parse error: %i\n", res);

                return PARSE_ERROR;
            }

            /* Check on min len on read */
            if (data_size == 0) {
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
            receive_length = at_recv_bytes(&simcom_dev->at_dev, (char*)data_for_receive, receive_length, SIMCOM_MAX_TIMEOUT);

            /* Read empty string */ 
            res = at_readline(&simcom_dev->at_dev, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, false, SIMCOM_MAX_TIMEOUT);
            DEBUG("res = %i, data: %s\n", res, simcom_dev->at_dev_resp);

            /* Check read len string */
            if (res != 0) {
                return READLINE_ERROR;
            }

            /* Read string with OK */
            res = at_readline(&simcom_dev->at_dev, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, false, SIMCOM_MAX_TIMEOUT);
            DEBUG("res = %i, data: %s\n", res, simcom_dev->at_dev_resp);
            /* Check read len string */
            if (res != 2) {
                return READLINE_ERROR;
            }

            /* Validation of the answer */
            if (strcmp(simcom_dev->at_dev_resp, "OK") != 0) {
                return UNKNOWN_RESP;
            }

#if ENABLE_DEBUG_DATA == 1
            /* Debug data */
            printf("[SIMCOM] Received %i byte:\n", receive_length);
            od_hex_dump(data_for_receive, receive_length, OD_WIDTH_DEFAULT);
#endif

            return receive_length;
        case 3:
            /* TODO: implement */

            return NOT_IMPLEMENTED;
        case 4:
            /* TODO: implement */

            return NOT_IMPLEMENTED;
        default:
            puts("[SIMCOM] Unknow mode");
            
            return UNDEFINED_ERROR;
    }
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Send data through TCP or UDP multi IP connection
 *
 * @param[in]   simcom_dev         Device to operate on
 * @param[in]   n                   A numeric parameter which indicates the connection number
 * @param[in]   data_for_send       Pointer to send buffer
 * @param[in]   data_size           Send data size
 * 
 * AT+CIPSEND Send data through TCP or UDP multi IP connection
 * Send data through TCP or UDP multi IP connection
 * 
 * @returns     res >= 0               - OK, receive_length (mode = 2)
 * @returns     SIMCOM_DEV_ERROR      - ERROR: simcom_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     ARGUMENT_NULL_ERROR    - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_ERROR         - ERROR: at_send_cmd() != 0
 * @returns     INVALID_DATA           - ERROR: Invalid data
 * @returns     UNKNOWN_RESP           - ERROR: Unknown response
 * @returns     TIMEOUT_EXPIRED        - ERROR: Timeout expired
 */
int simcom_send_data_through_multi_ip_connection(simcom_dev_t *simcom_dev,
                                                  uint8_t        n,
                                                  uint8_t       *data_for_send, 
                                                  size_t         data_size) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }     

    /* Test n */
    if (n > 7) {
        printf("[SIMCOM] simcom_send_data_through_multi_ip_connection() ERROR argument n: %i. (range 0-7)\n", n);

        return ARGUMENT_RANGE_ERROR;
    }  

    /* Test data_for_send on NULL ptr */
    if (data_for_send == NULL) {
        puts("data_for_send = NULL");
        return ARGUMENT_NULL_ERROR;
    }  

    /* Check data_size */
    if (data_size == 0) {
        return 0;
    }

    /* CMD with lenth data for send (AT+CIPSEND=n,data_size) */
    char cmd_CIPSEND[22];
    snprintf(cmd_CIPSEND, 22, "AT+CIPSEND=%i,%i", n, data_size);

    /* Send command */
    at_drain(&simcom_dev->at_dev);
    int res = at_send_cmd(&simcom_dev->at_dev, cmd_CIPSEND, SIMCOM_MAX_TIMEOUT);
    if (res != SIMCOM_OK) {
        return SEND_CMD_ERROR;
    } 

    /* Sleep on 30 ms */
    lptimer_usleep(30);

    /* Send data */
    at_send_bytes(&simcom_dev->at_dev, (char*)data_for_send, data_size);

    /* Check on valid data */
    res = at_recv_bytes(&simcom_dev->at_dev, simcom_dev->at_dev_resp, data_size + 4, 3000000);
    if ((!(memcmp(simcom_dev->at_dev_resp, "> ", 2)                                  == 0)) && 
        (!(memcmp((uint8_t*)(simcom_dev->at_dev_resp) + 2, data_for_send, data_size) == 0))) {
        puts("[SIMCOM] Data for send don't valid");

        return INVALID_DATA;
    } 

    /* Create string with resp */
    char resp_on_CIPSEND[11];
    snprintf(resp_on_CIPSEND, 11, "%i, SEND OK", n);

    /* Get time now */
    uint32_t timeout = lptimer_now_msec();
    DEBUG("In timeout: %li\n", timeout);

    /* Try get resp on timeout */
    while ((timeout + 5000) > lptimer_now_msec()) {
        DEBUG("Body timeout: %li\n", lptimer_now_msec());

        res = at_readline(&simcom_dev->at_dev, simcom_dev->at_dev_resp, simcom_dev->at_dev_resp_size, false, SIMCOM_MAX_TIMEOUT);
        DEBUG("res = %i, data: %s\n", res, simcom_dev->at_dev_resp);

        /* Check read len string */
        if (res != 10) {
            lptimer_usleep(10);
            continue;
        }

        /* Compare recv */
        if (memcmp(simcom_dev->at_dev_resp, resp_on_CIPSEND, 10) != 0) {
            return UNKNOWN_RESP;
        }

#if ENABLE_DEBUG_DATA == 1
    /* Debug data */
    printf("[SIMCOM] Sent %i byte:\n", data_size);
    od_hex_dump(data_for_send, data_size, OD_WIDTH_DEFAULT);
#endif

        return data_size;
    }

    return TIMEOUT_EXPIRED; 
}

/*---------------------------------------------------------------------------*/
/*------------------------- END LOW LEVEL FUNCTION --------------------------*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*---------------------------- START SOCKET API -----------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * @brief       Creating socket file descriptor
 *
 * @param[in]   simcom_dev                 Device to operate on
 * 
 * Creating socket file descriptor
 * 
 * @returns     res >= 0               - OK, Socket file descriptor
 * @returns     res <  0               - ERROR: See simcom_error
 */
int simcom_socket(simcom_dev_t *simcom_dev) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }   

    /* Search free socket */
    for(uint8_t i = 0; i < 8; i++) {
        if (!simcom_dev->socketfd[i]) {
            /* Mark as busy */
            simcom_dev->socketfd[i] = true;

            return i;
        }
    }

    return UNABLE_TO_CREATE_SOCKET;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Connect to server
 *
 * @param[in]   simcom_dev                 Device to operate on
 * @param[in]   sockfd                      Socket file descriptor
 * @param[in]   address                     String with address or domain for connect
 * @param[in]   port                        String with port number for connect
 * @param[in]   type                        String with type connection: TCP or UDP
 * 
 * Connect to server
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     res < 0                - ERROR: See simcom_error
 */
int simcom_connect(simcom_dev_t *simcom_dev,
                    int            sockfd, 
                    char          *address,
                    char          *port,
                    char          *type) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }   

    /* Test sockfd */
    if (sockfd > 7) {
        printf("[SIMCOM] simcom_connect() ERROR argument sockfd: %i. (range 0-7)\n", sockfd);

        return ARGUMENT_RANGE_ERROR;
    }

    /* Test NULL address */
    if (address == NULL) {
        puts("address = NULL");

        return ARGUMENT_NULL_ERROR;
    }   

    /* Test NULL port */
    if (port == NULL) {
        puts("port = NULL");

        return ARGUMENT_NULL_ERROR;
    }   

    /* Test NULL type */
    if (type == NULL) {
        puts("type = NULL");

        return ARGUMENT_NULL_ERROR;
    }   

    int res;

    /* Start up multi-IP TCP or UDP connection */
    res = simcom_start_up_multi_ip_up_connection(simcom_dev, sockfd, type, address, port);
    if (res < 0) {
        printf("[SIMCOM] Error start connection: %i\n", res);

        return res;
    }

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Send data via socket
 *
 * @param[in]   simcom_dev         Device to operate on
 * @param[in]   sockfd              Socket file descriptor
 * @param[in]   buffer              Pointer to send buffer
 * @param[in]   buffer_len          Send data size
 * 
 * Send data via socket
 * 
 * @returns     res >= 0               - OK, Send length 
 * @returns     res <  0               - ERROR: See simcom_error
 */
int simcom_send(simcom_dev_t *simcom_dev,
                 int            sockfd, 
                 uint8_t       *buffer,
                 size_t         buffer_len) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }   

    /* Test sockfd */
    if (sockfd > 7) {
        printf("[SIMCOM] simcom_send() ERROR argument sockfd: %i. (range 0-7)\n", sockfd);

        return ARGUMENT_RANGE_ERROR;
    } 

    /* Test NULL buffer */
    if (buffer == NULL) {
        puts("buffer = NULL");

        return ARGUMENT_NULL_ERROR;
    }
 
    int res = simcom_send_data_through_multi_ip_connection(simcom_dev, sockfd, buffer, buffer_len);
    if (res >= 0) {
        return res;
    }

    int error = res;

    /* Get state connection */
    simcom_set_cipstatus_resp_t simcom_set_cipstatus_resp;
    res = simcom_set_query_current_connection_status(simcom_dev,
                                                      0,
                                                      &simcom_set_cipstatus_resp);

    DEBUG("res: %i; resp: %s\n", res, simcom_set_cipstatus_resp.client_state);
    if (res < SIMCOM_OK) {
        return res;
    }

    if (strcmp(simcom_set_cipstatus_resp.client_state, "CONNECTED") == 0) {
        return error;
    } 

    if (strcmp(simcom_set_cipstatus_resp.client_state, "REMOTE CLOSING") == 0) {
        return SOCKET_REMOTE_CLOSING;
    } 

    if (strcmp(simcom_set_cipstatus_resp.client_state, "CLOSING") == 0) {
        return SOCKET_CLOSING;
    } 

    if (strcmp(simcom_set_cipstatus_resp.client_state, "CLOSED") == 0) {
        return SOCKET_CLOSED;
    } 

    return UNDEFINED_ERROR;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Received data via socket
 *
 * @param[in]   simcom_dev         Device to operate on
 * @param[in]   sockfd              Socket file descriptor
 * @param[out]  buffer              Pointer to a receive buffer
 * @param[in]   buffer_len          Received data size
 * 
 * Received data via socket
 * 
 * @returns     res >= 0               - OK, Receive length 
 * @returns     res <  0               - ERROR: See simcom_error
 */
int simcom_receive(simcom_dev_t *simcom_dev,
                    int            sockfd, 
                    uint8_t       *buffer,
                    size_t         buffer_len) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }   

    /* Test sockfd */
    if (sockfd > 7) {
        printf("[SIMCOM] simcom_send() ERROR argument sockfd: %i. (range 0-7)\n", sockfd);

        return ARGUMENT_RANGE_ERROR;
    } 

    /* Test NULL buffer */
    if (buffer == NULL) {
        puts("buffer = NULL");

        return ARGUMENT_NULL_ERROR;
    }   

    int    res;
    size_t recv_sz = 0;
    size_t recv_chank;
    printf("[SIMCOM] Recv need %i byte\n", buffer_len);

    /* Recv data */
    do {
        /* Calculate recv_chank */
        recv_chank = buffer_len - recv_sz; 
        if (recv_chank >= RECEIVE_MAX_LEN) {
            recv_chank = RECEIVE_MAX_LEN;
        }

        /* Recv chank */
        res = simcom_receive_data_through_multi_ip_connection(simcom_dev,
                                                               2,
                                                               sockfd,
                                                               buffer + recv_sz,
                                                               recv_chank);

        /* Exit on error */
        if (res < 0) {
            int error = res;

            /* Get state connection */
            simcom_set_cipstatus_resp_t simcom_set_cipstatus_resp;
            res = simcom_set_query_current_connection_status(simcom_dev,
                                                              0,
                                                              &simcom_set_cipstatus_resp);
            DEBUG("res: %i; resp: %s\n", res, simcom_set_cipstatus_resp.client_state);

            if (res < SIMCOM_OK) {
                return res;
            }

            if (strcmp(simcom_set_cipstatus_resp.client_state, "CONNECTED") == 0) {
                return error;
            } 

            if (strcmp(simcom_set_cipstatus_resp.client_state, "REMOTE CLOSING") == 0) {
                return SOCKET_REMOTE_CLOSING;
            } 

            if (strcmp(simcom_set_cipstatus_resp.client_state, "CLOSING") == 0) {
                return SOCKET_CLOSING;
            } 

            if (strcmp(simcom_set_cipstatus_resp.client_state, "CLOSED") == 0) {
                return SOCKET_CLOSED;
            } 

            return UNDEFINED_ERROR;
        }
        
        recv_sz += res;
        printf("[SIMCOM] Recv %i byte\n", recv_sz);

        /* Exit on no data for receptions */
        if (res == 0) {
            break;
        }
    } while (recv_sz < buffer_len);

    return recv_sz;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Close socket file descriptor
 *
 * @param[in]   simcom_dev         Device to operate on
 * @param[in]   sockfd              Socket file descriptor
 * 
 * Close socket file descriptor
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     res < 0                - ERROR: See simcom_error
 */
int simcom_close(simcom_dev_t *simcom_dev,
                  int            sockfd) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }   

    /* Test sockfd */
    if (sockfd > 7) {
        printf("[SIMCOM] simcom_close() ERROR argument sockfd: %i. (range 0-7)\n", sockfd);

        return ARGUMENT_RANGE_ERROR;
    }

    int res; 

    /* Get state connection */
    simcom_set_cipstatus_resp_t simcom_set_cipstatus_resp;
    res = simcom_set_query_current_connection_status(simcom_dev,
                                                      0,
                                                      &simcom_set_cipstatus_resp);

    DEBUG("res: %i; resp: %s\n", res, simcom_set_cipstatus_resp.client_state);
    if (res < SIMCOM_OK) {
        return res;
    }

    if (strcmp(simcom_set_cipstatus_resp.client_state, "CLOSED") == 0) {
        /* Free the socket */
        simcom_dev->socketfd[sockfd] = false;

        return SIMCOM_OK;
    } 

    if ((strcmp(simcom_set_cipstatus_resp.client_state, "REMOTE CLOSING") == 0) ||
        (strcmp(simcom_set_cipstatus_resp.client_state, "CLOSING")        == 0)) {
        /* Close socket */
        res = simcom_close_up_multi_ip_connection(simcom_dev, 
                                                sockfd,  
                                                0);

        if(res == SIMCOM_OK) {
            /* Free the socket */
            simcom_dev->socketfd[sockfd] = false;

            return SIMCOM_OK;
        }
        do {
            res = simcom_set_query_current_connection_status(simcom_dev,
                                                            0,
                                                            &simcom_set_cipstatus_resp);
            DEBUG("res: %i; resp: %s\n", res, simcom_set_cipstatus_resp.client_state);

            if (res < SIMCOM_OK) {
                return res;
            }

        } while (strcmp(simcom_set_cipstatus_resp.client_state, "CLOSED") != 0);

        /* Free the socket */
        simcom_dev->socketfd[sockfd] = false;

        return SIMCOM_OK;
    } 
    
    /* Close socket */
    res = simcom_close_up_multi_ip_connection(simcom_dev, 
                                               sockfd,  
                                               0);

    if(res != SIMCOM_OK) {
        printf("[SIMCOM] Error close socket %i\n", sockfd); 

        return res;
    }

    /* Free the socket */
    simcom_dev->socketfd[sockfd] = false;

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/*----------------------------- END SOCKET API ------------------------------*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*----------------------- START HIGH LEVEL FUNCTION -------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * @brief       Get internet settings from base
 *
 * @param[in]   simcom_dev                 Device to operate on
 * @param[in]   hni                         Home Network Identity (HNI)
 * @param[out]  simcom_internet_settings   Structure with response data
 * 
 * Get internet settings from base
 * 
 * @returns     SIMCOM_OK                  - OK
 * @returns     NO_INTERNET_SETTINGS_FOUND  - ERROR: No internet settings found  
 */
int simcom_get_internet_settings_from_base(simcom_dev_t               *simcom_dev,
                                            uint32_t                     hni,
                                            simcom_internet_settings_t *simcom_internet_settings) {
    (void)simcom_dev;

    printf("[SIMCOM] Get internet settings from base for HNI: %lu\n", hni);

    /* Get standart settings */
    switch (hni) {
        // +COPN: "25004","SIBCHALLENGE RUS"
        // +COPN: "25028","VOICE"
        // +COPN: "25092","Primetelefone RUS"

        /* +COPN: "25001","MTS RUS" */
        case 25001:
            snprintf(simcom_internet_settings->apn, 32, "internet.mts.ru");   /* Access Point Name */
            snprintf(simcom_internet_settings->username, 32, "mts");          /* Username */
            snprintf(simcom_internet_settings->password, 32, "mts");          /* Password */
            break;

        /* +COPN: "25002","MegaFon RUS" */
        case 25002:
            snprintf(simcom_internet_settings->apn, 32, "internet");      /* Access Point Name */
            memset(simcom_internet_settings->username, 0x00, 32);         /* Username */ 
            memset(simcom_internet_settings->password, 0x00, 32);         /* Password */
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
            snprintf(simcom_internet_settings->apn, 32, "internet.yota");     /* Access Point Name */
            memset(simcom_internet_settings->username, 0x00, 32);             /* Username */
            memset(simcom_internet_settings->password, 0x00, 32);             /* Password */   
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
            snprintf(simcom_internet_settings->apn, 32, "internet");  /* Access Point Name */
            memset(simcom_internet_settings->username, 0x00, 32);     /* Username */ 
            memset(simcom_internet_settings->password, 0x00, 32);     /* Password */
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
            snprintf(simcom_internet_settings->apn, 32, "internet.tele2.ru");     /* Access Point Name */
            memset(simcom_internet_settings->username, 0x00, 32);                 /* Username */
            memset(simcom_internet_settings->password, 0x00, 32);                 /* Password */  
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
            snprintf(simcom_internet_settings->apn, 32, "internet.beeline.ru");   /* Access Point Name */
            snprintf(simcom_internet_settings->username, 32, "beeline");          /* Username */
            snprintf(simcom_internet_settings->password, 32, "beeline");          /* Password */
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
            snprintf(simcom_internet_settings->apn, 32, "internet.rt.ru");    /* Access Point Name */
            memset(simcom_internet_settings->username, 0x00, 32);             /* Username */
            memset(simcom_internet_settings->password, 0x00, 32);             /* Password */ 
            break;

        // /* OJSC Multiregional TransitTelecom (MTT) */
        // case 25042:
        //     /*  */
        //     break;

        /* Tinkoff */
        case 25062:
            snprintf(simcom_internet_settings->apn, 32, "m.tinkoff");     /* Access Point Name */
            memset(simcom_internet_settings->username, 0x00, 32);         /* Username */
            memset(simcom_internet_settings->password, 0x00, 32);         /* Password */  
            break;

        /* +COPN: "25099","Beeline" */
        case 25099:
            snprintf(simcom_internet_settings->apn, 32, "internet.beeline.ru");   /* Access Point Name */
            snprintf(simcom_internet_settings->username, 32, "beeline");          /* Username */
            snprintf(simcom_internet_settings->password, 32, "beeline");          /* Password */
            break;    

        /* Unknown operator */
        default: 
            memset(simcom_internet_settings->apn,      0x00, 32);     /* Access Point Name */
            memset(simcom_internet_settings->username, 0x00, 32);     /* Username */
            memset(simcom_internet_settings->password, 0x00, 32);     /* Password */

            return NO_INTERNET_SETTINGS_FOUND;   
            break;
    } 
    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Start internet
 *
 * @param[in]   simcom_dev                 Device to operate on
 * @param[in]   registration_timeout        Maximum network registration time
 * @param[in]   simcom_internet_settings   Structure with internet settings
 * 
 * Start internet
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     res < 0                - ERROR: See simcom_error
 */
int simcom_start_internet(simcom_dev_t               *simcom_dev,
                           uint8_t                      registration_timeout,
                           simcom_internet_settings_t *simcom_internet_settings) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }     

    int res;

    /* Disabled showing an unsolicited event code */
    res = simcom_set_sim_inserted_status_reporting(simcom_dev, 0);
    if (res != SIMCOM_OK) {
        return res;
    }

    /* Is SIM card inserted? */
    simcom_csmins_resp_t simcom_csmins_resp;
    res = simcom_get_sim_inserted_status_reporting(simcom_dev, &simcom_csmins_resp);
    if (res != SIMCOM_OK) {
        return res;
    }

    /*  */
    if (simcom_csmins_resp.sim_inserted != 1) {
        return NO_SIM_CARD;
    }

    /* Is there a PIN code? */
    simcom_cpin_resp_t simcom_cpin_resp;
    res = simcom_get_pin_status(simcom_dev, &simcom_cpin_resp);

    /*  */
    if (res != SIMCOM_OK) {
        return res;
    }

    /* Check on req any password */
    if (simcom_cpin_resp != READY) {
        return NEED_PASSWORD_FOR_SIM_CARD;
    }

    /* Print message */
    puts("[SIMCOM] Waiting for registration");

    /* Have you registered in the cellular network? */
    /* Cycle counter */
    uint8_t counter = 0;

    /* Response on AT+CREG */
    simcom_creg_resp_t simcom_creg_resp;
    while(1) {
        if (counter >= registration_timeout) {
            puts("[SIMCOM] Registration timeout expired");
            return REGISTRATION_TIMEOUT_EXPIRED;
        }

        /*  */
        res = simcom_get_network_registration(simcom_dev, &simcom_creg_resp);
        if (res == SIMCOM_OK) {
            if (simcom_creg_resp.stat == 1) {
                break;
            }
        }

        lptimer_usleep(1000);
        counter++;
    } 
    puts("[SIMCOM] Registration OK");

    /* Reject Incoming Call */
    res = simcom_set_reject_incoming_call(simcom_dev, 1);
    if (res != SIMCOM_OK) {
        return res;
    }

    /* Start up multi-IP connection */
    res = simcom_start_up_multi_ip_connection(simcom_dev, 1);
    if (res != SIMCOM_OK) {
        return res;
    }

    /* Attach to the network */
    simcom_cgatt_resp_t simcom_cgatt_resp;
    res = simcom_get_gprs_service_state(simcom_dev, &simcom_cgatt_resp);
    if (res != SIMCOM_OK) {
        return res;
    }

    /* Attach to GPRS */
    if (simcom_cgatt_resp.state == 0) {
        res = simcom_set_gprs_service_state(simcom_dev, 1);
        if (res != SIMCOM_OK) {
            return res;
        }
    }

    /* Get data from network manually for multi IP connection */
    res = simcom_receive_data_through_multi_ip_connection(simcom_dev, 1, 1, NULL, 0);
    if(res != SIMCOM_OK) {
        puts("[SIMCOM] Set get data from network manually for multi IP connection ERROR");

        return res;
    } 

    /* Have internet settings? */
    if (simcom_internet_settings != NULL) {
        /* Start Task and Set APN, USER NAME, PASSWORD */
        res = simcom_set_network_settings(simcom_dev, 
                                           simcom_internet_settings->apn, 
                                           simcom_internet_settings->username, 
                                           simcom_internet_settings->password);
        if (res != SIMCOM_OK) {
            return res;
        }
    } else {
        /* Get internet settings from base */
        simcom_internet_settings_t simcom_get_internet_settings;
        res = simcom_get_internet_settings_from_base(simcom_dev,
                                                      simcom_get_hni(simcom_dev),
                                                      &simcom_get_internet_settings);
        if (res != SIMCOM_OK) {
            return res;
        }

        /* Start Task and Set APN, USER NAME, PASSWORD */
        res = simcom_set_network_settings(simcom_dev, 
                                           simcom_get_internet_settings.apn, 
                                           simcom_get_internet_settings.username, 
                                           simcom_get_internet_settings.password);
        if (res != SIMCOM_OK) {
            return res;
        }
    }

    /* Bring Up Wireless Connection with GPRS */
    res = simcom_bring_up_wireless_connection(simcom_dev);
    if (res != SIMCOM_OK) {
        return res;
    }

    /* Get local IP address */
    simcom_cifsr_resp_t simcom_cifsr_resp;
    res = simcom_get_local_ip_address(simcom_dev, &simcom_cifsr_resp);
    if (res != SIMCOM_OK) {
        return res;
    }

    /* Check local address */
    if ((simcom_cifsr_resp.local_ip_address[0] == 0) && 
        (simcom_cifsr_resp.local_ip_address[1] == 0) &&
        (simcom_cifsr_resp.local_ip_address[2] == 0) &&
        (simcom_cifsr_resp.local_ip_address[3] == 0)) {
        puts("[SIMCOM] Zero IP ERROR");

        return NO_LOCAL_IP_ADDRESS;
    }

    /* AT+CDNSCFG="8.8.8.8","8.8.4.4" */

    /* AT+CIPSTATUS */
    /* Get state connection */
    // simcom_set_cipstatus_resp_t simcom_set_cipstatus_resp;
    // res = simcom_set_query_current_connection_status(simcom_dev,
    //                                                   0,
    //                                                   &simcom_set_cipstatus_resp);
    // printf("res: %i\n", res);

    /* ping */
    // simcom_cipping_resp_t simcom_cipping_resp[3] = {};
    // if (simcom_ping_request(simcom_dev,
    //                          simcom_cipping_resp,
    //                          "8.8.8.8",
    //                          "3",
    //                          "32", 
    //                          "100",
    //                          "64")) {
    //     puts("ping true");

    //     for (uint8_t i = 0; i < 3; i++ ) {
    //         printf("simcom_cipping_resp[%i].reply_time = %i; simcom_cipping_resp[%i].ttl = %i\n", i, simcom_cipping_resp[i].reply_time, i, simcom_cipping_resp[i].ttl);
    //     }
    // } else {
    //     puts("ping false");

    //     return false;
    // }

    return SIMCOM_OK;
}

/*---------------------------------------------------------------------------*/
/**
 * @brief       Communication test between microcontroller and SIMCOM
 *
 * @param[in]   simcom_dev         Device to operate on
 *
 * Send 5 AT command
 * 
 * @returns     SIMCOM_OK             - OK
 * @returns     res < 0                - ERROR: See simcom_error
 */
int simcom_communication_test(simcom_dev_t *simcom_dev) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    }     

    int res; 

    /* SIMCOM connection */
    for(int i = 0; i < 5; i++) {
        res = simcom_send_at(simcom_dev);
        if (res == SIMCOM_OK) {
            puts("[SIMCOM] Connection OK");

            return SIMCOM_OK;
        }
        if (i == 4) {
            puts("[SIMCOM] Not answering");

            return SIMCOM_NOT_ANSWERING;
        }

        /* Sleep on 500 ms */
        lptimer_usleep(500);
    }

    return UNDEFINED_ERROR;
} 

/*---------------------------------------------------------------------------*/
/**
 * @brief       Initialize SIMCOM device
 *
 * @param[in]   simcom_dev         Struct to initialize 
 * @param[in]   uart                UART the device is connected to
 * @param[in]   baudrate            Baudrate of the device
 * @param[in]   buf                 Input buffer for AT driver
 * @param[in]   bufsize             Size of @p buf
 * @param[in]   at_dev_resp         Input buffer for parse response from Iridium
 * @param[in]   at_dev_resp_size    Size of @p at_dev_resp
 *
 * @returns     SIMCOM_OK             - OK
 * @returns     res < 0                - ERROR: See simcom_error
 */
int simcom_init(simcom_dev_t *simcom_dev, 
                 uart_t         uart, 
                 uint32_t       baudrate, 
                 char          *buf, 
                 size_t         bufsize, 
                 char          *at_dev_resp, 
                 uint16_t       at_dev_resp_size) {
    /* Test NULL device */
    if (simcom_dev == NULL) {
        puts("simcom_dev = NULL");

        return SIMCOM_DEV_ERROR;
    } 

    int res;
    
    puts("[SIMCOM] Initialization...");
    
    gpio_init(simcom_dev->power_en_pin, GPIO_OUT);
    gpio_init(simcom_dev->gsm_en_pin, GPIO_OUT);

    /* Power on SIMCOM */
    simcom_power_on(simcom_dev);

    /* Structure initialization */
    simcom_dev->at_dev.uart      = uart;
    simcom_dev->at_dev_resp      = at_dev_resp;
    simcom_dev->at_dev_resp_size = at_dev_resp_size;
    for(uint8_t i = 0; i < 8; i++) {
        simcom_dev->socketfd[i]  = false;
    }

    /* Initialization of UART */
    res = at_dev_init(&simcom_dev->at_dev, uart, baudrate, buf, bufsize);
    if (res != SIMCOM_OK) {
        DEBUG("[SIMCOM] at_dev_init() ERROR: %i\n", res);

        return res;
    }
    


    /* SIMCOM connection */
    res = simcom_communication_test(simcom_dev);
    if(res != SIMCOM_OK) {
        DEBUG("[SIMCOM] simcom_communication_test() ERROR: %i\n", res);

        return res;
    }

    puts("[SIMCOM] Init OK");

    return SIMCOM_OK;   
}

/*---------------------------------------------------------------------------*/
/*------------------------- END HIGH LEVEL FUNCTION -------------------------*/
/*---------------------------------------------------------------------------*/

