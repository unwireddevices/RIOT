/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    Iridium-9602 driver 
 * @ingroup     drivers
 * @brief       Iridium-9602 driver 
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
 * @brief       Iridium-9602 driver 
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

    /* Perform SBD session */
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
        /* Print result  */
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

