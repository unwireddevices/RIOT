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

#ifndef SIM5300_H
#define SIM5300_H

// #include <stdint.h>
// #include <unistd.h>
#include <stdbool.h>

// #include "isrpipe.h"
// #include "periph/uart.h"

#include "at.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SIM5300 device structure
 */
typedef struct {
    at_dev_t at_dev;            /**< AT device structure */
    char *at_dev_resp;          /**< Input buffer for parse response from SIM5300 */
    uint16_t at_dev_resp_size;  /**< Size of @p at_dev_resp */
} sim5300_dev_t;

/**
 * @brief SIM5300 response on AT+CSMINS
 */
typedef struct {
    int n;                      /**< A numeric parameter to show an unsolicited event code indicating whether the SIM has been inserted or removed */
    int sim_inserted;           /**< A numeric parameter which indicates whether SIM card has been inserted */
} sim5300_csmins_resp_t;

/*---------------------------------------------------------------------------*/
/**
 * @brief       Send ATtention Code
 *
 * @param[in]   sim5300_dev         Device to operate on
 * 
 * AT – ATtention Code
 * This is the prefix for all commands except A/. When entered on its own, the 9602 will respond OK.
 * 
 * @returns     true  - Iridium answered OK
 * @returns     false - Iridium not answered
 */
bool sim5300_send_at(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+CSMINS SIM Inserted Status Reporting */
bool sim5300_get_sim_inserted_status_reporting(sim5300_dev_t         *sim5300_dev,
                                               sim5300_csmins_resp_t *sim5300_csmins_resp);

/*---------------------------------------------------------------------------*/
/* AT+CSMINS SIM Inserted Status Reporting */
bool sim5300_set_sim_inserted_status_reporting(sim5300_dev_t *sim5300_dev, 
                                               uint8_t        n);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Communication test between microcontroller and SIM5300
 *
 * @param[in]   sim5300_dev         Device to operate on
 *
 * Send 5 AT command
 * 
 * @returns     true  - Communication test OK
 * @returns     false - Communication test failed ERROR
 */
bool sim5300_communication_test(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Initialize SIM5300 device
 *
 * @param[in]   sim5300_dev         Struct to initialize 
 * @param[in]   uart                UART the device is connected to
 * @param[in]   baudrate            Baudrate of the device
 * @param[in]   buf                 Input buffer for AT driver
 * @param[in]   bufsize             Size of @p buf
 * @param[in]   at_dev_resp         Input buffer for parse response from Iridium
 * @param[in]   at_dev_resp_size    Size of @p at_dev_resp
 *
 * @returns     true  - Initialization OK
 * @returns     false - Initialization ERROR
 */
bool sim5300_init(sim5300_dev_t *sim5300_dev, 
                  uart_t         uart, 
                  uint32_t       baudrate, 
                  char          *buf, 
                  size_t         bufsize, 
                  char          *at_dev_resp, 
                  uint16_t       at_dev_resp_size);

/*---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* SIM5300_H */
/** @} */