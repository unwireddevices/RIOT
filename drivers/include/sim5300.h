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
    at_dev_t  at_dev;           /**< AT device structure */
    char     *at_dev_resp;      /**< Input buffer for parse response from SIM5300 */
    uint16_t  at_dev_resp_size; /**< Size of @p at_dev_resp */
} sim5300_dev_t;

/**
 * @brief SIM5300 response on AT+CSMINS
 */
typedef struct {
    int n;                      /**< A numeric parameter to show an unsolicited event code indicating whether the SIM has been inserted or removed */
    int sim_inserted;           /**< A numeric parameter which indicates whether SIM card has been inserted */
} sim5300_csmins_resp_t;

/**
 * @brief SIM5300 response on AT+CSMINS
 */
typedef enum {
    READY      = 0,             /**< MT is not pending for any password */          
    SIM_PIN    = 1,             /**< MT is waiting SIM PIN to be given */
    SIM_PUK    = 2,             /**< MT is waiting for SIM PUK to be given */
    PH_SIM_PIN = 3,             /**< ME is waiting for phone to SIM card (antitheft) */
    PH_SIM_PUK = 4,             /**< ME is waiting for SIM PUK (antitheft) */
    SIM_PIN2   = 5,             /**< PIN2, e.g. for editing the FDN book possible only if preceding Command was acknowledged with +CME ERROR:17 */
    SIM_PUK2   = 6,             /**< Possible only if preceding Command was acknowledged with error +CME ERROR: 18. */
} sim5300_cpin_resp_t;

/**
 * @brief SIM5300 response on AT+CSMINS
 */
typedef struct {
    int n;                      /**< Unsolicited result code */
    int stat;                   /**< Status registration in network */
    // char lac[5];             /**< String type (string should be included in quotation marks); two byte location area code in hexadecimal format */
    // char ci[5];              /**< String type (string should be included in quotation marks); two byte cell ID in hexadecimal format */
    // int  act;                /**<  */
} sim5300_creg_resp_t;

/**
 * @brief SIM5300 response on AT+CSQ
 */
typedef struct {
    int rssi;                   /**< in -dBm */
    int ber;                    /**<  <ber> (in percent):
                                        0...7 As RXQUAL values in the table in GSM 05.08 [20] subclause
                                        7.2.4
                                        99 Not known or not detectable */
} sim5300_csq_resp_t;

/**
 * @brief SIM5300 response on AT+COPS
 */
typedef struct {
    int  mode;                  /**<  */
    int  format;                /**<  */
    char oper[32];              /**<  */
    // int  act;                /**<  */
} sim5300_cops_resp_t;

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
/* AT+CPIN Enter PIN */
int8_t sim5300_get_pin_status(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+CREG Network Registration */
bool sim5300_get_network_registration(sim5300_dev_t       *sim5300_dev,
                                      sim5300_creg_resp_t *sim5300_creg_resp);

/*---------------------------------------------------------------------------*/
/* AT+GSMBUSY Reject Incoming Call */
int8_t sim5300_get_reject_incoming_call(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+GSMBUSY Reject Incoming Call */
bool sim5300_set_reject_incoming_call(sim5300_dev_t *sim5300_dev, 
                                      uint8_t        mode);

/*---------------------------------------------------------------------------*/
/* AT+CSQ Signal Quality Report */
bool sim5300_get_signal_quality_report(sim5300_dev_t      *sim5300_dev,
                                       sim5300_csq_resp_t *sim5300_csq_resp);

/*---------------------------------------------------------------------------*/
/* AT+COPS Operator Selection */
bool sim5300_get_operator_selection(sim5300_dev_t       *sim5300_dev,
                                    sim5300_cops_resp_t *sim5300_cops_resp);

/*---------------------------------------------------------------------------*/
/* AT+CGACT PDP Context Activate or Deactivate */
bool sim5300_set_state_pdp_context(sim5300_dev_t *sim5300_dev,
                                   uint8_t        state,
                                   uint8_t        cid);
                                    
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