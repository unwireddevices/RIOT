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

#define RECEIVE_MAX_LEN (1460) // 1460 /**< Max requested number of data bytes (1-1460 bytes) to be read */

/**
 * @brief SIM5300 ERRORS
 */
enum sim5300_error {
    SIM5300_OK                   =  0,     /*  */
    SIM5300_DEV_ERROR            = -1,     /* sim5300_dev == NULL */
    ARGUMENT_NULL_ERROR          = -2,     /*  */
    ARGUMENT_RANGE_ERROR         = -3,     /*  */
    PARSE_ERROR                  = -4,     /*  */
    // _ERROR                       = -5,     /*  */
    // _ERROR                       = -6,     /*  */
    // _ERROR                       = -7,     /*  */
    // _ERROR                       = -8,     /*  */
    // _ERROR                       = -9,     /*  */
    // _ERROR                       = -0,     /*  */

};

/**
 * @brief SIM5300 device structure
 */
typedef struct {
    at_dev_t  at_dev;           /**< AT device structure */
    char     *at_dev_resp;      /**< Input buffer for parse response from SIM5300 */
    uint16_t  at_dev_resp_size; /**< Size of @p at_dev_resp */
    bool      socketfd[8];      /**< Socket status array */
} sim5300_dev_t;

/**
 * @brief SIM5300 response on AT+CSMINS
 */
typedef struct {
    int n;                      /**< A numeric parameter to show an unsolicited event code indicating whether the SIM has been inserted or removed */
    int sim_inserted;           /**< A numeric parameter which indicates whether SIM card has been inserted */
} sim5300_csmins_resp_t;

/**
 * @brief SIM5300 response on AT+CPIN
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
 * @brief SIM5300 response on AT+CREG
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

/**
 * @brief SIM5300 response on AT+CIFSR
 */
typedef struct {
    int local_ip_address[4];    /**< IP address assigned from GPRS */
} sim5300_cifsr_resp_t;

/**
 * @brief SIM5300 response on AT+CIPPING
 */
typedef struct {
    int reply_time;             /**< Time, in units of 100 ms, required to receive the response */
    int ttl;                    /**< Time to live (1 - 255, Default: 64) */
} sim5300_cipping_resp_t;

/**
 * @brief SIM5300 Internet settings
 */
typedef struct {
    char apn[32];               /**< Access Point Name */
    char username[32];          /**< Username */
    char password[32];          /**< Password */
} sim5300_internet_settings_t;

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
int sim5300_send_at(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+CSMINS SIM Inserted Status Reporting */
int sim5300_get_sim_inserted_status_reporting(sim5300_dev_t         *sim5300_dev,
                                              sim5300_csmins_resp_t *sim5300_csmins_resp);

/*---------------------------------------------------------------------------*/
/* AT+CSMINS SIM Inserted Status Reporting */
int sim5300_set_sim_inserted_status_reporting(sim5300_dev_t *sim5300_dev, 
                                              uint8_t        n);

/*---------------------------------------------------------------------------*/
/* AT+CPIN Enter PIN */
int sim5300_get_pin_status(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+CREG Network Registration */
int sim5300_get_network_registration(sim5300_dev_t       *sim5300_dev,
                                     sim5300_creg_resp_t *sim5300_creg_resp);

/*---------------------------------------------------------------------------*/
/* AT+GSMBUSY Reject Incoming Call */
int sim5300_get_reject_incoming_call(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+GSMBUSY Reject Incoming Call */
int sim5300_set_reject_incoming_call(sim5300_dev_t *sim5300_dev, 
                                     uint8_t        mode);

/*---------------------------------------------------------------------------*/
/* AT+CSQ Signal Quality Report */
int sim5300_get_signal_quality_report(sim5300_dev_t      *sim5300_dev,
                                      sim5300_csq_resp_t *sim5300_csq_resp);

/*---------------------------------------------------------------------------*/
/* AT+COPS Operator Selection */
int sim5300_get_operator_selection(sim5300_dev_t       *sim5300_dev,
                                   sim5300_cops_resp_t *sim5300_cops_resp);

/*---------------------------------------------------------------------------*/
/* AT+CGACT PDP Context Activate or Deactivate */
int sim5300_set_state_pdp_context(sim5300_dev_t *sim5300_dev,
                                  uint8_t        state,
                                  uint8_t        cid);

/*---------------------------------------------------------------------------*/
/* AT+CIMI Request International Mobile Subscriber Identity (IMSI) */
char *sim5300_get_imsi(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* Get Home Network Identity (HNI) */
int sim5300_get_hni(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+CGATT Get GPRS service state */
int sim5300_get_gprs_service_state(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+CGATT Set GPRS service state */
int sim5300_set_gprs_service_state(sim5300_dev_t *sim5300_dev, 
                                   uint8_t        state);

/*---------------------------------------------------------------------------*/
/* AT+CSTT Start Task and Set APN, USER NAME, PASSWORD */
int sim5300_set_network_settings(sim5300_dev_t *sim5300_dev,
                                 char          *apn,
                                 char          *user,
                                 char          *password);

/*---------------------------------------------------------------------------*/
/* AT+CIICR Bring up wireless connection with GPRS */
int sim5300_bring_up_wireless_connection(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/* AT+CIFSR Get local IP address */ 
int sim5300_get_local_ip_address(sim5300_dev_t        *sim5300_dev,
                                 sim5300_cifsr_resp_t *sim5300_cifsr_resp);

/*---------------------------------------------------------------------------*/
/* AT+CIPMUX Start up multi-IP connection */
int sim5300_start_up_multi_ip_connection(sim5300_dev_t *sim5300_dev, 
                                         uint8_t        n);

/*---------------------------------------------------------------------------*/
/* AT+CIPCLOSE Close up multi-IP connection */
int sim5300_close_up_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                         uint8_t        id,
                                         uint8_t        n);

/*---------------------------------------------------------------------------*/
/* AT+CIPPING PING request */
int sim5300_ping_request(sim5300_dev_t          *sim5300_dev,
                         sim5300_cipping_resp_t  sim5300_cipping_resp[],
                         char                   *address,
                         char                   *retr_num,
                         char                   *datalen, 
                         char                   *timeout,
                         char                   *ttl);

/*---------------------------------------------------------------------------*/
/* AT+CIPSTART Start up multi-IP TCP or UDP connection */
int sim5300_multi_ip_up_single_connection(sim5300_dev_t *sim5300_dev,
                                          uint8_t        n,
                                          char          *mode,
                                          char          *address,
                                          char          *port);

/*---------------------------------------------------------------------------*/
/* AT+CIPRXGET Get data from network manually for multi IP connection */
int sim5300_receive_data_through_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                                     uint8_t        mode,
                                                     uint8_t        n,
                                                     uint8_t       *data_for_receive,
                                                     size_t         data_size);

/*---------------------------------------------------------------------------*/
/* AT+CIPSEND Send data through TCP or UDP multi IP connection */
int sim5300_send_data_through_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                                  uint8_t        n,
                                                  uint8_t       *data_for_send, 
                                                  size_t         data_size);

/*---------------------------------------------------------------------------*/
/* Get internet settings from base */
int sim5300_get_internet_settings_from_base(sim5300_dev_t               *sim5300_dev,
                                            uint32_t                     hni,
                                            sim5300_internet_settings_t *sim5300_internet_settings);

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_start_internet(sim5300_dev_t               *sim5300_dev,
                           uint8_t                      registration_timeout,
                           sim5300_internet_settings_t *sim5300_internet_settings);

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_socket(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_connect(sim5300_dev_t *sim5300_dev,
                    int            sockfd, 
                    char          *address,
                    char          *port,
                    char          *type);

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_send(sim5300_dev_t *sim5300_dev,
                 int            sockfd, 
                 uint8_t       *buffer,
                 size_t         buffer_len);

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_receive(sim5300_dev_t *sim5300_dev,
                    int            sockfd, 
                    uint8_t       *buffer,
                    size_t         buffer_len);

/*---------------------------------------------------------------------------*/
/*  */
int sim5300_close(sim5300_dev_t *sim5300_dev,
                  int            sockfd);

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
int sim5300_communication_test(sim5300_dev_t *sim5300_dev);

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
int sim5300_init(sim5300_dev_t *sim5300_dev, 
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