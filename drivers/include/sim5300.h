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

#include <stdbool.h>

#include "at.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM5300_MAX_TIMEOUT         (1000000)   /**< Maximum time waiting for a response */ 

#if !defined(RECEIVE_MAX_LEN)
    #define RECEIVE_MAX_LEN         (1460)      /**< Max requested number of data bytes (1-1460 bytes) to be read */
#endif /* if !defined(RECEIVE_MAX_LEN) */

/**
 * @brief SIM5300 ERRORS
 */
enum sim5300_error {
    SIM5300_OK                   =  0,      /**< OK */

    SIM5300_DEV_ERROR            = -1,      /**< ERROR: sim5300_dev == NULL */
    ARGUMENT_NULL_ERROR          = -2,      /**< ERROR: Pointer to function argument == NULL */
    ARGUMENT_RANGE_ERROR         = -3,      /**< ERROR: Invalid argument value */
    PARSE_ERROR                  = -4,      /**< ERROR: sscanf() != desired number of variables */
    SEND_CMD_WAIT_OK_ERROR       = -5,      /**< ERROR: at_send_cmd_wait_ok() != 0 */
    SEND_CMD_GET_RESP_ERROR      = -6,      /**< ERROR: at_send_cmd_get_resp() < 0 */
    SEND_CMD_ERROR               = -7,      /**< ERROR: at_send_cmd() != 0 */
    READLINE_ERROR               = -8,      /**< ERROR: at_readline() < 0 */
    UNKNOWN_RESP                 = -9,      /**< ERROR: Unknown response */
    UNDEFINED_ERROR              = -10,     /**< ERROR: Undefined error */
    RECEIVE_MAX_LEN_ERROR        = -11,     /**< ERROR: data_size > RECEIVE_MAX_LEN */
    NOT_IMPLEMENTED              = -12,     /**< ERROR: Not implemented */
    TIMEOUT_EXPIRED              = -13,     /**< ERROR: Timeout expired */
    NO_INTERNET_SETTINGS_FOUND   = -14,     /**< ERROR: No internet settings found */
    NO_SIM_CARD                  = -15,     /**< ERROR: No SIM card */
    NEED_PASSWORD_FOR_SIM_CARD   = -16,     /**< ERROR: Need password for SIM card */
    REGISTRATION_TIMEOUT_EXPIRED = -17,     /**< ERROR: Registration timeout expired */
    NO_LOCAL_IP_ADDRESS          = -18,     /**< ERROR: No local IP address */
    SIM5300_NOT_ANSWERING        = -19,     /**< ERROR: SIM5300 not answering */
    UNABLE_TO_CREATE_SOCKET      = -20,     /**< ERROR: Unable to create socket */
    INVALID_DATA                 = -21,     /**< ERROR: Invalid data */
    SOCKET_REMOTE_CLOSING        = -22,     /**< ERROR: Socket remote closing */
    SOCKET_CLOSING               = -23,     /**< ERROR: Socket closing */
    SOCKET_CLOSED                = -24,     /**< ERROR: Socket closed */
    // NO_INTERNET_SETTINGS_FOUND   = -24,     /**< ERROR:  */
    // NO_INTERNET_SETTINGS_FOUND   = -25,     /**< ERROR:  */
    // NO_INTERNET_SETTINGS_FOUND   = -26,     /**< ERROR:  */
    // NO_INTERNET_SETTINGS_FOUND   = -27,     /**< ERROR:  */
    // NO_INTERNET_SETTINGS_FOUND   = -28,     /**< ERROR:  */
    
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
    // int  act;                /**< Network connection status */
} sim5300_creg_resp_t;

/**
 * @brief SIM5300 response on AT+GSMBUSY
 */
typedef enum {
    ENABLE_INCOMING_CALL                             = 0,   /**< Enable incoming call */          
    FORBID_ALL_INCOMING_CALLS                        = 1,   /**< Forbid all incoming calls */
    FORBID_INCOMING_VOICE_CALLS_BUT_ENABLE_CSD_CALLS = 2,   /**< Forbid incoming voice calls but enable CSD calls */
} sim5300_gsmbusy_resp_t;

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
    int  mode;                  /**< Mode */
    int  format;                /**< Format */
    char oper[32];              /**< Operator name */
    // int  act;                /**< Type cellular network */
} sim5300_cops_resp_t;

/**
 * @brief SIM5300 response on AT+CGATT
 */
typedef struct {
    int  state;                 /**< State */
} sim5300_cgatt_resp_t;

/**
 * @brief SIM5300 response on AT+CIFSR
 */
typedef struct {
    int local_ip_address[4];    /**< IP address assigned from GPRS */
} sim5300_cifsr_resp_t;

/**
 * @brief SIM5300 response on set AT+CIPSTATUS
 */
typedef struct {
    int  n;
    int  bearer;
    char type_connection[32];
    char ip_address[32];
    char port[32];
    char client_state[32];
} sim5300_set_cipstatus_resp_t;

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
/*------------------------ START LOW LEVEL FUNCTION -------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * @brief       Send ATtention Code
 *
 * @param[in]   sim5300_dev         Device to operate on
 * 
 * AT – ATtention Code
 * This is the prefix for all commands except A/. When entered on its own, the SIM5300 will respond OK.
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int sim5300_send_at(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get SIM inserted status reporting 
 *
 * @param[in]   sim5300_dev             Device to operate on
 * @param[out]  sim5300_csmins_resp     Structure with response data
 * 
 * AT+CSMINS – SIM Inserted Status Reporting
 * Get SIM inserted status reporting 
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int sim5300_get_sim_inserted_status_reporting(sim5300_dev_t         *sim5300_dev,
                                              sim5300_csmins_resp_t *sim5300_csmins_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set SIM inserted status reporting 
 *
 * @param[in]   sim5300_dev     Device to operate on
 * @param[in]   n               A numeric parameter to show an unsolicited event code indicating whether the SIM has been inserted or removed.
 * 
 * AT+CSMINS – SIM Inserted Status Reporting
 * Set SIM inserted status reporting 
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int sim5300_set_sim_inserted_status_reporting(sim5300_dev_t *sim5300_dev, 
                                              uint8_t        n);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get PIN status
 *
 * @param[in]   sim5300_dev             Device to operate on
 * @param[out]  sim5300_cpin_resp       Structure with response data
 * 
 * AT+CPIN Enter PIN
 * Get PIN status
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     UNKNOWN_RESP            - ERROR: Unknown response
 */
int sim5300_get_pin_status(sim5300_dev_t       *sim5300_dev,
                           sim5300_cpin_resp_t *sim5300_cpin_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get network registration
 *
 * @param[in]   sim5300_dev             Device to operate on
 * @param[out]  sim5300_creg_resp       Structure with response data
 * 
 * AT+CREG Network registration
 * Get network registration
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int sim5300_get_network_registration(sim5300_dev_t       *sim5300_dev,
                                     sim5300_creg_resp_t *sim5300_creg_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get reject incoming call 
 *
 * @param[in]   sim5300_dev             Device to operate on
 * @param[out]  sim5300_gsmbusy_resp    Structure with response data
 * 
 * AT+GSMBUSY Reject incoming call
 * Get reject incoming call 
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 * @returns     UNKNOWN_RESP            - ERROR: Unknown response
 */
int sim5300_get_reject_incoming_call(sim5300_dev_t          *sim5300_dev,
                                     sim5300_gsmbusy_resp_t *sim5300_gsmbusy_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set reject incoming call 
 *
 * @param[in]   sim5300_dev     Device to operate on
 * @param[in]   mode            Mode
 * 
 * AT+GSMBUSY Reject incoming call
 * Set reject incoming call 
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int sim5300_set_reject_incoming_call(sim5300_dev_t *sim5300_dev, 
                                     uint8_t        mode);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get signal quality report
 *
 * @param[in]   sim5300_dev             Device to operate on
 * @param[out]  sim5300_csq_resp        Structure with response data
 * 
 * AT+CSQ Signal quality report
 * Get signal quality report
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int sim5300_get_signal_quality_report(sim5300_dev_t      *sim5300_dev,
                                      sim5300_csq_resp_t *sim5300_csq_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get operator selection
 *
 * @param[in]   sim5300_dev             Device to operate on
 * @param[out]  sim5300_cops_resp       Structure with response data
 * 
 * AT+COPS Operator selection
 * Get operator selection
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int sim5300_get_operator_selection(sim5300_dev_t       *sim5300_dev,
                                   sim5300_cops_resp_t *sim5300_cops_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set PDP context activate or deactivate 
 *
 * @param[in]   sim5300_dev     Device to operate on
 * @param[in]   state           Indicates the state of PDP context activation
 * @param[in]   cid             A numeric parameter which specifies a particular PDP context definition (see +CGDCONT Command). 
 *                              If the <cid> is omitted, it only affects the first cid.
 * 
 * AT+CGACT PDP context activate or deactivate
 * Set PDP context activate or deactivate 
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int sim5300_set_state_pdp_context(sim5300_dev_t *sim5300_dev,
                                  uint8_t        state,
                                  uint8_t        cid);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get IMSI
 *
 * @param[in]   sim5300_dev     Device to operate on
 * 
 * AT+CIMI Request International Mobile Subscriber Identity (IMSI)
 * Get IMSI
 * 
 * @returns     Pointer to a string - OK
 * @returns     NULL                - ERROR
 */

char *sim5300_get_imsi(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get HNI
 *
 * @param[in]   sim5300_dev     Device to operate on
 * 
 * Get Home Network Identity (HNI)
 * 
 * @returns     SIM5300_OK >= 0   - HNI
 * @returns     SIM5300_DEV_ERROR - ERROR: sim5300_dev == NULL
 * @returns     UNDEFINED_ERROR   - ERROR: Undefined error 
 */
int sim5300_get_hni(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get GPRS service state
 *
 * @param[in]   sim5300_dev             Device to operate on
 * @param[out]  sim5300_cgatt_resp      Structure with response data
 * 
 * AT+CGATT Get GPRS service state
 * Get GPRS service state
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int sim5300_get_gprs_service_state(sim5300_dev_t        *sim5300_dev,
                                   sim5300_cgatt_resp_t *sim5300_cgatt_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set GPRS service state
 *
 * @param[in]   sim5300_dev     Device to operate on
 * @param[in]   state           Indicates the state of GPRS attachment
 * 
 * AT+CGATT Set GPRS service state
 * Set GPRS service state
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int sim5300_set_gprs_service_state(sim5300_dev_t *sim5300_dev, 
                                   uint8_t        state);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set start task and set APN, USER NAME, PASSWORD
 *
 * @param[in]   sim5300_dev     Device to operate on
 * @param[in]   apn             A string parameter which indicates the GPRS access point name
 * @param[in]   user            A string parameter which indicates the GPRS user name
 * @param[in]   password        A string parameter which indicates the GPRS password
 * 
 * AT+CSTT Set start task and set APN, USER NAME, PASSWORD
 * Set start task and set APN, USER NAME, PASSWORD
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR    - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int sim5300_set_network_settings(sim5300_dev_t *sim5300_dev,
                                 char          *apn,
                                 char          *user,
                                 char          *password);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Bring up wireless connection with GPRS
 *
 * @param[in]   sim5300_dev     Device to operate on
 * 
 * AT+CIICR Bring up wireless connection with GPRS
 * Bring up wireless connection with GPRS
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int sim5300_bring_up_wireless_connection(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Get local IP address
 *
 * @param[in]   sim5300_dev             Device to operate on
 * @param[out]  sim5300_cifsr_resp      Structure with response data
 * 
 * AT+CIFSR Get local IP address
 * Get local IP address
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int sim5300_get_local_ip_address(sim5300_dev_t        *sim5300_dev,
                                 sim5300_cifsr_resp_t *sim5300_cifsr_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Query current connection status
 *
 * @param[in]   sim5300_dev                 Device to operate on
 * @param[in]   n                           A numeric parameter which indicates the connection number
 * @param[out]  sim5300_set_cipstatus_resp  Structure with response data
 * 
 * AT+CIPSTATUS Query current connection status
 * Query current connection status
 * 
 * @returns     SIM5300_OK              - OK
 * @returns     SIM5300_DEV_ERROR       - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR    - ERROR: Invalid argument value
 * @returns     ARGUMENT_NULL_ERROR     - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_GET_RESP_ERROR - ERROR: at_send_cmd_get_resp() < 0
 * @returns     PARSE_ERROR             - ERROR: sscanf() != desired number of variables
 */
int sim5300_set_query_current_connection_status(sim5300_dev_t                *sim5300_dev,
                                                uint8_t                       n,
                                                sim5300_set_cipstatus_resp_t *sim5300_set_cipstatus_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Start up multi-IP connection
 *
 * @param[in]   sim5300_dev     Device to operate on
 * @param[in]   n               Mode: single IP connection or multi IP connection
 * 
 * AT+CIPMUX Start up multi-IP connection
 * Start up multi-IP connection
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_WAIT_OK_ERROR - ERROR: at_send_cmd_wait_ok() != 0
 */
int sim5300_start_up_multi_ip_connection(sim5300_dev_t *sim5300_dev, 
                                         uint8_t        n);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Close up multi-IP connection
 *
 * @param[in]   sim5300_dev     Device to operate on
 * @param[in]   id              A numeric parameter which indicates the connection number 
 * @param[in]   n               Closing type: slow or quick close 
 * 
 * AT+CIPCLOSE Close up multi-IP connection
 * Close up multi-IP connection
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     SEND_CMD_ERROR         - ERROR: at_send_cmd() != 0
 * @returns     READLINE_ERROR         - ERROR: at_readline() < 0
 * @returns     UNKNOWN_RESP           - ERROR: Unknown response
 */
int sim5300_close_up_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                         uint8_t        id,
                                         uint8_t        n);

/*---------------------------------------------------------------------------*/
/* AT+CIPPING PING request */
/* TODO: PARSE ERROR BECAUSE IP ADDREESS HAVE: "" */
int sim5300_ping_request(sim5300_dev_t          *sim5300_dev,
                         sim5300_cipping_resp_t  sim5300_cipping_resp[],
                         char                   *address,
                         char                   *retr_num,
                         char                   *datalen, 
                         char                   *timeout,
                         char                   *ttl);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Start up multi-IP TCP or UDP connection
 *
 * @param[in]   sim5300_dev     Device to operate on
 * @param[in]   n               A numeric parameter which indicates the connection number
 * @param[in]   mode            A string parameter which indicates the connection type: TCP or UDP
 * @param[in]   address         A string parameter which indicates remote server IP address or domain name
 * @param[in]   port            A string parameter which indicates remote server port 
 * 
 * AT+CIPSTART Start up multi-IP TCP or UDP connection
 * Start up multi-IP TCP or UDP connection
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     ARGUMENT_NULL_ERROR    - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_ERROR         - ERROR: at_send_cmd() != 0
 * @returns     READLINE_ERROR         - ERROR: at_readline() < 0
 * @returns     UNKNOWN_RESP           - ERROR: Unknown response
 * @returns     TIMEOUT_EXPIRED        - ERROR: Timeout expired
 */
int sim5300_start_up_multi_ip_up_connection(sim5300_dev_t *sim5300_dev,
                                            uint8_t        n,
                                            char          *mode,
                                            char          *address,
                                            char          *port);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Start up multi-IP TCP or UDP connection
 *
 * @param[in]   sim5300_dev         Device to operate on
 * @param[in]   mode                Mode
 * @param[in]   n                   A numeric parameter which indicates the connection number
 * @param[out]  data_for_receive    Pointer to a receive buffer
 * @param[in]   data_size           Received data size
 * 
 * AT+CIPSTART Start up multi-IP TCP or UDP connection
 * Start up multi-IP TCP or UDP connection
 * 
 * @returns     SIM5300_OK             - OK (mode = 1)
 * @returns     res >= 0               - OK, receive_length (mode = 2)
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
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
int sim5300_receive_data_through_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                                     uint8_t        mode,
                                                     uint8_t        n,
                                                     uint8_t       *data_for_receive,
                                                     size_t         data_size);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Send data through TCP or UDP multi IP connection
 *
 * @param[in]   sim5300_dev         Device to operate on
 * @param[in]   n                   A numeric parameter which indicates the connection number
 * @param[in]   data_for_send       Pointer to send buffer
 * @param[in]   data_size           Send data size
 * 
 * AT+CIPSEND Send data through TCP or UDP multi IP connection
 * Send data through TCP or UDP multi IP connection
 * 
 * @returns     res >= 0               - OK, receive_length (mode = 2)
 * @returns     SIM5300_DEV_ERROR      - ERROR: sim5300_dev == NULL
 * @returns     ARGUMENT_RANGE_ERROR   - ERROR: Invalid argument value
 * @returns     ARGUMENT_NULL_ERROR    - ERROR: Pointer to function argument == NULL
 * @returns     SEND_CMD_ERROR         - ERROR: at_send_cmd() != 0
 * @returns     INVALID_DATA           - ERROR: Invalid data
 * @returns     UNKNOWN_RESP           - ERROR: Unknown response
 * @returns     TIMEOUT_EXPIRED        - ERROR: Timeout expired
 */
int sim5300_send_data_through_multi_ip_connection(sim5300_dev_t *sim5300_dev,
                                                  uint8_t        n,
                                                  uint8_t       *data_for_send, 
                                                  size_t         data_size);

/*---------------------------------------------------------------------------*/
/*------------------------- END LOW LEVEL FUNCTION --------------------------*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*---------------------------- START SOCKET API -----------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * @brief       Creating socket file descriptor
 *
 * @param[in]   sim5300_dev                 Device to operate on
 * 
 * Creating socket file descriptor
 * 
 * @returns     res >= 0               - OK, Socket file descriptor
 * @returns     res <  0               - ERROR: See sim5300_error
 */
int sim5300_socket(sim5300_dev_t *sim5300_dev);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Connect to server
 *
 * @param[in]   sim5300_dev                 Device to operate on
 * @param[in]   sockfd                      Socket file descriptor
 * @param[in]   address                     String with address or domain for connect
 * @param[in]   port                        String with port number for connect
 * @param[in]   type                        String with type connection: TCP or UDP
 * 
 * Connect to server
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     res < 0                - ERROR: See sim5300_error
 */
int sim5300_connect(sim5300_dev_t *sim5300_dev,
                    int            sockfd, 
                    char          *address,
                    char          *port,
                    char          *type);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Send data via socket
 *
 * @param[in]   sim5300_dev         Device to operate on
 * @param[in]   sockfd              Socket file descriptor
 * @param[in]   buffer              Pointer to send buffer
 * @param[in]   buffer_len          Send data size
 * 
 * Send data via socket
 * 
 * @returns     res >= 0               - OK, Send length 
 * @returns     res <  0               - ERROR: See sim5300_error
 */
int sim5300_send(sim5300_dev_t *sim5300_dev,
                 int            sockfd, 
                 uint8_t       *buffer,
                 size_t         buffer_len);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Received data via socket
 *
 * @param[in]   sim5300_dev         Device to operate on
 * @param[in]   sockfd              Socket file descriptor
 * @param[out]  buffer              Pointer to a receive buffer
 * @param[in]   buffer_len          Received data size
 * 
 * Received data via socket
 * 
 * @returns     res >= 0               - OK, Receive length 
 * @returns     res <  0               - ERROR: See sim5300_error
 */
int sim5300_receive(sim5300_dev_t *sim5300_dev,
                    int            sockfd, 
                    uint8_t       *buffer,
                    size_t         buffer_len);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Close socket file descriptor
 *
 * @param[in]   sim5300_dev         Device to operate on
 * @param[in]   sockfd              Socket file descriptor
 * 
 * Close socket file descriptor
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     res < 0                - ERROR: See sim5300_error
 */
int sim5300_close(sim5300_dev_t *sim5300_dev,
                  int            sockfd);

/*---------------------------------------------------------------------------*/
/*----------------------------- END SOCKET API ------------------------------*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*----------------------- START HIGH LEVEL FUNCTION -------------------------*/
/*---------------------------------------------------------------------------*/
/**
 * @brief       Get internet settings from base
 *
 * @param[in]   sim5300_dev                 Device to operate on
 * @param[in]   hni                         Home Network Identity (HNI)
 * @param[out]  sim5300_internet_settings   Structure with response data
 * 
 * Get internet settings from base
 * 
 * @returns     SIM5300_OK                  - OK
 * @returns     NO_INTERNET_SETTINGS_FOUND  - ERROR: No internet settings found  
 */
int sim5300_get_internet_settings_from_base(sim5300_dev_t               *sim5300_dev,
                                            uint32_t                     hni,
                                            sim5300_internet_settings_t *sim5300_internet_settings);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Start internet
 *
 * @param[in]   sim5300_dev                 Device to operate on
 * @param[in]   registration_timeout        Maximum network registration time
 * @param[in]   sim5300_internet_settings   Structure with internet settings
 * 
 * Start internet
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     res < 0                - ERROR: See sim5300_error
 */
int sim5300_start_internet(sim5300_dev_t               *sim5300_dev,
                           uint8_t                      registration_timeout,
                           sim5300_internet_settings_t *sim5300_internet_settings);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Communication test between microcontroller and SIM5300
 *
 * @param[in]   sim5300_dev         Device to operate on
 *
 * Send 5 AT command
 * 
 * @returns     SIM5300_OK             - OK
 * @returns     res < 0                - ERROR: See sim5300_error
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
 * @returns     SIM5300_OK             - OK
 * @returns     res < 0                - ERROR: See sim5300_error
 */
int sim5300_init(sim5300_dev_t *sim5300_dev, 
                 uart_t         uart, 
                 uint32_t       baudrate, 
                 char          *buf, 
                 size_t         bufsize, 
                 char          *at_dev_resp, 
                 uint16_t       at_dev_resp_size);

/*---------------------------------------------------------------------------*/
/*------------------------- END HIGH LEVEL FUNCTION -------------------------*/
/*---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* SIM5300_H */
/** @} */