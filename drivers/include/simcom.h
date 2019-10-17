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

#ifndef SIMCOM_H
#define SIMCOM_H

#include <stdbool.h>

#include "at.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIMCOM_MAX_TIMEOUT         (1000000)   /**< Maximum time waiting for a response */ 
#define SIMCOM_TIME_ON             (500)       /**< The time of active low level impulse of PWRKEY pin to power on module. Min: 50ms, typ: 100ms */
#define SIMCOM_TIME_ON_UART        (3000)      /**< The time from power-on issue to UART port ready. Min: 3s, max: 5s */

#if !defined(RECEIVE_MAX_LEN)
    #define RECEIVE_MAX_LEN         (1460)      /**< Max requested number of data bytes (1-1460 bytes) to be read */
#endif /* if !defined(RECEIVE_MAX_LEN) */

/**
 * @brief SIMCOM ERRORS
 */
enum simcom_error {
    SIMCOM_OK                   =  0,      /**< OK */

    SIMCOM_DEV_ERROR            = -1,      /**< ERROR: simcom_dev == NULL */
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
    SIMCOM_NOT_ANSWERING        = -19,     /**< ERROR: SIMCOM not answering */
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
 * @brief SIMCOM device structure
 */
typedef struct {
    at_dev_t             at_dev;           /**< AT device structure */
    char                *at_dev_resp;      /**< Input buffer for parse response from SIMCOM */
    uint16_t             at_dev_resp_size; /**< Size of @p at_dev_resp */
    gpio_t               power_en_pin;     /**< Pin for power enable */
    gpio_t               gsm_en_pin;       /**< Pin for power enable SIN5300 */
    bool                 socketfd[8];      /**< Socket status array */
    uint8_t              power_act_level;  /**< Active level for power enable pin */
    uint8_t              gsm_act_level;    /**< Active level for power enable SIMCOM pin */
} simcom_dev_t;

/**
 * @brief SIMCOM response on AT+CSMINS
 */
typedef struct {
    int n;                      /**< A numeric parameter to show an unsolicited event code indicating whether the SIM has been inserted or removed */
    int sim_inserted;           /**< A numeric parameter which indicates whether SIM card has been inserted */
} simcom_csmins_resp_t;

/**
 * @brief SIMCOM response on AT+CPIN
 */
typedef enum {
    READY      = 0,             /**< MT is not pending for any password */          
    SIM_PIN    = 1,             /**< MT is waiting SIM PIN to be given */
    SIM_PUK    = 2,             /**< MT is waiting for SIM PUK to be given */
    PH_SIM_PIN = 3,             /**< ME is waiting for phone to SIM card (antitheft) */
    PH_SIM_PUK = 4,             /**< ME is waiting for SIM PUK (antitheft) */
    SIM_PIN2   = 5,             /**< PIN2, e.g. for editing the FDN book possible only if preceding Command was acknowledged with +CME ERROR:17 */
    SIM_PUK2   = 6,             /**< Possible only if preceding Command was acknowledged with error +CME ERROR: 18. */
} simcom_cpin_resp_t;

/**
 * @brief SIMCOM response on AT+CREG
 */
typedef struct {
    int n;                      /**< Unsolicited result code */
    int stat;                   /**< Status registration in network */
    // char lac[5];             /**< String type (string should be included in quotation marks); two byte location area code in hexadecimal format */
    // char ci[5];              /**< String type (string should be included in quotation marks); two byte cell ID in hexadecimal format */
    // int  act;                /**< Network connection status */
} simcom_creg_resp_t;

/**
 * @brief SIMCOM response on AT+GSMBUSY
 */
typedef enum {
    ENABLE_INCOMING_CALL                             = 0,   /**< Enable incoming call */          
    FORBID_ALL_INCOMING_CALLS                        = 1,   /**< Forbid all incoming calls */
    FORBID_INCOMING_VOICE_CALLS_BUT_ENABLE_CSD_CALLS = 2,   /**< Forbid incoming voice calls but enable CSD calls */
} simcom_gsmbusy_resp_t;

/**
 * @brief SIMCOM response on AT+CSQ
 */
typedef struct {
    int rssi;                   /**< in -dBm */
    int ber;                    /**<  <ber> (in percent):
                                        0...7 As RXQUAL values in the table in GSM 05.08 [20] subclause
                                        7.2.4
                                        99 Not known or not detectable */
} simcom_csq_resp_t;

/**
 * @brief SIMCOM response on AT+COPS
 */
typedef struct {
    int  mode;                  /**< Mode */
    int  format;                /**< Format */
    char oper[32];              /**< Operator name */
    // int  act;                /**< Type cellular network */
} simcom_cops_resp_t;

/**
 * @brief SIMCOM response on AT+CGATT
 */
typedef struct {
    int  state;                 /**< State */
} simcom_cgatt_resp_t;

/**
 * @brief SIMCOM response on AT+CIFSR
 */
typedef struct {
    int local_ip_address[4];    /**< IP address assigned from GPRS */
} simcom_cifsr_resp_t;

/**
 * @brief SIMCOM response on set AT+CIPSTATUS
 */
typedef struct {
    int  n;
    int  bearer;
    char type_connection[32];
    char ip_address[32];
    char port[32];
    char client_state[32];
} simcom_set_cipstatus_resp_t;

/**
 * @brief SIMCOM response on AT+CIPPING
 */
typedef struct {
    int reply_time;             /**< Time, in units of 100 ms, required to receive the response */
    int ttl;                    /**< Time to live (1 - 255, Default: 64) */
} simcom_cipping_resp_t;

/**
 * @brief SIMCOM Internet settings
 */
typedef struct {
    char apn[32];               /**< Access Point Name */
    char username[32];          /**< Username */
    char password[32];          /**< Password */
} simcom_internet_settings_t;

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
int simcom_power_on(simcom_dev_t *simcom_dev);

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
int simcom_power_off(simcom_dev_t *simcom_dev);

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
int simcom_send_at(simcom_dev_t *simcom_dev);

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
                                              simcom_csmins_resp_t *simcom_csmins_resp);

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
                                              uint8_t        n);

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
                           simcom_cpin_resp_t *simcom_cpin_resp);

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
                                     simcom_creg_resp_t *simcom_creg_resp);

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
                                     simcom_gsmbusy_resp_t *simcom_gsmbusy_resp);

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
                                     uint8_t        mode);

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
                                      simcom_csq_resp_t *simcom_csq_resp);

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
                                   simcom_cops_resp_t *simcom_cops_resp);

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
                                  uint8_t        cid);

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

char *simcom_get_imsi(simcom_dev_t *simcom_dev);

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
int simcom_get_hni(simcom_dev_t *simcom_dev);

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
                                   simcom_cgatt_resp_t *simcom_cgatt_resp);

/*---------------------------------------------------------------------------*/
/**
 * @brief       Set GPRS service state
 *
 * @param[in]   simcom_dev     Device to operate on
 * @param[in]   state           Indicates the state of GPRS attachment
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
                                   uint8_t        state);

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
                                 char          *password);

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
int simcom_bring_up_wireless_connection(simcom_dev_t *simcom_dev);

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
                                 simcom_cifsr_resp_t *simcom_cifsr_resp);

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
                                                simcom_set_cipstatus_resp_t *simcom_set_cipstatus_resp);

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
                                         uint8_t        n);

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
 * @returns     READLINE_ERROR         - ERROR: at_readline() < 0
 * @returns     UNKNOWN_RESP           - ERROR: Unknown response
 */
int simcom_close_up_multi_ip_connection(simcom_dev_t *simcom_dev,
                                         uint8_t        id,
                                         uint8_t        n);

/*---------------------------------------------------------------------------*/
/* AT+CIPPING PING request */
/* TODO: PARSE ERROR BECAUSE IP ADDREESS HAVE: "" */
int simcom_ping_request(simcom_dev_t          *simcom_dev,
                         simcom_cipping_resp_t  simcom_cipping_resp[],
                         char                   *address,
                         char                   *retr_num,
                         char                   *datalen, 
                         char                   *timeout,
                         char                   *ttl);

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
                                            char          *port);

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
                                                     size_t         data_size);

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
 * @param[in]   simcom_dev                 Device to operate on
 * 
 * Creating socket file descriptor
 * 
 * @returns     res >= 0               - OK, Socket file descriptor
 * @returns     res <  0               - ERROR: See simcom_error
 */
int simcom_socket(simcom_dev_t *simcom_dev);

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
                    char          *type);

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
                 size_t         buffer_len);

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
                    size_t         buffer_len);

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
                                            simcom_internet_settings_t *simcom_internet_settings);

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
                           simcom_internet_settings_t *simcom_internet_settings);

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
int simcom_communication_test(simcom_dev_t *simcom_dev);

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
                 uint16_t       at_dev_resp_size);

/*---------------------------------------------------------------------------*/
/*------------------------- END HIGH LEVEL FUNCTION -------------------------*/
/*---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* SIMCOM_H */
/** @} */