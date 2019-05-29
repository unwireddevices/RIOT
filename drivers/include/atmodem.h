/*
 * Copyright (C) 2019 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file
 * @brief       
 * @author      Oleg Manchenko
 */
#ifndef ATMODEM_MAIN_H_
#define ATMODEM_MAIN_H_

#define AT_DEV_BUF_SIZE         (2048)      /* The size of the buffer for all incoming data from modem */
#define AT_DEV_RESP_SIZE        (1024)      /* The size of the buffer to answer the command from the modem */

#ifndef ATMODEM_UART
#define ATMODEM_UART           UART_DEV(1) /* UART number for modem */
#endif

#define ATMODEM_TEST_IP_GOOGLE "8.8.8.8"
#define ATMODEM_TEST_IP_KROOT  "193.0.14.129"

#define ATMODEM_BAUDRATE          (115200)    /* UART baudrate for modem*/
#define ATMODEM_MAX_TIMEOUT       (1000000)   /* Maximum time waiting for a response */ 
#define ATMODEM_TIME_ON           (500)       /* The time of active low level impulse of PWRKEY pin to power on module. Min: 50ms, typ: 100ms */
#define ATMODEM_TIME_ON_UART      (3000)      /* The time from power-on issue to UART port ready. Min: 3s, max: 5s */
#define ATMODEM_CMD_TIMEOUT       (10)        /* Command timeout 10ms */

#define ATMODEM_READY             (0)
#define ATMODEM_NOT_INITIALIZED   (-1)        /* Modem is not initialized */
#define ATMODEM_NOT_ANSWERING     (-2)        /* If there is no modem, then sim_status.modem = MODEM_NOT_ANSWERING and display N/A on the screen. */
#define ATMODEM_WAIT_SMS_TIMEOUT  (400)       /* SMS waiting time with Internet settings in seconds */

/* Return code for modem_wait_sms_with_cmd() */
#define ATMODEM_NEW_INTERNET_SETTINGS_OK              (0)
#define ATMODEM_WAITING_SMS_INTERNET_SETTINGS_TIMEOUT (-1)
#define ATMODEM_ERROR_NEW_INTERNET_SETTINGS           (-2)

/* Get settings for selected SIM card */
bool atmodem_get_internet_settings(void);

/* Check the connection of the selected SIM card */
bool atmodem_test_connection(uint8_t *rssi);

/* Processing incoming SMS commands */
void atmodem_get_sms_command(void);

/* Power on for Modem */
void atmodem_power_on(void);

/* Power off for Modem */
void atmodem_power_off(void);

/* Modem initialization */
bool atmodem_init(void);

/*  */
int8_t atmodem_wait_sms_with_cmd(void);

/* Ping function */
bool atmodem_ping(char *ip_addr);

/* Sending data via Modem (HTTP GET) */
bool atmodem_send_httpget(uint8_t *data_for_send, size_t data_size);

/* Sending data via Modem (TCP or UDP) */
bool atmodem_send_tcpudp(char    *server_addr, 
                                char    *server_port, 
                                char    *connection_type,
                                uint8_t *data_for_send, 
                                size_t   data_size);

/* Sending data via Modem (HTTP POST) */
bool atmodem_send_httppost(char *url, uint8_t *data_for_send, size_t data_size);

#endif /* ATMODEM_MAIN_H_ */
