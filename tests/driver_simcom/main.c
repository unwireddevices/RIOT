/*
 * Copyright (C) 2019 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       SimCom AT modem example
 *
 * @author      Oleg Manchenko
 *
 * @}
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "periph/gpio.h"
#include "lptimer.h"
#include "simcom.h"
#include "od.h"

#define SIMCOM_UART            (2)             /* UART number for modem */
#define SIMCOM_BAUDRATE        (STDIO_UART_BAUDRATE)   /* UART baudrate for modem*/
#define AT_DEV_BUF_SIZE         (2048)          /* The size of the buffer for all incoming data from modem */
#define AT_DEV_RESP_SIZE        (2048)          /* The size of the buffer to answer the command from the modem */

#define SERVER_ADDRES           "tg.manchenkoos.ru"
#define SERVER_PORT             "8080"
#define SERVER_TYPE_CONNECTION  "TCP"

static simcom_dev_t simcom_dev;               /* Struct for SIMCOM */

static int sockfd;                              /* Socket */

static char at_dev_buf[AT_DEV_BUF_SIZE];        /* Buffer for incoming data from modem SIMCOM */
static char at_dev_resp[AT_DEV_RESP_SIZE];      /* Buffer for parse incoming data from modem SIMCOM */

/*---------------------------------------------------------------------------*/
int main(void)
{
    puts("Start SIMCOM test TCP");
    
    int res;

    /* Init SIMCOM */
    simcom_dev.power_en_pin    = RWCAR_GSM_POWER;
    simcom_dev.power_act_level = 1;
    simcom_dev.gsm_en_pin      = RWCAR_GSM_ENABLE;
    simcom_dev.gsm_act_level   = 1;

    res = simcom_init(&simcom_dev, SIMCOM_UART, SIMCOM_BAUDRATE, at_dev_buf, AT_DEV_RESP_SIZE, at_dev_resp, AT_DEV_RESP_SIZE);
    if (res != SIMCOM_OK) {
        puts("simcom_init ERROR");

        return -1;
    } 

    /* Set internet settings SIMCOM */
    res = simcom_start_internet(&simcom_dev, 30, NULL);
    if (res == SIMCOM_OK) {
        puts("[SIMCOM] Set internet settings OK");
    } else {
        puts("[SIMCOM] Set internet settings ERROR");

        return -1;
    }

    // uint8_t server_send[] = "GET /favicon.ico HTTP/1.1\r\nHost: tg.manchenkoos.ru:8080\r\nReferer: http://tg.manchenkoos.ru:8080/\r\n";
    uint8_t server_resp[200];
    uint32_t data_counter = 0;
    res = 1;
    
    puts("IN");
    while (res >= 0) {
        sockfd = simcom_socket(&simcom_dev);
        if (sockfd < SIMCOM_OK) {
            printf("Error get socket: %i\n", sockfd);

            return -1;
        }

        res = simcom_connect(&simcom_dev, 
                               sockfd, 
                               SERVER_ADDRES, 
                               SERVER_PORT, 
                               SERVER_TYPE_CONNECTION);
        printf("\t\tCONNECT RES: %i\n", res);
        if (res < SIMCOM_OK) {
            printf("Error start socket: %i\n", res);

            return -1;
        }

        printf("data_counter: %li\n", data_counter);

        // res = simcom_send(&simcom_dev, sockfd, server_send, sizeof(server_send)-1);
        // printf("\t\tSEND RES: %i\n", res);

        data_counter++;

        do {
            res = simcom_receive(&simcom_dev, sockfd, server_resp, sizeof(server_resp));
            lptimer_usleep(1000);
        } while (res == 0);
        
        printf("\t\tRESP RES: %i\n", res);
        if (res > 0) {
            printf("%s\n", server_resp);
        }

        res = simcom_close(&simcom_dev, sockfd);
        printf("\t\tCLOSE RES: %i\n", res);
        if (res != SIMCOM_OK) {
            printf("Error close socket: %i\n", res);

            return -1;
        }

        lptimer_usleep(7000);
    }
    puts("OUT");

    printf("send res: %i; data_counter: %li\n%s\n", res, data_counter, server_resp);

    return 0;
}
