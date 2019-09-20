/*
 * Copyright (C) 2006-2017 wolfSSL Inc.
 *
 * This file is part of wolfSSL.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       wolfSSL client example
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
#include "sim5300.h"
#include "od.h"

#define SIM5300_UART            (2)             /* UART number for modem */
#define SIM5300_BAUDRATE        (STDIO_UART_BAUDRATE)   /* UART baudrate for modem*/
#define AT_DEV_BUF_SIZE         (2048)          /* The size of the buffer for all incoming data from modem */
#define AT_DEV_RESP_SIZE        (2048)          /* The size of the buffer to answer the command from the modem */

#define SERVER_ADDRES           "tg.manchenkoos.ru"
#define SERVER_PORT             "8080"
#define SERVER_TYPE_CONNECTION  "TCP"

static sim5300_dev_t sim5300_dev;               /* Struct for SIM5300 */

static int sockfd;                              /* Socket */

static char at_dev_buf[AT_DEV_BUF_SIZE];        /* Buffer for incoming data from modem SIM5300 */
static char at_dev_resp[AT_DEV_RESP_SIZE];      /* Buffer for parse incoming data from modem SIM5300 */

/*---------------------------------------------------------------------------*/
int main(void)
{
    puts("Start SIM5300 test TCP");
    
    int res;

    /* Init SIM5300 */
    sim5300_dev.power_en_pin    = RWCAR_GSM_POWER;
    sim5300_dev.power_act_level = HIGH;
    sim5300_dev.gsm_en_pin      = RWCAR_GSM_ENABLE;
    sim5300_dev.gsm_act_level   = HIGH;

    res = sim5300_init(&sim5300_dev, SIM5300_UART, SIM5300_BAUDRATE, at_dev_buf, AT_DEV_RESP_SIZE, at_dev_resp, AT_DEV_RESP_SIZE);
    if (res != SIM5300_OK) {
        puts("sim5300_init ERROR");

        return -1;
    } 

    /* Set internet settings SIM5300 */
    res = sim5300_start_internet(&sim5300_dev, 30, NULL);
    if (res == SIM5300_OK) {
        puts("[SIM5300] Set internet settings OK");
    } else {
        puts("[SIM5300] Set internet settings ERROR");

        return -1;
    }

    // uint8_t server_send[] = "GET /favicon.ico HTTP/1.1\r\nHost: tg.manchenkoos.ru:8080\r\nReferer: http://tg.manchenkoos.ru:8080/\r\n";
    uint8_t server_resp[200];
    uint32_t data_counter = 0;
    res = 1;
    
    puts("IN");
    while (res >= 0) {
        sockfd = sim5300_socket(&sim5300_dev);
        if (sockfd < SIM5300_OK) {
            printf("Error get socket: %i\n", sockfd);

            return -1;
        }

        res = sim5300_connect(&sim5300_dev, 
                               sockfd, 
                               SERVER_ADDRES, 
                               SERVER_PORT, 
                               SERVER_TYPE_CONNECTION);
        printf("\t\tCONNECT RES: %i\n", res);
        if (res < SIM5300_OK) {
            printf("Error start socket: %i\n", res);

            return -1;
        }

        printf("data_counter: %li\n", data_counter);

        // res = sim5300_send(&sim5300_dev, sockfd, server_send, sizeof(server_send)-1);
        // printf("\t\tSEND RES: %i\n", res);

        data_counter++;

        do {
            res = sim5300_receive(&sim5300_dev, sockfd, server_resp, sizeof(server_resp));
            lptimer_usleep(1000);
        } while (res == 0);
        
        printf("\t\tRESP RES: %i\n", res);
        if (res > 0) {
            printf("%s\n", server_resp);
        }

        res = sim5300_close(&sim5300_dev, sockfd);
        printf("\t\tCLOSE RES: %i\n", res);
        if (res != SIM5300_OK) {
            printf("Error close socket: %i\n", res);

            return -1;
        }

        lptimer_usleep(7000);
    }
    puts("OUT");

    printf("send res: %i; data_counter: %li\n%s\n", res, data_counter, server_resp);

    return 0;
}
