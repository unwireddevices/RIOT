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

/* socket includes */
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <netinet/in.h>
// #include <unistd.h>

/* wolfSSL */
#include <wolfssl/ssl.h>
#include <wolfssl/certs_test.h>

#include "periph/gpio.h"
#include "rtctimers-millis.h"
#include "sim5300.h"

#include "od.h"

#define SIM5300_TIME_ON         (500)           /* The time of active low level impulse of PWRKEY pin to power on module. Min: 50ms, typ: 100ms */
#define SIM5300_UART            (T2M_UART_GSM)  /* UART number for modem */
#define SIM5300_BAUDRATE        (115200)        /* UART baudrate for modem*/
#define SIM5300_TIME_ON_UART    (3000)          /* The time from power-on issue to UART port ready. Min: 3s, max: 5s */
#define AT_DEV_BUF_SIZE         (1024)          /* The size of the buffer for all incoming data from modem */
#define AT_DEV_RESP_SIZE        (256)           /* The size of the buffer to answer the command from the modem */

static sim5300_dev_t sim5300_dev;               /* Struct for SIM5300 */

static char at_dev_buf[AT_DEV_BUF_SIZE];        /* Buffer for incoming data from modem SIM5300 */
static char at_dev_resp[AT_DEV_RESP_SIZE];      /* Buffer for parse incoming data from modem SIM5300 */

// #define DEFAULT_PORT 11111

time_t get_epoch_time(struct tm *time) {
    (void) time;
    return 1562869547;
}

int unwired_recv(WOLFSSL *ssl, char *buf, int sz, void *ctx) {
    (void) ssl;
    (void) buf;
    (void) sz;
    (void) ctx;
    printf("[Socket] Recv\n");

    return 0;
}

int unwired_send(WOLFSSL *ssl, char *buf, int sz, void *ctx) {
    (void) ssl;
    (void) ctx;

    printf("[Socket] Send\n");

    od_hex_dump(buf, sz, OD_WIDTH_DEFAULT);
    printf("\n");

    return sz;
}

/*---------------------------------------------------------------------------*/
/* Power on for SIM5300 */
void sim5300_power_on(void) {
    puts("[SIM5300] Power on");

    /* MODEM_POWER_ENABLE to Hi */
    gpio_init(T2M_GSMPOWER, GPIO_OUT);
    gpio_set(T2M_GSMPOWER);

    /* 3G_PWR to Hi on 100 ms*/
    gpio_init(T2M_GSMENABLE, GPIO_OUT);
    gpio_set(T2M_GSMENABLE);

    /* 500ms sleep and clear */
    rtctimers_millis_sleep(SIM5300_TIME_ON);
    gpio_clear(T2M_GSMENABLE);
}

/*---------------------------------------------------------------------------*/
/* Power off for SIM5300 */
// void sim5300_power_off(void) {
//     /* Power off UART for modem */
//     uart_poweroff(at_dev.uart);

//     /* T2M_GSMPOWER to Low */
//     gpio_init(T2M_GSMPOWER, GPIO_OUT);
//     gpio_clear(T2M_GSMPOWER);

//     /* Modem is not initialized */
//     sim_status.modem = MODEM_NOT_INITIALIZED;
//     puts("[SIM5300] Power off");
// }

/*---------------------------------------------------------------------------*/
/* Test driver SIM5300 */
bool sim5300_test_driver(sim5300_dev_t *sim5300_dev) {
    return true;

    if (!sim5300_set_sim_inserted_status_reporting(sim5300_dev, 1)) {
        return false;
    }

    sim5300_csmins_resp_t sim5300_csmins_resp;
    if (!sim5300_get_sim_inserted_status_reporting(sim5300_dev, &sim5300_csmins_resp)) {
        return false;
    }

    if (sim5300_get_pin_status(sim5300_dev) < 0) {
        return false;
    }

///
    if (!sim5300_set_network_settings(sim5300_dev, "internet", "", "")) {
        return false;
    }
    // if (!sim5300_set_network_settings(sim5300_dev, "apn", "", "password")) {
    //     return false;
    // }
    // if (!sim5300_set_network_settings(sim5300_dev, NULL, "user", "password")) {
    //     return false;
    // }
///

    rtctimers_millis_sleep(15000);

    sim5300_creg_resp_t sim5300_creg_resp;
    if (!sim5300_get_network_registration(sim5300_dev, &sim5300_creg_resp)) {
        return false;
    }

    if (!sim5300_set_reject_incoming_call(sim5300_dev, 1)) {
        return false;
    }

    if (sim5300_get_reject_incoming_call(sim5300_dev) < 0) {
        return false;
    }

    sim5300_csq_resp_t sim5300_csq_resp;
    if (!sim5300_get_signal_quality_report(sim5300_dev, &sim5300_csq_resp)) {
        return false;
    }

    sim5300_cops_resp_t sim5300_cops_resp;
    if (!sim5300_get_operator_selection(sim5300_dev, &sim5300_cops_resp)) {
        return false;
    }

    char *imsi = sim5300_get_imsi(sim5300_dev);
    if (imsi == NULL) {
        return false;
    }

    // while (1) {
    //     if (!sim5300_get_network_registration(sim5300_dev, &sim5300_creg_resp)) {
    //         return false;
    //     }

    //     if (!sim5300_get_operator_selection(sim5300_dev, &sim5300_cops_resp)) {
    //         return false;
    //     }

    //     rtctimers_millis_sleep(1000);
    // }
    
    if (!sim5300_set_state_pdp_context(sim5300_dev, 1, 0)) {
        return false;
    }

    if (!sim5300_bring_up_wireless_connection(sim5300_dev)) {
        return false;
    }

    return true;
}

/*---------------------------------------------------------------------------*/
/* Test driver SIM5300 */
bool sim5300_set_internet(sim5300_dev_t *sim5300_dev) {
    /* Disabled showing an unsolicited event code */
    if (!sim5300_set_sim_inserted_status_reporting(sim5300_dev, 0)) {
        return false;
    }

    /* Is SIM card inserted? */
    sim5300_csmins_resp_t sim5300_csmins_resp;
    if (!sim5300_get_sim_inserted_status_reporting(sim5300_dev, &sim5300_csmins_resp)) {
        return false;
    }
    if (sim5300_csmins_resp.sim_inserted != 1) {
        return false;
    }

    /* Is there a PIN code? */
    if (sim5300_get_pin_status(sim5300_dev) != READY) {
        return false;
    }

    int res;

    /* Have you registered in the cellular network? */
    /* Cycle counter */
    uint8_t counter = 0;

    /* Response on AT+CREG */
    sim5300_creg_resp_t sim5300_creg_resp;
    while(1) {
        if (counter >= 30) {
            puts("[SIM5300] Registration failed");
            return false;
        }

        if (sim5300_get_network_registration(sim5300_dev, &sim5300_creg_resp)) {
            if (sim5300_creg_resp.stat == 1) {
                break;
            }
        }

        rtctimers_millis_sleep(1000);
        counter++;
    } 
    puts("[SIM5300] Registration OK");

    /* Reject Incoming Call */
    if (!sim5300_set_reject_incoming_call(sim5300_dev, 1)) {
        return false;
    }

    /* Start up multi-IP connection */
    if (!sim5300_start_up_multi_ip_connection(sim5300_dev, 1)) {
        return false;
    }

    /* Attach to the network */
    int8_t gprs_state = sim5300_get_gprs_service_state(sim5300_dev);
    if (gprs_state == 0) {
        if (!sim5300_set_gprs_service_state(sim5300_dev, 1)) {
            return false;
        }
    } else if (gprs_state != 1) {
        return false;
    }

    /* Get data from network manually for multi IP connection */
    res = sim5300_receive_data_through_multi_ip_connection(sim5300_dev, 1, 1, NULL, 0);
    if(res != 0) {
        puts("[SIM5300] Set get data from network manually for multi IP connection ERROR");
    } 
    /////////////////DELETE/////////////////////////
    // /* Send AT command */
    // int res = at_send_cmd_wait_ok(&sim5300_dev->at_dev, "AT+CIPRXGET=1", 1000000);
    // if (res != 0) {
    //     puts("[SIM5300] AT+CIPRXGET=1 ERROR");
    // }
    /////////////////DELETE/////////////////////////

    /* Start Task and Set APN, USER NAME, PASSWORD */
    if (!sim5300_set_network_settings(sim5300_dev, "internet", "", "")) {
        return false;
    }

    // if (!sim5300_set_state_pdp_context(sim5300_dev, 1, 0)) {
    //     return false;
    // }

    /* Bring Up Wireless Connection with GPRS */
    if (!sim5300_bring_up_wireless_connection(sim5300_dev)) {
        return false;
    }

    /* Get local IP address */
    sim5300_cifsr_resp_t sim5300_cifsr_resp;
    if (!sim5300_get_local_ip_address(sim5300_dev, &sim5300_cifsr_resp)) {
        return false;
    }

    /* Check local address */
    if ((sim5300_cifsr_resp.local_ip_address[0] == 0) && 
        (sim5300_cifsr_resp.local_ip_address[1] == 0) &&
        (sim5300_cifsr_resp.local_ip_address[2] == 0) &&
        (sim5300_cifsr_resp.local_ip_address[3] == 0)) {
        puts("[SIM5300] Zero IP ERROR");

        return false;
    }

    /* AT+CDNSCFG="8.8.8.8","8.8.4.4" */

    /* AT+CIPSTATUS */

    /* ping */
    // sim5300_cipping_resp_t sim5300_cipping_resp[3] = {};
    // if (sim5300_ping_request(sim5300_dev,
    //                          sim5300_cipping_resp,
    //                          "8.8.8.8",
    //                          "3",
    //                          "32", 
    //                          "100",
    //                          "64")) {
    //     puts("ping true");

    //     for (uint8_t i = 0; i < 3; i++ ) {
    //         printf("sim5300_cipping_resp[%i].reply_time = %i; sim5300_cipping_resp[%i].ttl = %i\n", i, sim5300_cipping_resp[i].reply_time, i, sim5300_cipping_resp[i].ttl);
    //     }
    // } else {
    //     puts("ping false");

    //     return false;
    // }

    //////// START SOCKET //////////
    uint8_t data_for_send[256];
    uint8_t data_for_recv[256] = {};
    for (uint16_t i = 0; i < 256; i++) {
        data_for_send[i] = i;
    }

    int sockfd = sim5300_socket(sim5300_dev);
    if (res < 0) {
        printf("Error get socket: %i\n", res);

        return false;
    }

    res = sim5300_connect(sim5300_dev, 
                          sockfd, 
                          "176.15.5.90",
                          "666",
                          "TCP");
    if (res < 0) {
        printf("Error start socket: %i\n", res);

        return false;
    }

    res = sim5300_send(sim5300_dev,
                       sockfd, 
                       data_for_send,
                       sizeof(data_for_send));
    if (res != sizeof(data_for_send)) {
        printf("Error send data: %i\n", res);

        return false;
    }

    while (true) {
        res = sim5300_receive(sim5300_dev,
                              sockfd, 
                              data_for_recv,
                              sizeof(data_for_recv));

        if (res < 0) {
            printf("Error recv data: %i\n", res);

            // return false;
        }
        printf("Recv data: %i\n%s\n", res, data_for_recv);

        xtimer_usleep(1000000);
    }

    // uint8_t index = 1;

    // /* Start up multi-IP TCP or UDP connection */
    // res = sim5300_multi_ip_up_single_connection(sim5300_dev, index, "TCP", "176.15.5.90", "666");
    // if (!(res >= 0)) {
    //     printf("[SIM5300] Error start connection: %i\n", res);

    //     return false;
    // }

    // int error = 0;
    // int i = 0;
    // char data_for_send[100] = {};
    // char data_for_recv[256] = {};

    // while (true) {
    //     /* Test recv */
    //     if (i == 10 || i == 20 || i == 30 || i == 40 || i == 50 || i == 60 || i == 70 || i == 80 || i == 90 || i == 100) {
    //         res = sim5300_receive_data_through_multi_ip_connection(sim5300_dev, 2, index, (uint8_t*)data_for_recv, sizeof(data_for_recv));
    //         if(res < 0) {
    //             printf("Receive ERROR: %i\n", res);
    //         } else {
    //             printf("Receive OK: %i\n", res);
    //             od_hex_dump(data_for_recv, res, OD_WIDTH_DEFAULT);
    //             printf("%s", data_for_recv);
    //         }   
    //     }

    //     snprintf(data_for_send, 100, "%i;%i;\n", i, error);
    //     printf("%s\n", data_for_send);

    //     if (!sim5300_send_data_through_multi_ip_connection(sim5300_dev,
    //                                                        index,
    //                                                        (uint8_t*)data_for_send, 
    //                                                        strlen(data_for_send))) {
    //         printf("[SIM5300] Data send ERROR\n");
    //         error++;
        
    //     } else {
    //         printf("[SIM5300] Data send OK\n");
    //     }

    //     i++;
    //     // rtctimers_millis_sleep(1000);
    //     // xtimer_usleep(1000000);

    // }

    return true;
}

/*---------------------------------------------------------------------------*/
int main(void)
{
    /* SIM5300 power on */
    sim5300_power_on();

    /* We wait while SIM5300 is initialized */
    rtctimers_millis_sleep(SIM5300_TIME_ON_UART);

    /* Init SIM5300 */
    if (!sim5300_init(&sim5300_dev, SIM5300_UART, SIM5300_BAUDRATE, at_dev_buf, AT_DEV_RESP_SIZE, at_dev_resp, AT_DEV_RESP_SIZE)) {
        puts("sim5300_init ERROR");
    } 

    /* Test driver SIM5300 */
    if (sim5300_test_driver(&sim5300_dev)) {
        puts("[SIM5300] Test OK");
    } else {
        puts("[SIM5300] Test ERROR");
    }

    /* Set internet settings SIM5300 */
    if (sim5300_set_internet(&sim5300_dev)) {
        puts("[SIM5300] Set internet settings OK");
    } else {
        puts("[SIM5300] Set internet settings ERROR");
    }

    while (true);

    // modem_power_off();

    // /* Test Modem */
    // /* Checking modem connection */
    // puts("Setting up cellular connectivity");
    // if (!modem_init()) {
    //     puts("Modem initialization failed");
    // } else {
    //     puts("Modem successfully initialized");
        
    //     if (!modem_test_connection()) {
    //         puts("No Internet connection available");
    //     } else {
    //         puts("Internet connection established");
            
    //         uint8_t data[] = "{\r\n\"uniq_id\": \"7297af2f-a615-41d9-9e24-b487e71b4124\",\r\n\"soft_ver\": \"1.0.1.0\"\r\n}";
        
    //         if (!modem_send_tcp_or_udp_send("manchenkoos.ru", "777", "UDP", data, sizeof(data) - 1)) {
    //             puts("Error uploading data");
    //         } else {
    //             puts("Data successfully uploaded");
    //         }
    //     }
    // }

    // modem_power_off();

    puts("Start Tele2Med WolfSSL");

    int                sockfd = 1;
    // int                sockfd;
    // struct sockaddr_in servAddr;
    // char               buff[22] = "Hello wolfSSL Server!\0";
    // char               server_ip[10] = "127.0.0.1\0";
    // size_t             len;

    /* declare wolfSSL objects */
    WOLFSSL_CTX* ctx;
    WOLFSSL*     ssl;

/*----------------------------------------------------------------------------*/
/* TLS Setup:
 * This section will need resolved on a per-device basis depending on the
 * available TCP/IP stack
 */
/*----------------------------------------------------------------------------*/

    // /* Create a socket that uses an internet IPv4 address,
    //  * Sets the socket to be stream based (TCP),
    //  * 0 means choose the default protocol. */
    // if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    //     fprintf(stderr, "ERROR: failed to create the socket\n");
    //     exit(-1);
    // }

    // /* Initialize the server address struct with zeros */
    // memset(&servAddr, 0, sizeof(servAddr));

    // /* Fill in the server address */
    // servAddr.sin_family = AF_INET;             /* using IPv4      */
    // servAddr.sin_port   = htons(DEFAULT_PORT); /* on DEFAULT_PORT */

    // /* Get the server IPv4 address from the command line call */
    // if (inet_pton(AF_INET, server_ip, &servAddr.sin_addr) != 1) {
    //     fprintf(stderr, "ERROR: invalid address\n");
    //     exit(-1);
    // }

    // /* Connect to the server */
    // if (connect(sockfd, (struct sockaddr*) &servAddr, sizeof(servAddr)) == -1) {
    //     fprintf(stderr, "ERROR: failed to connect\n");
    //     exit(-1);
    // }
/*----------------------------------------------------------------------------*/
/* END TCP SETUP, BEGIN TLS */
/*----------------------------------------------------------------------------*/

    wolfSSL_Debugging_ON();

    /* Initialize wolfSSL */
    wolfSSL_Init();

    /* Create and initialize WOLFSSL_CTX */
    if ((ctx = wolfSSL_CTX_new(wolfTLSv1_2_client_method())) == NULL) {
        fprintf(stderr, "ERROR: failed to create WOLFSSL_CTX\n");
        exit(-1);
    }

    wolfSSL_SetIORecv(ctx, unwired_recv);
    wolfSSL_SetIOSend(ctx, unwired_send);

    /* Load client certificates into WOLFSSL_CTX */
    if (wolfSSL_CTX_load_verify_buffer(ctx, ca_cert_der_2048,
                                       sizeof_ca_cert_der_2048,
                                       SSL_FILETYPE_ASN1) != SSL_SUCCESS) {
        fprintf(stderr, "ERROR: failed to load ca buffer\n");
        exit(-1);
    }

    /* Create a WOLFSSL object */
    if ((ssl = wolfSSL_new(ctx)) == NULL) {
        fprintf(stderr, "ERROR: failed to create WOLFSSL object\n");
        exit(-1);
    }

    /* Attach wolfSSL to the socket */
    wolfSSL_set_fd(ssl, sockfd);

    /* Connect to wolfSSL on the server side */
    if (wolfSSL_connect(ssl) != SSL_SUCCESS) {
        fprintf(stderr, "ERROR: failed to connect to wolfSSL\n");
        exit(-1);
    }

    // /* Get a message for the server from stdin */
    // printf("Message for server: %s\n", buff);
    // len = strnlen(buff, sizeof(buff));

    // /* Send the message to the server */
    // if (wolfSSL_write(ssl, buff, len) != (int) len) {
    //     fprintf(stderr, "ERROR: failed to write\n");
    //     exit(-1);
    // }

    // /* Read the server data into our buff array */
    // memset(buff, 0, sizeof(buff));
    // if (wolfSSL_read(ssl, buff, sizeof(buff)-1) == -1) {
    //     fprintf(stderr, "ERROR: failed to read\n");
    //     exit(-1);
    // }

    // /* Print to stdout any data the server sends */
    // printf("Server sent a reply!\n");
    // printf("Server Response was:  %s\n", buff);

    // /* Cleanup and exit */
    // wolfSSL_free(ssl);      /* Free the wolfSSL object                  */
    // wolfSSL_CTX_free(ctx);  /* Free the wolfSSL context object          */
    // wolfSSL_Cleanup();      /* Cleanup the wolfSSL environment          */
    // close(sockfd);          /* Close the connection to the server       */

    // exit(0);               /* Return reporting a success               */
}
