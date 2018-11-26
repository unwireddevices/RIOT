/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Example application for demonstrating the RIOT network stack
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "unwds-udp.h"
#include <stdio.h>

#include "shell.h"
#include "msg.h"
#include "net/gnrc/netif.h"

#include "crypto/aes.h"

#define ENABLE_DEBUG		(0)
#include "debug.h"
#include "od.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "unwds_udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};


int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT Unwired Devices mesh network.");
	
	if(root_node_init() < 0)
    {
        puts("Error ROOT node init");
        return -1;
    }

	puts("Unwired Devices mesh network initialization is over\n");
	
	// ipv6_addr_t addr_dag;
	
	// addr_dag.u8[0] = 0xFE;
	// addr_dag.u8[1] = 0x80;
	// addr_dag.u8[2] = 0x00;
	// addr_dag.u8[3] = 0x00;
	// addr_dag.u8[4] = 0x00;
	// addr_dag.u8[5] = 0x00;
	// addr_dag.u8[6] = 0x00;
	// addr_dag.u8[7] = 0x00;
	// addr_dag.u8[8] = 0x1D;
	// addr_dag.u8[9] = 0x9A;
	// addr_dag.u8[10] = 0x5E;
	// addr_dag.u8[11] = 0x49;
	// addr_dag.u8[12] = 0x41;
	// addr_dag.u8[13] = 0x5B;
	// addr_dag.u8[14] = 0x29;
	// addr_dag.u8[15] = 0xA4;
	
	// unwds_pack_sender ( &addr_dag, 
	// 					UNWDS_OPT3001_MODULE_ID, 
	// 					LIT_MEASURE, 
	// 					LIT_MEASURE_LENGTH, 
	// 					NULL);	

	// uint8_t TEST_0_KEY[] = {
    // 	0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    // 	0x99, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
	// };
	// uint8_t TEST_0_INP[] = {
    // 	0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    // 	0x99, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
	// };
	// uint8_t TEST_0_ENC[] = {
	// 	0x20, 0x63, 0xB8, 0x45, 0xB7, 0x8A, 0xB9, 0xC8, 
	// 	0x04, 0x57, 0xED, 0x5E, 0x17, 0x3A, 0x09, 0xD1
	// };
	// uint8_t TEST_0_DEC[] = {
	// 	0xA9, 0x36, 0xB3, 0x40, 0x77, 0x4F, 0x87, 0x46, 
	// 	0xCD, 0xAD, 0xBC, 0x21, 0x66, 0xE8, 0x8C, 0xBD
	// };
	// cipher_context_t ctx;
    // int err;
    // uint8_t data[AES_BLOCK_SIZE];

	// puts("Test AES:");
	// od_hex_dump(TEST_0_KEY, sizeof(TEST_0_KEY), OD_WIDTH_DEFAULT);
	
	// puts("Input:");
	// od_hex_dump(TEST_0_INP, sizeof(TEST_0_INP), OD_WIDTH_DEFAULT);

    // err = aes_init(&ctx, TEST_0_KEY, AES_KEY_SIZE);
	// if(!(err)
    // 	printf("err: %i", err);

    // err = aes_encrypt(&ctx, TEST_0_INP, data);
	// if(!(err)
    // 	printf("err: %i", err);

	// puts("Encryption:");
	// od_hex_dump(TEST_0_ENC, sizeof(TEST_0_ENC), OD_WIDTH_DEFAULT);

	// puts("Output:");
	// od_hex_dump(data, sizeof(data), OD_WIDTH_DEFAULT);

	// err = aes_decrypt(&ctx, TEST_0_INP, data);
	// if(!(err)
    // 	printf("err: %i", err);

	// puts("Decryption:");
	// od_hex_dump(TEST_0_DEC, sizeof(TEST_0_ENC), OD_WIDTH_DEFAULT);

	// puts("Output:"); 
	// od_hex_dump(data, sizeof(data), OD_WIDTH_DEFAULT);
	
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
	
    /* should be never reached */
    return 0;
}
