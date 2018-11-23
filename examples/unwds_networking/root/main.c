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
#include "net/gnrc/rpl.h"

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
	
	if(unwds_udp_server_init() < 0)
	{
		puts("Error init unwds udp server.");	
		return -1;
	}
	
	puts("Init unwds udp server.");	
	start_unwds_udp_server();
	
	uint16_t flags = GNRC_NETIF_IPV6_ADDRS_FLAGS_STATE_VALID | (64 << 8);
	ipv6_addr_t addr;
	uint8_t instance_id = 1;
	
	addr.u8[0] = 0x20;
	addr.u8[1] = 0x01;
	addr.u8[2] = 0x0D;
	addr.u8[3] = 0xB8;
	addr.u8[4] = 0x00;
	addr.u8[5] = 0x00;
	addr.u8[6] = 0x00;
	addr.u8[7] = 0x00;
	addr.u8[8] = 0x00;
	addr.u8[9] = 0x00;
	addr.u8[10] = 0x00;
	addr.u8[11] = 0x00;
	addr.u8[12] = 0x00;
	addr.u8[13] = 0x00;
	addr.u8[14] = 0x00;
	addr.u8[15] = 0x01;
	
	// ifconfig 7 add 2001:db8::1  
	printf("Number of network interfaces: %i\n", gnrc_netif_numof()); 
	gnrc_netif_t *netif = gnrc_netif_iter(NULL);
	if(netif == NULL)
	{
		puts("Error. No interface");
		return -1;
	}

	if(gnrc_netapi_set(netif->pid, NETOPT_IPV6_ADDR, flags, &addr, sizeof(addr)) < 0) 
	{
		puts("Error: unable to add IPv6 address");
		return -1;
	}
	printf("Success: added root IPv6 address to interface %i\n", netif->pid);
	
	// rpl root 1 2001:db8::1
	gnrc_rpl_instance_t *inst = gnrc_rpl_root_init(instance_id, &addr, false, false);
    if (inst == NULL) {
        printf("Error: could not add DODAG to instance %i\n", instance_id);
        return -1;
    }

    puts("Successfully added a new RPL DODAG");
	puts("Unwired Devices mesh network initialization is over\n");
	
	ipv6_addr_t addr_dag;
	
	addr_dag.u8[0] = 0xFE;
	addr_dag.u8[1] = 0x80;
	addr_dag.u8[2] = 0x00;
	addr_dag.u8[3] = 0x00;
	addr_dag.u8[4] = 0x00;
	addr_dag.u8[5] = 0x00;
	addr_dag.u8[6] = 0x00;
	addr_dag.u8[7] = 0x00;
	addr_dag.u8[8] = 0x1D;
	addr_dag.u8[9] = 0x9A;
	addr_dag.u8[10] = 0x5E;
	addr_dag.u8[11] = 0x49;
	addr_dag.u8[12] = 0x41;
	addr_dag.u8[13] = 0x5B;
	addr_dag.u8[14] = 0x29;
	addr_dag.u8[15] = 0xA4;
	
	unwds_pack_sender ( &addr_dag, 
						UNWDS_OPT3001_MODULE_ID, 
						LIT_MEASURE, 
						LIT_MEASURE_LENGTH, 
						NULL);
	
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
	
    /* should be never reached */
    return 0;
}
