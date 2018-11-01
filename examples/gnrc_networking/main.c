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

#include <stdio.h>

#include "shell.h"
#include "msg.h"

#include "string.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/internal.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

	
	// ipv6_addr_t ipv6addrs[GNRC_NETIF_IPV6_ADDRS_NUMOF];
	// memset(ipv6addrs, 0, (sizeof(ipv6_addr_t) * GNRC_NETIF_IPV6_ADDRS_NUMOF));
	
	gnrc_netif_t *netif = gnrc_netif_get_by_pid(7);
	
	// gnrc_netapi_opt_t opt;
	// memset(&opt, 0, sizeof(gnrc_netapi_opt_t));
	
	// opt.opt = NETOPT_IPV6_ADDR;
	// opt.data = ipv6addrs;
	// opt.data_len = sizeof(ipv6_addr_t);
	
	// printf("\n"); 
	
	int res = 0;
	// res = gnrc_netif_get_from_netdev(netif, &opt);
	// printf("gnrc_netif_get_from_netdev: %d\n", res);
	// printf("IPv6 addr: ");
	// for(uint8_t i = 0; i < res; i++)
		// printf(" %x", ((uint8_t*)ipv6addrs)[i]);
	// printf("\n\n");
	// IPv6 addr:  fe 80 0 0 0 0 0 0 0 0 0 ff fe 0 df c8
	
	
	/*memset(ipv6addrs, 0, (sizeof(ipv6_addr_t) * GNRC_NETIF_IPV6_ADDRS_NUMOF));
	res = 0;
	
	res = gnrc_netif_ipv6_addrs_get(netif,
									ipv6addrs,
									sizeof(ipv6_addr_t) * GNRC_NETIF_IPV6_ADDRS_NUMOF);
	printf("gnrc_netif_ipv6_addrs_get: %d\n", res);
	printf("IPv6 addr: ");
	for(uint8_t i = 0; i < res; i++)
		printf(" %x", ((uint8_t*)ipv6addrs)[i]);
	printf("\n\n");*/
	// IPv6 addr:  fe 80 0 0 0 0 0 0 0 0 0 ff fe 0 df c8
	
	
	// uint8_t *ipv6addr = (uint8_t*)ipv6addrs;
	// ipv6addr[14] = 0xCA;
	// ipv6addr[15] = 0xFE;
	
	// printf("IPv6 addr: ");
	// for(uint8_t i = 0; i < res; i++)
		// printf(" %x", ((uint8_t*)ipv6addrs)[i]);
	// printf("\n\n");
	
	// netif->ipv6.addrs[0] = ipv6addrs[0];
	
	
	
	
	/*printf("l2addr_len: %i\n", netif->l2addr_len);
	printf("GNRC_NETIF_L2ADDR_MAXLEN: %i\n", GNRC_NETIF_L2ADDR_MAXLEN);
	
	printf("l2addr: ");
	for(uint8_t i = 0; i < GNRC_NETIF_L2ADDR_MAXLEN; i++)
		printf(" %x", netif->l2addr[i]);
	printf("\n\n");*/
	
	
	
	
	
	/**
 * @brief   Gets interface identifier (IID) of an interface's link-layer address
 *
 * @param[in] netif     the network interface
 * @param[out] eui64    the IID
 *
 * @return  0, on success
 * @return  -ENOTSUP, if interface has no link-layer address or if
 *          gnrc_netif_t::device_type is not supported.
 */
// int gnrc_netif_ipv6_get_iid(gnrc_netif_t *netif, eui64_t *eui64);
	
	eui64_t eui64;
	memset(&eui64, 0, sizeof(eui64_t));
	res = gnrc_netif_ipv6_get_iid(netif, &eui64);
	printf("gnrc_netif_ipv6_get_iid: %d\n", res);
	printf("eui64: ");
	for(uint8_t i = 0; i < sizeof(eui64_t); i++)
		printf(" %x", (((uint8_t*)&eui64)[i]));
	printf("\n\n");
	
	
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
	
    /* should be never reached */
    return 0;
}
