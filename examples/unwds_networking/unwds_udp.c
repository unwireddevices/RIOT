/*
 * Copyright (C) 2015-17 Freie Universität Berlin
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
 * @brief       Demonstrating the sending and receiving of UDP data
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Martine Lenders <m.lenders@fu-berlin.de>
 *
 * @}
 */

#include "unwds_udp.h"

#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "timex.h"
#include "utlist.h"
#include "xtimer.h"


#include <errno.h>
#include "byteorder.h"
#include "thread.h"
#include "net/icmpv6.h"
#include "net/ipv6/addr.h"
#include "net/tcp.h"
#include "net/sixlowpan.h"


#define ENABLE_DEBUG            (0)
#include "debug.h"
#include "od.h"

/**
 * @brief   Stack for the pktdump thread
 */
static char _stack[UNWDS_UDP_SERVER_STACKSIZE];

/**
 * @brief   The PID of the UNWDS_UDP_SERVER thread
 */
kernel_pid_t unwds_udp_server_pid = KERNEL_PID_UNDEF;

/**
 * @brief   gnrc_netreg_entry_t
 */
static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               KERNEL_PID_UNDEF);
															   
#if UNWDS_ROOT
extern void unwds_root_server(gnrc_pktsnip_t *pkt);
#endif /* UNWDS_ROOT */

#if UNWDS_DAG
extern void unwds_dag_server(gnrc_pktsnip_t *pkt);
#endif /* UNWDS_DAG */


#if ENABLE_DEBUG
static void _dump_snip(gnrc_pktsnip_t *pkt)
{
    size_t hdr_len = pkt->size;

    switch (pkt->type) {
        case GNRC_NETTYPE_UNDEF:
            printf("NETTYPE_UNDEF (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
            break;
			
#ifdef MODULE_GNRC_NETIF
        case GNRC_NETTYPE_NETIF:
            printf("NETTYPE_NETIF (%i)\n", pkt->type);
            gnrc_netif_hdr_print(pkt->data);
            break;
#endif /* MODULE_GNRC_NETIF */

#ifdef MODULE_GNRC_SIXLOWPAN
        case GNRC_NETTYPE_SIXLOWPAN:
            printf("NETTYPE_SIXLOWPAN (%i)\n", pkt->type);
            sixlowpan_print(pkt->data, pkt->size);
            break;
#endif /* MODULE_GNRC_SIXLOWPAN */

#ifdef MODULE_GNRC_IPV6
        case GNRC_NETTYPE_IPV6:
            printf("NETTYPE_IPV6 (%i)\n", pkt->type);
            ipv6_hdr_print(pkt->data);
            hdr_len = sizeof(ipv6_hdr_t);
            break;
#endif /* MODULE_GNRC_IPV6 */

#ifdef MODULE_GNRC_ICMPV6
        case GNRC_NETTYPE_ICMPV6:
            printf("NETTYPE_ICMPV6 (%i)\n", pkt->type);
            icmpv6_hdr_print(pkt->data);
            hdr_len = sizeof(icmpv6_hdr_t);
            break;
#endif /* MODULE_GNRC_ICMPV6 */

#ifdef MODULE_GNRC_TCP
        case GNRC_NETTYPE_TCP:
            printf("NETTYPE_TCP (%i)\n", pkt->type);
            tcp_hdr_print(pkt->data);
            hdr_len = sizeof(tcp_hdr_t);
            break;
#endif /* MODULE_GNRC_TCP */

#ifdef MODULE_GNRC_UDP
        case GNRC_NETTYPE_UDP:
            printf("NETTYPE_UDP (%i)\n", pkt->type);
            udp_hdr_print(pkt->data);
            hdr_len = sizeof(udp_hdr_t);
            break;
#endif /* MODULE_GNRC_UDP */

#ifdef MODULE_CCN_LITE_UTILS
        case GNRC_NETTYPE_CCN_CHUNK:
            printf("GNRC_NETTYPE_CCN_CHUNK (%i)\n", pkt->type);
            printf("Content is: %.*s\n", (int)pkt->size, (char*)pkt->data);
            break;
#endif /* MODULE_CCN_LITE_UTILS */

#ifdef MODULE_NDN_RIOT
    case GNRC_NETTYPE_NDN:
            printf("NETTYPE_NDN (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
        break;
#endif /* MODULE_NDN_RIOT */

#ifdef TEST_SUITES
        case GNRC_NETTYPE_TEST:
            printf("NETTYPE_TEST (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
            break;
#endif /* TEST_SUITES */

        default:
            printf("NETTYPE_UNKNOWN (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
            break;
    }
	
    if (hdr_len < pkt->size) {
        size_t size = pkt->size - hdr_len;

        od_hex_dump(((uint8_t *)pkt->data) + hdr_len, size, OD_WIDTH_DEFAULT);
    }
}

static void _dump(gnrc_pktsnip_t *pkt)
{
    int snips = 0;
    int size = 0;
    gnrc_pktsnip_t *snip = pkt;

    while (snip != NULL) {
        printf("~~ SNIP %2i - size: %3u byte, type: ", snips,
               (unsigned int)snip->size);
        _dump_snip(snip);
        ++snips;
        size += snip->size;
        snip = snip->next;
    }

    printf("~~ PKT    - %2i snips, total size: %3i byte\n", snips, size);
    // gnrc_pktbuf_release(pkt);
}
#endif /* ENABLE_DEBUG */

static void *_eventloop(void *arg)
{
    (void)arg;
    msg_t msg, reply;
    msg_t msg_queue[UNWDS_UDP_SERVER_MSG_QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, UNWDS_UDP_SERVER_MSG_QUEUE_SIZE);

    reply.content.value = (uint32_t)(-ENOTSUP);
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;

    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                DEBUG("UNWDS_UDP: data received:\n");
#if ENABLE_DEBUG
                _dump(msg.content.ptr);
#endif /* ENABLE_DEBUG */
#if UNWDS_ROOT
				unwds_root_server(msg.content.ptr);
#endif /* UNWDS_ROOT */
#if UNWDS_DAG
				unwds_dag_server(msg.content.ptr);
#endif /* UNWDS_DAG */
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                DEBUG("UNWDS_UDP: data to send:\n");
#if ENABLE_DEBUG
                _dump(msg.content.ptr);
#endif /* ENABLE_DEBUG */
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                msg_reply(&msg, &reply);
                break;
            default:
                DEBUG("UNWDS_UDP: received something unexpected\n");
                break;
        }
    }

    /* never reached */
    return NULL;
}

/**
 * @brief   Start unwds udp server thread and listening for incoming packets
 *
 * @return  PID of the UNWDS_UDP_SERVER thread
 * @return  negative value on error
 */
kernel_pid_t unwds_udp_server_init(void)
{
    if (unwds_udp_server_pid == KERNEL_PID_UNDEF) {
        unwds_udp_server_pid = thread_create(_stack, sizeof(_stack), UNWDS_UDP_SERVER_PRIO,
                             THREAD_CREATE_STACKTEST,
                             _eventloop, NULL, "unwds udp server");
    }
	
    return unwds_udp_server_pid;
}

static void udp_shell_send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay)
{
    int iface;
    uint16_t port;
    ipv6_addr_t addr;

    /* get interface, if available */
    iface = ipv6_addr_split_iface(addr_str);
    if ((iface < 0) && (gnrc_netif_numof() == 1)) {
        iface = gnrc_netif_iter(NULL)->pid;
    }
    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: unable to parse destination port");
        return;
    }

    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        unsigned payload_size;
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, data, strlen(data), GNRC_NETTYPE_UNDEF);
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return;
        }
        /* store size for output */
        payload_size = (unsigned)payload->size;
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            return;
        }
        /* add netif header, if interface was given */
        if (iface > 0) {
            gnrc_pktsnip_t *netif = gnrc_netif_hdr_build(NULL, 0, NULL, 0);

            ((gnrc_netif_hdr_t *)netif->data)->if_pid = (kernel_pid_t)iface;
            LL_PREPEND(ip, netif);
        }
        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            return;
        }
        /* access to `payload` was implicitly given up with the send operation above
         * => use temporary variable for output */
        printf("Success: sent %u byte(s) to [%s]:%u\n", payload_size, addr_str,
               port);
        xtimer_usleep(delay);
    }
}

void udp_send ( ipv6_addr_t *addr, 
				uint16_t port, 
				uint8_t  *data, 
				uint16_t len)
{	
    int iface;
    // uint16_t port;
    // ipv6_addr_t addr;

    /* get interface, if available */
    iface = 7;//ipv6_addr_split_iface(addr_str);
    if ((iface < 0) && (gnrc_netif_numof() == 1)) {
        iface = gnrc_netif_iter(NULL)->pid;
    }
	
    /* parse destination address */
    // if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        // puts("Error: unable to parse destination address");
        // return;
    // }
	
    /* parse port */
    // port = atoi(port_str);
    // if (port == 0) {
        // puts("Error: unable to parse destination port");
        // return;
    // }

	gnrc_pktsnip_t *payload, *udp, *ip;
	// unsigned payload_size;
	
	/* allocate payload */
	payload = gnrc_pktbuf_add(NULL, data, len, GNRC_NETTYPE_UNDEF);
	if (payload == NULL) {
		puts("Error: unable to copy data to packet buffer");
		return;
	}
	/* store size for output */
	// payload_size = (unsigned)payload->size;
	
	/* allocate UDP header, set source port := destination port */
	udp = gnrc_udp_hdr_build(payload, port, port);
	if (udp == NULL) {
		puts("Error: unable to allocate UDP header");
		gnrc_pktbuf_release(payload);
		return;
	}
	
	/* allocate IPv6 header */
	ip = gnrc_ipv6_hdr_build(udp, NULL, addr);
	if (ip == NULL) {
		puts("Error: unable to allocate IPv6 header");
		gnrc_pktbuf_release(udp);
		return;
	}
	
	/* add netif header, if interface was given */
	if (iface > 0) {
		gnrc_pktsnip_t *netif = gnrc_netif_hdr_build(NULL, 0, NULL, 0);

		((gnrc_netif_hdr_t *)netif->data)->if_pid = (kernel_pid_t)iface;
		LL_PREPEND(ip, netif);
	}
	
	/* send packet */
	if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
		puts("Error: unable to locate UDP thread");
		gnrc_pktbuf_release(ip);
		return;
	}
	/* access to `payload` was implicitly given up with the send operation above
	 * => use temporary variable for output */
	 
	// addr_str[16]
	// char *gnrc_netif_addr_to_str(const uint8_t *addr, size_t addr_len, char *out);
	// printf("Success: sent %u byte(s) to [%s]:%u\n", payload_size, addr_str, port);
	printf("Success: sent\n");
}

void start_unwds_udp_server(void)
{
    /* check if server is already running */
    if (server.target.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %" PRIu32 "\n",
               server.demux_ctx);
        return;
    }
	
	/* start server (which means registering pktdump for the chosen port) */
    server.target.pid = unwds_udp_server_pid;
    server.demux_ctx = UNWDS_UDP_SERVER_PORT; 
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", UNWDS_UDP_SERVER_PORT);
}

void stop_unwds_udp_server(void)
{
    /* check if server is running at all */
    if (server.target.pid == KERNEL_PID_UNDEF) {
        printf("Error: server was not running\n");
        return;
    }
    /* stop server */
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
    server.target.pid = KERNEL_PID_UNDEF;
    puts("Success: stopped UDP server");
}

int udp_cmd(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: %s [send|server]\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "send") == 0) {
        uint32_t num = 1;
        uint32_t delay = 1000000;
        if (argc < 5) {
            printf("Usage: %s send <addr> <port> <data> [<num> [<delay in us>]]\n",
                   argv[0]);
            return 1;
        }
        if (argc > 5) {
            num = atoi(argv[5]);
        }
        if (argc > 6) {
            delay = atoi(argv[6]);
        }
        udp_shell_send(argv[2], argv[3], argv[4], num, delay);
    }
    else if (strcmp(argv[1], "server") == 0) {
        if (argc < 3) {
            printf("Usage: %s server [start|stop]\n", argv[0]);
            return 1;
        }
        if (strcmp(argv[2], "start") == 0) {
            start_unwds_udp_server();
        }
        else if (strcmp(argv[2], "stop") == 0) {
            stop_unwds_udp_server();
        }
        else {
            puts("Error: invalid command");
        }
    }
    else {
        puts("Error: invalid command");
    }
    return 0;
}
