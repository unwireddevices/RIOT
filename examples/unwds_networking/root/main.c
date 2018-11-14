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

// #include <udp.h>
 
#include <stdio.h>

#include "shell.h"
#include "msg.h"


#include <inttypes.h>
#include <errno.h>
#include "byteorder.h"
#include "thread.h"
#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#include "net/icmpv6.h"
#include "net/ipv6/addr.h"
#include "net/ipv6/hdr.h"
#include "net/tcp.h"
#include "net/udp.h"
#include "net/sixlowpan.h"
#include "od.h"

/**
 * @brief   Message queue size for the UNWDS_UDP_SERVER thread
 */
#ifndef UNWDS_UDP_SERVER_MSG_QUEUE_SIZE
#define UNWDS_UDP_SERVER_MSG_QUEUE_SIZE     (8U)
#endif

/**
 * @brief   Priority of the UNWDS_UDP_SERVER thread
 */
#ifndef UNWDS_UDP_SERVER_PRIO
#define UNWDS_UDP_SERVER_PRIO               (THREAD_PRIORITY_MAIN - 1)
#endif

/**
 * @brief   Stack size used for the UNWDS_UDP_SERVER thread
 */
#ifndef UNWDS_UDP_SERVER_STACKSIZE
#define UNWDS_UDP_SERVER_STACKSIZE          (THREAD_STACKSIZE_MAIN)
#endif

/**
 * @brief   Port of unwds udp server 
 */
#ifndef UNWDS_UDP_SERVER_PORT
#define UNWDS_UDP_SERVER_PORT          		(0xF0B0) //‭61616‬
#endif

/**
 * @brief   The PID of the UNWDS_UDP_SERVER thread
 */
kernel_pid_t unwds_udp_server_pid = KERNEL_PID_UNDEF;

/**
 * @brief   Stack for the pktdump thread
 */
static char _stack[UNWDS_UDP_SERVER_STACKSIZE];

/**
 * @brief   gnrc_netreg_entry_t
 */
static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               KERNEL_PID_UNDEF);

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
#endif
#ifdef MODULE_GNRC_SIXLOWPAN
        case GNRC_NETTYPE_SIXLOWPAN:
            printf("NETTYPE_SIXLOWPAN (%i)\n", pkt->type);
            sixlowpan_print(pkt->data, pkt->size);
            break;
#endif
#ifdef MODULE_GNRC_IPV6
        case GNRC_NETTYPE_IPV6:
            printf("NETTYPE_IPV6 (%i)\n", pkt->type);
            ipv6_hdr_print(pkt->data);
            hdr_len = sizeof(ipv6_hdr_t);
            break;
#endif
#ifdef MODULE_GNRC_ICMPV6
        case GNRC_NETTYPE_ICMPV6:
            printf("NETTYPE_ICMPV6 (%i)\n", pkt->type);
            icmpv6_hdr_print(pkt->data);
            hdr_len = sizeof(icmpv6_hdr_t);
            break;
#endif
#ifdef MODULE_GNRC_TCP
        case GNRC_NETTYPE_TCP:
            printf("NETTYPE_TCP (%i)\n", pkt->type);
            tcp_hdr_print(pkt->data);
            hdr_len = sizeof(tcp_hdr_t);
            break;
#endif
#ifdef MODULE_GNRC_UDP
        case GNRC_NETTYPE_UDP:
            printf("NETTYPE_UDP (%i)\n", pkt->type);
            udp_hdr_print(pkt->data);
            hdr_len = sizeof(udp_hdr_t);
            break;
#endif
#ifdef MODULE_CCN_LITE_UTILS
        case GNRC_NETTYPE_CCN_CHUNK:
            printf("GNRC_NETTYPE_CCN_CHUNK (%i)\n", pkt->type);
            printf("Content is: %.*s\n", (int)pkt->size, (char*)pkt->data);
            break;
#endif
#ifdef MODULE_NDN_RIOT
    case GNRC_NETTYPE_NDN:
            printf("NETTYPE_NDN (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
        break;
#endif
#ifdef TEST_SUITES
        case GNRC_NETTYPE_TEST:
            printf("NETTYPE_TEST (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
            break;
#endif
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
    gnrc_pktbuf_release(pkt);
}

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
                puts("PKTDUMP: data received:");
                _dump(msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                puts("PKTDUMP: data to send:");
                _dump(msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                msg_reply(&msg, &reply);
                break;
            default:
                puts("PKTDUMP: received something unexpected");
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
    puts("RIOT network stack example application");
	
	
	printf("init unwds udp server: %i\n", unwds_udp_server_init());
	
	/* start server (which means registering pktdump for the chosen port) */
    server.target.pid = unwds_udp_server_pid;
    server.demux_ctx = UNWDS_UDP_SERVER_PORT; 
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", UNWDS_UDP_SERVER_PORT);
	
	
	
	
	
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
	
    /* should be never reached */
    return 0;
}
