/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_gnrc_pktdump Dump Network Packets
 * @ingroup     net_gnrc
 * @brief       Dump network packets to STDOUT for debugging
 *
 * @{
 *
 * @file
 * @brief       Interface for a generic network packet dumping module
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef UNWDS_UDP_H
#define UNWDS_UDP_H

#include "kernel_types.h"
#include "protocol.h"

#ifdef UNWDS_ROOT
	#include "root_node.h"
#endif /* UNWDS_ROOT */

#ifdef UNWDS_DAG
	#include "dag_node.h"
#endif /* UNWDS_DAG */

#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief   Port of UNWDS_UDP_SERVER
 */
#ifndef UNWDS_UDP_SERVER_PORT
#define UNWDS_UDP_SERVER_PORT          		(UDP_DATA_PORT) /* ‭61616‬ */
#endif


/**
 * @brief   The PID of the UNWDS_UDP_SERVER thread
 */
extern kernel_pid_t unwds_udp_server_pid;

/**
 * @brief   Start UNWDS_UDP_SERVER thread and listening for incoming packets
 *
 * @return  PID of the UNWDS_UDP_SERVER thread
 * @return  negative value on error
 */
kernel_pid_t unwds_udp_server_init(void);

/**
 * @brief   Start listening for incoming packets on ‭61616‬ port
 */
void start_unwds_udp_server(void);

/**
 * @brief   Stop listening for incoming packets on ‭61616‬ port
 */
void stop_unwds_udp_server(void);

/**
 * @brief udp_send
 *
 * @param[in]		addr	Address 
 * @param[in]       port	Port
 * @param[in]       data	Data
 * @param[in]       len		Len
 */
void udp_send ( ipv6_addr_t *addr, 
				uint16_t port, 
				uint8_t  *data, 
				uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* UNWDS_UDP_H */
/** @} */
