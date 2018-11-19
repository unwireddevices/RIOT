/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     unwds_networking
 * @{
 *
 * @file
 * @brief       DAG node 
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include <stdio.h>

#include "od.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"

#define ENABLE_DEBUG            (1)
#include "debug.h"

void unwds_dag_server(gnrc_pktsnip_t *pkt)
{
#if ENABLE_DEBUG
	DEBUG("UNWDS_UDP: data received:\n");
	od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

    gnrc_pktbuf_release(pkt);
    return;
}
