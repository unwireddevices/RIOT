/*
 * Copyright (C) 2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_nrf5x_nrfmax_gnrc
 * @{
 *
 * @file
 * @brief       GNRC adapter for the nrfmax radio driver
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include "net/gnrc.h"
#include "thread.h"
#include "net/gnrc/netif.h"

#include "nrfmax_gnrc.h"

#define ENABLE_DEBUG            (0)
#include "debug.h"

/**
 * @brief   Definition of default thread priority and stacksize
 * @{
 */
#ifndef NRFMAX_GNRC_THREAD_PRIO
#define NRFMAX_GNRC_THREAD_PRIO     GNRC_NETIF_PRIO
#endif

#ifndef NRFMAX_GNRC_STACKSIZE
#define NRFMAX_GNRC_STACKSIZE       THREAD_STACKSIZE_DEFAULT
#endif
/** @} */

/**
 * @brief
 */
#define BCAST   (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)

/**
 * @brief   Allocate the stack for the GNRC netdev thread to run in
 */
static char stack[NRFMAX_GNRC_STACKSIZE];

static int hdr_netif_to_nrfmax(nrfmax_hdr_t *nrfmax, gnrc_pktsnip_t *pkt)
{
    gnrc_netif_hdr_t *netif = (gnrc_netif_hdr_t *)pkt->data;

    if (!(netif->flags & BCAST) && (netif->dst_l2addr_len != sizeof(eui64_t))) {
        return -EINVAL;
    }

    nrfmax->len = gnrc_pkt_len(pkt->next) + NRFMAX_HDR_LEN;
    if (netif->flags & BCAST) {
		memset(&nrfmax->dst_addr, 0xFF, sizeof(eui64_t));
    }
    else {
        memcpy(&nrfmax->dst_addr, gnrc_netif_hdr_get_dst_addr(netif), sizeof(eui64_t));
    }
	
	nrfmax_get_eui64(&nrfmax->src_addr);
	
    if (pkt->next) {
        nrfmax->proto = (uint8_t)pkt->next->type;
    }
    else {
        nrfmax->proto = 0;
    }

    return 0;
}

static int gnrc_nrfmax_send(gnrc_netif_t *dev, gnrc_pktsnip_t *pkt) 
{
    int res;
    nrfmax_hdr_t nrfmax_hdr;

    assert(pkt);

    if (pkt->type != GNRC_NETTYPE_NETIF) {
        DEBUG("[nrfmax_gnrc] send: first header is not generic netif header\n");
        return -EBADMSG;
    }

    /* Build the nrfmax header from the generic netif header */
    res = hdr_netif_to_nrfmax(&nrfmax_hdr, pkt);
    if (res < 0) 
	{
        DEBUG("[nrfmax_gnrc] send: failed to build nrfmax header\n");
    }
	else
	{
		/* Link first entry after netif hdr of the pkt to the nrfmax header */
		iolist_t iolist = {
			.iol_next = (iolist_t *)pkt->next,
			.iol_base = &nrfmax_hdr,
			.iol_len = NRFMAX_HDR_LEN
		};

		/* And finally send out the data and release the packet */
		res = dev->dev->driver->send(dev->dev, &iolist);
	}

    gnrc_pktbuf_release(pkt);

    return res;
}

static gnrc_pktsnip_t *gnrc_nrfmax_recv(gnrc_netif_t *dev)
{
    int pktsize;
    nrfmax_hdr_t *nrfmax;
    gnrc_netif_hdr_t *netif;
    gnrc_pktsnip_t *pkt_snip;
    gnrc_pktsnip_t *hdr_snip;
    gnrc_pktsnip_t *netif_snip;

    /* Get the size of the new packet */
    pktsize = nrfmax_dev.driver->recv(NULL, NULL, 0, NULL);
    if (pktsize <= 0) {
        DEBUG("[nrfmax_gnrc] recv: error: tried to read empty packet\n");
        return NULL;
    }

    /* Allocate space in the packet buffer */
    pkt_snip = gnrc_pktbuf_add(NULL, NULL, pktsize, GNRC_NETTYPE_UNDEF);
    if (pkt_snip == NULL) {
        DEBUG("[nrfmax_gnrc] recv: unable to allocate pktsnip\n");
        return NULL;
    }

    /* Read the incoming data into the packet buffer */
    nrfmax_dev.driver->recv(NULL, pkt_snip->data, pktsize, NULL);

    /* Now we mark the nrfmax header */
    hdr_snip = gnrc_pktbuf_mark(pkt_snip, NRFMAX_HDR_LEN, GNRC_NETTYPE_UNDEF);
    if (hdr_snip == NULL) {
        DEBUG("[nrfmax_gnrc] recv: unable to mark the nrfmax header\n");
        gnrc_pktbuf_release(pkt_snip);
        return NULL;
    }

    /* Allocate the generic netif header and populate it with data from the
       nrfmax header */
    nrfmax = (nrfmax_hdr_t *)hdr_snip->data;
    netif_snip = gnrc_netif_hdr_build((uint8_t *)&nrfmax->src_addr, sizeof(eui64_t),
                                      (uint8_t *)&nrfmax->dst_addr, sizeof(eui64_t));
    if (netif_snip == NULL) {
        DEBUG("[nrfmax_gnrc] recv: unable to allocate netif header\n");
        gnrc_pktbuf_release(pkt_snip);
        return NULL;
    }

    netif = (gnrc_netif_hdr_t *)netif_snip->data;
	
	/* Broadcast check */
	eui64_t eui64_bcast;
	memset(&eui64_bcast, 0xFF, sizeof(eui64_t));
    if (memcmp(&nrfmax->dst_addr, &eui64_bcast, sizeof(eui64_t)) == 0) {
        netif->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
    }
	
    netif->lqi = 0;
    netif->rssi = 0;
    netif->if_pid = dev->pid;
    pkt_snip->type = nrfmax->proto;

    /* Finally: remove the nrfmax header and append the netif header */
    gnrc_pktbuf_remove_snip(pkt_snip, hdr_snip);
    LL_APPEND(pkt_snip, netif_snip);
	
    return pkt_snip;
}

/**
 * @see gnrc_netif_ops_t
 */
static const gnrc_netif_ops_t gnrc_nrfmax_ops = {
    .send = gnrc_nrfmax_send,			/* Send a @ref net_gnrc_pkt "packet" over the network interface */
    .recv = gnrc_nrfmax_recv,			/* Receives a @ref net_gnrc_pkt "packet" from the network interface */
    .get = gnrc_netif_get_from_netdev,	/* Gets an option from the network interface */
    .set = gnrc_netif_set_from_netdev,	/* Sets an option from the network interface */
};

void gnrc_nrfmax_init(void)
{
    /* Setup the NRFMAX driver */
    nrfmax_setup();
    gnrc_netif_create(stack, sizeof(stack), NRFMAX_GNRC_THREAD_PRIO, "nrfmax",
                     (netdev_t *)&nrfmax_dev, &gnrc_nrfmax_ops);
}
