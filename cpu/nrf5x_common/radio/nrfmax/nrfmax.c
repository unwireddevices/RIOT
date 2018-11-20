/*
 * Copyright (C) 2015-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_nrf5x_nrfmax
 * @{
 *
 * @file
 * @brief       Implementation of the nrfmax radio driver for nRF51 radios
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <errno.h>

#include "cpu.h"
#include "mutex.h"
#include "assert.h"

#include "periph_conf.h"
#include "periph/cpuid.h"

#include "nrfmax.h"
#include "net/netdev.h"

#ifdef MODULE_GNRC_SIXLOWPAN
#include "net/gnrc/nettype.h"
#endif

#define ENABLE_DEBUG            (1)
#include "debug.h"
#include "od.h"

/**
 * @brief   Driver specific device configuration
 * @{
 */
#define CONF_MODE               RADIO_MODE_MODE_Nrf_1Mbit
#define CONF_LEN                (8U)
#define CONF_S0                 (0U)
#define CONF_S1                 (0U)
#define CONF_STATLEN            (0U)
#define CONF_BASE_ADDR_LEN      (4U)
#define CONF_ENDIAN             RADIO_PCNF1_ENDIAN_Big
#define CONF_WHITENING          RADIO_PCNF1_WHITEEN_Disabled
#define CONF_CRC_LEN            (2U)
#define CONF_CRC_POLY           (0x00011021)
#define CONF_CRC_INIT           (0x00F0F0F0)
/** @} */

/**
 * @brief   Driver specific address configuration
 * @{
 */
#define CONF_ADDR_PREFIX0       (0x0000FF00)
#define CONF_ADDR_BCAST         (0xFFFFFFFF)
/** @} */

/**
 * @brief   We define a pseudo NID for compliance to 6LoWPAN
 */
#define CONF_PSEUDO_NID         (0xAFFE)

/**
 * @brief   Driver specific (interrupt) events (not all of them used currently)
 * @{
 */
#define ISR_EVENT_RX_START      (0x0001)
#define ISR_EVENT_RX_DONE       (0x0002)
#define ISR_EVENT_TX_START      (0x0004)
#define ISR_EVENT_TX_DONE       (0x0008)
#define ISR_EVENT_WRONG_CHKSUM  (0x0010)
/** @} */

/**
 * @brief   Since there can only be 1 nrfmax device, we allocate it right here
 */
netdev_t nrfmax_dev;

/**
 * @brief   For faster lookup we remember our own 64-bit address
 */
eui64_t nrfmax_eui64;

/**
 * @brief   We need to keep track of the radio state in SW (-> PAN ID 20)
 *
 * See nRF51822 PAN ID 20: RADIO State Register is not functional.
 */
volatile state_t nrfmax_state = STATE_OFF;

/**
 * @brief   We also remember the 'long-term' state, so we can resume after TX
 */
volatile state_t nrfmax_target_state = STATE_OFF;

/**
 * @brief   When sending out data, the data needs to be in one continuous memory
 *          region. So we need to buffer outgoing data on the driver level.
 */
static nrfmax_pkt_t tx_buf;

/**
 * @brief   As the device is memory mapped, we need some space to save incoming
 *          data to.
 *
 * @todo    Improve the RX buffering to at least use double buffering
 */
static nrfmax_pkt_t rx_buf;

/**
 * @brief   While we listen for incoming data, we lock the RX buffer
 */
static volatile uint8_t rx_lock = 0;

/**
 * @brief   Set radio into idle (DISABLED) state
 */
static void go_idle(void)
{
    /* Set device into basic disabled state */
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0) {}
	
    /* Also release any existing lock on the RX buffer */
    rx_lock = 0;
    nrfmax_state = STATE_IDLE;
}

/**
 * @brief   Set radio into the target state as defined by `nrfmax_target_state`
 *
 * Trick here is, that the driver can go back to it's previous state after a
 * send operation, so it can differentiate if the driver was in DISABLED or in
 * RX mode before the send process had started.
 */
void nrfmax_goto_target_state(void)
{
    go_idle();
	
	switch(nrfmax_target_state) 
	{
        case STATE_RX:  // if ((nrfmax_target_state == STATE_RX) //&& (rx_buf.pkt.hdr.len == 0))
			/* Set receive buffer and our own address */
			rx_lock = 1;
			NRF_RADIO->PACKETPTR = (uint32_t)(&rx_buf);
			NRF_RADIO->PREFIX0 = (CONF_ADDR_PREFIX0 | nrfmax_eui64.uint8[3]);
			NRF_RADIO->BASE0 = ((nrfmax_eui64.uint8[4] << 24) |
								(nrfmax_eui64.uint8[5] << 16) |
								(nrfmax_eui64.uint8[6] << 8)  |
								(nrfmax_eui64.uint8[7]));
					
			/* Goto RX mode */
			NRF_RADIO->TASKS_RXEN = 1;
			nrfmax_state = STATE_RX;
            break;
        case STATE_OFF:
			NRF_RADIO->POWER = 0;
			nrfmax_state = STATE_OFF;
            break;
		case NETOPT_STATE_SLEEP:
            break;
        default:
            printf("ERROR GOTO TARGET STATE\n");
			break;
    }
}

/**
 * @brief   Setup the device driver's data structures
 */
void nrfmax_setup(void)
{
    nrfmax_dev.driver = &nrfmax_netdev;
    nrfmax_dev.event_callback = NULL;
    nrfmax_dev.context = NULL;
#ifdef MODULE_NETSTATS_L2
    memset(&nrfmax_dev.stats, 0, sizeof(netstats_t));
#endif
}

/**
 * @brief   Radio interrupt routine
 */
void isr_radio(void)
{
    if (NRF_RADIO->EVENTS_END == 1) 
	{
        NRF_RADIO->EVENTS_END = 0;
		
        /* Did we just send or receive something? */
        if (nrfmax_state == STATE_RX) 
		{
            /* Drop packet on invalid CRC */
            if ((NRF_RADIO->CRCSTATUS != 1) || !(nrfmax_dev.event_callback)) 
			{
                rx_buf.pkt.hdr.len = 0;
                NRF_RADIO->TASKS_START = 1;
                return;
            }
			
            rx_lock = 0;
            nrfmax_dev.event_callback(&nrfmax_dev, NETDEV_EVENT_ISR, NULL);
        }
		
		/* If the transfer is over, then go to the target state.*/
        else if (nrfmax_state == STATE_TX) 
		{
            nrfmax_goto_target_state();
        }
    }

    cortexm_isr_end();
}

/**
 * @brief   Send frame
 */
static int nrfmax_send(netdev_t *dev, const iolist_t *iolist)
{
    (void)dev;

    assert((iolist) && (nrfmax_state != STATE_OFF));

    /* Wait for any ongoing transmission to finish and go into idle state */
    while (nrfmax_state == STATE_TX) {}
    go_idle();

    /* Copy packet data into the transmit buffer */
    int pos = 0;
    for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) 
	{
        if ((pos + iol->iol_len) > NRFMAX_PKT_MAX) 
		{
            DEBUG("[nrfmax] send: unable to do so, packet is too large!\n");
            return -EOVERFLOW;
        }
		
        memcpy(&tx_buf.raw[pos], iol->iol_base, iol->iol_len);	
        pos += iol->iol_len;
    }

    /* Set output buffer and destination address */
    nrfmax_hdr_t *hdr = (nrfmax_hdr_t *)iolist->iol_base;
    NRF_RADIO->PACKETPTR = (uint32_t)(&tx_buf);
	NRF_RADIO->PREFIX0 = (CONF_ADDR_PREFIX0 | tx_buf.raw[12]);
	NRF_RADIO->BASE0 = ((tx_buf.raw[13] << 24) |
						(tx_buf.raw[14] << 16) |
						(tx_buf.raw[15] << 8)  |
						(tx_buf.raw[16]));

#if ENABLE_DEBUG
    DEBUG("[nrfmax] send: putting %i byte into the ether\n", (int)hdr->len);
	DEBUG("[nrfmax] data send:\n");
	od_hex_dump(&tx_buf, hdr->len, OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */	

	/* Trigger the actual transmission */
    nrfmax_state = STATE_TX;
    NRF_RADIO->TASKS_TXEN = 1;

    return (int)pos;
}

/**
 * @brief   Get a received frame 
 */
static int nrfmax_recv(netdev_t *dev, void *buf, size_t len, void *info)
{
    (void)dev;
    (void)info;

    assert(nrfmax_state != STATE_OFF);

    unsigned pktlen = rx_buf.pkt.hdr.len;

    /* Check if packet data is readable */
    if (rx_lock || (pktlen == 0)) 
	{
        DEBUG("[nrfmax] recv: no packet data available\n");
        return 0;
    }

    if (buf == NULL) 
	{
        if (len > 0) 
		{
            /* Drop packet */
            DEBUG("[nrfmax] recv: dropping packet of length %i\n", pktlen);
            rx_buf.pkt.hdr.len = 0;
            nrfmax_goto_target_state();
        }
    }
    else 
	{     
#if ENABLE_DEBUG
		DEBUG("[nrfmax] recv: reading packet of length %i\n", pktlen);
		DEBUG("[nrfmax] data received:\n");
		od_hex_dump(&rx_buf, pktlen, OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

        pktlen = (len < pktlen) ? len : pktlen;
        memcpy(buf, rx_buf.raw, pktlen);
        rx_buf.pkt.hdr.len = 0;
        nrfmax_goto_target_state();
    }

    return pktlen;
}

/**
 * @brief   The driver's initialization function
 */
static int nrfmax_init(netdev_t *dev)
{
    (void)dev;
    uint8_t cpuid[CPUID_LEN];

    /* Check given device descriptor */
    assert(dev);

    /* Initialize our own address from the CPU ID */
    cpuid_get(cpuid);
	nrfmax_set_eui64((eui64_t*)cpuid);

    /* Power on the NRFs radio */
    NRF_RADIO->POWER = 1;
	
    /* Load driver specific configuration */
    NRF_RADIO->MODE = CONF_MODE;
	
    /* Configure variable parameters to default values */
    NRF_RADIO->TXPOWER = NRFMAX_TXPOWER_DEFAULT;
    NRF_RADIO->FREQUENCY = NRFMAX_CHAN_DEFAULT;
	
    /* Pre-configure radio addresses */
	NRF_RADIO->PREFIX0 = (CONF_ADDR_PREFIX0 | nrfmax_eui64.uint8[3]);
	NRF_RADIO->BASE0 = ((nrfmax_eui64.uint8[4] << 24) |
						(nrfmax_eui64.uint8[5] << 16) |
						(nrfmax_eui64.uint8[6] << 8)  |
						(nrfmax_eui64.uint8[7]));
    NRF_RADIO->BASE1   = CONF_ADDR_BCAST;
	
    /* Always send from logical address 0 */
    NRF_RADIO->TXADDRESS = 0x00UL;
	
    /* And listen to logical addresses 0 and 1 */
    NRF_RADIO->RXADDRESSES = 0x03UL;
	
    /* Configure data fields and packet length whitening and endianess */
    NRF_RADIO->PCNF0 = ((CONF_S1 << RADIO_PCNF0_S1LEN_Pos) |
                        (CONF_S0 << RADIO_PCNF0_S0LEN_Pos) |
                        (CONF_LEN << RADIO_PCNF0_LFLEN_Pos));
    NRF_RADIO->PCNF1 = ((CONF_WHITENING << RADIO_PCNF1_WHITEEN_Pos) |
                        (CONF_ENDIAN << RADIO_PCNF1_ENDIAN_Pos) |
                        (CONF_BASE_ADDR_LEN << RADIO_PCNF1_BALEN_Pos) |
                        (CONF_STATLEN << RADIO_PCNF1_STATLEN_Pos) |
                        (NRFMAX_PKT_MAX << RADIO_PCNF1_MAXLEN_Pos));
						
    /* Configure the CRC unit, we skip the address field as this seems to lead
     * to wrong checksum calculation on nRF52 devices in some cases */
    NRF_RADIO->CRCCNF = CONF_CRC_LEN | RADIO_CRCCNF_SKIPADDR_Msk;
    NRF_RADIO->CRCPOLY = CONF_CRC_POLY;
    NRF_RADIO->CRCINIT = CONF_CRC_INIT;
	
    /* Set shortcuts for more efficient transfer */
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
	
    /* Enable interrupts */
    NVIC_EnableIRQ(RADIO_IRQn);
	
    /* Enable END interrupt */
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
	
    /* Put device in receive mode */
    nrfmax_target_state = STATE_RX;
    nrfmax_goto_target_state();

    DEBUG("[nrfmax] initialization successful\n");

    return 0;
}

/**
 * @brief   A driver's user-space ISR handler
 */
static void nrfmax_isr(netdev_t *dev)
{
	(void)dev;
	
    if (nrfmax_dev.event_callback) 
	{
        nrfmax_dev.event_callback(dev, NETDEV_EVENT_RX_COMPLETE, NULL);
    }
}

/**
 * @brief   Get an option value from a given network device 
 */
static int nrfmax_get(netdev_t *dev, netopt_t opt, void *val, size_t max_len)
{
    (void)dev;
    (void)max_len;

    switch (opt) {
        case NETOPT_CHANNEL:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t *)val) = nrfmax_get_channel();
            return sizeof(uint16_t);
        case NETOPT_CHANNEL_FREQUENCY:
            assert(max_len >= sizeof(uint32_t));
            *((uint32_t *)val) = nrfmax_get_frequency();
            return sizeof(uint32_t);	
        case NETOPT_ADDRESS:
            assert(max_len >= sizeof(eui64_t));
            nrfmax_get_eui64((eui64_t*)val);
            return sizeof(eui64_t);
        case NETOPT_STATE:
            assert(max_len >= sizeof(netopt_state_t));
            *((netopt_state_t *)val) = nrfmax_get_state();
            return sizeof(netopt_state_t);
        case NETOPT_TX_POWER:
            assert(max_len >= sizeof(int16_t));
            *((int16_t *)val) = nrfmax_get_txpower();
            return sizeof(int16_t);
        case NETOPT_MAX_PACKET_SIZE:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t *)val) = NRFMAX_PAYLOAD_MAX;
            return sizeof(uint16_t);
        case NETOPT_ADDRESS_LONG:
            assert(max_len >= sizeof(eui64_t));
            nrfmax_get_eui64((eui64_t*)val);
            return sizeof(eui64_t);
        case NETOPT_ADDR_LEN:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t *)val) = 8;
            return sizeof(uint16_t);
        case NETOPT_NID:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t*)val) = CONF_PSEUDO_NID;
            return sizeof(uint16_t);	
		case NETOPT_SRC_LEN:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t*)val) = 8;
            return sizeof(uint16_t);
#ifdef MODULE_GNRC_SIXLOWPAN
        case NETOPT_PROTO:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t *)val) = GNRC_NETTYPE_SIXLOWPAN;
            return sizeof(uint16_t);
#endif
        case NETOPT_DEVICE_TYPE:
            assert(max_len >= sizeof(uint16_t));
            *((uint16_t *)val) = NETDEV_TYPE_NRFMAX;
            return sizeof(uint16_t);
        case NETOPT_IPV6_IID:
            // assert(max_len >= sizeof(uint64_t));
            // nrfmax_get_iid((uint16_t *)val);
            // return sizeof(uint64_t);
        default:
            return -ENOTSUP;
    }
}

/**
 * @brief   Set an option value for a given network device
 */
static int nrfmax_set(netdev_t *dev, netopt_t opt, const void *val, size_t len)
{
    (void)dev;
    (void)len;

    switch (opt) {
        case NETOPT_CHANNEL:
            assert(len == sizeof(uint16_t));
            return nrfmax_set_channel(*((const uint16_t *)val));
        case NETOPT_ADDRESS:
            // assert(len == sizeof(eui64_t));
            // nrfmax_set_eui64(*((const eui64_t *)val)); 
            // return sizeof(eui64_t);
        case NETOPT_ADDR_LEN:
        case NETOPT_SRC_LEN:
            assert(len == sizeof(uint16_t));
            if (*((const uint16_t *)val) != 8) {
                return -EAFNOSUPPORT;
            }
            return sizeof(uint16_t);
        case NETOPT_STATE:
            assert(len == sizeof(netopt_state_t));
            return nrfmax_set_state(*((const netopt_state_t *)val));
        case NETOPT_TX_POWER:
            assert(len == sizeof(int16_t));
            nrfmax_set_txpower(*((const int16_t *)val));
            return sizeof(int16_t);
        default:
            return -ENOTSUP;
    }
}

/**
 * @brief   Export of the netdev interface. 
 *
 * @see 	netdev_driver_t
 */
const netdev_driver_t nrfmax_netdev = {
    .send = nrfmax_send,	/* Send frame */
    .recv = nrfmax_recv,	/* Get a received frame */
    .init = nrfmax_init,	/* The driver's initialization function */
    .isr  = nrfmax_isr,		/* A driver's user-space ISR handler */
    .get  = nrfmax_get,		/* Get an option value from a given network device */
    .set  = nrfmax_set		/* Set an option value for a given network device */
};
