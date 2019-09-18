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

// #include "unwds_udp.h"

#include "cpu.h"
#include "system-common.h"

// #include <stdio.h>
// #include <inttypes.h>

// #include "net/gnrc.h"
// #include "net/gnrc/ipv6.h"
// #include "net/gnrc/netif.h"
// #include "net/gnrc/netif/hdr.h"
// #include "net/gnrc/udp.h"
// #include "net/gnrc/pktdump.h"
// #include "timex.h"
// #include "utlist.h"
// #include "xtimer.h"


// #include <errno.h>
// #include "byteorder.h"
// #include "thread.h"
// #include "net/icmpv6.h"
// #include "net/ipv6/addr.h"
// #include "net/tcp.h"
// #include "net/sixlowpan.h"

#define ENABLE_DEBUG            (0)
#include "debug.h"
#include "od.h"

/**
* @defgroup nrf_temp_hal TEMP HAL
* @{
* @ingroup nrf_temp temperature_example
* @brief   Temperature module init and read functions.
*/
#define MASK_SIGN           (0x00000200UL)
#define MASK_SIGN_EXTENSION (0xFFFFFC00UL)

/* Print IPv6 address */
void print_ipv6_addr(ipv6_addr_t *addr)
{
	char addr_str[IPV6_ADDR_MAX_STR_LEN];
	printf("[%s] ", ipv6_addr_to_str(addr_str, addr, sizeof(addr_str)));
}

/* Print Unknown command for UMDK-module! */
void print_unknown_command_for_umdk(const char *module)
{
	printf("Unknown command for UMDK-%s!\n", module);
}

/* Print Send packet_name packet */
void print_send_packet(const char *packet_name)
{
    printf("Send %s packet\n", packet_name);
}


/**
 * @brief Function for preparing the temp module for temperature measurement.
 *
 * This function initializes the TEMP module and writes to the hidden configuration register.
 */
void nrf_temp_init(void)
{
    /**@note Workaround for PAN_028 rev2.0A anomaly 31 - TEMP: Temperature offset value has to be manually loaded to the TEMP module */
    *(uint32_t *) 0x4000C504 = 0;
}

/**
 * @brief Function for reading temperature measurement.
 *
 * The function reads the 10 bit 2's complement value and transforms it to a 32 bit 2's complement value.
 *
 * @return  
 */
int32_t nrf_temp_read(void)
{
	NRF_TEMP->EVENTS_DATARDY = 0;
	NRF_TEMP->TASKS_START = 1;
	while (!(NRF_TEMP->EVENTS_DATARDY)) {}
    
    /**@note Workaround for PAN_028 rev2.0A anomaly 28 - TEMP: Negative measured values are not represented correctly */
    return ((NRF_TEMP->TEMP & MASK_SIGN) != 0) ? (int32_t)(NRF_TEMP->TEMP | MASK_SIGN_EXTENSION) : (NRF_TEMP->TEMP);
}

uint8_t iterator_to_byte(uint8_t iterator)
{
	if(iterator <= 16)
		return 16;
	if((iterator > 16) && (iterator <= 32))
		return 32;
	if((iterator > 32) && (iterator <= 48))
		return 48;
	if((iterator > 48) && (iterator <= 64))
		return 64;
	if((iterator > 64) && (iterator <= 80))
		return 80;
	if((iterator > 80) && (iterator <= 96))
		return 96;
	if((iterator > 96) && (iterator <= 112))
		return 112;
	if((iterator > 112) && (iterator <= 128))
		return 128;
	return 0;
}