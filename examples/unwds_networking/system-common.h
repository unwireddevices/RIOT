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

#ifndef SYSTEM_COMMON_H
#define SYSTEM_COMMON_H

// #include "kernel_types.h"
#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Print IPv6 address */
void print_ipv6_addr(ipv6_addr_t *src_addr);

/* Print Unknown command for UMDK-module! */
void print_unknown_command_for_umdk(const char *module);

/* Print Send packet_name packet */
void print_send_packet(const char *packet_name);


/**
 * @brief Function for preparing the temp module for temperature measurement.
 *
 * This function initializes the TEMP module and writes to the hidden configuration register.
 */
void nrf_temp_init(void);

/**
 * @brief Function for reading temperature measurement.
 *
 * The function reads the 10 bit 2's complement value and transforms it to a 32 bit 2's complement value.
 *
 * @return  
 */
int32_t nrf_temp_read(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_COMMON_H */
/** @} */
