/*
 * Copyright (C) 2016 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup
 * @ingroup
 * @brief
 * @{
 * @file        relay_mac.h
 * @brief       LoRa Relay Network MAC layer definitions
 * @author      Unwired Devices
 */
#ifndef TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_LRN_MAC_H_
#define TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_LRN_MAC_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "lrn_mac_types.h"
#include "lrn.h"

/**
 * Prepares frame to send.
 *
 * @param	[IN]	lrn			the LRN stack state
 * @param   [IN]    dest    	destination address
 * @param   [IN]    src			source address
 * @param	[IN]	next_hop	address of the next hop
 * @param   [IN]    *buffer 	data to send
 * @param   [IN]    buflen  	length of data buffer
 */
lrn_frame_t prepare_frame(lrn_t *lrn, lrn_addr_t dest, lrn_addr_t src, lrn_addr_t next_hop, lrn_type_t type, uint8_t *buffer, uint8_t buflen);

/**
 * Parse frame.
 *
 * Checks frame's MIC and composes lrn_frame_t structure from frame's data
 *
 * @param	[IN]	lrn		the LRN stack state
 * @param   [IN]    *buffer data buffer with frame data
 * @param   [IN]    buflen  length of the data buffer
 * @param   [OUT]   *frame  parsed frame
 *
 * @return  true if frame is ok, false if frame is invalid
 */
bool check_frame(lrn_t *lrn, uint8_t *buffer, uint8_t buflen);

#endif /* TESTS_DRIVER_SX1276_RELAY_NETWORK_LRN_LRN_MAC_H_ */
