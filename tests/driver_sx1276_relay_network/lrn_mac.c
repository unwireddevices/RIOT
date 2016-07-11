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
 * @file        relay_mac.c
 * @brief       LoRa Relay Network MAC layer implementation
 * @author      Unwired Devices
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "lrn.h"
#include "lrn_crypto.h"
#include "lrn_mac.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Prepares frame to send.
 *
 * @param	[IN]	lrn		the LRN stack state
 * @param   [IN]    dest    destination address
 * @param   [IN]    src     source address
 * @param   [IN]    *buffer data to send
 * @param   [IN]    buflen  length of data buffer
 *
 * @return  composed frame data or NULL if invalid parameters passed
 */
lrn_frame_t prepare_frame(lrn_t *lrn, lrn_addr_t dest, lrn_addr_t src, lrn_addr_t next_hop, lrn_type_t type, uint8_t *buffer, uint8_t buflen)
{
    lrn_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    /* Prepare frame header */
    frame.header.mhdr = MHDR_LORAWAN_RESERVED;
    frame.header.dest_addr = dest;
    frame.header.src_addr = src;
    frame.header.next_hop = next_hop;

    frame.header.type = type;

    /* Prepare payload */
    frame.payload.len = buflen;
    if (buflen > 0) {
        memcpy(frame.payload.data, buffer, buflen);
    }

    return frame;
}

/**
 * Checks frame.
 *
 * Checks frame's validity
 *
 * @param	[IN]	*lrn	the LRN stack state
 * @param   [IN]    *buffer data buffer with frame data
 * @param   [IN]    buflen  length of the data buffer, must be not less than frame minimum size
 *
 * @return  true if frame is ok, false if frame is invalid
 */
bool check_frame(lrn_t *lrn, uint8_t *buffer, uint8_t buflen)
{
  /* Input buffer less than minimum */
  if (buflen < LRN_FRAME_MINIMUM_SIZE)
    return false;

  /* Convert input buffer into frame structure */
  lrn_frame_t* f = ((lrn_frame_t*) buffer);

  /* LoRaWAN MHDR value is not as expected */
  if (f->header.mhdr != MHDR_LORAWAN_RESERVED)
    return false;

  /* Frame's MIC is invalid */
  if (!validate_frame_mic(lrn, f))
    return false;

  printf("Frame: mhdr=0x%02X, mic=0x%04X, 0x%02X <- 0x%02X <- 0x%02X [0x%02X], type=0x%02X, fid=0x%02X\n", (unsigned int) f->header.mhdr,
		  (unsigned int) f->header.mic,
		  (unsigned int) f->header.dest_addr, (unsigned int) f->header.next_hop, (unsigned int) f->header.src_addr,
		  (unsigned int) f->header.retr_src,
		  (unsigned int) f->header.type, (unsigned int) f->header.fid);

  return true;
}


#ifdef __cplusplus
}
#endif
