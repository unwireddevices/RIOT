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
 * @file		ls_mac.c
 * @brief       LoRa Star MAC frame assembly/disassembly implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>

#include "ls-mac-types.h"
#include "ls-mac.h"
#include "ls-crypto.h"

void ls_assemble_frame(ls_addr_t addr, ls_type_t type, uint8_t *buf, size_t buflen, ls_frame_t *frame)
{
    memset(frame, 0, sizeof(ls_frame_t));

    frame->header.mhdr = MHDR_LORAWAN_RESERVED;
    frame->header.dev_addr = addr;
    frame->header.type = type;

    frame->payload.len = buflen;
    if (buflen > 0) {
        memcpy(frame->payload.data, buf, buflen);
    }
}

bool ls_validate_frame(uint8_t *buf, size_t buflen)
{
    /* Incoming frame less than minimum possible size */
    if (buflen < LS_FRAME_MINIMUM_SIZE) {
        return false;
    }

    /* Convert input buffer into frame structure */
    ls_frame_t *f = (ls_frame_t *) buf;

    /* LoRaWAN MHDR value is not as expected */
    if (f->header.mhdr != MHDR_LORAWAN_RESERVED) {
        return false;
    }

    return true;
}

#ifdef __cplusplus
}
#endif
