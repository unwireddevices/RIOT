/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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

#if !defined(UNWDS_MAC_LORAWAN)

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

#endif

#ifdef __cplusplus
}
#endif
